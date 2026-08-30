"""
================================================================================
SAEINDIA AeroTHON 2026 — Track 1: Rotorcraft Systems Challenge (UAS)
Project: Mission 2: SkyScan — Autonomous Rapid Delivery
Author: Bhakti Revankar
================================================================================
Hardware & Software Architecture:
  - Companion Computer: NVIDIA Jetson TX2 NX (Ubuntu 20.04)
  - Vision Sensor: Luxonis OAK-D Lite Stereo Depth Camera (DepthAI / Webcam)
  - Autopilot: Pixhawk 6X Mini (ArduPilot Copter 4.5+ via MAVLink / DroneKit)
  - Hoist Mechanism: Continuous Servo Winch on Pixhawk AUX 1 (Channel 9)

Mission Execution Loop (Full 14-State State Machine):
  1. TAKEOFF (5m) -> 2. SCAN_START_QR -> 3. FIND_GREEN_BANNER ->
  4. ENTER_CORRIDOR (10ft) -> 5. FOLLOW_CORRIDOR (Wall Centering) ->
  6. REACH_DELIVERY (10m) -> 7. SCAN_TARGET_QR (Avoid Red Zones) ->
  8. ALIGN_PAYLOAD (Visual Servoing) -> 9. DROP_PAYLOAD (Winch Drop + CoG Trim) ->
  10. ASCEND_RETURN (10m & 180° Turn) -> 11. ALIGN_RETURN_CORRIDOR (10ft) ->
  12. FOLLOW_CORRIDOR_RETURN -> 13. APPROACH_HOME -> 14. PRECISION_LAND
================================================================================
"""

from __future__ import annotations

import abc
import argparse
import csv
import dataclasses
from enum import Enum, auto
import logging
import math
import sys
import time
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np

# ---------------------------------------------------------------------------
# Logging Setup
# ---------------------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s][%(levelname)s][BhaktiRevankar-AeroTHON] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("BhaktiRevankar-AeroTHON2026")


# ===========================================================================
# 1. MISSION STATES & CONFIGURATION PARAMETERS
# ===========================================================================
class MissionState(Enum):
    """Complete Mission State Machine including Forward & Backward passes."""
    INIT                    = auto()
    TAKEOFF                 = auto()
    SCAN_START_QR           = auto()
    FIND_GREEN_BANNER       = auto()
    ENTER_CORRIDOR          = auto()
    FOLLOW_CORRIDOR         = auto()
    REACH_DELIVERY          = auto()
    SCAN_TARGET_QR          = auto()
    ALIGN_PAYLOAD           = auto()
    DROP_PAYLOAD            = auto()
    ASCEND_RETURN           = auto()
    ALIGN_RETURN_CORRIDOR   = auto()
    FOLLOW_CORRIDOR_RETURN  = auto()
    EXIT_CORRIDOR           = auto()
    APPROACH_HOME           = auto()
    PRECISION_LAND          = auto()
    AVOID_OBSTACLES         = auto()
    COMPLETE                = auto()


@dataclasses.dataclass(frozen=True)
class MissionConfig:
    """Mission Parameters as mandated by SAEINDIA AeroTHON 2026 Rulebook."""
    # Altitudes (meters above ground)
    TAKEOFF_ALT_M           : float = 5.0    # 5.0m takeoff & initial QR scan
    CORRIDOR_ALT_M          : float = 3.048  # 10 ft (~3.05m) corridor navigation altitude
    DELIVERY_SEARCH_ALT_M   : float = 10.0   # 10.0m delivery zone QR search altitude
    PAYLOAD_DROP_ALT_M      : float = 5.0    # 5.0m precision winch drop altitude
    RETURN_CRUISE_ALT_M     : float = 10.0   # 10.0m return climb altitude
    RETURN_CORRIDOR_ALT_M   : float = 3.048  # 10 ft return corridor traversal altitude

    # Velocities (m/s)
    CRUISE_SPEED_MPS        : float = 1.8    # Standard cruise speed
    CORRIDOR_SPEED_MPS      : float = 1.5    # Controlled corridor traverse speed
    SLOW_SPEED_MPS          : float = 0.8    # Approach and slow maneuver speed
    MAX_SPEED_MPS           : float = 2.5    # Safety limit for body velocity commands
    YAW_RATE_DPS            : float = 25.0   # Maximum yaw turn rate (deg/s)

    # Obstacle & Proximity Distances (meters)
    OBS_SAFE_DIST_M         : float = 2.0    # > 2.0m: Full cruise speed
    OBS_SLOW_DIST_M         : float = 1.2    # 1.2m - 2.0m: Half speed & lateral steer
    OBS_CRITICAL_STOP_M     : float = 0.8    # < 0.8m: Immediate emergency stop & lateral evade
    CORRIDOR_WIDTH_M        : float = 3.5    # Specified arena corridor width
    CORRIDOR_LENGTH_EST_M   : float = 15.0   # Nominal corridor length

    # Tolerances
    ALTITUDE_TOLERANCE_M    : float = 0.30   # Altitude convergence threshold
    POSITION_TOLERANCE_M    : float = 0.35   # Waypoint convergence radius
    HEADING_TOLERANCE_DEG   : float = 5.0    # Yaw alignment tolerance

    # Loop & Flight Timeouts (seconds)
    ARM_TIMEOUT_S           : float = 10.0   # Max seconds to wait for FC arming
    TAKEOFF_TIMEOUT_S       : float = 25.0   # Max seconds for takeoff altitude climb
    ALTITUDE_CHANGE_TIMEOUT_S: float = 18.0  # Max seconds for in-flight altitude changes
    LANDING_TIMEOUT_S       : float = 30.0   # Max seconds for touchdown descent
    FC_CONNECTION_TIMEOUT_S : float = 30.0   # MAVLink heartbeat connection timeout
    MAX_MISSION_TIME_S      : float = 900.0  # 15 minutes competition time limit

    # Winch & Payload Drop Mechanism
    WINCH_SERVO_CHANNEL     : int   = 9      # MAVLink AUX servo channel (Pixhawk Pin 9)
    WINCH_LOWER_PWM         : int   = 1700   # Lower hoist PWM (µs)
    WINCH_NEUTRAL_PWM       : int   = 1500   # Stop/Hold PWM (µs)
    WINCH_RAISE_PWM         : int   = 1300   # Retract cable PWM (µs)
    WINCH_RELEASE_PWM       : int   = 1900   # Payload detach trigger PWM (µs)
    WINCH_LOWER_TIME_S      : float = 4.5    # Lowering duration
    WINCH_HOLD_TIME_S       : float = 1.5    # Ground contact settle duration
    WINCH_REEL_TIME_S       : float = 4.5    # Cable retraction duration

    # Mass & Center of Gravity (CoG) Compensation
    PAYLOAD_MASS_KG         : float = 0.100  # 100g specified payload mass
    DRONE_EMPTY_MASS_KG     : float = 1.850  # Dry weight of UAS (< 2.0kg MTOW)
    PAYLOAD_COG_OFFSET_X_M  : float = 0.12   # Longitudinal distance from CoG to payload mount

    # Vision & QR Code Parameters
    QR_CONFIRM_COUNT        : int   = 3      # Consecutive frames to confirm delivery QR
    QR_SCAN_TIMEOUT_S       : float = 10.0   # Time before starting yaw scan pattern


CFG = MissionConfig()


# ===========================================================================
# 2. DUAL COORDINATE SYSTEM: GLOBAL (NED) & LOCAL SECTION TRACKER
# ===========================================================================
@dataclasses.dataclass
class GlobalCoordinate:
    """Global Position in North-East-Down (NED) World Frame from Launch Pad (0, 0, 0)."""
    x: float = 0.0        # +X: North / Forward (meters)
    y: float = 0.0        # +Y: East / Right (meters)
    z: float = 0.0        # +Z: Down (meters) -> Altitude = -z (when z <= 0)
    yaw_deg: float = 0.0  # Yaw angle in degrees (0° = Initial launch forward heading)

    @property
    def altitude(self) -> float:
        return -self.z if self.z < 0 else self.z


@dataclasses.dataclass
class LocalSectionCoordinate:
    """Local Coordinate relative to the active mission section."""
    section_name: str = "START_PAD"
    x_rel: float = 0.0
    y_rel: float = 0.0
    z_rel: float = 0.0
    distance_to_goal: float = 0.0
    extra_info: str = ""


@dataclasses.dataclass
class TrajectoryPoint:
    """A timestamped 3D waypoint on the stored flight trajectory."""
    timestamp: float
    x: float
    y: float
    z: float
    yaw_deg: float
    state_name: str


class CoordinateManager:
    """Manages Dual Coordinate Tracking, Landmark Memory, and Complete Flight Path Storage."""

    def __init__(self, config: MissionConfig = CFG) -> None:
        self.config = config
        self.global_pos = GlobalCoordinate()
        self.local_pos = LocalSectionCoordinate()

        self.landmarks: Dict[str, Tuple[float, float, float]] = {
            "HOME": (0.0, 0.0, 0.0),
            "START_QR": (1.0, 0.0, -self.config.TAKEOFF_ALT_M),
            "CORRIDOR_ENTRY": (3.0, 0.0, -self.config.CORRIDOR_ALT_M),
            "CORRIDOR_EXIT": (3.0 + self.config.CORRIDOR_LENGTH_EST_M, 0.0, -self.config.CORRIDOR_ALT_M),
            "DELIVERY_CENTER": (3.0 + self.config.CORRIDOR_LENGTH_EST_M + 5.0, 0.0, -self.config.DELIVERY_SEARCH_ALT_M),
            "TARGET_QR": (0.0, 0.0, 0.0),
        }

        # Complete Stored Flight Trajectories (Forward and Return Paths)
        self.forward_path: List[TrajectoryPoint] = []
        self.return_path: List[TrajectoryPoint] = []
        self.full_trajectory: List[TrajectoryPoint] = []

    def update_from_fc(self, x: float, y: float, z: float, yaw: float) -> None:
        self.global_pos.x = float(x)
        self.global_pos.y = float(y)
        self.global_pos.z = float(z)
        self.global_pos.yaw_deg = float(yaw)

    def record_path_point(self, state: MissionState, x: float, y: float, z: float, yaw: float) -> None:
        """Records every single coordinate along the flight path from start to finish."""
        pt = TrajectoryPoint(
            timestamp=time.time(),
            x=float(x),
            y=float(y),
            z=float(z),
            yaw_deg=float(yaw),
            state_name=state.name,
        )
        self.full_trajectory.append(pt)

        forward_states = {
            MissionState.INIT,
            MissionState.TAKEOFF,
            MissionState.SCAN_START_QR,
            MissionState.FIND_GREEN_BANNER,
            MissionState.ENTER_CORRIDOR,
            MissionState.FOLLOW_CORRIDOR,
            MissionState.REACH_DELIVERY,
            MissionState.SCAN_TARGET_QR,
            MissionState.ALIGN_PAYLOAD,
            MissionState.DROP_PAYLOAD,
        }
        if state in forward_states:
            self.forward_path.append(pt)
        else:
            self.return_path.append(pt)

    def get_reverse_return_waypoints(self) -> List[Tuple[float, float, float]]:
        """Returns the recorded forward path in reverse order as 3D (x, y, z) waypoints for return navigation."""
        return [(pt.x, pt.y, pt.z) for pt in reversed(self.forward_path)]

    def register_landmark(self, name: str, x_global: float, y_global: float, z_global: float) -> None:
        self.landmarks[name] = (float(x_global), float(y_global), float(z_global))

    def compute_local_coordinates(
        self,
        state: MissionState,
        d_left_wall: float = 0.0,
        d_right_wall: float = 0.0,
        fwd_dist: float = float("inf"),
        target_qr_box_dist: Optional[float] = None,
    ) -> LocalSectionCoordinate:
        gx, gy, gz = self.global_pos.x, self.global_pos.y, self.global_pos.z
        alt = self.global_pos.altitude

        if state in (MissionState.INIT, MissionState.TAKEOFF, MissionState.SCAN_START_QR):
            sec = "START_ZONE"
            x_rel = gx
            y_rel = gy
            z_rel = alt - self.config.TAKEOFF_ALT_M
            d_goal = max(0.0, 1.0 - gx)
            info = f"Alt: {alt:.2f}m / Goal: {self.config.TAKEOFF_ALT_M}m"

        elif state == MissionState.FIND_GREEN_BANNER:
            sec = "BANNER_APPROACH"
            entry_x = self.landmarks["CORRIDOR_ENTRY"][0]
            x_rel = gx
            y_rel = gy
            z_rel = alt - self.config.TAKEOFF_ALT_M
            d_goal = fwd_dist if not math.isinf(fwd_dist) else max(0.0, entry_x - gx)
            info = f"Banner Dist: {d_goal:.2f}m"

        elif state in (MissionState.ENTER_CORRIDOR, MissionState.FOLLOW_CORRIDOR):
            sec = "CORRIDOR_FWD"
            if d_left_wall > 0 and d_right_wall > 0:
                y_center_err = (d_right_wall - d_left_wall) / 2.0
            else:
                y_center_err = gy
            entry_x = self.landmarks["CORRIDOR_ENTRY"][0]
            x_corr = max(0.0, gx - entry_x)
            x_rel = x_corr
            y_rel = y_center_err
            z_rel = alt - self.config.CORRIDOR_ALT_M
            d_goal = (
                fwd_dist
                if (fwd_dist < 15.0 and not math.isinf(fwd_dist))
                else max(0.0, self.config.CORRIDOR_LENGTH_EST_M - x_corr)
            )
            info = f"Corridor progress: {x_corr:.1f}m/{self.config.CORRIDOR_LENGTH_EST_M:.1f}m | LatErr: {y_center_err:+.2f}m"

        elif state in (
            MissionState.REACH_DELIVERY,
            MissionState.SCAN_TARGET_QR,
            MissionState.ALIGN_PAYLOAD,
            MissionState.DROP_PAYLOAD,
        ):
            sec = "DELIVERY_ZONE"
            deliv_entry = self.landmarks["CORRIDOR_EXIT"]
            x_rel = gx - deliv_entry[0]
            y_rel = gy - deliv_entry[1]
            target_alt = (
                self.config.PAYLOAD_DROP_ALT_M
                if state in (MissionState.ALIGN_PAYLOAD, MissionState.DROP_PAYLOAD)
                else self.config.DELIVERY_SEARCH_ALT_M
            )
            z_rel = alt - target_alt
            d_goal = target_qr_box_dist if target_qr_box_dist is not None else 0.0
            info = f"Delivery Arena (X:{x_rel:+.2f}m, Y:{y_rel:+.2f}m) | QR Dist: {d_goal:.2f}m"

        elif state in (
            MissionState.ASCEND_RETURN,
            MissionState.ALIGN_RETURN_CORRIDOR,
            MissionState.FOLLOW_CORRIDOR_RETURN,
            MissionState.EXIT_CORRIDOR,
        ):
            sec = "CORRIDOR_RETURN"
            exit_x = self.landmarks["CORRIDOR_EXIT"][0]
            ret_progress = max(0.0, exit_x - gx)
            if d_left_wall > 0 and d_right_wall > 0:
                y_center_err = (d_right_wall - d_left_wall) / 2.0
            else:
                y_center_err = gy
            x_rel = ret_progress
            y_rel = y_center_err
            z_rel = alt - self.config.RETURN_CORRIDOR_ALT_M
            d_goal = max(0.0, gx)
            info = f"Return Progress: {ret_progress:.1f}m | To Home: {d_goal:.1f}m"

        elif state in (MissionState.APPROACH_HOME, MissionState.PRECISION_LAND):
            sec = "HOME_APPROACH"
            x_rel = gx
            y_rel = gy
            z_rel = alt
            d_goal = math.sqrt(gx**2 + gy**2)
            info = f"Dist to Home Pad: {d_goal:.2f}m | Alt: {alt:.2f}m"

        else:
            sec = state.name
            x_rel, y_rel, z_rel, d_goal = gx, gy, gz, 0.0
            info = ""

        self.local_pos = LocalSectionCoordinate(
            section_name=sec,
            x_rel=x_rel,
            y_rel=y_rel,
            z_rel=z_rel,
            distance_to_goal=d_goal,
            extra_info=info,
        )
        return self.local_pos


# ===========================================================================
# 3. CAMERA INTERFACE & LAZY DRIVERS (OAK-D / WEBCAM / MOCK)
# ===========================================================================
class CameraInterface(abc.ABC):
    @abc.abstractmethod
    def read(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        pass

    @abc.abstractmethod
    def release(self) -> None:
        pass


class MockCamera(CameraInterface):
    def __init__(self, width: int = 640, height: int = 480) -> None:
        self.width = width
        self.height = height
        self._custom_frame: Optional[np.ndarray] = None
        self._custom_depth: Optional[np.ndarray] = None

    def set_frame(self, bgr: np.ndarray, depth_mm: Optional[np.ndarray] = None) -> None:
        self._custom_frame = bgr
        self._custom_depth = depth_mm

    def read(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        if self._custom_frame is not None:
            depth = (
                self._custom_depth
                if self._custom_depth is not None
                else np.full((self.height, self.width), 3500, dtype=np.uint16)
            )
            return self._custom_frame.copy(), depth.copy()

        bgr = np.full((self.height, self.width, 3), 150, dtype=np.uint8)
        depth = np.full((self.height, self.width), 3500, dtype=np.uint16)
        depth[:, :80] = 1750
        depth[:, -80:] = 1750
        return bgr, depth

    def release(self) -> None:
        pass


class WebcamCamera(CameraInterface):
    def __init__(self, camera_index: int = 0, width: int = 640, height: int = 480) -> None:
        self.camera_index = camera_index
        self.width = width
        self.height = height
        self._cap: Optional[cv2.VideoCapture] = None

    def _ensure_opened(self) -> bool:
        if self._cap is None:
            self._cap = cv2.VideoCapture(self.camera_index)
            if self._cap.isOpened():
                self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        return self._cap.isOpened()

    def read(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        if not self._ensure_opened() or self._cap is None:
            return None, None
        ret, frame = self._cap.read()
        if not ret or frame is None:
            return None, None
        frame = cv2.resize(frame, (self.width, self.height))
        h, w = frame.shape[:2]
        depth = np.full((h, w), 3500, dtype=np.uint16)
        depth[:, :80] = 1750
        depth[:, -80:] = 1750
        return frame, depth

    def release(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None


class OakDCamera(CameraInterface):
    def __init__(self, rgb_fps: int = 30, depth_fps: int = 30, width: int = 640, height: int = 480) -> None:
        self.rgb_fps = rgb_fps
        self.depth_fps = depth_fps
        self.width = width
        self.height = height
        self._device: Any = None
        self._q_rgb: Any = None
        self._q_depth: Any = None
        self._fallback: Optional[WebcamCamera] = None
        self._initialized = False

    def _init_pipeline(self) -> None:
        if self._initialized:
            return
        self._initialized = True
        try:
            import depthai as dai  # type: ignore
            pipeline = dai.Pipeline()

            cam_rgb = pipeline.create(dai.node.ColorCamera)
            cam_rgb.setPreviewSize(self.width, self.height)
            cam_rgb.setInterleaved(False)
            cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
            cam_rgb.setFps(self.rgb_fps)

            mono_left = pipeline.create(dai.node.MonoCamera)
            mono_right = pipeline.create(dai.node.MonoCamera)
            mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
            mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
            mono_left.setCamera("left")
            mono_right.setCamera("right")
            mono_left.setFps(self.depth_fps)
            mono_right.setFps(self.depth_fps)

            stereo = pipeline.create(dai.node.StereoDepth)
            stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
            stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
            stereo.setOutputSize(self.width, self.height)
            stereo.setLeftRightCheck(True)
            stereo.setExtendedDisparity(False)
            stereo.setSubpixel(False)

            mono_left.out.link(stereo.left)
            mono_right.out.link(stereo.right)

            xout_rgb = pipeline.create(dai.node.XLinkOut)
            xout_depth = pipeline.create(dai.node.XLinkOut)
            xout_rgb.setStreamName("rgb")
            xout_depth.setStreamName("depth")

            cam_rgb.preview.link(xout_rgb.input)
            stereo.depth.link(xout_depth.input)

            self._device = dai.Device(pipeline)
            self._q_rgb = self._device.getOutputQueue("rgb", maxSize=4, blocking=False)
            self._q_depth = self._device.getOutputQueue("depth", maxSize=4, blocking=False)
            log.info("[CAMERA] OAK-D Lite Stereo Depth Camera initialized successfully.")
        except Exception as err:
            log.warning(f"[CAMERA] OAK-D initialization failed ({err}). Falling back to Webcam.")
            self._fallback = WebcamCamera(width=self.width, height=self.height)

    def read(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        self._init_pipeline()
        if self._fallback is not None:
            return self._fallback.read()

        if self._device is not None:
            in_rgb = self._q_rgb.tryGet()
            in_depth = self._q_depth.tryGet()
            if in_rgb is None or in_depth is None:
                return None, None
            rgb = in_rgb.getCvFrame()
            depth = in_depth.getFrame().astype(np.uint16)
            return rgb, depth

        return None, None

    def release(self) -> None:
        if self._fallback is not None:
            self._fallback.release()
            self._fallback = None
        if self._device is not None:
            del self._device
            self._device = None
        self._initialized = False


# ===========================================================================
# 4. STEREO DEPTH SAMPLING & REASONING
# ===========================================================================
DEPTH_UNKNOWN_M = float("inf")


def depth_at_bbox(
    depth_mm: Optional[np.ndarray],
    bbox: Tuple[int, int, int, int],
    percentile: float = 50.0,
) -> float:
    if depth_mm is None:
        return DEPTH_UNKNOWN_M
    x, y, w, h = bbox
    fh, fw = depth_mm.shape[:2]

    x1 = max(0, x)
    y1 = max(0, y)
    x2 = min(fw, x + w)
    y2 = min(fh, y + h)

    if x2 <= x1 or y2 <= y1:
        return DEPTH_UNKNOWN_M

    roi = depth_mm[y1:y2, x1:x2]
    valid = roi[roi > 0].astype(np.float32)
    if valid.size == 0:
        return DEPTH_UNKNOWN_M

    return float(np.percentile(valid, percentile)) / 1000.0


def depth_action(distance_m: float, config: MissionConfig = CFG) -> str:
    if math.isinf(distance_m):
        return "SLOW"
    if distance_m > config.OBS_SAFE_DIST_M:
        return "CONTINUE"
    elif distance_m > config.OBS_CRITICAL_STOP_M:
        return "SLOW"
    else:
        return "STOP"


# ===========================================================================
# 5. CYCLING GEOMETRIC SHAPES FOR PEOPLE DETECTION
# ===========================================================================
PEOPLE_SHAPES = ["circle", "ellipse", "hexagon", "rounded_box"]


def draw_circle(img: np.ndarray, x1: int, y1: int, x2: int, y2: int, color: tuple) -> None:
    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
    radius = max((x2 - x1), (y2 - y1)) // 2
    cv2.circle(img, (cx, cy), radius, color, 3)


def draw_ellipse(img: np.ndarray, x1: int, y1: int, x2: int, y2: int, color: tuple) -> None:
    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
    ax, ay = max(1, (x2 - x1) // 2), max(1, (y2 - y1) // 2)
    cv2.ellipse(img, (cx, cy), (ax, ay), 0, 0, 360, color, 3)


def draw_hexagon(img: np.ndarray, x1: int, y1: int, x2: int, y2: int, color: tuple) -> None:
    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
    w, h = max(1, (x2 - x1) // 2), max(1, (y2 - y1) // 2)
    pts = []
    for i in range(6):
        angle = math.pi / 3 * i - math.pi / 2
        px = int(cx + w * math.cos(angle))
        py = int(cy + h * math.sin(angle))
        pts.append([px, py])
    poly = np.array(pts, np.int32).reshape((-1, 1, 2))
    cv2.polylines(img, [poly], isClosed=True, color=color, thickness=3)


def draw_rounded_box(img: np.ndarray, x1: int, y1: int, x2: int, y2: int, color: tuple, r: int = 15) -> None:
    r = min(r, max(1, (x2 - x1) // 2), max(1, (y2 - y1) // 2))
    cv2.line(img, (x1 + r, y1), (x2 - r, y1), color, 3)
    cv2.line(img, (x1 + r, y2), (x2 - r, y2), color, 3)
    cv2.line(img, (x1, y1 + r), (x1, y2 - r), color, 3)
    cv2.line(img, (x2, y1 + r), (x2, y2 - r), color, 3)
    cv2.ellipse(img, (x1 + r, y1 + r), (r, r), 180, 0, 90, color, 3)
    cv2.ellipse(img, (x2 - r, y1 + r), (r, r), 270, 0, 90, color, 3)
    cv2.ellipse(img, (x1 + r, y2 - r), (r, r), 90, 0, 90, color, 3)
    cv2.ellipse(img, (x2 - r, y2 - r), (r, r), 0, 0, 90, color, 3)


SHAPE_FUNCS = {
    "circle": draw_circle,
    "ellipse": draw_ellipse,
    "hexagon": draw_hexagon,
    "rounded_box": draw_rounded_box,
}


# ===========================================================================
# 6. PURE PERCEPTION FUNCTIONS
# ===========================================================================
def preprocess_clahe(bgr: np.ndarray) -> np.ndarray:
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2Lab)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    l = clahe.apply(l)
    return cv2.cvtColor(cv2.merge([l, a, b]), cv2.COLOR_Lab2BGR)


BANNER_HSV_LOWER = np.array([35, 75, 50])
BANNER_HSV_UPPER = np.array([85, 255, 255])
BANNER_MIN_AREA = 2500
BANNER_MIN_AR = 1.3


def detect_green_banner(
    bgr: np.ndarray,
    yolo_model: Any = None,
    yolo_results: Any = None,
) -> Optional[Tuple[int, int, int, int]]:
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, BANNER_HSV_LOWER, BANNER_HSV_UPPER)
    kernel = np.ones((7, 7), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best_box = None
    best_area = 0

    for c in contours:
        area = cv2.contourArea(c)
        if area < BANNER_MIN_AREA:
            continue
        x, y, w, h = cv2.boundingRect(c)
        ar = w / max(h, 1)
        if ar < BANNER_MIN_AR:
            continue
        if area > best_area:
            best_area = area
            best_box = (x, y, w, h)

    if yolo_results is not None and yolo_model is not None:
        for r in yolo_results:
            for box in r.boxes:
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                name = yolo_model.names[cls]
                if conf > 0.55 and "banner" in name.lower():
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    area_yolo = (x2 - x1) * (y2 - y1)
                    if area_yolo > best_area:
                        best_box = (x1, y1, x2 - x1, y2 - y1)
                        best_area = area_yolo

    return best_box


def detect_qr_codes(bgr: np.ndarray) -> List[Tuple[str, np.ndarray]]:
    results: List[Tuple[str, np.ndarray]] = []
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    upscaled = cv2.resize(gray, None, fx=2.5, fy=2.5, interpolation=cv2.INTER_CUBIC)
    thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 31, 4)

    stages = [(gray, 1.0), (upscaled, 2.5), (thresh, 1.0)]

    try:
        from pyzbar.pyzbar import decode as zbar_decode  # type: ignore
        for img_data, sc in stages:
            for obj in zbar_decode(img_data):
                if obj.type != "QRCODE":
                    continue
                text = obj.data.decode("utf-8", errors="ignore").strip()
                if not text:
                    continue
                quad = np.array([[int(p.x / sc), int(p.y / sc)] for p in obj.polygon], dtype=int)
                results.append((text, quad))
            if results:
                return results
    except Exception:
        pass

    qr_detector = cv2.QRCodeDetector()
    for img_data, sc in stages:
        var = cv2.cvtColor(img_data, cv2.COLOR_GRAY2BGR) if img_data.ndim == 2 else img_data
        ok, decoded_info, points, _ = qr_detector.detectAndDecodeMulti(var)
        if ok and points is not None and decoded_info:
            for t, pts in zip(decoded_info, points):
                if t:
                    results.append((t, (pts / sc).astype(int)))
            if results:
                return results

    return results


RED_HUE_RANGES = [(0, 10), (170, 180)]
RED_SAT_MIN = 130
RED_VAL_MIN = 80
RED_ZONE_MIN_AREA = 500


def find_red_zones(bgr: np.ndarray) -> List[Tuple[int, int, int, int]]:
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lo, hi in RED_HUE_RANGES:
        mask |= (((h >= lo) & (h <= hi) & (s >= RED_SAT_MIN) & (v >= RED_VAL_MIN)).astype(np.uint8)) * 255
    kernel = np.ones((9, 9), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.dilate(mask, kernel, iterations=1)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    boxes = []
    for c in contours:
        if cv2.contourArea(c) >= RED_ZONE_MIN_AREA:
            boxes.append(cv2.boundingRect(c))
    return boxes


def detect_obstacles_with_depth(
    bgr: np.ndarray,
    depth_mm: Optional[np.ndarray],
    yolo_model: Any = None,
    yolo_results: Any = None,
    config: MissionConfig = CFG,
) -> List[dict]:
    frame = preprocess_clahe(bgr)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, w = frame.shape[:2]
    frame_area = h * w

    _, _, val = cv2.split(hsv)
    dark_mask = ((val <= 90).astype(np.uint8)) * 255
    contours, _ = cv2.findContours(dark_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    obstacles = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < 350 or area > frame_area * 0.20:
            continue
        perimeter = cv2.arcLength(c, True)
        if perimeter == 0:
            continue
        circularity = 4 * math.pi * area / (perimeter**2)
        if circularity < 0.25:
            continue
        (cx, cy), radius = cv2.minEnclosingCircle(c)
        cx, cy, radius = int(cx), int(cy), int(radius)
        bbox = (cx - radius, cy - radius, radius * 2, radius * 2)
        dist = depth_at_bbox(depth_mm, bbox)
        obstacles.append({
            "cx": cx,
            "cy": cy,
            "radius": radius,
            "bbox": bbox,
            "dist_m": dist,
            "action": depth_action(dist, config),
            "source": "classical_cv",
            "class_name": "obstacle",
        })

    if yolo_results is not None and yolo_model is not None:
        for r in yolo_results:
            for box in r.boxes:
                conf = float(box.conf[0])
                if conf < 0.45:
                    continue
                cls = int(box.cls[0])
                name = yolo_model.names[cls]
                if name == "person":
                    continue
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                bw, bh = x2 - x1, y2 - y1
                bbox = (x1, y1, bw, bh)
                dist = depth_at_bbox(depth_mm, bbox)
                obstacles.append({
                    "cx": x1 + bw // 2,
                    "cy": y1 + bh // 2,
                    "radius": max(bw, bh) // 2,
                    "bbox": bbox,
                    "dist_m": dist,
                    "action": depth_action(dist, config),
                    "source": "yolo",
                    "class_name": name,
                    "conf": conf,
                })

    return obstacles


# ===========================================================================
# 7. UNIFIED PERCEPTION PIPELINE CLASS
# ===========================================================================
class PerceptionPipeline:
    def __init__(self, config: MissionConfig = CFG, enable_yolo: bool = True) -> None:
        self.config = config
        self.enable_yolo = enable_yolo
        self._yolo_model: Any = None
        self._yolo_initialized = False

    def _lazy_load_yolo(self) -> None:
        if self._yolo_initialized or not self.enable_yolo:
            return
        self._yolo_initialized = True
        try:
            from ultralytics import YOLO  # type: ignore
            self._yolo_model = YOLO("yolov8n.pt")
            log.info("[AI PERCEPTION] YOLOv8 model loaded successfully.")
        except Exception as err:
            log.warning(f"[AI PERCEPTION] YOLOv8 not available ({err}). Falling back to Classical CV.")
            self._yolo_model = None

    def perceive(self, bgr: np.ndarray, depth_mm: Optional[np.ndarray]) -> Dict[str, Any]:
        self._lazy_load_yolo()

        _yolo_res = None
        humans: List[dict] = []
        yolo_all: List[tuple] = []

        if self._yolo_model is not None:
            try:
                _yolo_res = self._yolo_model(bgr, stream=False, verbose=False)
                for r in _yolo_res:
                    for box in r.boxes:
                        conf = float(box.conf[0])
                        if conf < 0.45:
                            continue
                        cls = int(box.cls[0])
                        name = self._yolo_model.names[cls]
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        bbox = (x1, y1, x2 - x1, y2 - y1)
                        dist = depth_at_bbox(depth_mm, bbox)
                        yolo_all.append((name, conf, bbox, dist))
                        if name == "person":
                            bw, bh = x2 - x1, y2 - y1
                            humans.append({
                                "bbox": bbox,
                                "dist_m": dist,
                                "action": depth_action(dist, self.config),
                                "conf": conf,
                                "cx": x1 + bw // 2,
                                "cy": y1 + bh // 2,
                            })
            except Exception as err:
                log.debug(f"YOLO inference error: {err}")

        obstacles = detect_obstacles_with_depth(bgr, depth_mm, self._yolo_model, _yolo_res, self.config)
        red_zones = find_red_zones(bgr)
        qr_codes = detect_qr_codes(bgr)
        banner_box = detect_green_banner(bgr, self._yolo_model, _yolo_res)

        h_f, w_f = bgr.shape[:2]
        col_w = w_f // 3
        edge_w = w_f // 5

        def _get_col_depth(x_s: int, width: int) -> float:
            if depth_mm is None:
                return 3.5
            box = (x_s, 0, width, h_f)
            return depth_at_bbox(depth_mm, box)

        d_left = _get_col_depth(0, col_w)
        d_centre = _get_col_depth(col_w, col_w)
        d_right = _get_col_depth(col_w * 2, col_w)

        d_left_wall = _get_col_depth(0, edge_w)
        d_right_wall = _get_col_depth(w_f - edge_w, edge_w)

        centre_stop = any(
            o["action"] == "STOP" and abs(o["cx"] - w_f // 2) < col_w // 2 for o in obstacles
        )
        human_stop = any(h["action"] == "STOP" for h in humans)
        red_ahead = any(col_w <= (rx + rw // 2) <= col_w * 2 for rx, ry, rw, rh in red_zones)

        if centre_stop or human_stop or d_centre < self.config.OBS_CRITICAL_STOP_M:
            steering = "STOP"
        elif red_ahead:
            steering = "LEFT" if d_left > d_right else "RIGHT"
        elif d_centre >= d_left and d_centre >= d_right:
            steering = "FORWARD"
        elif d_left > d_right:
            steering = "LEFT"
        else:
            steering = "RIGHT"

        return {
            "obstacles": obstacles,
            "humans": humans,
            "yolo_all": yolo_all,
            "red_zones": red_zones,
            "qr_codes": qr_codes,
            "banner_box": banner_box,
            "steering": steering,
            "fwd_dist_m": d_centre,
            "d_left": d_left,
            "d_right": d_right,
            "d_left_wall": d_left_wall,
            "d_right_wall": d_right_wall,
        }


# ===========================================================================
# 8. HUD & DISPLAY ANNOTATION
# ===========================================================================
def annotate_mission_frame(
    bgr: np.ndarray,
    percept: dict,
    state: MissionState,
    target_id: str,
    gpos: GlobalCoordinate,
    lpos: LocalSectionCoordinate,
    config: MissionConfig = CFG,
) -> np.ndarray:
    out = bgr.copy()
    h, w = out.shape[:2]

    for rx, ry, rw, rh in percept.get("red_zones", []):
        cv2.rectangle(out, (rx, ry), (rx + rw, ry + rh), (0, 0, 240), 3)
        cv2.putText(out, "RED ZONE", (rx, max(15, ry - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 240), 2)

    for obs in percept.get("obstacles", []):
        if obs.get("source") == "yolo":
            continue
        cx, cy, rad = obs["cx"], obs["cy"], obs["radius"]
        dist = obs["dist_m"]
        act = obs["action"]
        col = (0, 220, 0) if act == "CONTINUE" else ((0, 165, 255) if act == "SLOW" else (0, 0, 255))
        cv2.circle(out, (cx, cy), rad, col, 2)
        cv2.putText(out, f"OBS {dist:.1f}m [{act}]", (cx - rad, max(15, cy - rad - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 2)

    for name, conf, (x1, y1, bw, bh), dist in percept.get("yolo_all", []):
        if name == "person":
            continue
        col = (255, 200, 0)
        cv2.rectangle(out, (x1, y1), (x1 + bw, y1 + bh), col, 2)
        cv2.putText(out, f"{name} {conf:.2f} {dist:.1f}m", (x1, max(15, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 2)

    for idx, human in enumerate(percept.get("humans", [])):
        x1, y1, bw, bh = human["bbox"]
        x2, y2 = x1 + bw, y1 + bh
        dist = human["dist_m"]
        act = human["action"]
        shape_type = PEOPLE_SHAPES[idx % len(PEOPLE_SHAPES)]
        col = (255, 0, 220)
        SHAPE_FUNCS[shape_type](out, x1, y1, x2, y2, col)
        cv2.putText(out, f"HUMAN [{shape_type}] {dist:.1f}m [{act}]", (x1, max(15, y1 - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)

    if percept.get("banner_box") is not None:
        bx, by, bw, bh = percept["banner_box"]
        cv2.rectangle(out, (bx, by), (bx + bw, by + bh), (0, 255, 100), 3)
        cv2.putText(out, "GREEN BANNER", (bx, max(15, by - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 100), 2)

    for text, quad in percept.get("qr_codes", []):
        cv2.polylines(out, [quad.reshape(-1, 1, 2)], True, (255, 255, 0), 3)
        qx, qy = int(quad[0][0]), int(quad[0][1])
        match_tag = " [TARGET MATCH!]" if (text == target_id and target_id) else ""
        cv2.putText(out, f"QR: {text[:20]}{match_tag}", (qx, max(15, qy - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2)

    bar_y = h - 22
    col_w3 = w // 3

    def _bar_c(d: float) -> tuple:
        if d > config.OBS_SAFE_DIST_M:
            return (0, 200, 0)
        elif d > config.OBS_CRITICAL_STOP_M:
            return (0, 165, 255)
        return (0, 0, 255)

    d_l = percept.get("d_left", 3.5)
    d_c = percept.get("fwd_dist_m", 3.5)
    d_r = percept.get("d_right", 3.5)

    for i, (d, tag) in enumerate([(d_l, "LEFT"), (d_c, "CTR"), (d_r, "RIGHT")]):
        x0 = i * col_w3
        cv2.rectangle(out, (x0, bar_y - 10), (x0 + col_w3, bar_y), _bar_c(d), -1)
        cv2.putText(out, f"{tag}: {d:.1f}m", (x0 + 6, bar_y - 1), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 255), 1)

    st = percept.get("steering", "FORWARD")
    st_col = (0, 255, 0) if st == "FORWARD" else ((0, 200, 255) if st in ("LEFT", "RIGHT") else (0, 0, 255))
    cv2.putText(out, f"STEER: {st}", (w // 2 - 50, h - 35), cv2.FONT_HERSHEY_SIMPLEX, 0.65, st_col, 2)

    cv2.rectangle(out, (0, 0), (w, 58), (20, 20, 20), -1)
    cv2.putText(out, f"STATE: {state.name} | PILOT: Bhakti Revankar", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 240, 240), 2)
    cv2.putText(out, f"TGT QR: {target_id if target_id else 'SEARCHING'}", (w - 240, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 220, 0), 2)

    global_str = f"GLOBAL NED: X:{gpos.x:+.1f}m | Y:{gpos.y:+.1f}m | Alt:{gpos.altitude:.1f}m | Yaw:{gpos.yaw_deg:.0f}°"
    local_str = f"LOCAL [{lpos.section_name}]: X_rel:{lpos.x_rel:+.2f}m | Y_lat:{lpos.y_rel:+.2f}m | Rem:{lpos.distance_to_goal:.1f}m"
    cv2.putText(out, global_str, (10, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180, 255, 180), 1)
    cv2.putText(out, local_str, (10, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 200, 150), 1)

    return out


# ===========================================================================
# 9. FLIGHT CONTROLLER INTERFACE & MOCKS
# ===========================================================================
class FlightControllerInterface(abc.ABC):
    @abc.abstractmethod
    def get_position_ned(self) -> Tuple[float, float, float, float]:
        pass

    @abc.abstractmethod
    def send_body_velocity(self, vx: float, vy: float, vz: float, duration: float = 0.2) -> None:
        pass

    @abc.abstractmethod
    def send_local_velocity(self, vx_n: float, vy_e: float, vz_d: float, duration: float = 0.2) -> None:
        pass

    @abc.abstractmethod
    def arm_and_takeoff(self, target_alt: float) -> bool:
        pass

    @abc.abstractmethod
    def set_altitude(self, target_alt: float) -> bool:
        pass

    @abc.abstractmethod
    def set_yaw(self, heading_deg: float, relative: bool = True) -> None:
        pass

    @abc.abstractmethod
    def set_winch(self, pwm_us: int) -> None:
        pass

    @abc.abstractmethod
    def stop(self) -> None:
        pass

    @abc.abstractmethod
    def return_to_launch(self) -> None:
        pass

    @abc.abstractmethod
    def land(self) -> bool:
        pass

    @abc.abstractmethod
    def close(self) -> None:
        pass


class MockFlightController(FlightControllerInterface):
    def __init__(self, config: MissionConfig = CFG) -> None:
        self.config = config
        self.sim_x = 0.0
        self.sim_y = 0.0
        self.sim_z = 0.0
        self.sim_yaw = 0.0
        self.armed = False
        self.mode = "GUIDED"
        self.command_log: List[str] = []
        self.winch_pwm_history: List[int] = []

    def get_position_ned(self) -> Tuple[float, float, float, float]:
        return (self.sim_x, self.sim_y, self.sim_z, self.sim_yaw)

    def send_body_velocity(self, vx: float, vy: float, vz: float, duration: float = 0.2) -> None:
        vx = max(-self.config.MAX_SPEED_MPS, min(self.config.MAX_SPEED_MPS, vx))
        vy = max(-self.config.MAX_SPEED_MPS, min(self.config.MAX_SPEED_MPS, vy))
        vz = max(-self.config.MAX_SPEED_MPS, min(self.config.MAX_SPEED_MPS, vz))

        yaw_rad = math.radians(self.sim_yaw)
        gx = vx * math.cos(yaw_rad) - vy * math.sin(yaw_rad)
        gy = vx * math.sin(yaw_rad) + vy * math.cos(yaw_rad)

        self.sim_x += gx * duration
        self.sim_y += gy * duration
        self.sim_z += vz * duration
        self.command_log.append(f"BODY_VEL vx={vx:.2f} vy={vy:.2f} vz={vz:.2f} dt={duration:.2f}")

    def send_local_velocity(self, vx_n: float, vy_e: float, vz_d: float, duration: float = 0.2) -> None:
        self.sim_x += vx_n * duration
        self.sim_y += vy_e * duration
        self.sim_z += vz_d * duration
        self.command_log.append(f"LOCAL_VEL vx={vx_n:.2f} vy={vy_e:.2f} vz={vz_d:.2f} dt={duration:.2f}")

    def arm_and_takeoff(self, target_alt: float) -> bool:
        self.armed = True
        self.sim_z = -target_alt
        self.command_log.append(f"TAKEOFF alt={target_alt:.2f}")
        log.info(f"[MOCK FC] Armed and climbed to {target_alt:.1f} m.")
        return True

    def set_altitude(self, target_alt: float) -> bool:
        self.sim_z = -target_alt
        self.command_log.append(f"SET_ALT alt={target_alt:.2f}")
        log.info(f"[MOCK FC] Set altitude -> {target_alt:.1f} m.")
        return True

    def set_yaw(self, heading_deg: float, relative: bool = True) -> None:
        if relative:
            self.sim_yaw = (self.sim_yaw + heading_deg) % 360.0
        else:
            self.sim_yaw = heading_deg % 360.0
        self.command_log.append(f"SET_YAW deg={heading_deg:.1f} rel={relative}")
        log.info(f"[MOCK FC] Yaw set to {self.sim_yaw:.1f}°.")

    def set_winch(self, pwm_us: int) -> None:
        self.winch_pwm_history.append(pwm_us)
        self.command_log.append(f"WINCH pwm={pwm_us}")
        log.info(f"[MOCK FC] Winch PWM -> {pwm_us} µs.")

    def stop(self) -> None:
        self.command_log.append("STOP")

    def return_to_launch(self) -> None:
        self.mode = "RTL"
        self.sim_x = 0.0
        self.sim_y = 0.0
        self.sim_z = 0.0
        self.command_log.append("RTL")
        log.info("[MOCK FC] Return to Launch executed.")

    def land(self) -> bool:
        self.mode = "LAND"
        self.sim_z = 0.0
        self.armed = False
        self.command_log.append("LAND")
        log.info("[MOCK FC] Landed and disarmed.")
        return True

    def close(self) -> None:
        self.command_log.append("CLOSE")


class MAVLinkFlightController(FlightControllerInterface):
    def __init__(self, connection_string: str, config: MissionConfig = CFG) -> None:
        self.config = config
        self.connection_string = connection_string
        self._vehicle: Any = None
        self._connect()

    def _connect(self) -> None:
        from dronekit import connect as dk_connect  # type: ignore
        log.info(f"[FC] Connecting to Flight Controller at '{self.connection_string}'...")
        start_time = time.time()
        self._vehicle = dk_connect(
            self.connection_string,
            wait_ready=True,
            heartbeat_timeout=int(self.config.FC_CONNECTION_TIMEOUT_S),
        )
        elapsed = time.time() - start_time
        log.info(f"[FC] Connected in {elapsed:.1f}s. Firmware: {self._vehicle.version}")

    def get_position_ned(self) -> Tuple[float, float, float, float]:
        if self._vehicle is None:
            return (0.0, 0.0, 0.0, 0.0)
        loc = self._vehicle.location.local_frame
        alt = float(self._vehicle.location.global_relative_frame.alt or 0.0)
        yaw = float(self._vehicle.attitude.yaw * 180.0 / math.pi) if self._vehicle.attitude else 0.0
        x = float(loc.north) if loc and loc.north is not None else 0.0
        y = float(loc.east) if loc and loc.east is not None else 0.0
        z = -alt
        return (x, y, z, yaw)

    def send_body_velocity(self, vx: float, vy: float, vz: float, duration: float = 0.2) -> None:
        if self._vehicle is None:
            return
        from pymavlink import mavutil  # type: ignore

        vx = max(-self.config.MAX_SPEED_MPS, min(self.config.MAX_SPEED_MPS, vx))
        vy = max(-self.config.MAX_SPEED_MPS, min(self.config.MAX_SPEED_MPS, vy))
        vz = max(-self.config.MAX_SPEED_MPS, min(self.config.MAX_SPEED_MPS, vz))

        msg = self._vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111,
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0, 0,
        )
        end_time = time.time() + duration
        while time.time() < end_time:
            self._vehicle.send_mavlink(msg)
            time.sleep(0.1)

    def send_local_velocity(self, vx_n: float, vy_e: float, vz_d: float, duration: float = 0.2) -> None:
        if self._vehicle is None:
            return
        from pymavlink import mavutil  # type: ignore

        msg = self._vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111,
            0, 0, 0,
            vx_n, vy_e, vz_d,
            0, 0, 0,
            0, 0,
        )
        end_time = time.time() + duration
        while time.time() < end_time:
            self._vehicle.send_mavlink(msg)
            time.sleep(0.1)

    def arm_and_takeoff(self, target_alt: float) -> bool:
        if self._vehicle is None:
            return False
        from dronekit import VehicleMode  # type: ignore

        log.info(f"[FC] Setting mode GUIDED and arming (Timeout: {self.config.ARM_TIMEOUT_S}s)...")
        self._vehicle.mode = VehicleMode("GUIDED")
        self._vehicle.armed = True

        arm_start = time.time()
        while not self._vehicle.armed:
            if time.time() - arm_start > self.config.ARM_TIMEOUT_S:
                log.error("[FC TIMEOUT] Arming timed out!")
                return False
            time.sleep(0.3)

        log.info(f"[FC] Taking off to {target_alt:.1f} m (Timeout: {self.config.TAKEOFF_TIMEOUT_S}s)...")
        self._vehicle.simple_takeoff(target_alt)

        climb_start = time.time()
        while True:
            if time.time() - climb_start > self.config.TAKEOFF_TIMEOUT_S:
                log.error("[FC TIMEOUT] Takeoff altitude climb timed out!")
                return False
            cur_alt = float(self._vehicle.location.global_relative_frame.alt or 0.0)
            if cur_alt >= target_alt - self.config.ALTITUDE_TOLERANCE_M:
                break
            time.sleep(0.2)

        log.info(f"[FC] Takeoff complete at {target_alt:.1f} m.")
        return True

    def set_altitude(self, target_alt: float) -> bool:
        if self._vehicle is None:
            return False
        from dronekit import LocationGlobalRelative  # type: ignore

        log.info(f"[FC] Changing altitude to {target_alt:.1f} m...")
        cur = self._vehicle.location.global_relative_frame
        target = LocationGlobalRelative(cur.lat, cur.lon, target_alt)
        self._vehicle.simple_goto(target)

        alt_start = time.time()
        while abs(float(self._vehicle.location.global_relative_frame.alt or 0.0) - target_alt) > self.config.ALTITUDE_TOLERANCE_M:
            if time.time() - alt_start > self.config.ALTITUDE_CHANGE_TIMEOUT_S:
                log.warning("[FC TIMEOUT] Altitude transition timed out; proceeding with current altitude.")
                return False
            time.sleep(0.2)
        return True

    def set_yaw(self, heading_deg: float, relative: bool = True) -> None:
        if self._vehicle is None:
            return
        from pymavlink import mavutil  # type: ignore

        msg = self._vehicle.message_factory.command_long_encode(
            0, 0,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            heading_deg,
            self.config.YAW_RATE_DPS,
            1 if relative else 0,
            1 if relative else 0,
            0, 0, 0,
        )
        self._vehicle.send_mavlink(msg)
        time.sleep(1.0)

    def set_winch(self, pwm_us: int) -> None:
        if self._vehicle is None:
            return
        from pymavlink import mavutil  # type: ignore

        msg = self._vehicle.message_factory.command_long_encode(
            0, 0,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,
            self.config.WINCH_SERVO_CHANNEL,
            pwm_us,
            0, 0, 0, 0, 0,
        )
        self._vehicle.send_mavlink(msg)

    def stop(self) -> None:
        self.send_body_velocity(0, 0, 0, 0.2)

    def return_to_launch(self) -> None:
        if self._vehicle is None:
            return
        from dronekit import VehicleMode  # type: ignore

        log.warning("[FC] Triggering RTL (Return to Launch)!")
        self._vehicle.mode = VehicleMode("RTL")

    def land(self) -> bool:
        if self._vehicle is None:
            return False
        from dronekit import VehicleMode  # type: ignore

        log.info(f"[FC] Switching to LAND mode (Timeout: {self.config.LANDING_TIMEOUT_S}s)...")
        self._vehicle.mode = VehicleMode("LAND")
        land_start = time.time()
        while float(self._vehicle.location.global_relative_frame.alt or 0.0) > 0.15:
            if time.time() - land_start > self.config.LANDING_TIMEOUT_S:
                log.warning("[FC TIMEOUT] Landing descent timed out.")
                return False
            time.sleep(0.3)
        log.info("[FC] Landed.")
        return True

    def close(self) -> None:
        if self._vehicle is not None:
            self._vehicle.close()
            self._vehicle = None


def create_flight_controller(
    connection_string: str = "",
    enable_real_flight: bool = False,
    config: MissionConfig = CFG,
) -> FlightControllerInterface:
    if not enable_real_flight or not connection_string:
        if connection_string and not enable_real_flight:
            log.warning(
                "[SAFETY WARNING] --connect was provided but --enable-real-flight is FALSE. "
                "Defaulting to safe MockFlightController. Pass --enable-real-flight to arm physical hardware."
            )
        return MockFlightController(config=config)

    try:
        return MAVLinkFlightController(connection_string=connection_string, config=config)
    except Exception as err:
        log.error(f"Failed to connect to MAVLink Flight Controller ({err}). Falling back to MockFlightController.")
        return MockFlightController(config=config)


# ===========================================================================
# 10. WINCH SEQUENCE & COG COMPENSATION
# ===========================================================================
def execute_winch_delivery(fc: FlightControllerInterface, config: MissionConfig = CFG) -> None:
    log.info("[WINCH] Commencing controlled payload lowering...")
    fc.set_winch(config.WINCH_LOWER_PWM)
    time.sleep(config.WINCH_LOWER_TIME_S)

    fc.set_winch(config.WINCH_NEUTRAL_PWM)
    time.sleep(config.WINCH_HOLD_TIME_S)

    log.info("[WINCH] Triggering mechanical release latch...")
    fc.set_winch(config.WINCH_RELEASE_PWM)
    time.sleep(0.6)
    fc.set_winch(config.WINCH_NEUTRAL_PWM)
    time.sleep(0.4)

    log.info("[WINCH] Retracting cable...")
    fc.set_winch(config.WINCH_RAISE_PWM)
    time.sleep(config.WINCH_REEL_TIME_S)
    fc.set_winch(config.WINCH_NEUTRAL_PWM)
    log.info("[WINCH] Hoist sequence completed.")

    cg_shift_m = (config.PAYLOAD_MASS_KG * config.PAYLOAD_COG_OFFSET_X_M) / config.DRONE_EMPTY_MASS_KG
    log.info(f"[COG] Post-drop CoG shift = {cg_shift_m * 1000.0:.2f} mm. Applying feed-forward forward pitch bias.")
    fc.send_body_velocity(vx=cg_shift_m * 0.8, vy=0.0, vz=0.0, duration=1.5)
    time.sleep(0.5)


# ===========================================================================
# 11. FLIGHT DATA RECORDER (CSV EXPORT)
# ===========================================================================
class FlightDataRecorder:
    def __init__(self, log_path: str = "flight_data_log.csv") -> None:
        self.filepath = log_path
        self.file = open(self.filepath, mode="w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            "timestamp", "state", "global_x", "global_y", "global_z", "altitude",
            "yaw_deg", "section", "local_x", "local_y", "local_z", "dist_to_goal",
            "fwd_depth_m", "left_wall_m", "right_wall_m", "target_id", "steering",
            "num_obstacles", "num_humans", "author"
        ])

    def record(
        self,
        state: MissionState,
        gpos: GlobalCoordinate,
        lpos: LocalSectionCoordinate,
        percept: Dict[str, Any],
        target_id: str = "",
    ) -> None:
        self.writer.writerow([
            f"{time.time():.3f}",
            state.name,
            f"{gpos.x:.3f}", f"{gpos.y:.3f}", f"{gpos.z:.3f}", f"{gpos.altitude:.3f}",
            f"{gpos.yaw_deg:.1f}", lpos.section_name,
            f"{lpos.x_rel:.3f}", f"{lpos.y_rel:.3f}", f"{lpos.z_rel:.3f}", f"{lpos.distance_to_goal:.3f}",
            f"{percept.get('fwd_dist_m', 0.0):.2f}",
            f"{percept.get('d_left_wall', 0.0):.2f}",
            f"{percept.get('d_right_wall', 0.0):.2f}",
            target_id,
            percept.get("steering", "FORWARD"),
            len(percept.get("obstacles", [])),
            len(percept.get("humans", [])),
            "Bhakti Revankar",
        ])
        self.file.flush()

    def close(self) -> None:
        if self.file and not self.file.closed:
            self.file.close()


# ===========================================================================
# 12. FULL MISSION FSM ENGINE (FORWARD & BACKWARD PASS)
# ===========================================================================
class MissionFSM:
    def __init__(
        self,
        fc: FlightControllerInterface,
        camera: CameraInterface,
        coord_mgr: Optional[CoordinateManager] = None,
        perception: Optional[PerceptionPipeline] = None,
        recorder: Optional[FlightDataRecorder] = None,
        config: MissionConfig = CFG,
    ) -> None:
        self.fc = fc
        self.camera = camera
        self.config = config
        self.coord_mgr = coord_mgr or CoordinateManager(config=config)
        self.perception = perception or PerceptionPipeline(config=config)
        self.recorder = recorder

        self.state = MissionState.TAKEOFF
        self.target_id = ""
        self.qr_fail_cnt = 0
        self.qr_confirm_cnt = 0
        self.banner_search_cnt = 0
        self.state_before_avoid = MissionState.FOLLOW_CORRIDOR
        self.mission_start_time = time.time()
        self.step_count = 0

    def step(self, bgr: np.ndarray, depth_mm: Optional[np.ndarray]) -> Dict[str, Any]:
        self.step_count += 1

        gx, gy, gz, yaw = self.fc.get_position_ned()
        self.coord_mgr.update_from_fc(gx, gy, gz, yaw)
        self.coord_mgr.record_path_point(self.state, gx, gy, gz, yaw)

        percept = self.perception.perceive(bgr, depth_mm)
        obstacles = percept["obstacles"]
        humans = percept["humans"]
        red_zones = percept["red_zones"]
        qr_codes = percept["qr_codes"]
        banner_box = percept["banner_box"]
        steering = percept["steering"]
        fwd_dist = percept["fwd_dist_m"]
        d_left_w = percept["d_left_wall"]
        d_right_w = percept["d_right_wall"]

        tgt_dist = None
        for txt, qd in qr_codes:
            if txt == self.target_id and self.target_id:
                qx, qy = int(np.mean(qd[:, 0])), int(np.mean(qd[:, 1]))
                tgt_dist = depth_at_bbox(depth_mm, (qx - 30, qy - 30, 60, 60))
                break

        lpos = self.coord_mgr.compute_local_coordinates(self.state, d_left_w, d_right_w, fwd_dist, tgt_dist)

        if self.recorder is not None:
            self.recorder.record(self.state, self.coord_mgr.global_pos, lpos, percept, self.target_id)

        safe_exempt = {
            MissionState.INIT,
            MissionState.TAKEOFF,
            MissionState.PRECISION_LAND,
            MissionState.DROP_PAYLOAD,
            MissionState.AVOID_OBSTACLES,
            MissionState.COMPLETE,
        }
        if self.state not in safe_exempt:
            threats = [t for t in (obstacles + humans) if t.get("action") == "STOP"]
            if threats:
                self.state_before_avoid = self.state
                log.warning(f"[SAFETY] Emergency STOP threat in {self.state.name} -> Entering AVOID_OBSTACLES!")
                self.fc.stop()
                self.state = MissionState.AVOID_OBSTACLES

        if self.state == MissionState.TAKEOFF:
            self.fc.arm_and_takeoff(self.config.TAKEOFF_ALT_M)
            log.info("[TAKEOFF] Target altitude reached -> SCAN_START_QR")
            self.state = MissionState.SCAN_START_QR

        elif self.state == MissionState.SCAN_START_QR:
            if qr_codes:
                self.target_id = qr_codes[0][0]
                self.coord_mgr.register_landmark(
                    "START_QR",
                    self.coord_mgr.global_pos.x,
                    self.coord_mgr.global_pos.y,
                    self.coord_mgr.global_pos.z,
                )
                log.info(f"[START_QR] Target Decoded: '{self.target_id}' -> FIND_GREEN_BANNER")
                self.state = MissionState.FIND_GREEN_BANNER
                self.qr_fail_cnt = 0
            else:
                self.qr_fail_cnt += 1
                if self.qr_fail_cnt < 8:
                    self.fc.send_body_velocity(vx=0.4, vy=0.0, vz=0.0, duration=0.3)
                elif self.qr_fail_cnt < 20:
                    self.fc.set_yaw(20.0, relative=True)
                else:
                    log.warning("[START_QR] Search timeout. Defaulting target to 'TARGET_A'")
                    self.target_id = "TARGET_A"
                    self.state = MissionState.FIND_GREEN_BANNER

        elif self.state == MissionState.FIND_GREEN_BANNER:
            if banner_box is not None:
                bx, by, bw, bh = banner_box
                frame_cx = bgr.shape[1] // 2
                banner_cx = bx + bw // 2
                banner_dist = depth_at_bbox(depth_mm, banner_box)
                if math.isinf(banner_dist) or banner_dist > 40.0:
                    banner_dist = 6.0

                lateral_err_px = banner_cx - frame_cx
                log.info(f"[BANNER] Green Banner locked: Dist={banner_dist:.2f}m, CenterErr={lateral_err_px}px")

                if abs(lateral_err_px) > 35:
                    vy = max(-0.4, min(0.4, 0.003 * lateral_err_px))
                    self.fc.send_body_velocity(vx=0.0, vy=vy, vz=0.0, duration=0.3)
                elif banner_dist > 2.5:
                    self.fc.send_body_velocity(vx=self.config.SLOW_SPEED_MPS, vy=0.0, vz=0.0, duration=0.3)
                else:
                    self.coord_mgr.register_landmark(
                        "CORRIDOR_ENTRY",
                        self.coord_mgr.global_pos.x,
                        self.coord_mgr.global_pos.y,
                        self.coord_mgr.global_pos.z,
                    )
                    log.info("[BANNER] Corridor entry reached. Descending to 10 ft (3m) -> ENTER_CORRIDOR")
                    self.fc.set_altitude(self.config.CORRIDOR_ALT_M)
                    self.state = MissionState.ENTER_CORRIDOR
                self.banner_search_cnt = 0
            else:
                self.banner_search_cnt += 1
                self.fc.set_yaw(15.0, relative=True)
                if self.banner_search_cnt > 24:
                    log.warning("[BANNER] Full 360° sweep completed. Forcing corridor entry.")
                    self.fc.set_altitude(self.config.CORRIDOR_ALT_M)
                    self.state = MissionState.ENTER_CORRIDOR

        elif self.state == MissionState.ENTER_CORRIDOR:
            if d_left_w < 3.0 and d_right_w < 3.0:
                log.info(f"[ENTER_CORRIDOR] Walls detected (L:{d_left_w:.2f}m, R:{d_right_w:.2f}m) -> FOLLOW_CORRIDOR")
                self.state = MissionState.FOLLOW_CORRIDOR
            else:
                self.fc.send_body_velocity(vx=self.config.SLOW_SPEED_MPS, vy=0.0, vz=0.0, duration=0.3)

        elif self.state == MissionState.FOLLOW_CORRIDOR:
            lat_center_err = (d_right_w - d_left_w) / 2.0 if (d_left_w > 0 and d_right_w > 0) else 0.0
            vy_corr = max(-0.4, min(0.4, 0.45 * lat_center_err))

            if (fwd_dist > 6.0 and d_left_w > 3.0 and d_right_w > 3.0) or (lpos.x_rel >= self.config.CORRIDOR_LENGTH_EST_M):
                self.coord_mgr.register_landmark(
                    "CORRIDOR_EXIT",
                    self.coord_mgr.global_pos.x,
                    self.coord_mgr.global_pos.y,
                    self.coord_mgr.global_pos.z,
                )
                log.info("[CORRIDOR_FWD] Exited corridor -> REACH_DELIVERY")
                self.fc.stop()
                self.state = MissionState.REACH_DELIVERY
            elif fwd_dist < self.config.OBS_SLOW_DIST_M:
                self.fc.send_body_velocity(vx=self.config.SLOW_SPEED_MPS * 0.5, vy=vy_corr, vz=0.0, duration=0.3)
            else:
                self.fc.send_body_velocity(vx=self.config.CORRIDOR_SPEED_MPS, vy=vy_corr, vz=0.0, duration=0.3)

        elif self.state == MissionState.REACH_DELIVERY:
            log.info(f"[DELIVERY] Ascending to {self.config.DELIVERY_SEARCH_ALT_M:.1f} m search altitude -> SCAN_TARGET_QR")
            self.fc.set_altitude(self.config.DELIVERY_SEARCH_ALT_M)
            self.state = MissionState.SCAN_TARGET_QR
            self.qr_confirm_cnt = 0

        elif self.state == MissionState.SCAN_TARGET_QR:
            matched_qr = None
            for txt, quad in qr_codes:
                if txt != self.target_id:
                    continue
                qx, qy = int(np.mean(quad[:, 0])), int(np.mean(quad[:, 1]))
                in_red = any(rx <= qx <= rx + rw and ry <= qy <= ry + rh for rx, ry, rw, rh in red_zones)
                if in_red:
                    log.warning(f"[SCAN_TARGET] QR '{txt}' is in a RED ZONE! Skipping.")
                    continue
                matched_qr = quad
                break

            if matched_qr is not None:
                self.qr_confirm_cnt += 1
                log.info(f"[SCAN_TARGET] Target matched ({self.qr_confirm_cnt}/{self.config.QR_CONFIRM_COUNT})")
                if self.qr_confirm_cnt >= self.config.QR_CONFIRM_COUNT:
                    self.coord_mgr.register_landmark(
                        "TARGET_QR",
                        self.coord_mgr.global_pos.x,
                        self.coord_mgr.global_pos.y,
                        self.coord_mgr.global_pos.z,
                    )
                    log.info("[SCAN_TARGET] Target Confirmed! Descending to 5m -> ALIGN_PAYLOAD")
                    self.fc.set_altitude(self.config.PAYLOAD_DROP_ALT_M)
                    self.state = MissionState.ALIGN_PAYLOAD
            else:
                self.qr_confirm_cnt = 0
                self.fc.send_body_velocity(vx=0.3, vy=0.2, vz=0.0, duration=0.4)

        elif self.state == MissionState.ALIGN_PAYLOAD:
            frame_cx = bgr.shape[1] // 2
            frame_cy = bgr.shape[0] // 2
            aligned = False

            for txt, quad in qr_codes:
                if txt != self.target_id:
                    continue
                qx = int(np.mean(quad[:, 0]))
                qy = int(np.mean(quad[:, 1]))
                err_x = qx - frame_cx
                err_y = qy - frame_cy

                vy = max(-0.25, min(0.25, 0.0025 * err_x))
                vx = max(-0.25, min(0.25, -0.0025 * err_y))

                if abs(err_x) < 25 and abs(err_y) < 25:
                    aligned = True
                else:
                    self.fc.send_body_velocity(vx=vx, vy=vy, vz=0.0, duration=0.2)
                break

            if aligned:
                log.info("[ALIGN] Centered over target -> DROP_PAYLOAD")
                self.fc.stop()
                self.state = MissionState.DROP_PAYLOAD

        elif self.state == MissionState.DROP_PAYLOAD:
            execute_winch_delivery(self.fc, self.config)
            log.info("[PAYLOAD] Winch drop complete -> ASCEND_RETURN")
            self.state = MissionState.ASCEND_RETURN

        elif self.state == MissionState.ASCEND_RETURN:
            self.fc.set_altitude(self.config.RETURN_CRUISE_ALT_M)
            log.info(f"[RETURN] Reached {self.config.RETURN_CRUISE_ALT_M}m. Rotating 180° -> ALIGN_RETURN_CORRIDOR")
            self.fc.set_yaw(180.0, relative=True)
            self.state = MissionState.ALIGN_RETURN_CORRIDOR

        elif self.state == MissionState.ALIGN_RETURN_CORRIDOR:
            self.fc.set_altitude(self.config.RETURN_CORRIDOR_ALT_M)
            if banner_box is not None or (d_left_w < 3.0 and d_right_w < 3.0):
                log.info("[RETURN_CORRIDOR] Entrance acquired -> FOLLOW_CORRIDOR_RETURN")
                self.state = MissionState.FOLLOW_CORRIDOR_RETURN
            else:
                self.fc.send_body_velocity(vx=self.config.SLOW_SPEED_MPS, vy=0.0, vz=0.0, duration=0.3)

        elif self.state == MissionState.FOLLOW_CORRIDOR_RETURN:
            lat_center_err = (d_right_w - d_left_w) / 2.0 if (d_left_w > 0 and d_right_w > 0) else 0.0
            vy_corr = max(-0.4, min(0.4, 0.45 * lat_center_err))

            if (d_left_w > 4.0 and d_right_w > 4.0) or (self.coord_mgr.global_pos.x <= 2.5):
                log.info("[RETURN_CORRIDOR] Cleared return corridor -> APPROACH_HOME")
                self.state = MissionState.APPROACH_HOME
            elif fwd_dist < self.config.OBS_SLOW_DIST_M:
                self.fc.send_body_velocity(vx=self.config.SLOW_SPEED_MPS * 0.5, vy=vy_corr, vz=0.0, duration=0.3)
            else:
                self.fc.send_body_velocity(vx=self.config.CORRIDOR_SPEED_MPS, vy=vy_corr, vz=0.0, duration=0.3)

        elif self.state == MissionState.APPROACH_HOME:
            gx, gy = self.coord_mgr.global_pos.x, self.coord_mgr.global_pos.y
            dist_home = math.sqrt(gx**2 + gy**2)
            log.info(f"[HOME_APPROACH] Dist to Home: {dist_home:.2f} m")

            if dist_home < 0.6:
                log.info("[HOME_APPROACH] Over Home Pad -> PRECISION_LAND")
                self.fc.stop()
                self.state = MissionState.PRECISION_LAND
            else:
                yaw_rad = math.radians(self.coord_mgr.global_pos.yaw_deg)
                dx, dy = -gx, -gy
                vx_body = dx * math.cos(yaw_rad) + dy * math.sin(yaw_rad)
                vy_body = -dx * math.sin(yaw_rad) + dy * math.cos(yaw_rad)
                spd = min(self.config.SLOW_SPEED_MPS, max(0.2, dist_home * 0.5))
                mag = math.sqrt(vx_body**2 + vy_body**2)
                if mag > 0.01:
                    vx_b = (vx_body / mag) * spd
                    vy_b = (vy_body / mag) * spd
                    self.fc.send_body_velocity(vx=vx_b, vy=vy_b, vz=0.0, duration=0.3)
                else:
                    self.fc.send_body_velocity(vx=0.0, vy=0.0, vz=0.0, duration=0.2)

        elif self.state == MissionState.PRECISION_LAND:
            self.fc.land()
            log.info("[LAND] Touchdown complete -> COMPLETE")
            self.state = MissionState.COMPLETE

        elif self.state == MissionState.AVOID_OBSTACLES:
            threats = [t for t in (obstacles + humans) if t.get("action") in ("STOP", "SLOW")]
            if not threats:
                log.info(f"[AVOID] Pathway clear -> Resuming {self.state_before_avoid.name}")
                self.state = self.state_before_avoid
            else:
                closest = min(threats, key=lambda t: t["dist_m"])
                cx = closest["cx"]
                fw = bgr.shape[1]
                if cx < fw // 2:
                    self.fc.send_body_velocity(vx=0.0, vy=self.config.SLOW_SPEED_MPS, vz=0.0, duration=0.3)
                else:
                    self.fc.send_body_velocity(vx=0.0, vy=-self.config.SLOW_SPEED_MPS, vz=0.0, duration=0.3)
                if closest["dist_m"] < 0.6:
                    self.fc.send_body_velocity(vx=-0.5, vy=0.0, vz=0.0, duration=0.3)

        return {
            "state": self.state,
            "target_id": self.target_id,
            "global_pos": self.coord_mgr.global_pos,
            "local_pos": lpos,
            "percept": percept,
        }

    def run(self, headless: bool = False, max_duration_s: Optional[float] = None) -> None:
        start_time = time.time()
        timeout = max_duration_s or self.config.MAX_MISSION_TIME_S

        while self.state != MissionState.COMPLETE:
            if time.time() - start_time > timeout:
                log.warning(f"[MISSION TIMEOUT] Exceeded {timeout:.1f}s limit. Triggering RTL failsafe.")
                self.fc.return_to_launch()
                break

            rgb, depth_mm = self.camera.read()
            if rgb is None:
                time.sleep(0.02)
                continue

            res = self.step(rgb, depth_mm)

            if not headless:
                hud = annotate_mission_frame(
                    rgb,
                    res["percept"],
                    res["state"],
                    res["target_id"],
                    res["global_pos"],
                    res["local_pos"],
                    self.config,
                )
                cv2.imshow("AeroTHON 2026 — Mission Control HUD (Bhakti Revankar)", hud)
                k = cv2.waitKey(1) & 0xFF
                if k == ord("q"):
                    log.warning("[OPERATOR ABORT] 'q' pressed -> Triggering RTL")
                    self.fc.return_to_launch()
                    break
                elif k == ord("e"):
                    log.warning("[EMERGENCY] 'e' pressed -> Immediate Land")
                    self.fc.land()
                    break

            time.sleep(0.02)

        if not headless:
            cv2.destroyAllWindows()


# ===========================================================================
# 13. CLI ENTRYPOINT
# ===========================================================================
def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="SAEINDIA AeroTHON 2026 Track 1: SkyScan Autonomous Rapid Delivery (Author: Bhakti Revankar)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--connect",
        default="",
        help="MAVLink connection URI (e.g. '/dev/ttyUSB0' or 'tcp:127.0.0.1:5760')",
    )
    parser.add_argument(
        "--enable-real-flight",
        action="store_true",
        default=False,
        help="SAFETY FLAG: Explicitly enable physical motor arming and flight control commands. "
             "If omitted, flight commands will run on safe MockFlightController.",
    )
    parser.add_argument(
        "--sim",
        action="store_true",
        default=False,
        help="Force pure internal software simulation mode.",
    )
    parser.add_argument(
        "--camera",
        choices=["oakd", "webcam", "mock", "multi"],
        default="oakd",
        help="Camera sensor backend: 'oakd' (Forward OAK-D Lite), 'multi' (3-Camera Suite: OAK-D + Arducam HQ + Skydroid FPV), 'webcam', or 'mock'.",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        default=False,
        help="Disable OpenCV GUI windows (for headless companion computer deployment).",
    )
    parser.add_argument(
        "--log-file",
        default="flight_data_log.csv",
        help="Output CSV path for flight telemetry and vision recordings.",
    )
    parser.add_argument(
        "--max-duration",
        type=float,
        default=CFG.MAX_MISSION_TIME_S,
        help="Maximum mission runtime in seconds (competition limit = 900s).",
    )
    return parser.parse_args()


def create_camera(cam_type: str, sim_mode: bool) -> CameraInterface:
    if sim_mode or cam_type == "mock":
        log.info("[CAMERA] Instantiating MockCamera.")
        return MockCamera()
    elif cam_type == "webcam":
        log.info("[CAMERA] Instantiating WebcamCamera.")
        return WebcamCamera()
    elif cam_type == "multi":
        log.info("[CAMERA] Instantiating 3-Camera Suite (Forward OAK-D Lite + Downward Arducam HQ + Skydroid FPV).")
        return ThreeCameraManager()
    else:
        log.info("[CAMERA] Instantiating OAK-D Lite Stereo Depth Camera.")
        return OakDCamera()


def main() -> None:
    args = parse_args()

    log.info("==================================================================")
    log.info("  SAEINDIA AeroTHON 2026 — Track 1 Autonomous Flight")
    log.info("  Software Engineer / System Architect: Bhakti Revankar")
    log.info("==================================================================")
    if not args.enable_real_flight:
        log.info("[SAFETY STATUS] Real flight execution is DISABLED (Sim / Dry-run Mode).")
    else:
        log.warning("[SAFETY STATUS] REAL FLIGHT EXECUTION IS ENABLED! Hardware will be armed.")

    camera = create_camera(args.camera, args.sim)
    fc = create_flight_controller(
        connection_string=args.connect,
        enable_real_flight=args.enable_real_flight and (not args.sim),
        config=CFG,
    )
    coord_mgr = CoordinateManager(config=CFG)
    perception = PerceptionPipeline(config=CFG)
    recorder = FlightDataRecorder(log_path=args.log_file)

    fsm = MissionFSM(
        fc=fc,
        camera=camera,
        coord_mgr=coord_mgr,
        perception=perception,
        recorder=recorder,
        config=CFG,
    )

    try:
        fsm.run(headless=args.headless, max_duration_s=args.max_duration)
    except KeyboardInterrupt:
        log.warning("[INTERRUPT] Keyboard interrupt received! Triggering RTL.")
        fc.return_to_launch()
    finally:
        camera.release()
        fc.close()
        recorder.close()
        log.info("[SHUTDOWN] Mission lifecycle completed cleanly.")


if __name__ == "__main__":
    main()
