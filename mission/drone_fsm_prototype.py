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
import logging
import math
import time
from enum import Enum, auto
from typing import Any

import cv2
import numpy as np

from aerothon.banner import detect_corridor_banner
from aerothon.depth_geometry import analyze_depth, pixel_to_ground
from aerothon.qr_candidates import find_qr_candidates
from aerothon.qr_decode import decode_qr_codes
from aerothon.field_spec import FIELD

# ---------------------------------------------------------------------------
# Logging Setup
# ---------------------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s][%(levelname)s][R2D2-AeroTHON] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("R2D2-AeroTHON2026")


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
    CORRIDOR_ALT_M          : float = FIELD.CORRIDOR_FLIGHT_ALT_M  # 1.5m: inside the 8 ft (2.44m) corridor walls
    DELIVERY_SEARCH_ALT_M   : float = 10.0   # 10.0m delivery zone QR search altitude
    PAYLOAD_DROP_ALT_M      : float = 5.0    # 5.0m precision winch drop altitude
    RETURN_CRUISE_ALT_M     : float = 10.0   # 10.0m return climb altitude
    RETURN_CORRIDOR_ALT_M   : float = FIELD.CORRIDOR_FLIGHT_ALT_M  # 1.5m return corridor traversal altitude

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
    CORRIDOR_WIDTH_M        : float = FIELD.CORRIDOR_WIDTH_M  # 11.5 ft (3.51m) between walls
    CORRIDOR_LENGTH_EST_M   : float = FIELD.CORRIDOR_LENGTH_M + 1.0  # 22 ft (6.71m) walls + 1m margin

    # Camera geometry (forward OAK-D Lite)
    CAMERA_PITCH_DEG        : float = 45.0   # OAK-D mount tilt below horizontal
    CAMERA_HFOV_DEG         : float = 73.0   # OAK-D Lite stereo horizontal FOV

    # Landing on the 4x4 ft (1.22m) raised podium
    HOME_TOLERANCE_M        : float = 0.25   # must be well inside the 0.61m pad half-width
    HOME_SETTLE_STEPS       : int = 3        # consecutive in-tolerance steps before LAND
    # The 45 deg camera only sees wall points from ~2.4m ahead (at 1.5m alt),
    # so walls 'disappear' before the corridor actually ends.
    CORRIDOR_BLIND_M        : float = 2.8    # keep going this far after walls vanish

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
    MAX_MISSION_TIME_S      : float = 1200.0  # 20 minutes competition time limit

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

        self.landmarks: dict[str, tuple[float, float, float]] = {
            "HOME": (0.0, 0.0, 0.0),
            "START_QR": (FIELD.START_QR_X_M, 0.0, -self.config.TAKEOFF_ALT_M),
            "CORRIDOR_ENTRY": (FIELD.CORRIDOR_ENTRY_X_M, FIELD.FORWARD_CORRIDOR_Y_M, -self.config.CORRIDOR_ALT_M),
            "CORRIDOR_EXIT": (FIELD.corridor_exit_x, FIELD.FORWARD_CORRIDOR_Y_M, -self.config.CORRIDOR_ALT_M),
            "DELIVERY_CENTER": (FIELD.corridor_exit_x + FIELD.DELIVERY_ZONE_GAP_M + 10.0, 0.0, -self.config.DELIVERY_SEARCH_ALT_M),
            "TARGET_QR": (0.0, 0.0, 0.0),
        }

        # Complete Stored Flight Trajectories (Forward and Return Paths)
        self.forward_path: list[TrajectoryPoint] = []
        self.return_path: list[TrajectoryPoint] = []
        self.full_trajectory: list[TrajectoryPoint] = []

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

    def get_reverse_return_waypoints(self) -> list[tuple[float, float, float]]:
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
        target_qr_box_dist: float | None = None,
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
    def read(self) -> tuple[np.ndarray | None, np.ndarray | None]:
        pass

    @abc.abstractmethod
    def release(self) -> None:
        pass


class MockCamera(CameraInterface):
    def __init__(self, width: int = 640, height: int = 480) -> None:
        self.width = width
        self.height = height
        self._custom_frame: np.ndarray | None = None
        self._custom_depth: np.ndarray | None = None

    def set_frame(self, bgr: np.ndarray, depth_mm: np.ndarray | None = None) -> None:
        self._custom_frame = bgr
        self._custom_depth = depth_mm

    def read(self) -> tuple[np.ndarray | None, np.ndarray | None]:
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
        self._cap: cv2.VideoCapture | None = None

    def _ensure_opened(self) -> bool:
        if self._cap is None:
            self._cap = cv2.VideoCapture(self.camera_index)
            if self._cap.isOpened():
                self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        return self._cap.isOpened()

    def read(self) -> tuple[np.ndarray | None, np.ndarray | None]:
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
        self._fallback: WebcamCamera | None = None
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

    def read(self) -> tuple[np.ndarray | None, np.ndarray | None]:
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


ThreeCameraManager = OakDCamera


# ===========================================================================
# 4. STEREO DEPTH SAMPLING & REASONING
# ===========================================================================
DEPTH_UNKNOWN_M = float("inf")


def depth_at_bbox(
    depth_mm: np.ndarray | None,
    bbox: tuple[int, int, int, int],
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
    l_ch, a_ch, b_ch = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    l_ch = clahe.apply(l_ch)
    return cv2.cvtColor(cv2.merge([l_ch, a_ch, b_ch]), cv2.COLOR_Lab2BGR)


BANNER_HSV_LOWER = np.array([35, 75, 50])
BANNER_HSV_UPPER = np.array([85, 255, 255])
BANNER_MIN_AREA = 2500
BANNER_MIN_AR = 1.3


def detect_green_banner(
    bgr: np.ndarray,
    yolo_model: Any = None,
    yolo_results: Any = None,
) -> tuple[int, int, int, int] | None:
    # Field spec banners are long strips (11.5x1.5 ft / 11.5x1 ft, #00B140);
    # see aerothon/banner.py for why the old HSV + axis-aligned check failed.
    best_box = detect_corridor_banner(bgr, "green")
    best_area = best_box[2] * best_box[3] if best_box is not None else 0

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


def detect_qr_codes(bgr: np.ndarray) -> list[tuple[str, np.ndarray]]:
    # Field QRs have no white border; see aerothon/qr_decode.py.
    return decode_qr_codes(bgr)


RED_HUE_RANGES = [(0, 10), (170, 180)]
RED_SAT_MIN = 130
RED_VAL_MIN = 80
RED_ZONE_MIN_AREA = 500


def find_red_zones(bgr: np.ndarray) -> list[tuple[int, int, int, int]]:
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
    depth_mm: np.ndarray | None,
    yolo_model: Any = None,
    yolo_results: Any = None,
    config: MissionConfig = CFG,
) -> list[dict]:
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

    def perceive(
        self, bgr: np.ndarray, depth_mm: np.ndarray | None, alt_m: float | None = None
    ) -> dict[str, Any]:
        self._lazy_load_yolo()

        _yolo_res = None
        humans: list[dict] = []
        yolo_all: list[tuple] = []

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
        exit_banner_box = detect_corridor_banner(bgr, "blue")

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

        # With altitude known, use ground-aware geometry (camera is pitched
        # down, so raw edge-column depth is mostly floor, not wall).
        if alt_m is not None and depth_mm is not None:
            geo = analyze_depth(
                depth_mm,
                alt_m,
                pitch_deg=self.config.CAMERA_PITCH_DEG,
                hfov_deg=self.config.CAMERA_HFOV_DEG,
                stop_m=self.config.OBS_SLOW_DIST_M,
                slow_m=self.config.OBS_SAFE_DIST_M,
            )
            d_left_wall, d_right_wall = geo["d_left_wall"], geo["d_right_wall"]
            d_centre = geo["fwd_dist_m"]
            obstacles = obstacles + geo["obstacles"]

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
            "exit_banner_box": exit_banner_box,
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
    cv2.putText(out, f"STATE: {state.name} | PILOT: R2D2", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 240, 240), 2)
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
    def get_position_ned(self) -> tuple[float, float, float, float]:
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
        self.command_log: list[str] = []
        self.winch_pwm_history: list[int] = []

    def get_position_ned(self) -> tuple[float, float, float, float]:
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
        self._yaw_hold_deg: float | None = None  # heading every setpoint commands (course frame)
        # Course frame: x = from the pad towards the corridors, y = right.
        # Set at arming from the way the drone faces on the pad, unless
        # FIELD.COURSE_HEADING_DEG fixes it. Everything the FSM sees and
        # commands is in this frame, so the field need not face north.
        self._course_deg = 0.0
        self._connect()

    def _connect(self) -> None:
        import collections
        import collections.abc

        # dronekit 2.9 still imports collections.MutableMapping (removed in Python 3.10)
        if not hasattr(collections, "MutableMapping"):
            collections.MutableMapping = collections.abc.MutableMapping  # type: ignore[attr-defined]
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

    def get_position_ned(self) -> tuple[float, float, float, float]:
        """(x, y, z, yaw) in the course frame (see _course_deg), z down."""
        if self._vehicle is None:
            return (0.0, 0.0, 0.0, 0.0)
        loc = self._vehicle.location.local_frame
        alt = float(self._vehicle.location.global_relative_frame.alt or 0.0)
        n = float(loc.north) if loc and loc.north is not None else 0.0
        e = float(loc.east) if loc and loc.east is not None else 0.0
        c = math.radians(self._course_deg)
        x = n * math.cos(c) + e * math.sin(c)
        y = -n * math.sin(c) + e * math.cos(c)
        return (x, y, -alt, self._heading_now())

    # ArduCopter GUIDED: a velocity setpoint that leaves yaw unspecified
    # cancels any CONDITION_YAW and (with the default WP_YAW_BEHAVIOR) turns
    # the nose toward the direction of travel. With body-frame velocities that
    # have a sideways part, the frame then rotates with the nose and the drone
    # spins in place. So every setpoint below carries an explicit heading.
    _MASK_VEL_YAW = 0b0000100111000111  # use vx,vy,vz + yaw; ignore pos, accel, yaw rate
    _MASK_POS_YAW = 0b0000100111111000  # use x,y,z + yaw; ignore vel, accel, yaw rate

    def _raw_heading(self) -> float:
        """Compass heading, degrees 0-360."""
        att = self._vehicle.attitude if self._vehicle is not None else None
        if att is None or att.yaw is None:
            return 0.0
        return math.degrees(att.yaw) % 360.0

    def _heading_now(self) -> float:
        """Heading in the course frame."""
        return (self._raw_heading() - self._course_deg) % 360.0

    def _yaw_target(self) -> float:
        if self._yaw_hold_deg is None:
            self._yaw_hold_deg = self._heading_now()
        return self._yaw_hold_deg

    def _send_local(self, vn: float, ve: float, vd: float, duration: float) -> None:
        from pymavlink import mavutil  # type: ignore

        lim = self.config.MAX_SPEED_MPS
        vx, vy, vd = (max(-lim, min(lim, v)) for v in (vn, ve, vd))
        c = math.radians(self._course_deg)  # course frame -> NED
        vn = vx * math.cos(c) - vy * math.sin(c)
        ve = vx * math.sin(c) + vy * math.cos(c)
        msg = self._vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            self._MASK_VEL_YAW,
            0, 0, 0,
            vn, ve, vd,
            0, 0, 0,
            math.radians(self._yaw_target() + self._course_deg), 0,
        )
        end_time = time.time() + duration
        while True:
            self._vehicle.send_mavlink(msg)
            if time.time() >= end_time:
                break
            time.sleep(0.1)

    def send_body_velocity(self, vx: float, vy: float, vz: float, duration: float = 0.2) -> None:
        """Velocity relative to the commanded heading (camera forward / right)."""
        if self._vehicle is None:
            return
        h = math.radians(self._yaw_target())
        vn = vx * math.cos(h) - vy * math.sin(h)
        ve = vx * math.sin(h) + vy * math.cos(h)
        self._send_local(vn, ve, vz, duration)

    def send_local_velocity(self, vx_n: float, vy_e: float, vz_d: float, duration: float = 0.2) -> None:
        if self._vehicle is None:
            return
        self._send_local(vx_n, vy_e, vz_d, duration)

    def arm_and_takeoff(self, target_alt: float) -> bool:
        if self._vehicle is None:
            return False
        from dronekit import VehicleMode  # type: ignore

        self._course_deg = (
            FIELD.COURSE_HEADING_DEG if FIELD.COURSE_HEADING_DEG is not None else self._raw_heading()
        ) % 360.0
        self._yaw_hold_deg = 0.0
        log.info(f"[FC] Course heading {self._course_deg:.0f} deg (x axis = pad -> corridors)")
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

        log.info(f"[FC] Takeoff complete at {target_alt:.1f} m, facing the corridors.")
        return True

    def set_altitude(self, target_alt: float) -> bool:
        if self._vehicle is None:
            return False
        from pymavlink import mavutil  # type: ignore

        log.info(f"[FC] Changing altitude to {target_alt:.1f} m...")
        loc = self._vehicle.location.local_frame
        n = float(loc.north or 0.0) if loc else 0.0
        e = float(loc.east or 0.0) if loc else 0.0
        # z in LOCAL_NED is relative to the EKF origin (home), down positive
        msg = self._vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            self._MASK_POS_YAW,
            n, e, -target_alt,
            0, 0, 0,
            0, 0, 0,
            math.radians(self._yaw_target() + self._course_deg), 0,
        )
        alt_start = time.time()
        while abs(float(self._vehicle.location.global_relative_frame.alt or 0.0) - target_alt) > self.config.ALTITUDE_TOLERANCE_M:
            if time.time() - alt_start > self.config.ALTITUDE_CHANGE_TIMEOUT_S:
                log.warning("[FC TIMEOUT] Altitude transition timed out; proceeding with current altitude.")
                return False
            self._vehicle.send_mavlink(msg)
            time.sleep(0.2)
        return True

    def set_yaw(self, heading_deg: float, relative: bool = True) -> None:
        """Turn on the spot to a heading and keep holding it afterwards.

        Relative turns add to the commanded heading (not the measured one), so
        a 15-deg-per-step scan really completes 360 deg in 24 steps.
        """
        if self._vehicle is None:
            return
        base = self._yaw_target() if relative else 0.0
        self._yaw_hold_deg = (base + heading_deg) % 360.0
        turn = abs(((self._heading_now() - self._yaw_hold_deg) + 180.0) % 360.0 - 180.0)
        deadline = time.time() + turn / max(self.config.YAW_RATE_DPS, 1.0) + 2.0
        while time.time() < deadline:
            self._send_local(0.0, 0.0, 0.0, 0.0)  # hold position, turn to heading
            err = abs(((self._heading_now() - self._yaw_hold_deg) + 180.0) % 360.0 - 180.0)
            if err <= self.config.HEADING_TOLERANCE_DEG:
                break
            time.sleep(0.1)

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
        # Touchdown = ArduPilot's own land detector disarming the motors.
        # (The old "relative alt < 0.15m" check could fire before contact,
        # or never fire if the baro reads slightly high on the 6 in podium.)
        while self._vehicle.armed:
            if time.time() - land_start > self.config.LANDING_TIMEOUT_S:
                alt = float(self._vehicle.location.global_relative_frame.alt or 0.0)
                log.warning(f"[FC TIMEOUT] Still armed after {self.config.LANDING_TIMEOUT_S:.0f}s (alt {alt:.2f}m).")
                return False
            time.sleep(0.3)
        log.info("[FC] Landed and disarmed.")
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
        percept: dict[str, Any],
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
            "R2D2",
        ])
        self.file.flush()

    def close(self) -> None:
        if self.file and not self.file.closed:
            self.file.close()


# ===========================================================================
# 12. FULL MISSION FSM ENGINE (FORWARD & BACKWARD PASS)
# ===========================================================================
def delivery_search_waypoints(config: MissionConfig = CFG) -> list[tuple[float, float, float]]:
    """Snake (lawnmower) sweep over the 40 x 30 m delivery zone: (x, y, heading).

    The forward camera is pitched 45 deg, so from the search altitude it looks
    ~alt metres ahead; lanes start that far before the zone edge and stop
    that far before its end. Lanes 10 m apart cover the 30 m width.
    """
    alt = config.DELIVERY_SEARCH_ALT_M
    x0 = FIELD.corridor_exit_x + FIELD.DELIVERY_ZONE_GAP_M
    x_near = x0 - 0.6 * alt
    x_far = x0 + FIELD.DELIVERY_ZONE_LENGTH_M - 0.6 * alt
    half_w = FIELD.DELIVERY_ZONE_WIDTH_M / 2
    lanes = (0.0, half_w - 5.0, -(half_w - 5.0))  # centre lane first
    wps: list[tuple[float, float, float]] = []
    for i, y in enumerate(lanes):
        if i % 2 == 0:
            wps += [(x_near, y, 0.0), (x_far, y, 0.0)]
        else:
            wps += [(x_far + alt, y, 180.0), (x0 + 0.6 * alt, y, 180.0)]
    return wps


class MissionFSM:
    def __init__(
        self,
        fc: FlightControllerInterface,
        camera: CameraInterface,
        coord_mgr: CoordinateManager | None = None,
        perception: PerceptionPipeline | None = None,
        recorder: FlightDataRecorder | None = None,
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
        self.home_settle_cnt = 0
        self.target_xy: tuple[float, float] | None = None
        self.align_settle_cnt = 0
        self.walls_lost_at: tuple[float, float] | None = None
        self.search_wps: list[tuple[float, float, float]] = []
        self.search_idx = 0
        self.qr_cands: list[dict] = []  # QR-like patches seen in the delivery zone (world x, y)
        self.inspecting: dict | None = None
        self.return_mouth_reached = False
        self.state_before_avoid = MissionState.FOLLOW_CORRIDOR
        self.mission_start_time = time.time()
        self.step_count = 0

    def _fly_towards(self, tx: float, ty: float, speed: float) -> float:
        """One velocity step towards local point (tx, ty). Returns distance."""
        gx, gy = self.coord_mgr.global_pos.x, self.coord_mgr.global_pos.y
        dx, dy = tx - gx, ty - gy
        dist = math.hypot(dx, dy)
        if dist > 0.05:
            spd = min(speed, max(0.15, dist * 0.6))
            self.fc.send_local_velocity(dx / dist * spd, dy / dist * spd, 0.0, duration=0.3)
        return dist

    def _to_world(self, px: float, py: float, w: int, h: int, alt: float, yaw: float):
        off = pixel_to_ground(px, py, w, h, alt, self.config.CAMERA_PITCH_DEG, self.config.CAMERA_HFOV_DEG)
        if off is None:
            return None
        yr = math.radians(yaw)
        gx, gy = self.coord_mgr.global_pos.x, self.coord_mgr.global_pos.y
        return (gx + off[0] * math.cos(yr) - off[1] * math.sin(yr), gy + off[0] * math.sin(yr) + off[1] * math.cos(yr))

    def _search_for_target(self, bgr, qr_codes, gx: float, gy: float, alt: float, yaw: float) -> None:
        """Delivery-zone search: spot QR-like patches from afar, fly to each
        one and read it; sweep lanes only when nothing is left to inspect.

        A fixed heading is always commanded. (The old "vx=0.3, vy=0.2 in body
        frame" drift made a real ArduCopter turn its nose toward the velocity,
        rotating the body frame, so the drone circled instead of searching.)
        """
        h_img, w_img = bgr.shape[:2]
        x0 = FIELD.corridor_exit_x + FIELD.DELIVERY_ZONE_GAP_M
        x1 = x0 + FIELD.DELIVERY_ZONE_LENGTH_M
        half_w = FIELD.DELIVERY_ZONE_WIDTH_M / 2

        def _nearest(x, y, r=1.5):
            best = None
            for c in self.qr_cands:
                d = math.hypot(c["x"] - x, c["y"] - y)
                if d < r and (best is None or d < best[0]):
                    best = (d, c)
            return best[1] if best else None

        # 1. remember new QR-like patches inside the delivery zone
        for cx, cy, _a in find_qr_candidates(bgr):
            p = self._to_world(cx, cy, w_img, h_img, alt, yaw)
            if p is None or not (x0 - 1 <= p[0] <= x1 + 1 and abs(p[1]) <= half_w + 1):
                continue
            c = _nearest(*p)
            if c is None:
                self.qr_cands.append({"x": p[0], "y": p[1], "done": False, "n": 1})
                log.info(f"[SCAN_TARGET] QR-like patch at ({p[0]:.1f}, {p[1]:.1f}) -> queued for inspection")
            elif not c["done"]:  # refine the estimate while it is far away
                c["x"] = (c["x"] * c["n"] + p[0]) / (c["n"] + 1)
                c["y"] = (c["y"] * c["n"] + p[1]) / (c["n"] + 1)
                c["n"] = min(c["n"] + 1, 5)

        # 2. any QR we can read that is not ours: mark that patch done
        for txt, quad in qr_codes:
            if txt == self.target_id:
                continue
            p = self._to_world(float(np.mean(quad[:, 0])), float(np.mean(quad[:, 1])), w_img, h_img, alt, yaw)
            c = _nearest(*p, r=2.0) if p else None
            if c is not None and not c["done"]:
                c["done"] = True
                log.info(f"[SCAN_TARGET] Patch at ({c['x']:.1f}, {c['y']:.1f}) is '{txt}', not ours")
                if self.inspecting is c:
                    self.inspecting = None

        # 3. inspect the nearest unread patch: stand 5 m short of it, facing +x
        #    (at the 10 m search altitude the 45 deg camera reads a 1 m QR best
        #    when it is 4-9 m ahead and nearly straight on)
        if self.inspecting is None:
            todo = [c for c in self.qr_cands if not c["done"]]
            if todo:
                self.inspecting = min(todo, key=lambda c: math.hypot(c["x"] - 5.0 - gx, c["y"] - gy))
                self.inspecting["dwell"] = 0
        if self.inspecting is not None:
            c = self.inspecting
            if abs(((yaw - 0.0) + 180.0) % 360.0 - 180.0) > 10.0:
                self.fc.set_yaw(0.0, relative=False)
            if self._fly_towards(c["x"] - 5.0, c["y"], self.config.CRUISE_SPEED_MPS) < 0.5:
                self.fc.stop()
                c["dwell"] += 1
                if c["dwell"] >= 4:  # could not read it from the sweet spot
                    log.info(f"[SCAN_TARGET] Patch at ({c['x']:.1f}, {c['y']:.1f}) unreadable/not ours -> next")
                    c["done"] = True
                    self.inspecting = None
            return

        # 4. nothing to inspect: sweep the zone lane by lane
        if not self.search_wps:
            self.search_wps = delivery_search_waypoints(self.config)
        wx, wy, wyaw = self.search_wps[self.search_idx]
        if abs(((yaw - wyaw) + 180.0) % 360.0 - 180.0) > 10.0:
            self.fc.set_yaw(wyaw, relative=False)
        if self._fly_towards(wx, wy, self.config.CRUISE_SPEED_MPS) < 0.6:
            self.search_idx = (self.search_idx + 1) % len(self.search_wps)
            if self.search_idx == 0:
                log.warning("[SCAN_TARGET] Full sweep done without a match -> re-checking all patches")
                for c in self.qr_cands:
                    c["done"] = False

    def _walls_lost_for(self, d_left_w: float, d_right_w: float) -> float:
        """Distance flown since both corridor walls were last seen (0 if seen)."""
        gx, gy = self.coord_mgr.global_pos.x, self.coord_mgr.global_pos.y
        if d_left_w > 3.0 and d_right_w > 3.0:
            if self.walls_lost_at is None:
                self.walls_lost_at = (gx, gy)
            return math.hypot(gx - self.walls_lost_at[0], gy - self.walls_lost_at[1])
        self.walls_lost_at = None
        return 0.0

    def step(self, bgr: np.ndarray, depth_mm: np.ndarray | None) -> dict[str, Any]:
        self.step_count += 1

        gx, gy, gz, yaw = self.fc.get_position_ned()
        self.coord_mgr.update_from_fc(gx, gy, gz, yaw)
        self.coord_mgr.record_path_point(self.state, gx, gy, gz, yaw)

        percept = self.perception.perceive(bgr, depth_mm, alt_m=-gz)
        obstacles = percept["obstacles"]
        humans = percept["humans"]
        red_zones = percept["red_zones"]
        qr_codes = percept["qr_codes"]
        banner_box = percept["banner_box"]
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
                self.fc.set_yaw(0.0, relative=False)  # corridors lie along +x from take-off
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
                    self.fc.set_yaw(0.0, relative=False)

        elif self.state == MissionState.FIND_GREEN_BANNER:
            if banner_box is not None:
                bx, _by, bw, _bh = banner_box
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
                    log.info(f"[BANNER] Corridor entry reached. Descending to {self.config.CORRIDOR_ALT_M:.1f}m -> ENTER_CORRIDOR")
                    self.fc.set_altitude(self.config.CORRIDOR_ALT_M)
                    self.state = MissionState.ENTER_CORRIDOR
                self.banner_search_cnt = 0
            else:
                self.banner_search_cnt += 1
                if self.banner_search_cnt <= 24:
                    self.fc.set_yaw(15.0, relative=True)
                else:
                    # Override: banner never seen in a full sweep. Fly to the
                    # corridor mouth from the measured field layout instead
                    # of blindly creeping forward from wherever we are.
                    if self.banner_search_cnt == 25:
                        log.warning("[BANNER] Not found after 360 sweep -> flying to corridor mouth from field layout")
                        self.fc.set_yaw(0.0, relative=False)
                    dist = self._fly_towards(
                        FIELD.CORRIDOR_ENTRY_X_M - 1.0, FIELD.FORWARD_CORRIDOR_Y_M, self.config.SLOW_SPEED_MPS
                    )
                    if dist < 0.4:
                        self.fc.set_altitude(self.config.CORRIDOR_ALT_M)
                        self.state = MissionState.ENTER_CORRIDOR
                        self.banner_search_cnt = 0

        elif self.state == MissionState.ENTER_CORRIDOR:
            if d_left_w < 3.0 and d_right_w < 3.0:
                log.info(f"[ENTER_CORRIDOR] Walls detected (L:{d_left_w:.2f}m, R:{d_right_w:.2f}m) -> FOLLOW_CORRIDOR")
                self.state = MissionState.FOLLOW_CORRIDOR
            else:
                self.fc.send_body_velocity(vx=self.config.SLOW_SPEED_MPS, vy=0.0, vz=0.0, duration=0.3)

        elif self.state == MissionState.FOLLOW_CORRIDOR:
            lat_center_err = (d_right_w - d_left_w) / 2.0 if (d_left_w > 0 and d_right_w > 0) else 0.0
            vy_corr = max(-0.4, min(0.4, 0.45 * lat_center_err))

            lost_for = self._walls_lost_for(d_left_w, d_right_w)
            if (fwd_dist > 6.0 and lost_for >= self.config.CORRIDOR_BLIND_M) or (
                lpos.x_rel >= self.config.CORRIDOR_LENGTH_EST_M
            ):
                self.walls_lost_at = None
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
            self.search_wps = delivery_search_waypoints(self.config)
            self.search_idx = 0
            self.qr_cands, self.inspecting = [], None

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
                if self.qr_confirm_cnt == 0:
                    self.fc.stop()  # hold still while confirming over 3 frames
                self.qr_confirm_cnt += 1
                log.info(f"[SCAN_TARGET] Target matched ({self.qr_confirm_cnt}/{self.config.QR_CONFIRM_COUNT})")
                if self.qr_confirm_cnt >= self.config.QR_CONFIRM_COUNT:
                    self.coord_mgr.register_landmark(
                        "TARGET_QR",
                        self.coord_mgr.global_pos.x,
                        self.coord_mgr.global_pos.y,
                        self.coord_mgr.global_pos.z,
                    )
                    qx = float(np.mean(matched_qr[:, 0]))
                    qy = float(np.mean(matched_qr[:, 1]))
                    off = pixel_to_ground(
                        qx, qy, bgr.shape[1], bgr.shape[0], -gz,
                        self.config.CAMERA_PITCH_DEG, self.config.CAMERA_HFOV_DEG,
                    )
                    if off is not None:
                        yaw_r = math.radians(yaw)
                        f_m, r_m = off
                        self.target_xy = (
                            gx + f_m * math.cos(yaw_r) - r_m * math.sin(yaw_r),
                            gy + f_m * math.sin(yaw_r) + r_m * math.cos(yaw_r),
                        )
                        log.info(f"[SCAN_TARGET] Target ground position ({self.target_xy[0]:.2f}, {self.target_xy[1]:.2f})")
                    log.info("[SCAN_TARGET] Target Confirmed! Descending to 5m -> ALIGN_PAYLOAD")
                    self.fc.set_altitude(self.config.PAYLOAD_DROP_ALT_M)
                    self.state = MissionState.ALIGN_PAYLOAD
            else:
                self.qr_confirm_cnt = 0
                self._search_for_target(bgr, qr_codes, gx, gy, -gz, yaw)

        elif self.state == MissionState.ALIGN_PAYLOAD:
            # The forward camera is pitched 45 deg: "QR in image centre" means
            # the QR is ~alt metres AHEAD, not below. Fly to the QR's ground
            # position (estimated at confirmation, refined while visible).
            for txt, quad in qr_codes:
                if txt != self.target_id:
                    continue
                off = pixel_to_ground(
                    float(np.mean(quad[:, 0])), float(np.mean(quad[:, 1])),
                    bgr.shape[1], bgr.shape[0], -gz,
                    self.config.CAMERA_PITCH_DEG, self.config.CAMERA_HFOV_DEG,
                )
                if off is not None and off[0] > 1.0:  # far enough for a reliable fix
                    yaw_r = math.radians(yaw)
                    nx = gx + off[0] * math.cos(yaw_r) - off[1] * math.sin(yaw_r)
                    ny = gy + off[0] * math.sin(yaw_r) + off[1] * math.cos(yaw_r)
                    if self.target_xy is None:
                        self.target_xy = (nx, ny)
                    else:  # light smoothing
                        self.target_xy = (0.7 * self.target_xy[0] + 0.3 * nx, 0.7 * self.target_xy[1] + 0.3 * ny)
                break

            if self.target_xy is None:
                log.warning("[ALIGN] No target position yet -> back to SCAN_TARGET_QR")
                self.state = MissionState.SCAN_TARGET_QR
            else:
                dist = self._fly_towards(self.target_xy[0], self.target_xy[1], self.config.SLOW_SPEED_MPS)
                if dist < 0.3:
                    self.fc.stop()
                    self.align_settle_cnt += 1
                    if self.align_settle_cnt >= 3:
                        log.info(f"[ALIGN] Over target (err {dist:.2f}m) -> DROP_PAYLOAD")
                        self.state = MissionState.DROP_PAYLOAD
                else:
                    self.align_settle_cnt = 0

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
            # The return lap uses the EXIT corridor (blue banners, obstacles),
            # which sits beside the forward corridor. Go to its mouth first.
            mouth_x = FIELD.corridor_exit_x + 1.5
            mouth_y = FIELD.RETURN_CORRIDOR_Y_M
            if not self.return_mouth_reached:
                if self._fly_towards(mouth_x, mouth_y, self.config.CRUISE_SPEED_MPS) < 0.4:
                    self.fc.stop()
                    self.fc.set_yaw(180.0, relative=False)
                    self.fc.set_altitude(self.config.RETURN_CORRIDOR_ALT_M)
                    self.return_mouth_reached = True
                    log.info("[RETURN_CORRIDOR] At exit-corridor mouth -> aligning")
            elif d_left_w < 3.0 and d_right_w < 3.0:
                log.info("[RETURN_CORRIDOR] Walls acquired -> FOLLOW_CORRIDOR_RETURN")
                self.state = MissionState.FOLLOW_CORRIDOR_RETURN
            else:
                vy = 0.0
                exit_box = percept.get("exit_banner_box")
                if exit_box is not None:
                    err = (exit_box[0] + exit_box[2] // 2) - bgr.shape[1] // 2
                    vy = max(-0.3, min(0.3, 0.003 * err))
                self.fc.send_body_velocity(vx=self.config.SLOW_SPEED_MPS, vy=vy, vz=0.0, duration=0.3)

        elif self.state == MissionState.FOLLOW_CORRIDOR_RETURN:
            lat_center_err = (d_right_w - d_left_w) / 2.0 if (d_left_w > 0 and d_right_w > 0) else 0.0
            vy_corr = max(-0.4, min(0.4, 0.45 * lat_center_err))

            lost_for = self._walls_lost_for(d_left_w, d_right_w)
            if lost_for >= self.config.CORRIDOR_BLIND_M or (
                self.coord_mgr.global_pos.x <= FIELD.CORRIDOR_ENTRY_X_M - 0.5
            ):
                self.walls_lost_at = None
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

            if dist_home < self.config.HOME_TOLERANCE_M:
                # Pad is only 1.22m square: hold and confirm we are settled
                # over it before committing to LAND.
                self.fc.stop()
                self.home_settle_cnt += 1
                if self.home_settle_cnt >= self.config.HOME_SETTLE_STEPS:
                    log.info("[HOME_APPROACH] Settled over 4x4 ft pad -> PRECISION_LAND")
                    self.state = MissionState.PRECISION_LAND
            else:
                yaw_rad = math.radians(self.coord_mgr.global_pos.yaw_deg)
                dx, dy = -gx, -gy
                vx_body = dx * math.cos(yaw_rad) + dy * math.sin(yaw_rad)
                vy_body = -dx * math.sin(yaw_rad) + dy * math.cos(yaw_rad)
                self.home_settle_cnt = 0
                spd = min(self.config.SLOW_SPEED_MPS, max(0.15, dist_home * 0.6))
                mag = math.sqrt(vx_body**2 + vy_body**2)
                if mag > 0.01:
                    vx_b = (vx_body / mag) * spd
                    vy_b = (vy_body / mag) * spd
                    self.fc.send_body_velocity(vx=vx_b, vy=vy_b, vz=0.0, duration=0.3)
                else:
                    self.fc.send_body_velocity(vx=0.0, vy=0.0, vz=0.0, duration=0.2)

        elif self.state == MissionState.PRECISION_LAND:
            if self.fc.land():
                log.info("[LAND] Touchdown complete -> COMPLETE")
            else:
                log.warning("[LAND] Land did not confirm disarm in time -> COMPLETE (check vehicle)")
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

    def run(self, headless: bool = False, max_duration_s: float | None = None) -> None:
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
                cv2.imshow("AeroTHON 2026 — Mission Control HUD (R2D2)", hud)
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
        description="SAEINDIA AeroTHON 2026 Track 1: SkyScan Autonomous Rapid Delivery (Author: R2D2)",
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
        choices=["oakd", "webcam", "mock", "multi", "sim"],
        default="oakd",
        help="Camera sensor backend: 'oakd' (Forward OAK-D Lite), 'multi' (3-Camera Suite: OAK-D + Arducam HQ + Skydroid FPV), 'webcam', 'mock', or 'sim' (renders the field from the FC pose; default with --sim, use with SITL).",
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


def create_camera(cam_type: str, sim_mode: bool, fc: Any = None) -> CameraInterface:
    if cam_type == "sim" or (sim_mode and cam_type != "mock"):
        from aerothon.sim_field_camera import FieldSimCamera

        log.info("[CAMERA] Instantiating FieldSimCamera (renders AeroTHON field from FC pose).")
        return FieldSimCamera(fc, pitch_deg=CFG.CAMERA_PITCH_DEG, hfov_deg=CFG.CAMERA_HFOV_DEG)
    if cam_type == "mock":
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
    log.info("  Software Engineer / System Architect: R2D2")
    log.info("==================================================================")
    if not args.enable_real_flight:
        log.info("[SAFETY STATUS] Real flight execution is DISABLED (Sim / Dry-run Mode).")
    else:
        log.warning("[SAFETY STATUS] REAL FLIGHT EXECUTION IS ENABLED! Hardware will be armed.")

    fc = create_flight_controller(
        connection_string=args.connect,
        enable_real_flight=args.enable_real_flight and (not args.sim),
        config=CFG,
    )
    camera = create_camera(args.camera, args.sim, fc)
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
