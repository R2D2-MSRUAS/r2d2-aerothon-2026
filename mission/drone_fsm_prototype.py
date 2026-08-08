"""
AeroTHON 2026 — Track 1;
=========================================================

Hardware stack (as per technical report):
  OAK-D Lite  -->  Myriad X VPU  -->  DepthAI pipeline  -->  This script
  Jetson TX2 NX runs this script and dispatches MAVLink to Pixhawk 6X Mini.

Mission state machine (complete loop):
  TAKEOFF -> SCAN_QR -> FIND_GREEN_BANNER -> ENTER_CORRIDOR ->
  FOLLOW_CORRIDOR -> REACH_DELIVERY -> SCAN_TARGET_QR ->
  DROP_PAYLOAD -> RETURN -> AVOID_OBSTACLES -> LAND

Depth-aware obstacle handling (per specification):
  OAK-D produces simultaneous RGB + depth map.
  Vision models annotate bounding boxes on the RGB image.
  The depth inside each bounding box is sampled to get real-world distance.
  Controller thresholds:
    > 2.0 m  --> Continue at current speed
    1.0–2.0 m --> Slow down (50 % speed)
    < 1.0 m  --> Stop / Avoid (lateral escape)

Run:
  python drone_mission.py --connect /dev/ttyUSB0     # real FC
  python drone_mission.py --connect tcp:127.0.0.1:5760  # SITL / simulator

Optional dependencies (all degrade gracefully if absent):
  pip install depthai          -- OAK-D integration
  pip install dronekit         -- MAVLink / flight control
  pip install ultralytics      -- YOLOv8 banner + obstacle detection
  pip install pyzbar           -- Robust QR decoding
  pip install opencv-python numpy
"""

from __future__ import annotations

import argparse
import logging
import math
import time
from enum import Enum, auto
from typing import List, Optional, Tuple

import cv2
import numpy as np

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s][%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("drone_mission")

# ---------------------------------------------------------------------------
# Optional: DepthAI / OAK-D
# ---------------------------------------------------------------------------
try:
    import depthai as dai  # type: ignore
    DEPTHAI_AVAILABLE = True
    log.info("DepthAI available — OAK-D stereo depth pipeline active.")
except ImportError:
    dai = None  # type: ignore
    DEPTHAI_AVAILABLE = False
    log.warning("DepthAI not installed. Webcam fallback active (no real depth).")

# ---------------------------------------------------------------------------
# Optional: DroneKit (MAVLink)
# ---------------------------------------------------------------------------
try:
    from dronekit import connect as dk_connect, VehicleMode, LocationGlobalRelative  # type: ignore
    DRONEKIT_AVAILABLE = True
    log.info("DroneKit available — real MAVLink commands enabled.")
except ImportError:
    dk_connect = None  # type: ignore
    DRONEKIT_AVAILABLE = False
    log.warning("DroneKit not installed. Running in SIMULATION / display-only mode.")

# ---------------------------------------------------------------------------
# Optional: YOLOv8
# ---------------------------------------------------------------------------
try:
    from ultralytics import YOLO  # type: ignore
    _yolo_model = YOLO("yolov8n.pt")
    YOLO_AVAILABLE = True
    log.info("YOLOv8 loaded.")
except Exception as _e:
    _yolo_model = None  # type: ignore
    YOLO_AVAILABLE = False
    log.warning(f"YOLOv8 not available ({_e}). Classical CV obstacle/banner detection only.")

# ---------------------------------------------------------------------------
# Optional: pyzbar (robust QR decoding)
# ---------------------------------------------------------------------------
try:
    from pyzbar.pyzbar import decode as _zbar_decode  # type: ignore
    ZBAR_AVAILABLE = True
    log.info("pyzbar available — robust QR decoding active.")
except ImportError:
    _zbar_decode = None  # type: ignore
    ZBAR_AVAILABLE = False
    log.warning("pyzbar not installed. Using OpenCV QR fallback.")

# ===========================================================================
# MISSION STATE MACHINE
# ===========================================================================
class MissionState(Enum):
    TAKEOFF          = auto()
    SCAN_QR          = auto()
    FIND_GREEN_BANNER= auto()
    ENTER_CORRIDOR   = auto()
    FOLLOW_CORRIDOR  = auto()
    REACH_DELIVERY   = auto()
    SCAN_TARGET_QR   = auto()
    DROP_PAYLOAD     = auto()
    RETURN           = auto()
    AVOID_OBSTACLES  = auto()
    LAND             = auto()
    COMPLETE         = auto()

# ===========================================================================
# MISSION PARAMETERS
# ===========================================================================
class MissionParams:
    # Altitudes (metres, relative)
    TAKEOFF_ALT      = 5.0
    CORRIDOR_ALT     = 3.0   # 10 ft as per rulebook
    DELIVERY_ALT     = 10.0
    WINCH_LOWER_ALT  = 5.0

    # Speeds (m/s)
    CRUISE_SPEED     = 2.0
    SLOW_SPEED       = 1.0
    CORRIDOR_SPEED   = 2.0

    # Obstacle distance thresholds (metres)
    OBS_CONTINUE_M   = 2.0   # > 2 m → continue
    OBS_SLOW_M       = 1.0   # 1–2 m → slow down
    # < OBS_SLOW_M   → stop / avoid

    # Tolerances
    ALT_TOL_M        = 0.3
    LAT_TOL_M        = 0.25
    CORRIDOR_CENTRE_TOL_M = 0.3

    # QR scan
    QR_SCAN_ATTEMPTS = 8    # failures before switching to Arducam backup
    QR_CONFIRM_FRAMES = 3   # consecutive frames for delivery zone QR

    # Winch timing (seconds)
    WINCH_LOWER_TIME = 5.0
    WINCH_HOLD_TIME  = 2.0
    WINCH_REEL_TIME  = 5.0
    WINCH_SERVO_CH   = 9    # AUX servo channel

    # Payload mass and CG offset (kg, metres) — used after drop
    PAYLOAD_MASS_KG  = 0.2
    FRAME_MASS_KG    = 1.8
    PAYLOAD_OFFSET_M = 0.15  # distance from CoG where payload hangs

P = MissionParams()

# ===========================================================================
# 1. OAK-D PIPELINE  —  RGB + Stereo Depth
# ===========================================================================

class OakDCamera:
    """
    Wraps the DepthAI OAK-D pipeline.
    Produces:
      rgb_frame  : BGR uint8 (H×W×3)
      depth_frame: uint16 in millimetres (H×W), aligned to RGB space
    Falls back to webcam + synthetic depth if DepthAI is not available.
    """

    def __init__(self, rgb_fps: int = 30, depth_fps: int = 30) -> None:
        self._device = None
        self._q_rgb   = None
        self._q_depth = None
        self._cap     = None  # webcam fallback

        if DEPTHAI_AVAILABLE:
            self._setup_oak(rgb_fps, depth_fps)
        else:
            log.info("OAK-D: using webcam fallback (no real depth).")
            self._cap = cv2.VideoCapture(0)

    # ------------------------------------------------------------------
    def _setup_oak(self, rgb_fps: int, depth_fps: int) -> None:
        pipeline = dai.Pipeline()

        # RGB camera
        cam_rgb = pipeline.create(dai.node.ColorCamera)
        cam_rgb.setPreviewSize(640, 480)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam_rgb.setFps(rgb_fps)

        # Stereo depth
        mono_left  = pipeline.create(dai.node.MonoCamera)
        mono_right = pipeline.create(dai.node.MonoCamera)
        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_left.setCamera("left")
        mono_right.setCamera("right")
        mono_left.setFps(depth_fps)
        mono_right.setFps(depth_fps)

        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)  # align depth to RGB
        stereo.setOutputSize(640, 480)
        stereo.setLeftRightCheck(True)
        stereo.setExtendedDisparity(False)
        stereo.setSubpixel(False)

        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)

        # XLinkOut nodes
        xout_rgb   = pipeline.create(dai.node.XLinkOut)
        xout_depth = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")
        xout_depth.setStreamName("depth")

        cam_rgb.preview.link(xout_rgb.input)
        stereo.depth.link(xout_depth.input)

        self._device  = dai.Device(pipeline)
        self._q_rgb   = self._device.getOutputQueue("rgb",   maxSize=4, blocking=False)
        self._q_depth = self._device.getOutputQueue("depth", maxSize=4, blocking=False)
        log.info("OAK-D pipeline started.")

    # ------------------------------------------------------------------
    def read(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Returns (rgb_bgr [H,W,3 uint8], depth_mm [H,W uint16]) or (None, None).
        depth_mm values are in millimetres; 0 = invalid/no measurement.
        """
        if self._device is not None:
            in_rgb   = self._q_rgb.tryGet()
            in_depth = self._q_depth.tryGet()
            if in_rgb is None or in_depth is None:
                return None, None
            rgb   = in_rgb.getCvFrame()                    # BGR uint8
            depth = in_depth.getFrame().astype(np.uint16)  # mm uint16
            return rgb, depth

        # Webcam fallback — synthetic flat depth (4 m everywhere)
        ret, frame = self._cap.read()
        if not ret:
            return None, None
        h, w = frame.shape[:2]
        synthetic_depth = np.full((h, w), 4000, dtype=np.uint16)
        return frame, synthetic_depth

    # ------------------------------------------------------------------
    def release(self) -> None:
        if self._cap is not None:
            self._cap.release()
        if self._device is not None:
            del self._device
        log.info("Camera released.")


# ===========================================================================
# 2. DEPTH QUERY  —  Bounding Box → Median Distance
# ===========================================================================

DEPTH_UNKNOWN_M = float("inf")  # sentinel: "no valid depth reading" — NEVER treat as safe


def depth_at_bbox(
    depth_mm: np.ndarray,
    bbox: Tuple[int, int, int, int],
    percentile: float = 50.0,
) -> float:
    """
    Query the depth map inside a bounding box and return the median (or chosen
    percentile) distance in metres.

    Args:
        depth_mm : Depth frame from OAK-D in millimetres (uint16, H×W).
        bbox     : (x, y, w, h) in pixel coordinates of the RGB frame.
        percentile: Which percentile of valid depth values to return (default 50 = median).

    Returns:
        Distance in metres (float).  Returns DEPTH_UNKNOWN_M (inf) if no valid
        pixels exist — callers MUST treat this as "unknown", never as "clear/far".

    Example (from the specification):
        bbox = (250, 180, 60, 80)   # obstacle bounding box
        depth grid inside:
            1800  1900  1800
            1800  1900  2000
            1900  1800  1900
        median → 1900 mm → 1.9 m
    """
    x, y, w, h = bbox
    frame_h, frame_w = depth_mm.shape[:2]

    # Clamp bounding box to frame boundaries
    x1 = max(0, x)
    y1 = max(0, y)
    x2 = min(frame_w, x + w)
    y2 = min(frame_h, y + h)

    if x2 <= x1 or y2 <= y1:
        return DEPTH_UNKNOWN_M

    roi = depth_mm[y1:y2, x1:x2]

    # Valid pixels: non-zero (0 means "no measurement" from OAK-D)
    valid = roi[roi > 0].astype(np.float32)

    if valid.size == 0:
        return DEPTH_UNKNOWN_M

    return float(np.percentile(valid, percentile)) / 1000.0  # mm → m


def depth_action(distance_m: float) -> str:
    """
    Map a distance to an action string (used by the obstacle / corridor logic).
      unknown (inf) --> "STOP"   (never treat missing depth as safe)
      > 2.0 m        --> "CONTINUE"
      1.0-2.0 m      --> "SLOW"
      < 1.0 m        --> "STOP"
    """
    if math.isinf(distance_m):
        return "STOP"
    if distance_m > P.OBS_CONTINUE_M:
        return "CONTINUE"
    elif distance_m > P.OBS_SLOW_M:
        return "SLOW"
    else:
        return "STOP"


# ===========================================================================
# 3. PREPROCESSING
# ===========================================================================

def preprocess(bgr: np.ndarray) -> np.ndarray:
    """
    CLAHE on the L channel (LAB colour space) to normalise brightness across
    shade → bright-sun transitions without shifting hue, keeping HSV masks reliable.
    """
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2Lab)
    l_ch, a_ch, b_ch = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    l_ch  = clahe.apply(l_ch)
    return cv2.cvtColor(cv2.merge([l_ch, a_ch, b_ch]), cv2.COLOR_Lab2BGR)


# ===========================================================================
# 4. GREEN BANNER DETECTION
# ===========================================================================

# HSV range for the AeroTHON 2026 green banner
BANNER_HSV_LOWER = np.array([35,  80,  60])
BANNER_HSV_UPPER = np.array([85, 255, 255])
BANNER_MIN_AREA  = 3000   # px²  — ignore small green blobs
BANNER_MIN_AR    = 1.5    # minimum width/height aspect ratio (banner is wide)


def detect_green_banner(bgr: np.ndarray, yolo_results=None) -> Optional[Tuple[int, int, int, int]]:
    """
    Detect the AeroTHON green banner using HSV colour segmentation.
    Returns the largest qualifying bounding box (x, y, w, h) or None.

    YOLOv8 is used in parallel if available; the result with higher confidence
    (or the one that returns a valid box) takes precedence.
    """
    hsv  = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, BANNER_HSV_LOWER, BANNER_HSV_UPPER)

    # Morphological cleanup — close holes, remove noise
    kernel = np.ones((9, 9), np.uint8)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best_box  = None
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
            best_box  = (x, y, w, h)

    # If YOLO is available, also check for a banner-like detection
    # Use pre-computed results — do NOT call _yolo_model() again here
    if yolo_results is not None:
        for r in yolo_results:
            for box in r.boxes:
                cls  = int(box.cls[0])
                conf = float(box.conf[0])
                name = _yolo_model.names[cls]
                if conf > 0.6 and "banner" in name.lower():
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    yolo_box  = (x1, y1, x2 - x1, y2 - y1)
                    yolo_area = (x2 - x1) * (y2 - y1)
                    if yolo_area > best_area:
                        best_box  = yolo_box
                        best_area = yolo_area

    return best_box


# ===========================================================================
# 5. QR CODE DETECTION
# ===========================================================================

_qr_detector = cv2.QRCodeDetector()


def _zbar_attempt(gray: np.ndarray, scale: float = 1.0) -> List[Tuple[str, np.ndarray]]:
    """Run pyzbar on a greyscale image; map polygon coords back to original scale."""
    results = []
    for obj in _zbar_decode(gray):
        if obj.type != "QRCODE":
            continue
        text = obj.data.decode("utf-8", errors="ignore").strip()
        if not text:
            continue
        quad = np.array([[int(p.x / scale), int(p.y / scale)]
                         for p in obj.polygon], dtype=int)
        results.append((text, quad))
    return results


def detect_qr_codes(bgr: np.ndarray) -> List[Tuple[str, np.ndarray]]:
    """
    Detect and fully decode all QR codes in the frame.

    Strategy — three attempts in order (each more aggressive):
      1. Raw greyscale
      2. 4× upscale (handles small / distant QR codes)
      3. Adaptive threshold (handles uneven lighting)

    pyzbar is tried first (more robust); OpenCV QRCodeDetector is the fallback.
    Only returns codes that decode to non-empty text.
    """
    gray      = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    upscaled  = cv2.resize(gray, None, fx=4.0, fy=4.0,
                           interpolation=cv2.INTER_CUBIC)
    threshold = cv2.adaptiveThreshold(
        gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 35, 5)

    attempts = [(gray, 1.0), (upscaled, 4.0), (threshold, 1.0)]

    if ZBAR_AVAILABLE and _zbar_decode is not None:
        for img, scale in attempts:
            results = _zbar_attempt(img, scale)
            if results:
                return results

    # OpenCV fallback
    for img, scale in attempts:
        bgr_var = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR) if img.ndim == 2 else img
        ok, decoded_info, points, _ = _qr_detector.detectAndDecodeMulti(bgr_var)
        if ok and points is not None and decoded_info:
            results = [
                (t, (pts / scale).astype(int))
                for t, pts in zip(decoded_info, points)
                if t
            ]
            if results:
                return results

    return []


# ===========================================================================
# 6. OBSTACLE DETECTION (with depth)
# ===========================================================================

# Dark-blob / circular obstacle constants
OBS_VAL_MAX          = 90
OBS_MIN_AREA         = 400
OBS_MAX_AREA_RATIO   = 0.15
OBS_MIN_CIRCULARITY  = 0.35

# Red zone (no-fly / danger) constants
RED_HUE_RANGES    = [(0, 8), (172, 180)]
RED_SAT_MIN       = 150
RED_VAL_MIN       = 90
RED_ZONE_MIN_AREA = 600
RED_MERGE_DIST_PX = 60

# Sky mask
SKY_HUE_RANGE = (90, 135)
SKY_SAT_MAX   = 60
SKY_VAL_MIN   = 150


def _build_sky_mask(hsv: np.ndarray) -> np.ndarray:
    h, s, v = cv2.split(hsv)
    lo, hi   = SKY_HUE_RANGE
    blue_sky = (h >= lo) & (h <= hi) & (v >= SKY_VAL_MIN)
    hazy_sky = (s <= SKY_SAT_MAX) & (v >= SKY_VAL_MIN)
    mask     = ((blue_sky | hazy_sky).astype(np.uint8)) * 255
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))


def _build_red_mask(hsv: np.ndarray) -> np.ndarray:
    h, s, v = cv2.split(hsv)
    mask     = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lo, hi in RED_HUE_RANGES:
        mask |= (((h >= lo) & (h <= hi) & (s >= RED_SAT_MIN) &
                  (v >= RED_VAL_MIN)).astype(np.uint8)) * 255
    kernel = np.ones((15, 15), np.uint8)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return cv2.dilate(mask, kernel, iterations=1)


def _merge_boxes(boxes: List[tuple], max_dist: int) -> List[tuple]:
    """Iteratively merge bounding boxes closer than max_dist pixels."""
    boxes  = [list(b) for b in boxes]
    merged = True
    while merged:
        merged = False
        for i in range(len(boxes)):
            for j in range(i + 1, len(boxes)):
                x1, y1, w1, h1 = boxes[i]
                x2, y2, w2, h2 = boxes[j]
                ax1, ay1 = x1,      y1
                ax2, ay2 = x1 + w1, y1 + h1
                bx1, by1 = x2,      y2
                bx2, by2 = x2 + w2, y2 + h2
                gap_x = max(bx1 - ax2, ax1 - bx2, 0)
                gap_y = max(by1 - ay2, ay1 - by2, 0)
                if gap_x <= max_dist and gap_y <= max_dist:
                    nx1 = min(ax1, bx1)
                    ny1 = min(ay1, by1)
                    nx2 = max(ax2, bx2)
                    ny2 = max(ay2, by2)
                    boxes[i] = [nx1, ny1, nx2 - nx1, ny2 - ny1]
                    del boxes[j]
                    merged = True
                    break
            if merged:
                break
    return [tuple(b) for b in boxes]


def detect_obstacles_with_depth(
    bgr: np.ndarray,
    depth_mm: np.ndarray,
    yolo_results=None,          # pre-computed YOLO results from perceive_frame
) -> List[dict]:
    """
    Detect obstacles and return a list of dicts with:
      {
        'cx': int,        # centre x (px)
        'cy': int,        # centre y (px)
        'radius': int,    # bounding radius (px)
        'bbox': tuple,    # (x, y, w, h) in pixels
        'dist_m': float,  # median distance from depth map (metres)
        'action': str,    # "CONTINUE" / "SLOW" / "STOP"
        'source': str,    # "classical_cv" or "yolo"
      }
    """
    frame = preprocess(bgr)
    hsv   = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, w  = frame.shape[:2]
    frame_area = h * w

    sky_mask = _build_sky_mask(hsv)
    red_mask = _build_red_mask(hsv)
    hue_ch, sat_ch, val_ch = cv2.split(hsv)

    # Dark-blob detection (tires, rocks, compact objects)
    dark_mask = ((val_ch <= OBS_VAL_MAX).astype(np.uint8)) * 255
    dark_mask = cv2.bitwise_and(dark_mask, cv2.bitwise_not(red_mask))
    dark_mask = cv2.bitwise_and(dark_mask, cv2.bitwise_not(sky_mask))

    contours, _ = cv2.findContours(dark_mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    obstacles = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < OBS_MIN_AREA or area > frame_area * OBS_MAX_AREA_RATIO:
            continue
        perimeter = cv2.arcLength(c, True)
        if perimeter == 0:
            continue
        circularity = 4 * math.pi * area / (perimeter ** 2)
        if circularity < OBS_MIN_CIRCULARITY:
            continue
        (cx, cy), radius = cv2.minEnclosingCircle(c)
        cx, cy, radius = int(cx), int(cy), int(radius)
        bbox = (cx - radius, cy - radius, radius * 2, radius * 2)
        dist = depth_at_bbox(depth_mm, bbox)
        obstacles.append({
            "cx": cx, "cy": cy, "radius": radius,
            "bbox": bbox, "dist_m": dist,
            "action": depth_action(dist),
            "source": "classical_cv",
        })

    # YOLOv8 supplement — use pre-computed results to avoid a second inference call
    if yolo_results is not None:
        for r in yolo_results:
            for box in r.boxes:
                conf = float(box.conf[0])
                if conf < 0.5:
                    continue
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                bw, bh = x2 - x1, y2 - y1
                bbox   = (x1, y1, bw, bh)
                dist   = depth_at_bbox(depth_mm, bbox)
                cx     = x1 + bw // 2
                cy     = y1 + bh // 2
                radius = max(bw, bh) // 2
                obstacles.append({
                    "cx": cx, "cy": cy, "radius": radius,
                    "bbox": bbox, "dist_m": dist,
                    "action": depth_action(dist),
                    "source": "yolo",
                })

    return obstacles


def find_red_zones(bgr: np.ndarray) -> List[Tuple[int, int, int, int]]:
    """Return merged red-zone bounding boxes (x, y, w, h)."""
    hsv      = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    red_mask = _build_red_mask(hsv)
    contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    zones = []
    for c in contours:
        if cv2.contourArea(c) < RED_ZONE_MIN_AREA:
            continue
        zones.append(cv2.boundingRect(c))
    return _merge_boxes(zones, RED_MERGE_DIST_PX)


# ===========================================================================
# 7. CORRIDOR NAVIGATION (wall-following with depth)
# ===========================================================================

def corridor_lateral_error(depth_mm: np.ndarray) -> float:
    """
    Compute a lateral correction error for corridor-following.

    Samples the left and right 20 % columns of the depth frame (walls).
    Returns a signed lateral error (metres difference):
      positive  → right wall is farther away → drone is left-of-centre → nudge RIGHT
      negative  → left  wall is farther away → drone is right-of-centre → nudge LEFT
      zero      → perfectly centred
    Returns 0.0 if depth is unavailable.
    """
    if depth_mm is None:
        return 0.0

    h, w = depth_mm.shape[:2]
    col_w = w // 5  # 20 % of width

    left_roi  = depth_mm[:, :col_w]
    right_roi = depth_mm[:, w - col_w:]

    valid_left  = left_roi[left_roi   > 0].astype(float)
    valid_right = right_roi[right_roi > 0].astype(float)

    if valid_left.size == 0 or valid_right.size == 0:
        return 0.0

    d_left  = np.median(valid_left)   / 1000.0  # mm → m
    d_right = np.median(valid_right)  / 1000.0

    # d_right - d_left:
    #   positive → right wall farther → drone is left of centre → caller nudges RIGHT (+vy)
    #   negative → left wall farther  → drone is right of centre → caller nudges LEFT  (-vy)
    return float(d_right - d_left)


# ===========================================================================
# 8. DRONEKIT FLIGHT CONTROLLER INTERFACE
# ===========================================================================

class FlightController:
    """
    Wraps DroneKit commands.  Falls back to logging-only mode when DroneKit is
    absent so the vision/FSM code can be tested without a real flight controller.
    """

    def __init__(self, connection_string: str) -> None:
        self._vehicle = None
        self._sim     = not DRONEKIT_AVAILABLE

        if DRONEKIT_AVAILABLE and connection_string:
            log.info(f"Connecting to FC at {connection_string} …")
            self._vehicle = dk_connect(
                connection_string, wait_ready=True, heartbeat_timeout=30)
            log.info("FC connected.  Firmware: %s", self._vehicle.version)
        else:
            log.info("FC: simulation / display-only mode.")

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    def _send_ned_velocity(self, vx: float, vy: float, vz: float,
                           duration: float = 0.5) -> None:
        """
        Send body-frame velocity setpoint (NED: +x=North/forward, +y=East/right,
        +z=Down) via MAVLink SET_POSITION_TARGET_LOCAL_NED.
        """
        if self._sim or self._vehicle is None:
            log.debug("SIM velocity: vx=%.2f vy=%.2f vz=%.2f", vx, vy, vz)
            return
        from pymavlink import mavutil  # type: ignore  (comes with dronekit)
        msg = self._vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111,   # velocity only
            0, 0, 0,              # position (ignored)
            vx, vy, vz,           # velocity
            0, 0, 0,              # acceleration (ignored)
            0, 0,                 # yaw, yaw_rate (ignored)
        )
        end_time = time.time() + duration
        while time.time() < end_time:
            self._vehicle.send_mavlink(msg)
            time.sleep(0.1)

    def _current_alt(self) -> float:
        if self._sim or self._vehicle is None:
            return 0.0
        return float(self._vehicle.location.global_relative_frame.alt or 0.0)

    # ------------------------------------------------------------------
    # Public mission commands
    # ------------------------------------------------------------------
    def arm_and_takeoff(self, target_alt: float) -> None:
        if self._sim or self._vehicle is None:
            log.info("SIM TAKEOFF to %.1f m", target_alt)
            time.sleep(0.5)
            return
        log.info("Arming …")
        self._vehicle.mode = VehicleMode("GUIDED")
        self._vehicle.armed = True
        while not self._vehicle.armed:
            log.debug("Waiting for arm …")
            time.sleep(0.5)
        log.info("Taking off to %.1f m …", target_alt)
        self._vehicle.simple_takeoff(target_alt)
        while True:
            alt = self._current_alt()
            log.debug("Alt: %.2f m", alt)
            if alt >= target_alt - P.ALT_TOL_M:
                break
            time.sleep(0.2)
        log.info("Takeoff complete.")

    def move_forward(self, speed: float = P.CRUISE_SPEED,
                     duration: float = 1.0) -> None:
        """Move forward (North in body frame) at given speed for duration seconds."""
        log.debug("FORWARD %.2f m/s × %.1f s", speed, duration)
        self._send_ned_velocity(speed, 0, 0, duration)

    def move_lateral(self, vy: float, duration: float = 0.5) -> None:
        """Lateral correction: positive vy = right, negative = left."""
        log.debug("LATERAL vy=%.2f × %.1f s", vy, duration)
        self._send_ned_velocity(0, vy, 0, duration)

    def stop(self) -> None:
        """Halt all velocity — hover in place."""
        log.info("STOP (hover)")
        self._send_ned_velocity(0, 0, 0, 0.5)

    def set_altitude(self, target_alt: float) -> None:
        if self._sim or self._vehicle is None:
            log.info("SIM ALT CHANGE → %.1f m", target_alt)
            time.sleep(0.5)
            return
        log.info("Changing altitude to %.1f m …", target_alt)
        cur = self._vehicle.location.global_relative_frame
        target = LocationGlobalRelative(cur.lat, cur.lon, target_alt)
        self._vehicle.simple_goto(target)
        while abs(self._current_alt() - target_alt) > P.ALT_TOL_M:
            time.sleep(0.2)
        log.info("Altitude %.1f m reached.", target_alt)

    def yaw_to(self, heading_deg: float, relative: bool = True) -> None:
        if self._sim or self._vehicle is None:
            log.info("SIM YAW to %.1f° (relative=%s)", heading_deg, relative)
            return
        from pymavlink import mavutil  # type: ignore
        is_relative = 1 if relative else 0
        msg = self._vehicle.message_factory.command_long_encode(
            0, 0,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0, heading_deg, 20, 1, is_relative, 0, 0, 0)
        self._vehicle.send_mavlink(msg)
        time.sleep(1.5)

    def set_winch(self, pwm_us: int) -> None:
        """Send winch servo command via MAV_CMD_DO_SET_SERVO."""
        if self._sim or self._vehicle is None:
            log.info("SIM WINCH servo %d → %d µs", P.WINCH_SERVO_CH, pwm_us)
            return
        from pymavlink import mavutil  # type: ignore
        msg = self._vehicle.message_factory.command_long_encode(
            0, 0,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,
            P.WINCH_SERVO_CH,  # servo channel
            pwm_us,            # PWM value
            0, 0, 0, 0, 0,
        )
        self._vehicle.send_mavlink(msg)

    def return_to_launch(self) -> None:
        if self._sim or self._vehicle is None:
            log.info("SIM RTL")
            return
        log.info("RETURN TO LAUNCH")
        self._vehicle.mode = VehicleMode("RTL")

    def land(self) -> None:
        if self._sim or self._vehicle is None:
            log.info("SIM LAND")
            time.sleep(0.5)
            return
        log.info("LAND")
        self._vehicle.mode = VehicleMode("LAND")
        while self._current_alt() > 0.2:
            time.sleep(0.5)
        log.info("Landed.")

    def close(self) -> None:
        if self._vehicle is not None:
            self._vehicle.close()


# ===========================================================================
# 9. WINCH / PAYLOAD DROP SEQUENCE
# ===========================================================================

def winch_lower_and_release(fc: FlightController) -> None:
    """
    ADRC-modelled winch sequence:
      1. Ramp-lower payload
      2. Wait for ground contact (current spike or timer)
      3. Mechanical release (servo detach PWM)
      4. Ramp reel-back
    """
    log.info("[WINCH] Ramp lower …")
    PWM_LOWER   = 1700   # µs — lower direction
    PWM_NEUTRAL = 1500   # µs — stop
    PWM_RAISE   = 1300   # µs — raise direction
    PWM_RELEASE = 1900   # µs — mechanical release trigger

    fc.set_winch(PWM_LOWER)
    time.sleep(P.WINCH_LOWER_TIME)

    fc.set_winch(PWM_NEUTRAL)
    time.sleep(P.WINCH_HOLD_TIME)

    log.info("[WINCH] Mechanical release …")
    fc.set_winch(PWM_RELEASE)
    time.sleep(0.5)
    fc.set_winch(PWM_NEUTRAL)
    time.sleep(0.5)

    log.info("[WINCH] Reel back …")
    fc.set_winch(PWM_RAISE)
    time.sleep(P.WINCH_REEL_TIME)

    fc.set_winch(PWM_NEUTRAL)
    log.info("[WINCH] Sequence complete.")


def apply_cg_correction(fc: FlightController) -> None:
    """
    After payload drop the CG shifts.  Apply a brief feed-forward correction:
      - Compute forward trim needed based on payload offset
      - Hold correction for 2 s then allow EKF to stabilise naturally
    """
    cg_shift_m = (P.PAYLOAD_MASS_KG * P.PAYLOAD_OFFSET_M) / P.FRAME_MASS_KG
    log.info("[CG] Post-drop CG shift %.3f m — applying feed-forward trim.", cg_shift_m)
    # In practice this would be a pitch bias command; simulated with a
    # small forward velocity pulse to compensate for CoG moving aft.
    fc.move_forward(speed=cg_shift_m * 0.5, duration=2.0)
    time.sleep(1.0)  # Stability gate
    log.info("[CG] Correction applied.")


# ===========================================================================
# 10a. ALWAYS-ON PERCEPTION  — runs every frame regardless of mission state
# ===========================================================================

def perceive_frame(
    bgr: np.ndarray,
    depth_mm: np.ndarray,
) -> dict:
    """
    Run ALL perception on a single frame and return a unified perception dict.
    This is called at the top of EVERY loop iteration before the state machine
    so that obstacles, red zones, humans, banner, QR codes, and steering are
    always visible — not just in specific states.

    Returns:
        {
          "obstacles"  : List[dict]   — depth-annotated obstacle list
          "humans"     : List[dict]   — YOLO-detected people (separate label)
          "yolo_all"   : List[tuple]  — all YOLO detections (name, conf, box)
          "red_zones"  : List[tuple]  — merged red-zone bounding boxes
          "qr_codes"   : List[tuple]  — decoded (text, quad) pairs
          "banner_box" : Optional[tuple] — green banner (x,y,w,h) or None
          "steering"   : str          — "FORWARD" / "LEFT" / "RIGHT" / "STOP"
          "fwd_dist_m" : float        — forward centre distance in metres
        }
    """
    # --- 1. Run YOLO exactly ONCE for this frame ------------------------------
    # All subsystems (obstacles, humans, banner) share this single inference.
    _yolo_results = None
    humans: List[dict] = []
    yolo_all: List[tuple] = []
    if YOLO_AVAILABLE and _yolo_model is not None:
        _yolo_results = _yolo_model(bgr, stream=False, verbose=False)
        for r in _yolo_results:
            for box in r.boxes:
                conf = float(box.conf[0])
                if conf < 0.45:
                    continue
                cls  = int(box.cls[0])
                name = _yolo_model.names[cls]
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                bbox = (x1, y1, x2 - x1, y2 - y1)
                dist = depth_at_bbox(depth_mm, bbox)
                yolo_all.append((name, conf, bbox, dist))
                if name == "person":
                    bw, bh = x2 - x1, y2 - y1
                    humans.append({
                        "bbox": bbox, "dist_m": dist,
                        "action": depth_action(dist),
                        "conf": conf,
                        "cx": x1 + bw // 2,
                        "cy": y1 + bh // 2,
                    })

    # --- 2. Obstacles (classical CV + shared YOLO results) ----------------
    obstacles = detect_obstacles_with_depth(bgr, depth_mm,
                                            yolo_results=_yolo_results)

    # --- 3. Red zones (HSV colour) ----------------------------------------
    red_zones = find_red_zones(bgr)

    # --- 4. QR codes ----------------------------------------------------------
    qr_codes = detect_qr_codes(bgr)

    # --- 5. Green banner (shared YOLO results) --------------------------------
    banner_box = detect_green_banner(bgr, yolo_results=_yolo_results)

    # --- 6. Steering direction ------------------------------------------------
    # Sample depth in three vertical columns (left / centre / right)
    h_f, w_f = depth_mm.shape[:2]
    col_w = w_f // 3
    def _col_dist(x_start: int) -> float:
        bbox = (x_start, 0, col_w, h_f)
        return depth_at_bbox(depth_mm, bbox)

    d_left   = _col_dist(0)
    d_centre = _col_dist(col_w)
    d_right  = _col_dist(col_w * 2)

    # Also check if any obstacle is in STOP range directly ahead
    centre_stop = any(
        o["action"] == "STOP" and
        abs(o["cx"] - w_f // 2) < w_f // 3
        for o in obstacles
    )
    human_stop = any(h["action"] == "STOP" for h in humans)

    # Check if any red zone centroid is in the centre third of the frame
    red_zone_ahead = any(
        col_w <= (rx + rw // 2) <= col_w * 2
        for rx, ry, rw, rh in red_zones
    )

    if centre_stop or human_stop or d_centre < P.OBS_SLOW_M:
        steering = "STOP"
    elif red_zone_ahead:
        # Red zone blocks the path — divert to whichever side has more space
        steering = "LEFT" if d_left > d_right else "RIGHT"
    elif d_centre >= d_left and d_centre >= d_right:
        steering = "FORWARD"
    elif d_left > d_right:
        steering = "LEFT"
    else:
        steering = "RIGHT"

    return {
        "obstacles" : obstacles,
        "humans"    : humans,
        "yolo_all"  : yolo_all,
        "red_zones" : red_zones,
        "qr_codes"  : qr_codes,
        "banner_box": banner_box,
        "steering"  : steering,
        "fwd_dist_m": d_centre,
        "d_left"    : d_left,
        "d_right"   : d_right,
    }


# ===========================================================================
# 10b. ANNOTATION  — draw everything onto the frame
# ===========================================================================

def annotate_frame(
    bgr: np.ndarray,
    percept: dict,
    state: MissionState,
    target_id: str,
) -> np.ndarray:
    """
    Draw ALL perception results onto the frame and add the HUD.
    Uses the unified percept dict from perceive_frame().
    """
    out = bgr.copy()
    h, w = out.shape[:2]

    obstacles  = percept["obstacles"]
    humans     = percept["humans"]
    yolo_all   = percept["yolo_all"]
    red_zones  = percept["red_zones"]
    qr_codes   = percept["qr_codes"]
    banner_box = percept["banner_box"]
    steering   = percept["steering"]
    fwd_dist   = percept["fwd_dist_m"]
    d_left     = percept["d_left"]
    d_right    = percept["d_right"]

    # ---- Red zones --------------------------------------------------------
    for x, y, bw, bh in red_zones:
        cv2.rectangle(out, (x, y), (x + bw, y + bh), (0, 0, 220), 3)
        cv2.putText(out, "RED ZONE", (x, max(0, y - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 220), 2)

    # ---- Obstacles (classical CV blobs) -----------------------------------
    for obs in obstacles:
        if obs["source"] == "yolo":
            continue   # drawn separately below with YOLO labels
        cx, cy, radius = obs["cx"], obs["cy"], obs["radius"]
        dist_m = obs["dist_m"]
        action = obs["action"]
        colour = {"CONTINUE": (0, 200, 0),
                  "SLOW":     (0, 165, 255),
                  "STOP":     (0, 0, 255)}.get(action, (0, 165, 255))
        cv2.circle(out, (cx, cy), radius, colour, 3)
        cv2.putText(out, f"OBS {dist_m:.1f}m [{action}]",
                    (cx - radius, max(0, cy - radius - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, colour, 2)

    # ---- ALL YOLO detections (objects, vehicles, animals, etc.) ----------
    for name, conf, (x1, y1, bw, bh), dist in yolo_all:
        if name == "person":
            continue  # drawn with dedicated human box below
        colour = (0, 200, 255)   # cyan for generic YOLO objects
        cv2.rectangle(out, (x1, y1), (x1 + bw, y1 + bh), colour, 2)
        label = f"{name} {conf:.2f} {dist:.1f}m [{depth_action(dist)}]"
        cv2.putText(out, label, (x1, max(0, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, colour, 2)

    # ---- Humans (bright magenta + explicit HUMAN label) ------------------
    for human in humans:
        x1, y1, bw, bh = human["bbox"]
        dist_m = human["dist_m"]
        action = human["action"]
        hcol   = (255, 0, 200)   # bright magenta — stands out from everything
        cv2.rectangle(out, (x1, y1), (x1 + bw, y1 + bh), hcol, 3)
        cv2.putText(out, f"HUMAN {dist_m:.1f}m [{action}]",
                    (x1, max(0, y1 - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.58, hcol, 2)

    # ---- QR codes ---------------------------------------------------------
    for text, quad in qr_codes:
        cv2.polylines(out, [quad.reshape(-1, 1, 2)], True, (255, 0, 255), 3)
        qx, qy = int(quad[0][0]), int(quad[0][1])
        cv2.putText(out, text[:30], (qx, max(0, qy - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)

    # ---- Green banner -----------------------------------------------------
    if banner_box is not None:
        bx, by, bw2, bh2 = banner_box
        cv2.rectangle(out, (bx, by), (bx + bw2, by + bh2), (0, 255, 80), 3)
        cv2.putText(out, "GREEN BANNER", (bx, max(0, by - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 80), 2)

    # ---- Depth column indicators (L / C / R) shown as vertical bars ------
    bar_y  = h - 18
    bar_h  = 12
    col_w3 = w // 3

    def _bar_colour(d: float) -> tuple:
        if d > P.OBS_CONTINUE_M:
            return (0, 220, 0)
        elif d > P.OBS_SLOW_M:
            return (0, 165, 255)
        else:
            return (0, 0, 255)

    for i, (d, label_d) in enumerate([(d_left, "L"), (fwd_dist, "C"), (d_right, "R")]):
        x0 = i * col_w3
        cv2.rectangle(out, (x0, bar_y - bar_h), (x0 + col_w3, bar_y),
                      _bar_colour(d), -1)
        cv2.putText(out, f"{label_d}:{d:.1f}m", (x0 + 4, bar_y - 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 255), 1)

    # ---- Steering arrow in centre -----------------------------------------
    arrow_colour = {"FORWARD": (0, 255, 0),
                    "LEFT":    (255, 200, 0),
                    "RIGHT":   (0, 200, 255),
                    "STOP":    (0, 0, 255)}.get(steering, (200, 200, 200))
    arrow_text   = {"FORWARD": "^  FWD",
                    "LEFT":    "<  LEFT",
                    "RIGHT":   "RIGHT  >",
                    "STOP":    "!! STOP !!"}.get(steering, steering)
    cv2.putText(out, arrow_text, (w // 2 - 60, h - 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.75, arrow_colour, 2)

    # ---- Top HUD bar (state + target) ------------------------------------
    cv2.rectangle(out, (0, 0), (w, 42), (0, 0, 0), -1)
    cv2.putText(out, f"STATE: {state.name}", (8, 26),
                cv2.FONT_HERSHEY_SIMPLEX, 0.72, (0, 230, 230), 2)
    if target_id:
        cv2.putText(out, f"TGT: {target_id[:18]}", (w // 2, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 220, 0), 2)

    return out


# ===========================================================================
# 11. FULL MISSION FSM LOOP
# ===========================================================================

def run_mission(fc: FlightController, camera: OakDCamera,
                headless: bool = False) -> None:
    """
    Main mission loop.

    Every iteration:
      1. Grab RGB + depth from OAK-D.
      2. Run perceive_frame() — obstacles, humans, red zones, QR, banner,
         steering — ALL computed EVERY frame regardless of state.
      3. GLOBAL SAFETY CHECK — if any obstacle/human is critically close,
         stop the drone immediately regardless of current state.
      4. Run the state-specific logic.
      5. Annotate and display.
    """
    state       = MissionState.TAKEOFF
    target_id   = ""    # text decoded from start QR
    qr_fail_cnt      = 0  # consecutive QR scan failures
    qr_confirm       = 0  # consecutive matching delivery-zone QR frames
    banner_fail_cnt  = 0  # frames without banner in FIND_GREEN_BANNER
    drop_align_fails = 0  # frames without QR lock during DROP_PAYLOAD alignment
    state_before_avoid = MissionState.RETURN   # where to resume after avoidance

    log.info("=== MISSION START ===")

    while state != MissionState.COMPLETE:

        # ----------------------------------------------------------------
        # 1. GRAB FRAME
        # ----------------------------------------------------------------
        rgb, depth_mm = camera.read()
        if rgb is None:
            time.sleep(0.03)
            continue
        bgr = rgb

        # ----------------------------------------------------------------
        # 2. ALWAYS-ON PERCEPTION — runs every single frame
        # ----------------------------------------------------------------
        percept    = perceive_frame(bgr, depth_mm)
        obstacles  = percept["obstacles"]
        humans     = percept["humans"]
        red_zones  = percept["red_zones"]
        qr_codes   = percept["qr_codes"]
        banner_box = percept["banner_box"]
        steering   = percept["steering"]
        fwd_dist   = percept["fwd_dist_m"]

        # ----------------------------------------------------------------
        # 3. GLOBAL SAFETY INTERRUPT
        #    Any obstacle OR human < 1 m in ANY state → stop & avoid.
        #    Skipped during TAKEOFF, LAND, DROP_PAYLOAD (can't dodge those).
        # ----------------------------------------------------------------
        _safe_states = {MissionState.TAKEOFF, MissionState.LAND,
                        MissionState.DROP_PAYLOAD, MissionState.AVOID_OBSTACLES}
        if state not in _safe_states:
            all_threats = obstacles + humans
            critical = [t for t in all_threats
                        if t.get("action") == "STOP" and
                        t.get("cy", bgr.shape[0]) < bgr.shape[0] * 0.8]
            if critical:
                if state != MissionState.AVOID_OBSTACLES:
                    state_before_avoid = state
                    log.warning("[SAFETY] Critical threat detected in state %s "
                                "— emergency stop → AVOID_OBSTACLES", state.name)
                    fc.stop()
                    state = MissionState.AVOID_OBSTACLES

        # ================================================================
        # 4. STATE MACHINE
        # ================================================================

        # ----------------------------------------------------------------
        if state == MissionState.TAKEOFF:
        # ----------------------------------------------------------------
            log.info("[TAKEOFF] Arming and ascending to %.1f m …", P.TAKEOFF_ALT)
            fc.arm_and_takeoff(P.TAKEOFF_ALT)
            log.info("[TAKEOFF] Complete → SCAN_QR")
            state = MissionState.SCAN_QR

        # ----------------------------------------------------------------
        elif state == MissionState.SCAN_QR:
        # ----------------------------------------------------------------
            # qr_codes already detected by perceive_frame()
            if qr_codes:
                target_id   = qr_codes[0][0]
                qr_fail_cnt = 0
                log.info("[SCAN_QR] target_id='%s' → FIND_GREEN_BANNER", target_id)
                state = MissionState.FIND_GREEN_BANNER
            else:
                qr_fail_cnt += 1
                log.debug("[SCAN_QR] No QR (attempt %d/%d)",
                          qr_fail_cnt, P.QR_SCAN_ATTEMPTS)
                if qr_fail_cnt >= P.QR_SCAN_ATTEMPTS:
                    log.warning("[SCAN_QR] %d failures — rotating 45°", qr_fail_cnt)
                    fc.yaw_to(45.0, relative=True)
                    qr_fail_cnt = 0
                else:
                    fc.move_forward(speed=0.5, duration=0.8)

        # ----------------------------------------------------------------
        elif state == MissionState.FIND_GREEN_BANNER:
        # ----------------------------------------------------------------
            # banner_box already found by perceive_frame()
            if banner_box is not None:
                bx, by, bw, bh = banner_box
                frame_cx  = bgr.shape[1] // 2
                banner_cx = bx + bw // 2
                banner_fail_cnt = 0   # reset sweep counter — banner found
                dist      = depth_at_bbox(depth_mm, banner_box)
                # depth_at_bbox returns DEPTH_UNKNOWN_M (inf) when depth is
                # unavailable (webcam fallback / no valid pixels). This is NOT
                # a safety-critical path (the global obstacle/human safety
                # interrupt still runs independently every frame), so here we
                # deliberately treat "unknown" as "assume far, approach slowly"
                # rather than stopping — that lets banner approach proceed
                # even without real depth (e.g. webcam-only dev/test runs).
                if math.isinf(dist) or dist < 0.1 or dist > 80.0:
                    dist = 10.0
                log.info("[FIND_BANNER] dist=%.2f m  cx_err=%d px",
                         dist, banner_cx - frame_cx)

                lateral_err_px = banner_cx - frame_cx
                if abs(lateral_err_px) > 30:
                    vy = max(-0.5, min(0.5, 0.003 * lateral_err_px))
                    fc.move_lateral(vy, duration=0.3)
                elif dist > 2.5:
                    spd = P.CRUISE_SPEED if dist > 5.0 else P.SLOW_SPEED
                    fc.move_forward(speed=spd, duration=0.5)
                else:
                    fc.set_altitude(P.CORRIDOR_ALT)
                    log.info("[FIND_BANNER] Aligned → ENTER_CORRIDOR")
                    state = MissionState.ENTER_CORRIDOR
            else:
                banner_fail_cnt += 1
                log.debug("[FIND_BANNER] No banner — rotating 15° (fail %d)",
                          banner_fail_cnt)
                fc.yaw_to(15.0, relative=True)
                # After a full 360° sweep (24 × 15°) without finding the
                # banner, assume it is out of camera range or not present
                # (e.g. sim / test environment) and proceed to corridor entry.
                if banner_fail_cnt >= 24:
                    log.warning("[FIND_BANNER] Banner not found after full "
                                "sweep — forcing ENTER_CORRIDOR")
                    banner_fail_cnt = 0
                    state = MissionState.ENTER_CORRIDOR

        # ----------------------------------------------------------------
        elif state == MissionState.ENTER_CORRIDOR:
        # ----------------------------------------------------------------
            d_left  = percept["d_left"]
            d_right = percept["d_right"]
            if d_left < 3.0 and d_right < 3.0:
                log.info("[ENTER_CORRIDOR] Walls L=%.2f m R=%.2f m → FOLLOW_CORRIDOR",
                         d_left, d_right)
                state = MissionState.FOLLOW_CORRIDOR
            else:
                # Use global steering suggestion to navigate into corridor
                if steering == "LEFT":
                    fc.move_lateral(-P.SLOW_SPEED, duration=0.3)
                elif steering == "RIGHT":
                    fc.move_lateral(P.SLOW_SPEED, duration=0.3)
                else:
                    fc.move_forward(speed=P.CRUISE_SPEED, duration=0.5)

        # ----------------------------------------------------------------
        elif state == MissionState.FOLLOW_CORRIDOR:
        # ----------------------------------------------------------------
            lateral_err = corridor_lateral_error(depth_mm)
            vy = max(-P.SLOW_SPEED, min(P.SLOW_SPEED, 0.5 * lateral_err))

            if fwd_dist < P.OBS_SLOW_M:
                fc.stop()
                log.info("[FOLLOW_CORRIDOR] End wall at %.2f m → REACH_DELIVERY",
                         fwd_dist)
                state = MissionState.REACH_DELIVERY
            elif fwd_dist < P.OBS_CONTINUE_M:
                fc.move_forward(speed=P.SLOW_SPEED, duration=0.3)
                fc.move_lateral(vy, duration=0.1)
            else:
                fc.move_forward(speed=P.CORRIDOR_SPEED, duration=0.3)
                fc.move_lateral(vy, duration=0.1)

        # ----------------------------------------------------------------
        elif state == MissionState.REACH_DELIVERY:
        # ----------------------------------------------------------------
            fc.set_altitude(P.DELIVERY_ALT)
            log.info("[REACH_DELIVERY] At %.1f m → SCAN_TARGET_QR", P.DELIVERY_ALT)
            state      = MissionState.SCAN_TARGET_QR
            qr_confirm = 0

        # ----------------------------------------------------------------
        elif state == MissionState.SCAN_TARGET_QR:
        # ----------------------------------------------------------------
            # qr_codes and red_zones already from perceive_frame()
            matched_box = None
            for text, quad in qr_codes:
                if text != target_id:
                    continue
                qx = int(np.mean(quad[:, 0]))
                qy = int(np.mean(quad[:, 1]))
                # Reject QR inside red zone
                in_red = any(
                    rx <= qx <= rx + rw and ry <= qy <= ry + rh
                    for rx, ry, rw, rh in red_zones
                )
                if in_red:
                    log.warning("[SCAN_TARGET_QR] QR '%s' inside red zone — skip", text)
                    continue
                qw  = int(np.max(quad[:, 0]) - np.min(quad[:, 0]))
                qh  = int(np.max(quad[:, 1]) - np.min(quad[:, 1]))
                qrb = (qx - qw // 2, qy - qh // 2, max(qw, 10), max(qh, 10))
                dist = depth_at_bbox(depth_mm, qrb)
                if dist > 15.0:
                    continue
                matched_box = qrb
                qr_confirm += 1
                log.info("[SCAN_TARGET_QR] Match '%s' (%d/%d) dist=%.1f m",
                         text, qr_confirm, P.QR_CONFIRM_FRAMES, dist)
                break

            if matched_box is not None and qr_confirm >= P.QR_CONFIRM_FRAMES:
                log.info("[SCAN_TARGET_QR] Confirmed → DROP_PAYLOAD")
                state = MissionState.DROP_PAYLOAD
            elif matched_box is None:
                qr_confirm = 0
                # Slow spiral search
                fc.move_forward(speed=0.4, duration=0.5)
                fc.move_lateral(0.25, duration=0.5)

        # ----------------------------------------------------------------
        elif state == MissionState.DROP_PAYLOAD:
        # ----------------------------------------------------------------
            fc.set_altitude(P.WINCH_LOWER_ALT)
            # Fine alignment — use QR centroid from perceive_frame() result
            frame_cx = bgr.shape[1] // 2
            frame_cy = bgr.shape[0] // 2
            aligned  = False
            for text, quad in qr_codes:
                if text != target_id:
                    continue
                qx      = int(np.mean(quad[:, 0]))
                qy      = int(np.mean(quad[:, 1]))
                lat_err = qx - frame_cx
                fwd_err = qy - frame_cy
                vy = max(-0.3, min(0.3, 0.003 * lat_err))
                vx = max(-0.3, min(0.3, 0.003 * fwd_err))
                if abs(lat_err) < 20 and abs(fwd_err) < 20:
                    aligned = True
                else:
                    fc.move_lateral(vy, duration=0.3)
                    fc.move_forward(speed=vx, duration=0.3)
                break

            if aligned:
                log.info("[DROP_PAYLOAD] Aligned — winch sequence")
                drop_align_fails = 0
                winch_lower_and_release(fc)
                apply_cg_correction(fc)
                fc.set_altitude(P.DELIVERY_ALT)
                log.info("[DROP_PAYLOAD] Complete → RETURN")
                state_before_avoid = MissionState.RETURN  # resume RETURN if avoidance fires
                state = MissionState.RETURN
            else:
                fc.stop()   # hover while aligning
                if not any(text == target_id for text, _ in qr_codes):
                    # QR drifted out of frame — count and timeout after ~5 s
                    drop_align_fails += 1
                    log.debug("[DROP_PAYLOAD] QR not visible (fail %d/50)",
                              drop_align_fails)
                    if drop_align_fails >= 50:
                        log.warning("[DROP_PAYLOAD] QR lost for 50 frames — "
                                    "dropping anyway and returning")
                        drop_align_fails = 0
                        winch_lower_and_release(fc)
                        apply_cg_correction(fc)
                        fc.set_altitude(P.DELIVERY_ALT)
                        state_before_avoid = MissionState.RETURN
                        state = MissionState.RETURN
                else:
                    drop_align_fails = 0   # QR visible but still aligning — reset

        # ----------------------------------------------------------------
        elif state == MissionState.RETURN:
        # ----------------------------------------------------------------
            # Use the always-on perception steering for return flight
            if steering == "STOP":
                log.info("[RETURN] STOP threat → AVOID_OBSTACLES")
                state_before_avoid = MissionState.RETURN
                fc.stop()
                state = MissionState.AVOID_OBSTACLES
            elif steering == "LEFT":
                fc.move_lateral(-P.SLOW_SPEED, duration=0.4)
            elif steering == "RIGHT":
                fc.move_lateral(P.SLOW_SPEED, duration=0.4)
            else:
                slow_obs = [o for o in obstacles if o["action"] == "SLOW"]
                speed    = P.SLOW_SPEED if (slow_obs or humans) else P.CRUISE_SPEED
                fc.move_forward(speed=speed, duration=0.5)

            # Open space on both sides → we've exited the arena → LAND
            if percept["d_left"] > 5.0 and percept["d_right"] > 5.0:
                log.info("[RETURN] Open space → LAND")
                state = MissionState.LAND

        # ----------------------------------------------------------------
        elif state == MissionState.AVOID_OBSTACLES:
        # ----------------------------------------------------------------
            all_threats = obstacles + humans
            critical = [t for t in all_threats
                        if t.get("action") in ("STOP", "SLOW") and
                        t.get("cy", bgr.shape[0]) < bgr.shape[0] * 0.8]

            if not critical:
                log.info("[AVOID_OBSTACLES] Clear → resuming %s",
                         state_before_avoid.name)
                state = state_before_avoid
            else:
                closest = min(critical, key=lambda t: t["dist_m"])
                cx_obs  = closest["cx"]
                w_f     = bgr.shape[1]
                if cx_obs < w_f // 2:
                    fc.move_lateral(P.SLOW_SPEED, duration=0.5)
                    log.debug("[AVOID] Threat LEFT → move RIGHT")
                else:
                    fc.move_lateral(-P.SLOW_SPEED, duration=0.5)
                    log.debug("[AVOID] Threat RIGHT → move LEFT")
                if closest["dist_m"] < 0.7:
                    fc.move_forward(speed=-P.SLOW_SPEED, duration=0.3)

        # ----------------------------------------------------------------
        elif state == MissionState.LAND:
        # ----------------------------------------------------------------
            log.info("[LAND] Landing …")
            fc.land()
            log.info("[LAND] Complete")
            state = MissionState.COMPLETE

        # ================================================================
        # 5. ANNOTATE + DISPLAY  (every frame, all detections visible)
        # ================================================================
        if not headless:
            frame_out = annotate_frame(bgr, percept, state, target_id)
            cv2.imshow("AeroTHON 2026 — Mission View", frame_out)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                log.info("Operator abort (q) → RTL")
                fc.return_to_launch()
                break
            if key == ord('e'):
                log.info("Emergency RTL (e) → RTL")
                fc.return_to_launch()
                break

        time.sleep(0.03)   # ~30 fps

    log.info("=== MISSION COMPLETE ===")
    if not headless:
        cv2.destroyAllWindows()


# ===========================================================================
# ENTRY POINT
# ===========================================================================
def main() -> None:
    parser = argparse.ArgumentParser(
        description="AeroTHON 2026 — Full Autonomous Drone Mission")
    parser.add_argument(
        "--connect",
        default="",
        help="MAVLink connection string: /dev/ttyUSB0  or  tcp:127.0.0.1:5760  "
             "(leave empty for display-only / SITL simulation mode)",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Disable OpenCV display window (useful on Jetson without desktop).",
    )
    args = parser.parse_args()

    camera = OakDCamera()
    fc     = FlightController(args.connect)

    try:
        run_mission(fc, camera, headless=args.headless)
    except KeyboardInterrupt:
        log.info("Interrupted — RTL")
        fc.return_to_launch()
    finally:
        camera.release()
        fc.close()
        if not args.headless:
            cv2.destroyAllWindows()
        log.info("Shutdown complete.")


if __name__ == "__main__":
    main()
