"""
AeroTHON 2026 — Vision, Stereo Depth & Perception Pipeline Package
================================================================================
Author: Bhakti Revankar
"""

from __future__ import annotations

import abc
import logging
import math
from typing import Any

import cv2
import numpy as np

from aerothon.config import DEFAULT_CONFIG, MissionConfig, MissionState
from aerothon.coordinates import GlobalCoordinate, LocalSectionCoordinate

log = logging.getLogger("AeroTHON2026.Vision")

DEPTH_UNKNOWN_M = float("inf")


# ===========================================================================
# CAMERA INTERFACE & IMPLEMENTATIONS (SAFE / LAZY)
# ===========================================================================
class CameraInterface(abc.ABC):
    """Abstract interface for RGB + Depth video capture."""

    @abc.abstractmethod
    def read(self) -> tuple[np.ndarray | None, np.ndarray | None]:
        """Returns (bgr_frame [H,W,3 uint8], depth_mm [H,W uint16]) or (None, None)."""

    @abc.abstractmethod
    def release(self) -> None:
        pass


class MockCamera(CameraInterface):
    """Safe drone-free mock camera for testing and simulation."""

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

        # Generate default synthetic room/corridor frame
        bgr = np.full((self.height, self.width, 3), 150, dtype=np.uint8)
        depth = np.full((self.height, self.width), 3500, dtype=np.uint16)
        # Add synthetic walls
        depth[:, :80] = 1750
        depth[:, -80:] = 1750
        return bgr, depth

    def release(self) -> None:
        pass


class WebcamCamera(CameraInterface):
    """Standard OpenCV webcam capture with synthetic flat depth."""

    def __init__(
        self, camera_index: int = 0, width: int = 640, height: int = 480
    ) -> None:
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
    """OAK-D Lite DepthAI camera pipeline (Lazy initialization)."""

    def __init__(
        self,
        rgb_fps: int = 30,
        depth_fps: int = 30,
        width: int = 640,
        height: int = 480,
    ) -> None:
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
            mono_right.setResolution(
                dai.MonoCameraProperties.SensorResolution.THE_400_P
            )
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
            self._q_depth = self._device.getOutputQueue(
                "depth", maxSize=4, blocking=False
            )
            log.info("OAK-D Hardware stereo pipeline connected successfully.")
        except Exception as err:  # noqa: BLE001
            log.warning(
                f"OAK-D initialization failed ({err}). Falling back to WebcamCamera."
            )
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


class ThreeCameraManager(CameraInterface):
    """
    3-Camera Vision Suite Manager:
      1. Forward Camera (OAK-D Lite): Wall centering, Green Banner, Forward stereo depth
      2. Downward Camera (Arducam B0251 12.3MP HQ): Ground QR target scan & drop visual servoing
      3. Secondary Camera (Skydroid FPV Mini): FPV pilot monitoring & Return pass safety
    """

    def __init__(
        self,
        forward_cam: CameraInterface | None = None,
        downward_cam: CameraInterface | None = None,
        fpv_cam: CameraInterface | None = None,
    ) -> None:
        self.forward_cam = forward_cam or OakDCamera()
        self.downward_cam = downward_cam or WebcamCamera(camera_index=1)
        self.fpv_cam = fpv_cam or WebcamCamera(camera_index=2)

    def read(self) -> tuple[np.ndarray | None, np.ndarray | None]:
        return self.forward_cam.read()

    def read_downward(self) -> tuple[np.ndarray | None, np.ndarray | None]:
        return self.downward_cam.read()

    def read_fpv(self) -> tuple[np.ndarray | None, np.ndarray | None]:
        return self.fpv_cam.read()

    def release(self) -> None:
        self.forward_cam.release()
        self.downward_cam.release()
        self.fpv_cam.release()


# ===========================================================================
# DEPTH SAMPLING & ACTION LOGIC
# ===========================================================================
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


def depth_action(distance_m: float, config: MissionConfig = DEFAULT_CONFIG) -> str:
    if math.isinf(distance_m):
        return "SLOW"
    if distance_m > config.OBS_SAFE_DIST_M:
        return "CONTINUE"
    elif distance_m > config.OBS_CRITICAL_STOP_M:
        return "SLOW"
    else:
        return "STOP"


# ===========================================================================
# HUMAN SHAPES CYCLING DRAWING UTILITIES
# ===========================================================================
PEOPLE_SHAPES = ["circle", "ellipse", "hexagon", "rounded_box"]


def draw_circle(
    img: np.ndarray, x1: int, y1: int, x2: int, y2: int, color: tuple
) -> None:
    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
    radius = max((x2 - x1), (y2 - y1)) // 2
    cv2.circle(img, (cx, cy), radius, color, 3)


def draw_ellipse(
    img: np.ndarray, x1: int, y1: int, x2: int, y2: int, color: tuple
) -> None:
    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
    ax, ay = max(1, (x2 - x1) // 2), max(1, (y2 - y1) // 2)
    cv2.ellipse(img, (cx, cy), (ax, ay), 0, 0, 360, color, 3)


def draw_hexagon(
    img: np.ndarray, x1: int, y1: int, x2: int, y2: int, color: tuple
) -> None:
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


def draw_rounded_box(
    img: np.ndarray, x1: int, y1: int, x2: int, y2: int, color: tuple, r: int = 15
) -> None:
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
# PURE PERCEPTION FUNCTIONS
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
) -> tuple[int, int, int, int] | None:
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


def detect_qr_codes(bgr: np.ndarray) -> list[tuple[str, np.ndarray]]:
    results: list[tuple[str, np.ndarray]] = []
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    upscaled = cv2.resize(gray, None, fx=2.5, fy=2.5, interpolation=cv2.INTER_CUBIC)
    thresh = cv2.adaptiveThreshold(
        gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 31, 4
    )

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
                quad = np.array(
                    [[int(p.x / sc), int(p.y / sc)] for p in obj.polygon], dtype=int
                )
                results.append((text, quad))
            if results:
                return results
    except Exception:  # noqa: BLE001, S110
        pass

    qr_detector = cv2.QRCodeDetector()
    for img_data, sc in stages:
        var = (
            cv2.cvtColor(img_data, cv2.COLOR_GRAY2BGR)
            if img_data.ndim == 2
            else img_data
        )
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


def find_red_zones(bgr: np.ndarray) -> list[tuple[int, int, int, int]]:
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lo, hi in RED_HUE_RANGES:
        mask |= (
            ((h >= lo) & (h <= hi) & (s >= RED_SAT_MIN) & (v >= RED_VAL_MIN)).astype(
                np.uint8
            )
        ) * 255
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
    config: MissionConfig = DEFAULT_CONFIG,
) -> list[dict]:
    frame = preprocess_clahe(bgr)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, w = frame.shape[:2]
    frame_area = h * w

    _, _, val = cv2.split(hsv)
    dark_mask = ((val <= 90).astype(np.uint8)) * 255
    contours, _ = cv2.findContours(
        dark_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

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
        obstacles.append(
            {
                "cx": cx,
                "cy": cy,
                "radius": radius,
                "bbox": bbox,
                "dist_m": dist,
                "action": depth_action(dist, config),
                "source": "classical_cv",
                "class_name": "obstacle",
            }
        )

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
                obstacles.append(
                    {
                        "cx": x1 + bw // 2,
                        "cy": y1 + bh // 2,
                        "radius": max(bw, bh) // 2,
                        "bbox": bbox,
                        "dist_m": dist,
                        "action": depth_action(dist, config),
                        "source": "yolo",
                        "class_name": name,
                        "conf": conf,
                    }
                )

    return obstacles


# ===========================================================================
# UNIFIED PERCEPTION PIPELINE CLASS
# ===========================================================================
class PerceptionPipeline:
    def __init__(
        self, config: MissionConfig = DEFAULT_CONFIG, enable_yolo: bool = True
    ) -> None:
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
            log.info("YOLOv8 model loaded on first perception call.")
        except Exception as err:  # noqa: BLE001
            log.warning(
                f"Could not load YOLOv8 model ({err}). Using classical CV only."
            )
            self._yolo_model = None

    def perceive(self, bgr: np.ndarray, depth_mm: np.ndarray | None) -> dict[str, Any]:
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
                            humans.append(
                                {
                                    "bbox": bbox,
                                    "dist_m": dist,
                                    "action": depth_action(dist, self.config),
                                    "conf": conf,
                                    "cx": x1 + bw // 2,
                                    "cy": y1 + bh // 2,
                                }
                            )
            except Exception as err:  # noqa: BLE001
                log.debug(f"YOLO inference error: {err}")

        obstacles = detect_obstacles_with_depth(
            bgr, depth_mm, self._yolo_model, _yolo_res, self.config
        )
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
            o["action"] == "STOP" and abs(o["cx"] - w_f // 2) < col_w // 2
            for o in obstacles
        )
        human_stop = any(h["action"] == "STOP" for h in humans)
        red_ahead = any(
            col_w <= (rx + rw // 2) <= col_w * 2 for rx, ry, rw, rh in red_zones
        )

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
# HUD ANNOTATION
# ===========================================================================
def annotate_mission_frame(
    bgr: np.ndarray,
    percept: dict,
    state: MissionState,
    target_id: str,
    gpos: GlobalCoordinate,
    lpos: LocalSectionCoordinate,
    config: MissionConfig = DEFAULT_CONFIG,
) -> np.ndarray:
    out = bgr.copy()
    h, w = out.shape[:2]

    # Red Zones
    for rx, ry, rw, rh in percept.get("red_zones", []):
        cv2.rectangle(out, (rx, ry), (rx + rw, ry + rh), (0, 0, 240), 3)
        cv2.putText(
            out,
            "RED ZONE",
            (rx, max(15, ry - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 240),
            2,
        )

    # Obstacles
    for obs in percept.get("obstacles", []):
        if obs.get("source") == "yolo":
            continue
        cx, cy, rad = obs["cx"], obs["cy"], obs["radius"]
        dist = obs["dist_m"]
        act = obs["action"]
        col = (
            (0, 220, 0)
            if act == "CONTINUE"
            else ((0, 165, 255) if act == "SLOW" else (0, 0, 255))
        )
        cv2.circle(out, (cx, cy), rad, col, 2)
        cv2.putText(
            out,
            f"OBS {dist:.1f}m [{act}]",
            (cx - rad, max(15, cy - rad - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            col,
            2,
        )

    # YOLO Objects
    for name, conf, (x1, y1, bw, bh), dist in percept.get("yolo_all", []):
        if name == "person":
            continue
        col = (255, 200, 0)
        cv2.rectangle(out, (x1, y1), (x1 + bw, y1 + bh), col, 2)
        cv2.putText(
            out,
            f"{name} {conf:.2f} {dist:.1f}m",
            (x1, max(15, y1 - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            col,
            2,
        )

    # Humans with cycling shapes
    for idx, human in enumerate(percept.get("humans", [])):
        x1, y1, bw, bh = human["bbox"]
        x2, y2 = x1 + bw, y1 + bh
        dist = human["dist_m"]
        act = human["action"]
        shape_type = PEOPLE_SHAPES[idx % len(PEOPLE_SHAPES)]
        col = (255, 0, 220)
        SHAPE_FUNCS[shape_type](out, x1, y1, x2, y2, col)
        cv2.putText(
            out,
            f"HUMAN [{shape_type}] {dist:.1f}m [{act}]",
            (x1, max(15, y1 - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            col,
            2,
        )

    # Green Banner
    if percept.get("banner_box") is not None:
        bx, by, bw, bh = percept["banner_box"]
        cv2.rectangle(out, (bx, by), (bx + bw, by + bh), (0, 255, 100), 3)
        cv2.putText(
            out,
            "GREEN BANNER",
            (bx, max(15, by - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 100),
            2,
        )

    # QR Codes
    for text, quad in percept.get("qr_codes", []):
        cv2.polylines(out, [quad.reshape(-1, 1, 2)], True, (255, 255, 0), 3)
        qx, qy = int(quad[0][0]), int(quad[0][1])
        match_tag = " [TARGET MATCH!]" if (text == target_id and target_id) else ""
        cv2.putText(
            out,
            f"QR: {text[:20]}{match_tag}",
            (qx, max(15, qy - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 0),
            2,
        )

    # Bottom Depth Bars
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
        cv2.putText(
            out,
            f"{tag}: {d:.1f}m",
            (x0 + 6, bar_y - 1),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.38,
            (255, 255, 255),
            1,
        )

    # Steering
    st = percept.get("steering", "FORWARD")
    st_col = (
        (0, 255, 0)
        if st == "FORWARD"
        else ((0, 200, 255) if st in ("LEFT", "RIGHT") else (0, 0, 255))
    )
    cv2.putText(
        out,
        f"STEER: {st}",
        (w // 2 - 50, h - 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        st_col,
        2,
    )

    # Top Telemetry Dashboard
    cv2.rectangle(out, (0, 0), (w, 58), (20, 20, 20), -1)
    cv2.putText(
        out,
        f"STATE: {state.name} | PILOT: Bhakti Revankar",
        (10, 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (0, 240, 240),
        2,
    )
    cv2.putText(
        out,
        f"TGT QR: {target_id if target_id else 'SEARCHING'}",
        (w - 240, 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 220, 0),
        2,
    )

    global_str = f"GLOBAL NED: X:{gpos.x:+.1f}m | Y:{gpos.y:+.1f}m | Alt:{gpos.altitude:.1f}m | Yaw:{gpos.yaw_deg:.0f}°"
    local_str = f"LOCAL [{lpos.section_name}]: X_rel:{lpos.x_rel:+.2f}m | Y_lat:{lpos.y_rel:+.2f}m | Rem:{lpos.distance_to_goal:.1f}m"
    cv2.putText(
        out, global_str, (10, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180, 255, 180), 1
    )
    cv2.putText(
        out, local_str, (10, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 200, 150), 1
    )

    return out
