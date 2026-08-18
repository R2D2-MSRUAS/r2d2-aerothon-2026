"""Camera source abstractions for the R2D2 vision middleware.

Every source exposes the same tiny interface so the CameraManager can swap
them freely:

    open() -> bool          bring the device up
    read() -> frame | None  latest BGR frame (non-blocking-ish)
    close()                 release the device
    .name / .healthy / .last_frame_ts

Sources:
    OakDLiteSource    — OAK-D Lite RGB via DepthAI (USB3). Primary.
    ArducamCSISource  — Arducam B0251 (IMX477) via nvarguscamerasrc GStreamer
                        on Jetson. Backup, bottom-facing.
    OpenCVSource      — plain cv2.VideoCapture (webcam index / video file /
                        GStreamer string). Bench-testing stand-in for either.

DepthAI is imported lazily so this module works on a laptop without an OAK
attached. All sources are single-consumer; the manager owns them.
"""
import logging
import time

import cv2

from .config import CameraConfig

log = logging.getLogger("middleware.cameras")


class CameraSource:
    """Base class: bookkeeping shared by all sources."""

    name = "base"

    def __init__(self, cfg: CameraConfig):
        self.cfg = cfg
        self.last_frame_ts = 0.0
        self.consecutive_errors = 0
        self._open = False

    # -- interface -----------------------------------------------------
    def open(self) -> bool:
        raise NotImplementedError

    def _grab(self):
        """Return a BGR frame or None. Subclasses implement."""
        raise NotImplementedError

    def close(self):
        self._open = False

    # -- shared logic ---------------------------------------------------
    def read(self):
        if not self._open:
            return None
        try:
            frame = self._grab()
        except Exception as e:
            log.warning("%s: read error: %s", self.name, e)
            frame = None
        if frame is None:
            self.consecutive_errors += 1
            return None
        self.consecutive_errors = 0
        self.last_frame_ts = time.monotonic()
        return frame

    @property
    def is_open(self) -> bool:
        return self._open

    @property
    def healthy(self) -> bool:
        """Open, producing frames recently, and not erroring repeatedly."""
        if not self._open:
            return False
        if self.consecutive_errors >= self.cfg.max_consecutive_errors:
            return False
        if self.last_frame_ts == 0.0:
            return True  # just opened, give it a chance
        return (time.monotonic() - self.last_frame_ts) < self.cfg.frame_timeout_s


class OakDLiteSource(CameraSource):
    """OAK-D Lite RGB stream via DepthAI. Detection/inference runs on the
    Myriad X inside the camera; this source only pulls the RGB preview the
    QR/zone pipeline consumes on the Jetson."""

    name = "oak-d-lite"

    def __init__(self, cfg: CameraConfig):
        super().__init__(cfg)
        self._device = None
        self._queue = None

    def open(self) -> bool:
        try:
            import depthai as dai
        except ImportError:
            log.error("%s: depthai not installed", self.name)
            return False
        try:
            pipeline = dai.Pipeline()
            cam = pipeline.create(dai.node.ColorCamera)
            cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
            cam.setResolution(
                dai.ColorCameraProperties.SensorResolution.THE_1080_P)
            cam.setVideoSize(self.cfg.oak_rgb_width, self.cfg.oak_rgb_height)
            cam.setFps(self.cfg.oak_fps)
            xout = pipeline.create(dai.node.XLinkOut)
            xout.setStreamName("rgb")
            cam.video.link(xout.input)

            self._device = dai.Device(pipeline)
            self._queue = self._device.getOutputQueue(
                "rgb", maxSize=2, blocking=False)
            self._open = True
            self.consecutive_errors = 0
            log.info("%s: opened (%dx%d@%d)", self.name,
                     self.cfg.oak_rgb_width, self.cfg.oak_rgb_height,
                     self.cfg.oak_fps)
            return True
        except Exception as e:
            log.error("%s: open failed: %s", self.name, e)
            self.close()
            return False

    def _grab(self):
        msg = self._queue.tryGet()
        return msg.getCvFrame() if msg is not None else None

    def close(self):
        super().close()
        if self._device is not None:
            try:
                self._device.close()
            except Exception:
                pass
        self._device = None
        self._queue = None


class ArducamCSISource(CameraSource):
    """Arducam B0251 (IMX477) on the Jetson CSI port through nvarguscamerasrc.
    Bottom-facing backup camera — used when the OAK-D drops off USB."""

    name = "arducam-csi"

    def __init__(self, cfg: CameraConfig):
        super().__init__(cfg)
        self._cap = None

    def _gst(self) -> str:
        c = self.cfg
        return (
            f"nvarguscamerasrc sensor-id={c.csi_sensor_id} ! "
            f"video/x-raw(memory:NVMM), width={c.csi_width}, "
            f"height={c.csi_height}, framerate={c.csi_fps}/1 ! "
            "nvvidconv ! video/x-raw, format=BGRx ! "
            "videoconvert ! video/x-raw, format=BGR ! "
            "appsink drop=true max-buffers=2"
        )

    def open(self) -> bool:
        try:
            self._cap = cv2.VideoCapture(self._gst(), cv2.CAP_GSTREAMER)
            if not self._cap.isOpened():
                log.error("%s: GStreamer pipeline failed to open", self.name)
                self.close()
                return False
            self._open = True
            self.consecutive_errors = 0
            log.info("%s: opened (%dx%d@%d)", self.name, self.cfg.csi_width,
                     self.cfg.csi_height, self.cfg.csi_fps)
            return True
        except Exception as e:
            log.error("%s: open failed: %s", self.name, e)
            self.close()
            return False

    def _grab(self):
        ok, frame = self._cap.read()
        return frame if ok else None

    def close(self):
        super().close()
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                pass
        self._cap = None


class OpenCVSource(CameraSource):
    """Generic cv2.VideoCapture source for bench testing on a laptop:
    webcam index, video file, or explicit GStreamer string."""

    def __init__(self, cfg: CameraConfig, source, name="opencv", loop=False):
        super().__init__(cfg)
        self.name = name
        self._source = source
        self._loop = loop  # loop video files forever (bench soak tests)
        self._cap = None

    def open(self) -> bool:
        try:
            self._cap = cv2.VideoCapture(self._source)
            if not self._cap.isOpened():
                self.close()
                return False
            self._open = True
            self.consecutive_errors = 0
            log.info("%s: opened (%s)", self.name, self._source)
            return True
        except Exception as e:
            log.error("%s: open failed: %s", self.name, e)
            self.close()
            return False

    def _grab(self):
        ok, frame = self._cap.read()
        if not ok and self._loop:
            self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, frame = self._cap.read()
        return frame if ok else None

    def close(self):
        super().close()
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                pass
        self._cap = None
