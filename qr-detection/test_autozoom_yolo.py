"""Auto-zoom with a NEURAL detector: QReader's pretrained YOLOv8 QR
detector locates the code at 1080p (even when far too small to decode),
then we 'zoom' by cropping that region from the 4K sensor frame and
decoding the crop with ZBar.

This is the full loop the mission needs:
  see it far away -> zoom in -> read it clearly.
"""
import time
import numpy as np
import cv2
import synth

synth.FRAME_W, synth.FRAME_H = 3840, 2160
synth.FOCAL_PX = synth.FRAME_W / (2 * np.tan(np.radians(69 / 2)))
from synth import generate_frame, qr_pixel_size

from pyzbar import pyzbar
from qreader import QReader

TARGET = "AT2026RC35_M2_TARGET_07"
DISTANCES = [5, 8, 10, 12, 15, 18]
N = 12

detector = QReader(model_size="s")   # YOLOv8-small QR detector
wc = cv2.wechat_qrcode_WeChatQRCode(
    "wechat_models/detect.prototxt", "wechat_models/detect.caffemodel",
    "wechat_models/sr.prototxt", "wechat_models/sr.caffemodel")


def zbar_texts(img):
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
    return [r.data.decode("utf-8", "ignore") for r in pyzbar.decode(g)]


def baseline_1080(frame4k):
    small = cv2.resize(frame4k, (1920, 1080), interpolation=cv2.INTER_AREA)
    texts, _ = wc.detectAndDecode(small)
    return TARGET in texts or TARGET in zbar_texts(small)


def autozoom_yolo(frame4k):
    """YOLO locates at 1080p -> crop 4K ROI -> upscale -> decode."""
    small = cv2.resize(frame4k, (1920, 1080), interpolation=cv2.INTER_AREA)
    rgb = cv2.cvtColor(small, cv2.COLOR_BGR2RGB)
    dets = detector.detect(image=rgb, is_bgr=False)
    if not dets:
        return False, "no-detect"
    for det in dets:
        x0, y0, x1, y1 = np.array(det["bbox_xyxy"]) * 2.0   # -> 4K coords
        m = 0.35 * max(x1 - x0, y1 - y0) + 12
        xa, ya = max(0, int(x0 - m)), max(0, int(y0 - m))
        xb = min(frame4k.shape[1], int(x1 + m))
        yb = min(frame4k.shape[0], int(y1 + m))
        crop = frame4k[ya:yb, xa:xb]
        if crop.size == 0:
            continue
        scale = max(1, int(np.ceil(300 / max(crop.shape[:2]))))
        if scale > 1:
            crop = cv2.resize(crop, None, fx=scale, fy=scale,
                              interpolation=cv2.INTER_CUBIC)
        if TARGET in zbar_texts(crop):
            return True, "zoom+zbar"
        texts, _ = wc.detectAndDecode(crop)
        if TARGET in texts:
            return True, "zoom+wechat"
    return False, "detect-no-decode"


print(f"{'dist':>5} {'qr@1080p':>9} | {'baseline 1080p':>14} | "
      f"{'YOLO detect':>11} | {'zoom decode':>11} | {'ms':>7}")
seed = 90_000
for d in DISTANCES:
    base_ok = det_ok = zoom_ok = 0
    ms = []
    for i in range(N):
        f, _ = generate_frame(TARGET, d, ["normal", "shade", "sun"][i % 3],
                              [1, 1, 3][i % 3], seed)
        seed += 1
        if baseline_1080(f):
            base_ok += 1
        t0 = time.perf_counter()
        ok, why = autozoom_yolo(f)
        ms.append((time.perf_counter() - t0) * 1000)
        det_ok += (why != "no-detect")
        zoom_ok += ok
    px = qr_pixel_size(d) // 2
    print(f"{d:>4}m {px:>8}px | {base_ok:>10}/{N}   | {det_ok:>8}/{N}  | "
          f"{zoom_ok:>8}/{N}  | {np.mean(ms):>7.0f}")
