"""Tiled auto-zoom: run the YOLO QR detector on 2x2 tiles of the 4K frame
(so the detector sees native-resolution pixels instead of a shrunken
overview), then zoom-crop + ZBar decode any detection.

This is the hover-scan configuration: latency ~4x single detect, fine for
a stationary scan, and it extends detection range well past decode range.
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
DISTANCES = [8, 10, 12, 15, 18]
N = 12
OVERLAP = 160   # px overlap so a QR on a tile seam is fully inside one tile

detector = QReader(model_size="s")
wc = cv2.wechat_qrcode_WeChatQRCode(
    "wechat_models/detect.prototxt", "wechat_models/detect.caffemodel",
    "wechat_models/sr.prototxt", "wechat_models/sr.caffemodel")


def zbar_texts(img):
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
    return [r.data.decode("utf-8", "ignore") for r in pyzbar.decode(g)]


def tiles_4k(frame):
    H, W = frame.shape[:2]
    tw, th = W // 2 + OVERLAP, H // 2 + OVERLAP
    for ty in (0, H // 2 - OVERLAP):
        for tx in (0, W // 2 - OVERLAP):
            yield tx, ty, frame[ty:ty + th, tx:tx + tw]


def autozoom_tiled(frame4k):
    boxes = []
    for tx, ty, tile in tiles_4k(frame4k):
        rgb = cv2.cvtColor(tile, cv2.COLOR_BGR2RGB)
        for det in detector.detect(image=rgb, is_bgr=False):
            x0, y0, x1, y1 = det["bbox_xyxy"]
            boxes.append((x0 + tx, y0 + ty, x1 + tx, y1 + ty))
    if not boxes:
        return False, "no-detect"
    for x0, y0, x1, y1 in boxes:
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


print(f"{'dist':>5} {'qr@4K':>7} | {'tiled detect':>12} | {'zoom decode':>11} | {'ms':>7}")
seed = 130_000
for d in DISTANCES:
    det_ok = zoom_ok = 0
    ms = []
    for i in range(N):
        f, _ = generate_frame(TARGET, d, ["normal", "shade", "sun"][i % 3],
                              [1, 1, 3][i % 3], seed)
        seed += 1
        t0 = time.perf_counter()
        ok, why = autozoom_tiled(f)
        ms.append((time.perf_counter() - t0) * 1000)
        det_ok += (why != "no-detect")
        zoom_ok += ok
    print(f"{d:>4}m {qr_pixel_size(d):>6}px | {det_ok:>9}/{N}  | "
          f"{zoom_ok:>8}/{N}  | {np.mean(ms):>7.0f}")
