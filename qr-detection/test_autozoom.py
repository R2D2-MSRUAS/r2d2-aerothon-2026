"""Auto-zoom demo: detect a far QR at 1080p, then 'zoom' by cropping the
same region from the full-resolution sensor frame and decoding the crop.

Simulates what the OAK-D Lite can do in DepthAI: stream 1080p for speed,
and on detection request a full-res ISP crop (digital zoom) of the QR
region — no optical zoom needed.

Pipeline per frame:
  1. sensor frame at 4K  (stand-in for the 13 MP sensor)
  2. downscale to 1080p  -> OpenCV QRCodeDetector.detect() (locate only)
  3. map the detected quad back to 4K coordinates
  4. crop with margin from the 4K frame, upscale 2x  ("zoom")
  5. decode the crop with ZBar (WeChat as fallback)

Baseline for comparison: decode straight off the 1080p frame (flight mode).
"""
import time

import cv2
import numpy as np
import synth

# run the synthetic camera at 4K = "full sensor"
synth.FRAME_W, synth.FRAME_H = 3840, 2160
synth.FOCAL_PX = synth.FRAME_W / (2 * np.tan(np.radians(69 / 2)))
from pyzbar import pyzbar
from synth import generate_frame, qr_pixel_size

TARGET = "AT2026RC35_M2_TARGET_07"
DISTANCES = [5, 8, 10, 12, 15]
N = 12

cv_det = cv2.QRCodeDetector()
wc = cv2.wechat_qrcode_WeChatQRCode(
    "wechat_models/detect.prototxt", "wechat_models/detect.caffemodel",
    "wechat_models/sr.prototxt", "wechat_models/sr.caffemodel")


def zbar_texts(img):
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
    return [r.data.decode("utf-8", "ignore") for r in pyzbar.decode(g)]


def baseline_1080(frame4k):
    """Flight mode as-is: decode on the downscaled 1080p frame."""
    small = cv2.resize(frame4k, (1920, 1080), interpolation=cv2.INTER_AREA)
    texts, _ = wc.detectAndDecode(small)
    return TARGET in texts or TARGET in zbar_texts(small)


def autozoom(frame4k, detect_scale=0.5):
    """Detect at reduced scale, zoom-crop from 4K, decode the crop.
    detect_scale=0.5 -> detect at 1080p; 1.0 -> detect at full 4K."""
    if detect_scale != 1.0:
        small = cv2.resize(frame4k, None, fx=detect_scale, fy=detect_scale,
                           interpolation=cv2.INTER_AREA)
    else:
        small = frame4k
    found, points = cv_det.detect(small)          # locate only, no decode
    if not found or points is None:
        return False, "no-detect"
    for quad in points.reshape(-1, 4, 2):
        q4k = quad / detect_scale                 # -> 4K coords
        x0, y0 = q4k.min(axis=0)
        x1, y1 = q4k.max(axis=0)
        m = 0.35 * max(x1 - x0, y1 - y0) + 12     # margin incl. quiet zone
        x0, y0 = max(0, int(x0 - m)), max(0, int(y0 - m))
        x1 = min(frame4k.shape[1], int(x1 + m))
        y1 = min(frame4k.shape[0], int(y1 + m))
        crop = frame4k[y0:y1, x0:x1]
        if crop.size == 0:
            continue
        crop = cv2.resize(crop, None, fx=2, fy=2,   # gentle upscale for ZBar
                          interpolation=cv2.INTER_CUBIC)
        if TARGET in zbar_texts(crop):
            return True, "zoom+zbar"
        texts, _ = wc.detectAndDecode(crop)
        if TARGET in texts:
            return True, "zoom+wechat"
    return False, "detect-no-decode"


print(f"{'dist':>5} {'qr@1080p':>9} | {'base1080':>9} | "
      f"{'zoom(det@1080)':>14} {'nodet':>6} | {'zoom(det@4K)':>13} {'nodet':>6} {'ms':>8}")
seed = 50_000
for d in DISTANCES:
    base_ok = z1_ok = z1_nd = z4_ok = z4_nd = 0
    ms4 = []
    for i in range(N):
        f, _ = generate_frame(TARGET, d, ["normal", "shade", "sun"][i % 3],
                              [1, 1, 3][i % 3], seed)
        seed += 1
        if baseline_1080(f):
            base_ok += 1
        ok, why = autozoom(f, 0.5)
        z1_ok += ok
        z1_nd += (why == "no-detect")
        t0 = time.perf_counter()
        ok, why = autozoom(f, 1.0)
        ms4.append((time.perf_counter() - t0) * 1000)
        z4_ok += ok
        z4_nd += (why == "no-detect")
    px1080 = qr_pixel_size(d) // 2
    print(f"{d:>4}m {px1080:>8}px | {base_ok:>6}/{N} | "
          f"{z1_ok:>11}/{N} {z1_nd:>4}/{N} | {z4_ok:>10}/{N} {z4_nd:>4}/{N} "
          f"{np.mean(ms4):>8.1f}")
