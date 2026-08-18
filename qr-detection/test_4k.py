"""Same benchmark at 4K capture resolution (the mission's stationary-scan mode).

Confirms whether stepping up to 4K recovers decode range at the 10 m
delivery-zone scan altitude.
"""
import time

import cv2
import numpy as np
import synth
from synth import generate_frame

# switch the synthetic camera to 4K
synth.FRAME_W, synth.FRAME_H = 3840, 2160
synth.FOCAL_PX = synth.FRAME_W / (2 * np.tan(np.radians(69 / 2)))

from pyzbar import pyzbar

TARGET = "AT2026RC35_M2_TARGET_07"
DISTANCES = [5, 8, 10, 12, 15, 18]
N = 12

wc = cv2.wechat_qrcode_WeChatQRCode(
    "wechat_models/detect.prototxt", "wechat_models/detect.caffemodel",
    "wechat_models/sr.prototxt", "wechat_models/sr.caffemodel")


def run_zbar(frame):
    res = pyzbar.decode(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
    return [r.data.decode("utf-8", "ignore") for r in res]


print(f"{'dist':>5} {'qr_px':>6} {'wechat':>8} {'zbar':>8} {'wc ms':>8} {'zb ms':>8}")
seed = 10_000
for d in DISTANCES:
    wc_ok = zb_ok = 0
    wc_ms, zb_ms = [], []
    for i in range(N):
        f, _ = generate_frame(TARGET, d, "normal", [1, 1, 3][i % 3], seed)
        seed += 1
        t0 = time.perf_counter()
        texts, _ = wc.detectAndDecode(f)
        wc_ms.append((time.perf_counter() - t0) * 1000)
        if TARGET in texts:
            wc_ok += 1
        t0 = time.perf_counter()
        if TARGET in run_zbar(f):
            zb_ok += 1
        zb_ms.append((time.perf_counter() - t0) * 1000)
    px = synth.qr_pixel_size(d)
    print(f"{d:>4}m {px:>6} {wc_ok:>5}/{N} {zb_ok:>5}/{N} "
          f"{np.mean(wc_ms):>8.1f} {np.mean(zb_ms):>8.1f}")
