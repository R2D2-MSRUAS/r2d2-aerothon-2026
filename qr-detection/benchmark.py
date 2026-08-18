"""Benchmark QR detect+decode methods on synthetic aerial mission frames.

Methods:
  opencv  - cv2.QRCodeDetector.detectAndDecodeMulti (classical, built-in)
  wechat  - cv2.wechat_qrcode (CNN detector + super-resolution + decode)
  zbar    - pyzbar on the full frame (what the report runs on TX2 NX)
  hybrid  - WeChat detect -> crop ROI + upscale -> ZBar decode
            (mirrors the mission architecture: VPU crop -> ZBar on Jetson)

Usage: python benchmark.py            (needs DYLD_LIBRARY_PATH=/opt/homebrew/lib for zbar)
"""
import csv
import os
import time

import cv2
import numpy as np
from synth import generate_frame, qr_pixel_size

try:
    from pyzbar import pyzbar
    HAVE_ZBAR = True
except ImportError:
    HAVE_ZBAR = False

TARGET = "AT2026RC35_M2_TARGET_07"
DISTANCES = [1, 3, 5, 8, 10, 12, 15]          # metres (slant range)
LIGHTING = ["normal", "shade", "sun"]
N_PER_CELL = 12                                # frames per condition cell
BLUR_CHOICES = [1, 1, 3, 5]                    # mostly sharp, some motion blur

cv_det = cv2.QRCodeDetector()
wc_det = cv2.wechat_qrcode_WeChatQRCode(
    "wechat_models/detect.prototxt", "wechat_models/detect.caffemodel",
    "wechat_models/sr.prototxt", "wechat_models/sr.caffemodel")


def run_opencv(frame):
    ok, texts, points, _ = cv_det.detectAndDecodeMulti(frame)
    if ok and texts:
        return [t for t in texts if t], points
    return [], None


def run_wechat(frame):
    texts, points = wc_det.detectAndDecode(frame)
    return [t for t in texts if t], points


def run_zbar(frame):
    if not HAVE_ZBAR:
        return [], None
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    res = pyzbar.decode(gray)
    return [r.data.decode("utf-8", "ignore") for r in res], None


def run_hybrid(frame):
    """WeChat locates the code; ZBar decodes an upscaled crop.
    Mirrors mission split: Myriad X crops ROI -> ZBar on TX2 NX decodes."""
    if not HAVE_ZBAR:
        return [], None
    texts, points = wc_det.detectAndDecode(frame)
    out = [t for t in texts if t]          # WeChat may already decode
    if points is not None:
        for i, quad in enumerate(points):
            if i < len(texts) and texts[i]:
                continue                        # already decoded
            q = np.array(quad).reshape(-1, 2)
            x0, y0 = q.min(axis=0).astype(int) - 8
            x1, y1 = q.max(axis=0).astype(int) + 8
            x0, y0 = max(0, x0), max(0, y0)
            crop = frame[y0:y1, x0:x1]
            if crop.size == 0:
                continue
            scale = max(1, int(160 / max(1, crop.shape[0])))
            if scale > 1:
                crop = cv2.resize(crop, None, fx=scale, fy=scale,
                                  interpolation=cv2.INTER_CUBIC)
            gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
            for r in pyzbar.decode(gray):
                out.append(r.data.decode("utf-8", "ignore"))
    return out, points


METHODS = {"opencv": run_opencv, "wechat": run_wechat,
           "zbar": run_zbar, "hybrid": run_hybrid}


def main():
    os.makedirs("samples", exist_ok=True)
    rows = []
    seed = 0
    for dist in DISTANCES:
        for light in LIGHTING:
            frames = []
            for i in range(N_PER_CELL):
                blur = BLUR_CHOICES[i % len(BLUR_CHOICES)]
                f, _ = generate_frame(TARGET, dist, light, blur, seed)
                frames.append(f)
                seed += 1
            if light == "normal":  # save one sample per distance
                cv2.imwrite(f"samples/dist{dist}m.jpg", frames[0],
                            [cv2.IMWRITE_JPEG_QUALITY, 85])
            for name, fn in METHODS.items():
                dec, lat = 0, []
                for f in frames:
                    t0 = time.perf_counter()
                    texts, _ = fn(f)
                    lat.append((time.perf_counter() - t0) * 1000)
                    if TARGET in texts:
                        dec += 1
                rows.append({"method": name, "distance_m": dist,
                             "qr_px": qr_pixel_size(dist), "lighting": light,
                             "n": N_PER_CELL, "decoded": dec,
                             "decode_rate": round(dec / N_PER_CELL, 3),
                             "avg_ms": round(float(np.mean(lat)), 1)})
                print(f"d={dist:>2}m ({qr_pixel_size(dist):>3}px) {light:<6} "
                      f"{name:<7} decode {dec:>2}/{N_PER_CELL}  "
                      f"{np.mean(lat):6.1f} ms")

    with open("results.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=rows[0].keys())
        w.writeheader()
        w.writerows(rows)

    # summary: decode rate by method x distance (all lighting pooled)
    print("\n=== Decode rate by method x distance (pooled lighting) ===")
    print(f"{'method':<8}" + "".join(f"{d:>7}m" for d in DISTANCES) + f"{'avg ms':>9}")
    for name in METHODS:
        r = [x for x in rows if x["method"] == name]
        line = f"{name:<8}"
        for d in DISTANCES:
            cell = [x for x in r if x["distance_m"] == d]
            rate = sum(x["decoded"] for x in cell) / sum(x["n"] for x in cell)
            line += f"{rate:>8.0%}"
        line += f"{np.mean([x['avg_ms'] for x in r]):>9.1f}"
        print(line)


if __name__ == "__main__":
    main()
