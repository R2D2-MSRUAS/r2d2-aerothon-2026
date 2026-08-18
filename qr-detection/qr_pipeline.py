"""QR detect + scan pipeline for Mission 2 (SkyScan).

Two operating modes, matching the mission's adaptive-resolution design:

  flight  - fast per-frame detect+decode for moving flight (WeChat CNN
            detector + super-resolution). Works down to ~5 m slant range
            at 1080p. ~10-140 ms/frame on laptop CPU.
  scan    - maximum-range decode for the stationary hover scan (ZBar over
            the full frame at native resolution, WeChat as fallback).
            Decodes reliably to ~8-10 m slant range at 4K.

Usage:
  python qr_pipeline.py --input path/to/video.mp4 --mode flight
  python qr_pipeline.py --input path/to/image.jpg --mode scan --target AT2026RC35_M2_TARGET_07
  python qr_pipeline.py --input 0 --mode flight            # webcam
  python qr_pipeline.py --input frames_dir/ --mode scan

Requires: opencv-contrib-python==4.11.0.86, pyzbar (+ `brew install zbar`,
run with DYLD_LIBRARY_PATH=/opt/homebrew/lib on macOS).
"""
import argparse
import json
import os
import sys
import time

import cv2
import numpy as np

try:
    from pyzbar import pyzbar
    HAVE_ZBAR = True
except ImportError:
    HAVE_ZBAR = False

WECHAT_FILES = ["detect.prototxt", "detect.caffemodel",
                "sr.prototxt", "sr.caffemodel"]


def load_wechat(model_dir="wechat_models"):
    paths = [os.path.join(model_dir, f) for f in WECHAT_FILES]
    if not all(os.path.exists(p) for p in paths):
        sys.exit(f"WeChat model files missing in {model_dir}/ "
                 f"(need {', '.join(WECHAT_FILES)})")
    return cv2.wechat_qrcode_WeChatQRCode(*paths)


def decode_flight(frame, wechat):
    """Fast mode: WeChat detect+decode. Downscale 4K to 1080p first —
    WeChat's detector performs worse and slower on very large frames."""
    scale = 1.0
    if frame.shape[1] > 2000:
        scale = 1920 / frame.shape[1]
        frame = cv2.resize(frame, None, fx=scale, fy=scale,
                           interpolation=cv2.INTER_AREA)
    texts, points = wechat.detectAndDecode(frame)
    out = []
    for i, t in enumerate(texts):
        if not t:
            continue
        pts = (np.array(points[i]).reshape(-1, 2) / scale).tolist() \
            if points is not None else None
        out.append({"text": t, "corners": pts})
    return out


def decode_scan(frame, wechat):
    """Max-range mode: ZBar on the full native-resolution frame
    (best long-range performer at 4K), WeChat as fallback."""
    out = []
    if HAVE_ZBAR:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        for r in pyzbar.decode(gray):
            corners = [[p.x, p.y] for p in r.polygon] if r.polygon else None
            out.append({"text": r.data.decode("utf-8", "ignore"),
                        "corners": corners})
    if not out:                       # fallback
        out = decode_flight(frame, wechat)
    return out


def annotate(frame, results, target=None):
    for r in results:
        hit = target is not None and r["text"] == target
        col = (0, 200, 0) if hit else (0, 165, 255)
        if r["corners"]:
            pts = np.array(r["corners"], np.int32)
            cv2.polylines(frame, [pts], True, col, 3)
            org = tuple(pts.min(axis=0) - [0, 10])
        else:
            org = (30, 60)
        label = ("TARGET " if hit else "") + r["text"]
        cv2.putText(frame, label, org, cv2.FONT_HERSHEY_SIMPLEX, 0.9, col, 2)
    return frame


def iter_frames(inp):
    """Yield (name, frame) from an image, a directory, a video, or a camera."""
    if inp.isdigit():
        cap = cv2.VideoCapture(int(inp))
        i = 0
        while True:
            ok, f = cap.read()
            if not ok:
                break
            yield f"cam{i}", f
            i += 1
    elif os.path.isdir(inp):
        for n in sorted(os.listdir(inp)):
            if n.lower().endswith((".jpg", ".jpeg", ".png", ".bmp")):
                yield n, cv2.imread(os.path.join(inp, n))
    elif inp.lower().endswith((".jpg", ".jpeg", ".png", ".bmp")):
        yield os.path.basename(inp), cv2.imread(inp)
    else:
        cap = cv2.VideoCapture(inp)
        i = 0
        while True:
            ok, f = cap.read()
            if not ok:
                break
            yield f"frame{i:06d}", f
            i += 1


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--input", required=True,
                    help="image / dir / video path, or camera index")
    ap.add_argument("--mode", choices=["flight", "scan"], default="flight")
    ap.add_argument("--target", help="target_id to match (green box on hit)")
    ap.add_argument("--save", help="dir to save annotated frames")
    ap.add_argument("--show", action="store_true", help="display window")
    args = ap.parse_args()

    wechat = load_wechat()
    fn = decode_flight if args.mode == "flight" else decode_scan
    if args.save:
        os.makedirs(args.save, exist_ok=True)

    n_frames = n_decoded = 0
    for name, frame in iter_frames(args.input):
        if frame is None:
            continue
        t0 = time.perf_counter()
        results = fn(frame, wechat)
        ms = (time.perf_counter() - t0) * 1000
        n_frames += 1
        if results:
            n_decoded += 1
        hit = any(args.target == r["text"] for r in results) if args.target else None
        print(json.dumps({"frame": name, "ms": round(ms, 1),
                          "decoded": [r["text"] for r in results],
                          "target_hit": hit}))
        if args.save or args.show:
            vis = annotate(frame.copy(), results, args.target)
            if args.save:
                cv2.imwrite(os.path.join(args.save, f"{name}.jpg"), vis)
            if args.show:
                cv2.imshow("qr", vis)
                if cv2.waitKey(1) == 27:
                    break

    print(f"\n{n_decoded}/{n_frames} frames had at least one decode "
          f"({args.mode} mode)", file=sys.stderr)


if __name__ == "__main__":
    main()
