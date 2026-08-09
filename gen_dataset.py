"""Generate a YOLO-format training dataset for a QR *detector*.

Answers the "can a model be trained on detecting QRs and then scanning
them" question: this produces synthetic aerial frames with QR codes
composited at mission-realistic scales/angles/lighting, labelled with
bounding boxes (class 0 = qr). Train any YOLO on it, use the trained
detector for long-range ROI proposal, and decode the cropped ROI with ZBar
— the same detect-then-decode split as QReader and as our mission
architecture (Myriad X crop -> ZBar on TX2 NX).

For best results mix in real footage: paste the same generated QRs onto
frames from the Kaggle semantic-drone-dataset instead of procedural
backgrounds (drop images into real_backgrounds/ and they will be used).

Usage: python gen_dataset.py --n 500 --out qr_dataset
"""
import argparse
import os
import random

import cv2
import numpy as np

import synth
from synth import make_qr, make_background, composite_qr, apply_conditions, qr_pixel_size

WORDS = ["TARGET", "ZONE", "DROP", "ALPHA", "BRAVO", "DELTA", "ECHO"]


def random_payload(rng):
    return f"AT2026RC35_M2_{rng.choice(WORDS)}_{rng.randrange(100):02d}"


def load_real_backgrounds(d="real_backgrounds"):
    if not os.path.isdir(d):
        return []
    out = []
    for n in os.listdir(d):
        if n.lower().endswith((".jpg", ".jpeg", ".png")):
            img = cv2.imread(os.path.join(d, n))
            if img is not None:
                out.append(cv2.resize(img, (synth.FRAME_W, synth.FRAME_H)))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=500)
    ap.add_argument("--out", default="qr_dataset")
    ap.add_argument("--val-frac", type=float, default=0.15)
    args = ap.parse_args()

    real_bgs = load_real_backgrounds()
    print(f"{len(real_bgs)} real background(s) found" if real_bgs
          else "no real_backgrounds/ - using procedural textures")

    for split in ("train", "val"):
        os.makedirs(f"{args.out}/images/{split}", exist_ok=True)
        os.makedirs(f"{args.out}/labels/{split}", exist_ok=True)

    rng = random.Random(42)
    for i in range(args.n):
        split = "val" if rng.random() < args.val_frac else "train"
        bg = rng.choice(real_bgs).copy() if real_bgs and rng.random() < 0.6 \
            else make_background(rng)

        labels = []
        frame = bg
        for _ in range(rng.randint(1, 3)):          # 1-3 QRs per frame
            dist = rng.uniform(2, 15)               # slant range, metres
            size = qr_pixel_size(dist)
            qr = make_qr(random_payload(rng))
            tilt = rng.uniform(30, 55)
            yaw = rng.uniform(-45, 45)
            frame, pts = composite_qr(frame, qr, size, tilt, yaw, rng)
            x0, y0 = pts.min(axis=0)
            x1, y1 = pts.max(axis=0)
            cx = (x0 + x1) / 2 / synth.FRAME_W
            cy = (y0 + y1) / 2 / synth.FRAME_H
            w = (x1 - x0) / synth.FRAME_W
            h = (y1 - y0) / synth.FRAME_H
            labels.append(f"0 {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}")

        light = rng.choice(["normal", "normal", "shade", "sun"])
        blur = rng.choice([1, 1, 1, 3, 5])
        frame = apply_conditions(frame, light, blur, rng)

        name = f"qr_{i:05d}"
        cv2.imwrite(f"{args.out}/images/{split}/{name}.jpg", frame,
                    [cv2.IMWRITE_JPEG_QUALITY, 90])
        with open(f"{args.out}/labels/{split}/{name}.txt", "w") as fh:
            fh.write("\n".join(labels) + "\n")
        if (i + 1) % 100 == 0:
            print(f"{i + 1}/{args.n}")

    with open(f"{args.out}/dataset.yaml", "w") as fh:
        fh.write(f"path: {os.path.abspath(args.out)}\n"
                 "train: images/train\nval: images/val\n"
                 "names:\n  0: qr\n")
    print(f"done -> {args.out}/  (train with: yolo detect train "
          f"data={args.out}/dataset.yaml model=yolov8n.pt imgsz=1280)")


if __name__ == "__main__":
    main()
