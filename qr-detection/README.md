# QR Detector + Scanner — AeroTHON 2026 Mission 2 (Team R2D2, AT2026RC35)

Detect-and-decode pipeline for the SkyScan mission, benchmarked on synthetic
aerial frames that simulate the OAK-D Lite's actual viewing geometry
(45° camera tilt, 1–18 m slant range, sun/shade lighting, motion blur).

**TL;DR recommendation: two-mode pipeline, with neural auto-zoom for the scan.**

| Mode | When | Method | Range (0.4 m QR) |
|---|---|---|---|
| `flight` | moving flight, 30 fps budget | WeChat CNN detect + decode @ 1080p | reliable ≤ 3 m, 75% @ 5 m |
| `scan` (auto-zoom) | stationary hover scan | YOLO QR detector on 4K tiles → ROI "zoom" crop → ZBar decode | **100% @ 8–12 m, 75% @ 15 m** |

This mirrors and *validates* the report's adaptive-resolution design: decode
range is pixel-limited, so the stationary QR scans must run at 4K, and the
delivery-zone scan at 10 m altitude is **marginal** with a 0.4 m code —
see "Implication for the mission" below.

## Results

Synthetic benchmark, 12 frames per cell, payload `AT2026RC35_M2_TARGET_07`
(25-module QR), OAK-D Lite FOV model (69° HFOV), distances are slant range.

### 1080p capture — decode rate (sun/shade/normal pooled)

| method | 1 m | 3 m | 5 m | 8 m | 10 m | 12 m | 15 m | avg ms* |
|---|---|---|---|---|---|---|---|---|
| OpenCV `QRCodeDetector` | 75% | 31% | 0% | 0% | 0% | 0% | 0% | 49 |
| **WeChat (CNN + SR)** | **100%** | **100%** | **75%** | 14% | 3% | 3% | 0% | 138 |
| ZBar full frame | 97% | 94% | 67% | 0% | 0% | 0% | 0% | 87 |
| Hybrid (WeChat crop → ZBar) | 100% | 100% | 75% | 14% | 3% | 3% | 0% | 138 |

### 4K capture — decode rate (normal lighting)

| method | 5 m | 8 m | 10 m | 12 m | 15 m | 18 m | avg ms* |
|---|---|---|---|---|---|---|---|
| WeChat | 50% | 50% | 42% | 8% | 8% | 0% | ~900 |
| **ZBar full frame** | **100%** | **100%** | **58%** | **67%** | 0% | 0% | ~380 |

\* Laptop CPU (Apple Silicon). TX2 NX numbers will differ; ratios should hold.

### Auto-zoom (detect far → digital zoom → decode) — the winner

The OAK-D Lite has no optical zoom, but its 13 MP sensor supports the
software equivalent: stream a downscaled view for speed, and on detection
crop the QR's region at native sensor resolution (DepthAI ISP/ImageManip
ROI crop) — recovering the pixels flight mode throws away.

Chain: **QReader's pretrained YOLOv8 QR detector on 2×2 tiles of the 4K
frame → map boxes back → crop with margin → upscale → ZBar decode.**

| method | 8 m | 10 m | 12 m | 15 m | 18 m | ms/frame* |
|---|---|---|---|---|---|---|
| baseline 1080p (flight mode) | 58% | 8% | 0% | 0% | 0% | ~100 |
| ZBar full-frame @ 4K | 100% | 58% | 67% | 0% | 0% | ~380 |
| **tiled YOLO detect + zoom + ZBar** | **100%** | **100%** | **100%** | **75%** | 8% | ~500 |

Two facts make this work:
- **Detection outranges decoding.** The neural detector finds a QR it
  cannot possibly read; the zoom crop then supplies the missing pixels.
- **Conversion is 100%**: in every test at 8–15 m, once the detector fired,
  the zoomed crop decoded. Detection range is now the only limiter — and a
  custom YOLO trained on `gen_dataset.py` output at higher input size
  should push it past 15 m.

~0.5 s/frame is too slow for flight but fine for the stationary P6 hover
scan, and drops further if detection runs as a blob on the Myriad X VPU.
See `test_autozoom_tiled.py` (also `test_autozoom.py`/`test_autozoom_yolo.py`
for the negative results: classical detectors fail as far-range spotters,
and single-shot YOLO at 640 px input goes blind past 8 m).

### Key findings

1. **Decode range is pixel-budget-limited, not algorithm-limited.** A 0.4 m
   QR at 10 m slant range subtends ~56 px at 1080p → ~2 px per module →
   physically undecodable by anything. At 4K it's ~112 px (~4 px/module) →
   decodable. This is the quantitative justification for the 4K stationary
   scan mode in our report, and for the pixel-size validation flagged as a
   pending item.
2. **WeChat is the best flight-mode detector** — CNN detection + built-in
   super-resolution wins at 1080p short range, and it's the only method
   that also returns corner points for alignment PID.
3. **WeChat degrades at 4K** (its detector rescales large inputs; latency
   ~1 s). At 4K, plain ZBar over the full frame is both faster and far more
   reliable. So: WeChat for flight, ZBar for the hover scan — which is
   exactly the VPU-crop → ZBar-decode split already in our architecture.
4. **OpenCV's built-in QRCodeDetector is not competitive** at any range.

### Implication for the mission (raise with team)

At the P6 delivery-zone scan (10 m altitude, 45° tilt ⇒ ~14 m slant range
to the footprint centre), even 4K ZBar decodes only ~58–67% per frame at
10–12 m — workable with the 3-consecutive-frame confirmation but with
little margin, and **0% beyond 12 m**. Options, pending the rulebook's QR
physical dimensions:
- scan closer to nadir (reduce slant range) or descend to ~8 m for the sweep;
- or accept per-frame ~60% and rely on multi-frame confirmation over a slow sweep;
- if the rulebook QR is smaller than 0.4 m, descending is mandatory.

## Files

| file | purpose |
|---|---|
| `qr_pipeline.py` | the deliverable: detect+scan on image / dir / video / webcam, `--mode flight\|scan`, `--target <id>` matching, annotated output |
| `benchmark.py` | 1080p benchmark across 7 distances × 3 lighting × 4 methods → `results.csv` |
| `test_4k.py` | 4K decode-range test |
| `test_autozoom_tiled.py` | **the auto-zoom pipeline** (tiled YOLO detect → ROI zoom crop → ZBar) |
| `test_autozoom.py`, `test_autozoom_yolo.py` | auto-zoom ablations (classical / single-shot detectors) |
| `synth.py` | synthetic aerial frame generator (OAK-D geometry model) |
| `gen_dataset.py` | YOLO-format training-set generator for a custom QR *detector* |
| `wechat_models/` | WeChat QR CNN weights (from opencv_3rdparty) |
| `samples/`, `annotated/` | example frames per distance + pipeline output |
| `results.csv` | full benchmark data |

## Setup

```bash
python3 -m venv .venv
./.venv/bin/pip install "opencv-contrib-python==4.11.0.86" pyzbar qrcode pillow numpy
brew install zbar                      # macOS; apt install libzbar0 on Jetson/Linux
# macOS only: run python with DYLD_LIBRARY_PATH=/opt/homebrew/lib
```

Note: opencv-contrib is pinned to 4.x — OpenCV 5 removed the Caffe loader
the WeChat models need.

## Usage

```bash
# fast flight mode on a video, matching the mission target id
DYLD_LIBRARY_PATH=/opt/homebrew/lib ./.venv/bin/python qr_pipeline.py \
  --input flight_footage.mp4 --mode flight --target AT2026RC35_M2_TARGET_07 --save out/

# max-range scan mode on still frames
DYLD_LIBRARY_PATH=/opt/homebrew/lib ./.venv/bin/python qr_pipeline.py \
  --input samples/ --mode scan
```

Output is one JSON line per frame: decoded strings, latency, target hit.

## Training a QR detector (the "can a model be trained" question)

Yes — and it's the right long-range play: a trained detector *finds* codes
far beyond decode range (useful for approach guidance), then ZBar decodes
once close enough. Two routes:

1. **Off the shelf:** [QReader](https://github.com/Eric-Canas/qreader) —
   a YOLOv8 model already trained for QR detection, ZBar decode built in.
   Heavier dependency (PyTorch), but zero training effort. Good baseline.
2. **Our own (recommended for the OAK-D):** `gen_dataset.py` produces a
   YOLO dataset at mission-realistic scales/angles/lighting:

   ```bash
   ./.venv/bin/python gen_dataset.py --n 2000 --out qr_dataset
   # drop Kaggle semantic-drone-dataset images into real_backgrounds/ first —
   # the generator composites QRs onto them for 60% of frames (domain realism)
   yolo detect train data=qr_dataset/dataset.yaml model=yolov8n.pt imgsz=1280
   ```

   Export PyTorch → ONNX → OpenVINO IR → Myriad X blob (Luxonis
   blobconverter) to run detection on the OAK-D's VPU, keeping the
   TX2 NX free — same deployment path as the PIDNet segmentation model.

## Next steps

- [ ] Get QR physical dimensions from the rulebook → recompute range table
- [ ] Re-run benchmark on real footage (record OAK-D flight clips with printed QRs)
- [ ] Validate WeChat latency on the TX2 NX (or move detection to a trained
      YOLO blob on the Myriad X and keep only ZBar on the Jetson)
- [ ] Mix `real_backgrounds/` from the semantic-drone-dataset into training data
