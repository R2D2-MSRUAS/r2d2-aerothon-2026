"""
Find QR-code-like patches from far away, before they are decodable.

From the 10 m search altitude a 1 m borderless QR is only ~30-60 px across
and decodes only when almost straight ahead. But it is still obviously a
"busy" black-and-white square on plain ground. This finds such patches so
the FSM can fly over and read each one, instead of hoping a lane passes
exactly over the target.

Cues: high edge density + both very dark and very bright pixels in the
patch + roughly square (allowing for the 45 deg camera foreshortening).
"""

from __future__ import annotations

import cv2
import numpy as np

MIN_AREA_PX = 120
MAX_AREA_FRAC = 0.10


def find_qr_candidates(bgr: np.ndarray) -> list[tuple[int, int, int]]:
    """[(cx, cy, area_px), ...] largest first."""
    if bgr is None or bgr.size == 0:
        return []
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    h, w = gray.shape
    edges = cv2.Canny(gray, 60, 160)
    density = cv2.blur(edges.astype(np.float32) / 255.0, (9, 9))
    dark = cv2.blur((gray < 70).astype(np.float32), (9, 9))
    bright = cv2.blur((gray > 170).astype(np.float32), (9, 9))
    mask = ((density > 0.22) & (dark > 0.12) & (bright > 0.12)).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < MIN_AREA_PX or area > MAX_AREA_FRAC * h * w:
            continue
        (cx, cy), (rw, rh), _ = cv2.minAreaRect(c)
        if min(rw, rh) < 6 or max(rw, rh) / max(min(rw, rh), 1) > 3.0:
            continue
        out.append((int(cx), int(cy), int(area)))
    out.sort(key=lambda t: -t[2])
    return out
