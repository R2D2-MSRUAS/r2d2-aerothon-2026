"""
Corridor banner detection for the AeroTHON 2026 field spec.

The organisers' banners are long thin strips:
  * arch banner   11.5 ft x 1.5 ft  (aspect ~7.7)
  * ground banner 11.5 ft x 1.0 ft  (aspect ~11.5, more when foreshortened)
  * entry corridor = GREEN #00B140, exit corridor = BLUE #0000FF
  * 1-inch white border on every banner

The old detector used hue 35-85 / sat >= 75, which also matches grass and
any green-painted sim geometry, and judged shape from the axis-aligned box.
This one uses a tighter colour window around the printed colours, judges
shape from the rotated rectangle (so a tilted or yawed banner still reads
as a strip), requires the blob to actually fill its rectangle, and rejects
blobs that swallow the frame (ground, walls).
"""

from __future__ import annotations

import cv2
import numpy as np

# HSV windows (OpenCV hue 0-179). #00B140 -> (71, 255, 177); #0000FF -> (120, 255, 255).
BANNER_HSV = {
    "green": ((58, 110, 60), (88, 255, 255)),
    "blue": ((105, 120, 60), (130, 255, 255)),
}

MIN_AREA_FRAC = 0.0004  # ~120 px on 640x480: a banner ~15 m away still counts
MAX_AREA_FRAC = 0.45  # bigger than this is ground / wall, not a banner
MIN_ASPECT = 2.5  # rotated-rect long/short side; spec strips are 7.7 and 11.5
MAX_ASPECT = 40.0
MIN_FILL = 0.55  # contour area / rotated-rect area: must be a solid strip


def banner_mask(bgr: np.ndarray, color: str = "green") -> np.ndarray:
    lo, hi = BANNER_HSV[color]
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
    k = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    return mask


def detect_banners(bgr: np.ndarray, color: str = "green") -> list[dict]:
    """All banner-shaped blobs of one colour, largest first.

    Each result: {"box": (x, y, w, h), "center": (cx, cy), "area": px,
    "aspect": float, "angle": deg, "color": str}.
    """
    if bgr is None or bgr.size == 0:
        return []
    h_f, w_f = bgr.shape[:2]
    frame_area = float(h_f * w_f)
    mask = banner_mask(bgr, color)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    found = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < MIN_AREA_FRAC * frame_area or area > MAX_AREA_FRAC * frame_area:
            continue
        (cx, cy), (rw, rh), angle = cv2.minAreaRect(c)
        short, long_ = min(rw, rh), max(rw, rh)
        if short < 2:
            continue
        aspect = long_ / short
        if not (MIN_ASPECT <= aspect <= MAX_ASPECT):
            continue
        if area / (rw * rh) < MIN_FILL:
            continue
        x, y, w, h = cv2.boundingRect(c)
        found.append(
            {
                "box": (int(x), int(y), int(w), int(h)),
                "center": (int(cx), int(cy)),
                "area": float(area),
                "aspect": float(aspect),
                "angle": float(angle),
                "color": color,
            }
        )
    found.sort(key=lambda d: d["area"], reverse=True)
    return found


def detect_corridor_banner(
    bgr: np.ndarray, color: str = "green"
) -> tuple[int, int, int, int] | None:
    """Box of the best banner of `color`, or None. When both the arch and the
    ground banner are visible, they are merged into one box so the centre
    used for lateral alignment sits on the corridor centreline."""
    hits = detect_banners(bgr, color)
    if not hits:
        return None
    best = hits[0]
    x, y, w, h = best["box"]
    bcx = best["center"][0]
    for other in hits[1:3]:
        ox, oy, ow, oh = other["box"]
        # same corridor if horizontally overlapping by at least half
        overlap = min(x + w, ox + ow) - max(x, ox)
        if overlap > 0.5 * min(w, ow) and abs(other["center"][0] - bcx) < 0.5 * w:
            nx, ny = min(x, ox), min(y, oy)
            w, h = max(x + w, ox + ow) - nx, max(y + h, oy + oh) - ny
            x, y = nx, ny
    return (x, y, w, h)
