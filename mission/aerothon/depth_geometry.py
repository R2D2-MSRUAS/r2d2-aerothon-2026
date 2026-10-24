"""
Ground-aware depth analysis for corridor flight.

The FSM originally read wall distance as "median depth of the left/right
fifth of the image". With the camera pitched down (OAK-D mount is 45 deg)
most of those pixels are ground, so "walls detected" fired with no walls
and wall-centring was driven by floor depth. Light-coloured obstacle boards
were also invisible, because the classical detector only finds dark blobs.

Here every depth pixel is back-projected using the camera pitch and the
drone's altitude into (forward, right, height-above-ground). Then:
  * walls     = structure at the drone's altitude, beside it
  * obstacles = structure at the drone's altitude, in its flight lane
  * ground is ignored entirely.
"""

from __future__ import annotations

import math

import numpy as np

WALL_MIN_H = 0.30  # m above ground to count as a wall/obstacle point
ALT_BELOW_M = 1.0  # structure more than this below the drone is passed over, not hit
ALT_ABOVE_M = 0.7  # ...and more than this above it is overhead
LOOK_AHEAD_M = 4.0  # only consider points this far forward
LANE_HALF_M = 0.65  # half-width of the drone's flight lane
NO_RETURN_M = 99.0  # reported when nothing is seen


def analyze_depth(
    depth_mm: np.ndarray,
    alt_m: float,
    pitch_deg: float = 45.0,
    hfov_deg: float = 73.0,
    step: int = 4,
    stop_m: float = 0.8,
    slow_m: float = 2.0,
) -> dict:
    h, w = depth_mm.shape[:2]
    fx = (w / 2) / math.tan(math.radians(hfov_deg) / 2)
    d = depth_mm[::step, ::step].astype(np.float32) / 1000.0
    vs, us = np.mgrid[0:h:step, 0:w:step]
    xn = (us - w / 2) / fx
    yn = (vs - h / 2) / fx
    p = math.radians(pitch_deg)
    valid = d > 0.15
    fwd = d * (math.cos(p) - yn * math.sin(p))
    down = d * (math.sin(p) + yn * math.cos(p))
    right = d * xn
    height = alt_m - down

    # Things well below us (the corridor arch seen from 5 m, the pad) are not
    # collision threats. Only a pitched camera's view of the LOWER part of a
    # wall/board is often available, so the band reaches 1 m below the drone.
    band_lo = max(WALL_MIN_H, alt_m - ALT_BELOW_M)
    tall = (
        valid
        & (height > band_lo)
        & (height < alt_m + ALT_ABOVE_M)
        & (fwd > 0.1)
        & (fwd < LOOK_AHEAD_M)
    )

    lane = tall & (np.abs(right) < LANE_HALF_M)
    left = tall & (right <= -LANE_HALF_M)
    rightside = tall & (right >= LANE_HALF_M)

    def _robust_min(v):
        return float(np.percentile(v, 10)) if v.size >= 6 else NO_RETURN_M

    d_left = _robust_min(-right[left])
    d_right = _robust_min(right[rightside])
    fwd_free = _robust_min(fwd[lane])

    obstacles = []
    if fwd_free < slow_m:
        sel = lane & (fwd < fwd_free + 0.4)
        cx = int(np.median(us[sel])) if sel.any() else w // 2
        cy = int(np.median(vs[sel])) if sel.any() else h // 2
        action = "STOP" if fwd_free < stop_m else "SLOW"
        lat = float(np.median(right[sel])) if sel.any() else 0.0
        obstacles.append(
            {
                "cx": cx,
                "cy": cy,
                "radius": 20,
                "bbox": (cx - 20, cy - 20, 40, 40),
                "dist_m": fwd_free,
                "lateral_m": lat,
                "action": action,
                "source": "depth_geometry",
                "class_name": "obstacle",
            }
        )
    return {
        "d_left_wall": d_left,
        "d_right_wall": d_right,
        "fwd_dist_m": fwd_free,
        "obstacles": obstacles,
    }


def pixel_to_ground(
    px: float,
    py: float,
    width: int,
    height: int,
    alt_m: float,
    pitch_deg: float = 45.0,
    hfov_deg: float = 73.0,
) -> tuple[float, float] | None:
    """Where on the ground (forward_m, right_m from the drone) a pixel looks.

    With a pitched-down forward camera, a target centred in the image is NOT
    under the drone (at 45 deg it is `alt` metres ahead), so payload
    alignment must use this ground offset rather than "centre the QR".
    """
    fx = (width / 2) / math.tan(math.radians(hfov_deg) / 2)
    xn = (px - width / 2) / fx
    yn = (py - height / 2) / fx
    p = math.radians(pitch_deg)
    down = math.sin(p) + yn * math.cos(p)
    if down <= 1e-3:
        return None  # at or above the horizon
    t = alt_m / down
    return t * (math.cos(p) - yn * math.sin(p)), t * xn
