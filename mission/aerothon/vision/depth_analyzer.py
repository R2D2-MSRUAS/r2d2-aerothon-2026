"""
OAK-D Stereo Depth Analyzer
R2D2 Vision Subsystem Component
"""

import numpy as np


class DepthAnalyzer:
    """Analyzes stereo depth matrix across 3 vertical columns (left, center, right)."""

    def __init__(self, max_depth_m: float = 10.0):
        self.max_depth_m = max_depth_m

    def analyze_3column_depth(self, depth_mm: np.ndarray) -> dict[str, float]:
        """
        Calculates median distance in meters for forward (center), left, and right channels.
        """
        if depth_mm is None or depth_mm.size == 0:
            return {
                "forward": float("inf"),
                "left": float("inf"),
                "right": float("inf"),
            }

        _h, w = depth_mm.shape
        col_w = w // 3

        left_crop = depth_mm[:, :col_w]
        center_crop = depth_mm[:, col_w : 2 * col_w]
        right_crop = depth_mm[:, 2 * col_w :]

        def get_valid_median(crop: np.ndarray) -> float:
            valid = crop[(crop > 200) & (crop < self.max_depth_m * 1000)]
            if valid.size == 0:
                return self.max_depth_m
            return float(np.median(valid)) / 1000.0

        return {
            "left": get_valid_median(left_crop),
            "forward": get_valid_median(center_crop),
            "right": get_valid_median(right_crop),
        }
