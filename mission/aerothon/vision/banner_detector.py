"""
Corridor Entrance Banner Detector
R2D2 Vision Subsystem Component
"""

import numpy as np

from aerothon.banner import detect_corridor_banner


class BannerDetector:
    """Detects the corridor banners (green entry #00B140 / blue exit #0000FF).

    Thin wrapper over aerothon.banner, which is tuned to the 2026 field
    spec (11.5 ft x 1.5 ft arch and 11.5 ft x 1 ft ground strips).
    """

    def __init__(self, color: str = "green"):
        self.color = color

    def detect(
        self, bgr_frame: np.ndarray
    ) -> tuple[tuple[int, int] | None, tuple[int, int, int, int] | None]:
        """
        Returns (banner_center_xy, bounding_box_xywh) or (None, None).
        """
        if bgr_frame is None:
            return None, None
        box = detect_corridor_banner(bgr_frame, self.color)
        if box is None:
            return None, None
        x, y, w, h = box
        return (x + w // 2, y + h // 2), box
