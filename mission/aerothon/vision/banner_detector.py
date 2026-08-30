"""
Corridor Entrance Banner Detector
R2D2 Vision Subsystem Component
"""


import cv2
import numpy as np


class BannerDetector:
    """Detects green corridor entry banner using HSV color segmentation and morphological filtering."""

    def __init__(self, lower_hsv: tuple[int, int, int] = (35, 75, 50), upper_hsv: tuple[int, int, int] = (85, 255, 255)):
        self.lower_hsv = np.array(lower_hsv, dtype=np.uint8)
        self.upper_hsv = np.array(upper_hsv, dtype=np.uint8)
        self.min_area = 2500

    def detect(self, bgr_frame: np.ndarray) -> tuple[tuple[int, int] | None, tuple[int, int, int, int] | None]:
        """
        Returns (banner_center_xy, bounding_box_xywh) or (None, None).
        """
        if bgr_frame is None:
            return None, None

        hsv = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_hsv, self.upper_hsv)

        kernel = np.ones((7, 7), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best_box = None
        best_area = 0

        for c in contours:
            area = cv2.contourArea(c)
            if area > self.min_area and area > best_area:
                x, y, w, h = cv2.boundingRect(c)
                if (w / max(h, 1)) >= 1.2:
                    best_area = area
                    best_box = (x, y, w, h)

        if best_box is not None:
            x, y, w, h = best_box
            center = (x + w // 2, y + h // 2)
            return center, best_box

        return None, None
