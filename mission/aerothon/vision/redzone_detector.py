"""
Red Zone Forbidden Area Detector
R2D2 Vision Subsystem Component
"""

import cv2
import numpy as np


class RedZoneDetector:
    """Detects red-colored penalty / forbidden landing zones using dual-hue HSV color thresholding."""

    def __init__(self):
        self.red_hue_ranges = [
            ((0, 130, 80), (10, 255, 255)),
            ((170, 130, 80), (180, 255, 255)),
        ]
        self.min_area = 500

    def detect(self, bgr_frame: np.ndarray) -> list[tuple[int, int, int, int]]:
        """
        Returns list of red zone bounding boxes [(x, y, w, h)]
        """
        if bgr_frame is None:
            return []

        hsv = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2HSV)
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)

        for lower, upper in self.red_hue_ranges:
            m = cv2.inRange(
                hsv, np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8)
            )
            mask = cv2.bitwise_or(mask, m)

        kernel = np.ones((7, 7), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        red_boxes = []

        for c in contours:
            if cv2.contourArea(c) >= self.min_area:
                red_boxes.append(cv2.boundingRect(c))

        return red_boxes
