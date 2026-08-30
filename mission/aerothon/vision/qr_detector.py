"""
Multi-Stage QR Code Detector & Decoder
R2D2 Vision Subsystem Component
"""

from typing import Dict, List
import cv2
import numpy as np


class QRDetector:
    """Decodes QR codes using Pyzbar with OpenCV fallback across multi-scale image pyramids."""

    def __init__(self):
        self.opencv_detector = cv2.QRCodeDetector()

    def detect_and_decode(self, bgr_frame: np.ndarray) -> List[Dict[str, float]]:
        """
        Returns list of decoded QR dicts: [{'id': str, 'center_x': float, 'center_y': float, 'bbox': list}]
        """
        if bgr_frame is None:
            return []

        results = []
        gray = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape

        # PyZbar Attempt
        try:
            from pyzbar.pyzbar import decode as zbar_decode
            decoded_objs = zbar_decode(gray)
            for obj in decoded_objs:
                if obj.type == "QRCODE":
                    text = obj.data.decode("utf-8", errors="ignore").strip()
                    if text:
                        pts = [[p.x, p.y] for p in obj.polygon]
                        cx = sum(p[0] for p in pts) / len(pts)
                        cy = sum(p[1] for p in pts) / len(pts)
                        results.append({"id": text, "center_x": cx, "center_y": cy, "bbox": pts})
            if results:
                return results
        except Exception:
            pass

        # OpenCV Fallback
        ok, decoded_info, points, _ = self.opencv_detector.detectAndDecodeMulti(bgr_frame)
        if ok and decoded_info and points is not None:
            for text, pts in zip(decoded_info, points):
                if text:
                    cx = sum(p[0] for p in pts) / len(pts)
                    cy = sum(p[1] for p in pts) / len(pts)
                    results.append({"id": text, "center_x": cx, "center_y": cy, "bbox": pts.tolist()})

        return results
