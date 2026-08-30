"""
Unit tests for Pure Perception, Stereo Depth Reasoning & Vision Helpers.
100% drone-free, offline, deterministic.
"""

from __future__ import annotations

import math
import unittest

import numpy as np
from aerothon.config import DEFAULT_CONFIG
from aerothon.vision import (
    DEPTH_UNKNOWN_M,
    MockCamera,
    depth_action,
    depth_at_bbox,
    detect_green_banner,
    draw_circle,
    draw_ellipse,
    draw_hexagon,
    draw_rounded_box,
    find_red_zones,
)


class TestVision(unittest.TestCase):
    def test_depth_at_bbox_median(self) -> None:
        depth = np.zeros((480, 640), dtype=np.uint16)
        # Put 1900mm inside bbox
        depth[180:260, 250:310] = 1900

        bbox = (250, 180, 60, 80)
        d_m = depth_at_bbox(depth, bbox, percentile=50.0)
        self.assertAlmostEqual(d_m, 1.9, places=2)

    def test_depth_at_bbox_empty_or_zeros(self) -> None:
        depth = np.zeros((480, 640), dtype=np.uint16)
        bbox = (100, 100, 50, 50)
        d_m = depth_at_bbox(depth, bbox)
        self.assertTrue(math.isinf(d_m))
        self.assertEqual(d_m, DEPTH_UNKNOWN_M)

    def test_depth_action_classification(self) -> None:
        self.assertEqual(depth_action(3.5, DEFAULT_CONFIG), "CONTINUE")
        self.assertEqual(depth_action(1.5, DEFAULT_CONFIG), "SLOW")
        self.assertEqual(depth_action(0.5, DEFAULT_CONFIG), "STOP")
        self.assertEqual(depth_action(float("inf"), DEFAULT_CONFIG), "SLOW")

    def test_shape_drawing_functions(self) -> None:
        img = np.zeros((200, 200, 3), dtype=np.uint8)
        color = (0, 255, 0)

        draw_circle(img, 20, 20, 80, 80, color)
        draw_ellipse(img, 20, 20, 80, 80, color)
        draw_hexagon(img, 20, 20, 80, 80, color)
        draw_rounded_box(img, 20, 20, 80, 80, color, r=10)

        self.assertGreater(np.count_nonzero(img), 0)

    def test_green_banner_detection(self) -> None:
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        # Draw green banner in BGR
        img[150:230, 200:440] = [40, 210, 40]

        box = detect_green_banner(img)
        self.assertIsNotNone(box)
        x, y, w, h = box
        self.assertLess(abs(x - 200), 5)
        self.assertLess(abs(y - 150), 5)
        self.assertLess(abs(w - 240), 5)
        self.assertLess(abs(h - 80), 5)

    def test_red_zone_detection(self) -> None:
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        # Draw Red zone in BGR
        img[200:300, 300:450] = [20, 20, 230]

        boxes = find_red_zones(img)
        self.assertGreaterEqual(len(boxes), 1)
        rx, ry, _rw, _rh = boxes[0]
        self.assertLess(abs(rx - 300), 10)
        self.assertLess(abs(ry - 200), 10)

    def test_mock_camera(self) -> None:
        cam = MockCamera(width=640, height=480)
        frame, depth = cam.read()
        self.assertIsNotNone(frame)
        self.assertIsNotNone(depth)
        self.assertEqual(frame.shape, (480, 640, 3))
        self.assertEqual(depth.shape, (480, 640))
        cam.release()


if __name__ == "__main__":
    unittest.main()
