"""
Tests for the Oct 2026 field-spec update: banner detection, depth geometry,
borderless QR decoding, and a full simulated mission on the rendered field.
"""

from __future__ import annotations

import logging
import unittest

import numpy as np

from aerothon.banner import detect_corridor_banner
from aerothon.depth_geometry import analyze_depth, pixel_to_ground
from aerothon.field_spec import FIELD
from aerothon.qr_decode import decode_qr_codes
from aerothon.sim_field_camera import FieldSimCamera


class _Pose:
    def __init__(self, x, y, alt, yaw):
        self.p = (x, y, -alt, yaw)

    def get_position_ned(self):
        return self.p


def _frame(x, y, alt, yaw, **kw):
    return FieldSimCamera(_Pose(x, y, alt, yaw), noise=0, **kw).read()


class TestFieldSpec(unittest.TestCase):
    def test_dimensions_from_document(self):
        self.assertAlmostEqual(FIELD.CORRIDOR_LENGTH_M, 6.706, places=3)  # 22 ft
        self.assertAlmostEqual(FIELD.CORRIDOR_WIDTH_M, 3.505, places=3)  # 11.5 ft
        self.assertAlmostEqual(FIELD.CORRIDOR_WALL_HEIGHT_M, 2.438, places=3)  # 8 ft
        self.assertAlmostEqual(FIELD.PAD_SIZE_M, 1.219, places=3)  # 4 ft
        self.assertLess(FIELD.CORRIDOR_FLIGHT_ALT_M, FIELD.CORRIDOR_WALL_HEIGHT_M - 0.5)


class TestBanner(unittest.TestCase):
    def test_spec_strip_banners_are_found(self):
        # 11.5 x 1.5 ft and 11.5 x 1 ft strips (aspect 7.7 / 11.5) on grey.
        for h_ft in (1.5, 1.0):
            img = np.full((480, 640, 3), 110, np.uint8)
            w, h = 345, int(345 * h_ft / 11.5)
            img[200 : 200 + h, 147 : 147 + w] = (64, 177, 0)  # #00B140 as BGR
            self.assertIsNotNone(detect_corridor_banner(img, "green"), h_ft)

    def test_green_grass_is_not_a_banner(self):
        img = np.full((480, 640, 3), 110, np.uint8)
        img[240:] = (50, 140, 50)  # the old sim_test "grass"
        self.assertIsNone(detect_corridor_banner(img, "green"))

    def test_banner_seen_from_takeoff(self):
        bgr, _ = _frame(0, 0, 5, 0)
        self.assertIsNotNone(detect_corridor_banner(bgr, "green"))

    def test_blue_exit_banner_on_return(self):
        bgr, _ = _frame(FIELD.corridor_exit_x + 4, FIELD.RETURN_CORRIDOR_Y_M, 3, 180)
        self.assertIsNotNone(detect_corridor_banner(bgr, "blue"))


class TestDepthGeometry(unittest.TestCase):
    def test_walls_in_corridor(self):
        _, d = _frame(FIELD.CORRIDOR_ENTRY_X_M + 1, FIELD.FORWARD_CORRIDOR_Y_M, 1.5, 0)
        r = analyze_depth(d, 1.5)
        self.assertAlmostEqual(r["d_left_wall"], FIELD.corridor_half_width, delta=0.15)
        self.assertAlmostEqual(r["d_right_wall"], FIELD.corridor_half_width, delta=0.15)

    def test_no_walls_in_open_field(self):
        _, d = _frame(0, 0, 1.5, 0)
        r = analyze_depth(d, 1.5)
        self.assertGreater(r["d_left_wall"], 3.0)
        self.assertGreater(r["d_right_wall"], 3.0)

    def test_structure_far_below_is_not_a_threat(self):
        _, d = _frame(3.0, 0, 5.0, 0)  # above the corridor mouth at 5 m
        self.assertEqual(analyze_depth(d, 5.0)["obstacles"], [])

    def test_obstacle_board_ahead(self):
        _, d = _frame(FIELD.corridor_exit_x - 0.3, FIELD.RETURN_CORRIDOR_Y_M + 0.9, 1.5, 180)
        r = analyze_depth(d, 1.5, stop_m=1.2, slow_m=2.0)
        self.assertTrue(r["obstacles"])
        self.assertLess(r["fwd_dist_m"], 2.0)

    def test_pixel_to_ground_centre_is_ahead(self):
        f, r = pixel_to_ground(320, 240, 640, 480, alt_m=5.0, pitch_deg=45.0)
        self.assertAlmostEqual(f, 5.0, places=3)
        self.assertAlmostEqual(r, 0.0, places=3)


class TestBorderlessQR(unittest.TestCase):
    def test_decode_start_qr(self):
        bgr, _ = _frame(0, 0, 5, 0)
        texts = [t for t, _ in decode_qr_codes(bgr)]
        if not texts:
            self.skipTest("WeChat decoder unavailable (needs opencv-contrib-python)")
        self.assertIn("R2D2_TARGET", texts)


class TestSimMission(unittest.TestCase):
    def test_full_mission_on_rendered_field(self):
        from aerothon.qr_decode import _get_wechat

        if _get_wechat() is None:
            self.skipTest("WeChat decoder unavailable (needs opencv-contrib-python)")
        import os
        import sys

        sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        import drone_fsm_prototype as P

        logging.disable(logging.CRITICAL)
        try:
            fc = P.MockFlightController()
            cam = FieldSimCamera(fc)
            fsm = P.MissionFSM(fc, cam, perception=P.PerceptionPipeline(enable_yolo=False))
            visited = set()
            for _ in range(600):
                bgr, d = cam.read()
                visited.add(fsm.step(bgr, d)["state"].name)
                if fsm.state.name == "COMPLETE":
                    break
        finally:
            logging.disable(logging.NOTSET)
        self.assertEqual(fsm.state.name, "COMPLETE")
        for st in ("FOLLOW_CORRIDOR", "DROP_PAYLOAD", "FOLLOW_CORRIDOR_RETURN", "PRECISION_LAND"):
            self.assertIn(st, visited)
        x, y, _, _ = fc.get_position_ned()
        self.assertLess(np.hypot(x, y), FIELD.PAD_SIZE_M / 2)  # landed on the pad


if __name__ == "__main__":
    unittest.main()
