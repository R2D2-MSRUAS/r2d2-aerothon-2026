"""
MAVLinkFlightController against an ArduCopter-like fake vehicle.

Regression for "drone just spins instead of moving forward to find the QR":
body-frame velocity setpoints without a yaw target let ArduCopter turn the
nose toward the velocity, which rotates the body frame, so any command with a
sideways part turned into spinning on the spot.
"""

from __future__ import annotations

import logging
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import drone_fsm_prototype as P  # noqa: E402
from fake_arducopter import FakeArduCopter  # noqa: E402


def _controller(vehicle: FakeArduCopter) -> P.MAVLinkFlightController:
    fc = P.MAVLinkFlightController.__new__(P.MAVLinkFlightController)
    fc.config = P.CFG
    fc.connection_string = "fake"
    fc._vehicle = vehicle
    fc._yaw_hold_deg = None
    fc._course_deg = 0.0
    return fc


class _Clocked(unittest.TestCase):
    def setUp(self) -> None:
        self.v = FakeArduCopter()
        self._real_time = P.time
        P.time = self.v.clock
        logging.disable(logging.CRITICAL)

    def tearDown(self) -> None:
        P.time = self._real_time
        logging.disable(logging.NOTSET)


class TestYawHold(_Clocked):
    def test_body_velocity_with_sideways_part_does_not_spin(self):
        fc = _controller(self.v)
        self._arm()
        fc.arm_and_takeoff(10.0)
        start = self.v.heading
        for _ in range(60):  # the old SCAN_TARGET_QR search command, 24 s
            fc.send_body_velocity(0.3, 0.2, 0.0, duration=0.4)
        turned = abs(math.degrees(math.atan2(math.sin(self.v.heading - start), math.cos(self.v.heading - start))))
        self.assertLess(turned, 2.0)
        # and it actually travelled forward-right, ~0.36 m/s for 24 s
        self.assertGreater(self.v.n, 5.0)
        self.assertGreater(self.v.e, 3.0)

    def test_relative_scan_completes_full_turn(self):
        fc = _controller(self.v)
        self._arm()
        fc.arm_and_takeoff(5.0)
        total = 0.0
        prev = self.v.heading
        for _ in range(24):  # FIND_GREEN_BANNER sweep: 24 x 15 deg
            fc.set_yaw(15.0, relative=True)
            d = math.atan2(math.sin(self.v.heading - prev), math.cos(self.v.heading - prev))
            total += math.degrees(d)
            prev = self.v.heading
        self.assertAlmostEqual(total, 360.0, delta=8.0)

    def test_set_altitude_keeps_heading_and_position(self):
        fc = _controller(self.v)
        self._arm()
        fc.arm_and_takeoff(5.0)
        fc.set_yaw(90.0, relative=False)
        n0, e0 = self.v.n, self.v.e
        self.assertTrue(fc.set_altitude(1.5))
        self.assertAlmostEqual(math.degrees(self.v.heading) % 360, 90.0, delta=6.0)
        self.assertLess(math.hypot(self.v.n - n0, self.v.e - e0), 0.3)

    def _arm(self) -> bool:
        import dronekit_stub  # noqa: F401

        self.v.mode = type("M", (), {"name": "GUIDED"})()
        self.v.armed = True
        return True


class TestFullMissionArduCopterLike(_Clocked):
    def test_mission_completes_on_fake_arducopter(self):
        self._run_mission()

    def test_field_not_facing_north_and_far_target(self):
        # Pad faces 73 deg; target QR in the far corner of the delivery zone.
        self.v = FakeArduCopter(heading_deg=73.0)
        P.time = self.v.clock
        self._run_mission(target_box="qr_TEAM_D")

    def _run_mission(self, target_box: str | None = None):
        from aerothon.qr_decode import _get_wechat
        from aerothon.sim_field_camera import FieldSimCamera

        if _get_wechat() is None:
            self.skipTest("WeChat decoder unavailable (needs opencv-contrib-python)")
        import dronekit_stub  # noqa: F401  (VehicleMode for arm/land)

        self.v.mode = type("M", (), {"name": "GUIDED"})()
        self.v.armed = True  # dronekit sets armed asynchronously; pre-armed here
        fc = _controller(self.v)
        cam = FieldSimCamera(fc)
        if target_box:  # move our team's QR to another of the 5 spots
            a = next(b for b in cam.boxes if b.name == "qr_R2D2_TARGET")
            b = next(b for b in cam.boxes if b.name == target_box)
            a.texture, b.texture = b.texture, a.texture
        fsm = P.MissionFSM(fc, cam, perception=P.PerceptionPipeline(enable_yolo=False))
        seen = []
        for _ in range(1000):
            bgr, d = cam.read()
            st = fsm.step(bgr, d)["state"].name
            if not seen or seen[-1] != st:
                seen.append(st)
            if st == "COMPLETE":
                break
        self.assertEqual(fsm.state.name, "COMPLETE", seen)
        self.assertIn("DROP_PAYLOAD", seen)
        self.assertLess(math.hypot(self.v.n, self.v.e), 0.61)  # on the 4x4 ft pad
        # heading never spun: total yaw travel stays small
        tr = self.v.heading_trace
        travel = sum(abs(math.atan2(math.sin(b - a), math.cos(b - a))) for a, b in zip(tr, tr[1:]))
        self.assertLess(math.degrees(travel), 1080.0)


if __name__ == "__main__":
    unittest.main()
