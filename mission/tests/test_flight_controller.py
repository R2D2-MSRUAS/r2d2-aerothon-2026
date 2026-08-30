"""
Unit tests for Flight Controller Interface, Mock FC, and Winch Actuation.
100% drone-free, fast, deterministic.
"""

from __future__ import annotations

import unittest

from aerothon.config import DEFAULT_CONFIG, MissionConfig
from aerothon.flight_controller import (
    MockFlightController,
    create_flight_controller,
    execute_winch_delivery,
)


class TestFlightController(unittest.TestCase):

    def test_mock_fc_initial_state(self) -> None:
        fc = MockFlightController()
        x, y, z, yaw = fc.get_position_ned()
        self.assertEqual(x, 0.0)
        self.assertEqual(y, 0.0)
        self.assertEqual(z, 0.0)
        self.assertEqual(yaw, 0.0)
        self.assertFalse(fc.armed)

    def test_mock_fc_takeoff_and_altitude(self) -> None:
        fc = MockFlightController()
        ok = fc.arm_and_takeoff(5.0)
        self.assertTrue(ok)
        self.assertTrue(fc.armed)
        _, _, z, _ = fc.get_position_ned()
        self.assertEqual(z, -5.0)

        ok_alt = fc.set_altitude(10.0)
        self.assertTrue(ok_alt)
        _, _, z2, _ = fc.get_position_ned()
        self.assertEqual(z2, -10.0)

    def test_mock_fc_velocity_integration(self) -> None:
        fc = MockFlightController()
        fc.arm_and_takeoff(5.0)

        # Move forward 2 m/s for 1.0s at heading 0 deg (North)
        fc.send_body_velocity(vx=2.0, vy=0.0, vz=0.0, duration=1.0)
        x, y, z, _ = fc.get_position_ned()
        self.assertAlmostEqual(x, 2.0, places=2)
        self.assertAlmostEqual(y, 0.0, places=2)

        # Turn 90 deg right (East) and move forward 1 m/s for 2.0s
        fc.set_yaw(90.0, relative=False)
        fc.send_body_velocity(vx=1.0, vy=0.0, vz=0.0, duration=2.0)
        x2, y2, _, yaw2 = fc.get_position_ned()
        self.assertEqual(yaw2, 90.0)
        self.assertAlmostEqual(x2, 2.0, places=1)
        self.assertAlmostEqual(y2, 2.0, places=1)

    def test_mock_fc_factory_safety_default(self) -> None:
        # When --connect is provided but enable_real_flight=False, MUST return MockFlightController
        fc = create_flight_controller(
            connection_string="/dev/ttyUSB0",
            enable_real_flight=False,
        )
        self.assertIsInstance(fc, MockFlightController)

    def test_winch_delivery_sequence(self) -> None:
        test_cfg = MissionConfig(
            WINCH_LOWER_TIME_S=0.01,
            WINCH_HOLD_TIME_S=0.01,
            WINCH_REEL_TIME_S=0.01,
        )
        fc = MockFlightController(config=test_cfg)
        execute_winch_delivery(fc, config=test_cfg)

        self.assertIn(test_cfg.WINCH_LOWER_PWM, fc.winch_pwm_history)
        self.assertIn(test_cfg.WINCH_RELEASE_PWM, fc.winch_pwm_history)
        self.assertIn(test_cfg.WINCH_RAISE_PWM, fc.winch_pwm_history)
        self.assertEqual(fc.winch_pwm_history[-1], test_cfg.WINCH_NEUTRAL_PWM)
        self.assertTrue(any("BODY_VEL vx=" in cmd for cmd in fc.command_log))


if __name__ == "__main__":
    unittest.main()
