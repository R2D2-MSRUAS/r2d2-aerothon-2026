"""
Unit Tests for R2D2 Modular Architecture & Perception / FSM Logic
Ensures drone-free execution without hardware or camera attached.
"""

import unittest

from aerothon.mavlink.interface import MAVLinkInterface
from aerothon.mission.fsm import MissionFSM, R2D2MissionState
from aerothon.navigation.corridor_controller import CorridorController
from aerothon.navigation.obstacle_avoidance import ObstacleAvoidance
from aerothon.payload.winch_control import WinchController
from aerothon.perception.manager import PerceptionManager, PerceptionResult
from aerothon.slam.pose_estimator import PoseEstimate


class TestR2D2Architecture(unittest.TestCase):

    def test_perception_manager_aggregation(self):
        pm = PerceptionManager()
        result = pm.update(
            banner_center=(320, 240),
            banner_distance=2.5,
            qr_codes=[{"id": "DELIVERY_TARGET_A", "center_x": 300, "center_y": 200}],
            obstacles={"left": 1.2, "forward": 4.0, "right": 2.8},
            corridor_offset=0.2
        )
        self.assertTrue(result.banner_detected)
        self.assertEqual(result.banner_center, (320, 240))
        self.assertEqual(len(result.qr_codes), 1)
        self.assertEqual(result.qr_codes[0]["id"], "DELIVERY_TARGET_A")
        self.assertEqual(result.obstacles["left"], 1.2)

    def test_corridor_controller_lateral_steering(self):
        ctrl = CorridorController(k_p=0.5)
        # Closer to left wall (left=0.8, right=2.8) -> Should steer right (vy > 0)
        p1 = PerceptionResult(obstacles={"left": 0.8, "forward": 5.0, "right": 2.8})
        vy1 = ctrl.compute_lateral_correction(p1)
        self.assertGreater(vy1, 0.0)

        # Closer to right wall (left=2.8, right=0.8) -> Should steer left (vy < 0)
        p2 = PerceptionResult(obstacles={"left": 2.8, "forward": 5.0, "right": 0.8})
        vy2 = ctrl.compute_lateral_correction(p2)
        self.assertLess(vy2, 0.0)

    def test_obstacle_avoidance_thresholds(self):
        avoid = ObstacleAvoidance(safe_dist_m=2.0, critical_stop_m=0.8)
        
        # Clear path
        p_clear = PerceptionResult(obstacles={"forward": 3.5})
        self.assertEqual(avoid.evaluate_clearance(p_clear), "CLEAR")

        # Caution zone
        p_slow = PerceptionResult(obstacles={"forward": 1.5})
        self.assertEqual(avoid.evaluate_clearance(p_slow), "SLOW")

        # Critical stop zone
        p_stop = PerceptionResult(obstacles={"forward": 0.5})
        self.assertEqual(avoid.evaluate_clearance(p_stop), "STOP")

        # Red zone detected -> Mandatory STOP
        p_red = PerceptionResult(red_zone_detected=True, obstacles={"forward": 5.0})
        self.assertEqual(avoid.evaluate_clearance(p_red), "STOP")

    def test_fsm_state_transitions(self):
        fsm = MissionFSM()
        self.assertEqual(fsm.state, R2D2MissionState.INIT)

        pose = PoseEstimate(position=(0.0, 0.0, 5.0))
        perception = PerceptionResult()

        # INIT -> TAKEOFF -> SCAN_START_QR
        fsm.update(perception, pose)
        self.assertEqual(fsm.state, R2D2MissionState.TAKEOFF)

        fsm.update(perception, pose)
        self.assertEqual(fsm.state, R2D2MissionState.SCAN_START_QR)

    def test_winch_controller_safe_mock(self):
        mavlink = MAVLinkInterface()
        winch = WinchController(mavlink, servo_channel=9)
        # Verify winch sequence executes without raising exceptions
        winch.execute_full_drop_sequence()
        self.assertTrue(mavlink.is_armed is False)


if __name__ == "__main__":
    unittest.main()
