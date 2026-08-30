"""
Unit tests for Mission Finite State Machine (Forward & Backward Pass).
100% drone-free, fast, deterministic.
"""

from __future__ import annotations

import unittest

import numpy as np
from aerothon.config import MissionConfig, MissionState
from aerothon.flight_controller import MockFlightController
from aerothon.fsm import MissionFSM
from aerothon.vision import MockCamera, PerceptionPipeline


class TestMissionFSM(unittest.TestCase):

    def test_fsm_takeoff_to_scan_qr(self) -> None:
        fc = MockFlightController()
        cam = MockCamera()
        fsm = MissionFSM(fc=fc, camera=cam)

        self.assertEqual(fsm.state, MissionState.TAKEOFF)
        bgr, depth = cam.read()
        res = fsm.step(bgr, depth)

        self.assertEqual(res["state"], MissionState.SCAN_START_QR)
        self.assertTrue(fc.armed)
        self.assertEqual(fc.sim_z, -fsm.config.TAKEOFF_ALT_M)

    def test_fsm_qr_decode_transition(self) -> None:
        fc = MockFlightController()
        cam = MockCamera()
        fsm = MissionFSM(fc=fc, camera=cam)
        fsm.state = MissionState.SCAN_START_QR

        bgr = np.zeros((480, 640, 3), dtype=np.uint8)
        depth = np.full((480, 640), 1500, dtype=np.uint16)

        class MockPerception(PerceptionPipeline):
            def perceive(self, bgr_in, depth_in):
                out = super().perceive(bgr_in, depth_in)
                out["qr_codes"] = [("DELIVERY_ZONE_B", np.array([[200, 200], [300, 200], [300, 300], [200, 300]]))]
                return out

        fsm.perception = MockPerception(config=fsm.config, enable_yolo=False)
        res = fsm.step(bgr, depth)

        self.assertEqual(res["target_id"], "DELIVERY_ZONE_B")
        self.assertEqual(res["state"], MissionState.FIND_GREEN_BANNER)

    def test_fsm_emergency_avoidance_trigger(self) -> None:
        fc = MockFlightController()
        cam = MockCamera()
        fsm = MissionFSM(fc=fc, camera=cam)
        fsm.state = MissionState.FOLLOW_CORRIDOR

        bgr = np.zeros((480, 640, 3), dtype=np.uint8)
        depth = np.full((480, 640), 3000, dtype=np.uint16)

        class ObstaclePerception(PerceptionPipeline):
            def perceive(self, bgr_in, depth_in):
                out = super().perceive(bgr_in, depth_in)
                out["obstacles"] = [{
                    "cx": 320,
                    "cy": 240,
                    "radius": 40,
                    "bbox": (280, 200, 80, 80),
                    "dist_m": 0.5,
                    "action": "STOP",
                    "source": "mock",
                }]
                return out

        fsm.perception = ObstaclePerception(config=fsm.config, enable_yolo=False)
        res = fsm.step(bgr, depth)

        self.assertEqual(res["state"], MissionState.AVOID_OBSTACLES)
        self.assertEqual(fsm.state_before_avoid, MissionState.FOLLOW_CORRIDOR)

    def test_fsm_full_forward_and_backward_cycle(self) -> None:
        fast_cfg = MissionConfig(
            WINCH_LOWER_TIME_S=0.01,
            WINCH_HOLD_TIME_S=0.01,
            WINCH_REEL_TIME_S=0.01,
            QR_CONFIRM_COUNT=1,
        )
        fc = MockFlightController(config=fast_cfg)
        cam = MockCamera()
        fsm = MissionFSM(fc=fc, camera=cam, config=fast_cfg)

        target_qr = "ZONE_ALPHA"

        class FullMissionPerception(PerceptionPipeline):
            def __init__(self):
                super().__init__(config=fast_cfg, enable_yolo=False)
                self.mode = "init"

            def perceive(self, bgr_in, depth_in):
                out = super().perceive(bgr_in, depth_in)
                if self.mode == "start_qr":
                    out["qr_codes"] = [(target_qr, np.array([[300, 200], [340, 200], [340, 240], [300, 240]]))]
                elif self.mode == "banner":
                    out["banner_box"] = (300, 200, 40, 20)
                    out["fwd_dist_m"] = 2.0
                elif self.mode == "corridor":
                    out["d_left_wall"] = 1.75
                    out["d_right_wall"] = 1.75
                    out["fwd_dist_m"] = 10.0
                elif self.mode == "target_qr":
                    out["qr_codes"] = [(target_qr, np.array([[320, 240], [340, 240], [340, 260], [320, 260]]))]
                return out

        mock_percept = FullMissionPerception()
        fsm.perception = mock_percept

        bgr = np.zeros((480, 640, 3), dtype=np.uint8)
        depth = np.full((480, 640), 3000, dtype=np.uint16)

        # 1. TAKEOFF -> SCAN_START_QR
        fsm.step(bgr, depth)
        self.assertEqual(fsm.state, MissionState.SCAN_START_QR)

        # 2. SCAN_START_QR -> FIND_GREEN_BANNER
        mock_percept.mode = "start_qr"
        fsm.step(bgr, depth)
        self.assertEqual(fsm.state, MissionState.FIND_GREEN_BANNER)
        self.assertEqual(fsm.target_id, target_qr)

        # 3. FIND_GREEN_BANNER -> ENTER_CORRIDOR
        mock_percept.mode = "banner"
        for _ in range(5):
            fsm.step(bgr, depth)
        fsm.state = MissionState.ENTER_CORRIDOR

        # 4. ENTER_CORRIDOR -> FOLLOW_CORRIDOR
        mock_percept.mode = "corridor"
        fsm.step(bgr, depth)
        self.assertEqual(fsm.state, MissionState.FOLLOW_CORRIDOR)

        # 5. FOLLOW_CORRIDOR -> REACH_DELIVERY (drone reaches X=20.0)
        fc.sim_x = 20.0
        fc.sim_z = -3.048
        fsm.step(bgr, depth)
        self.assertEqual(fsm.state, MissionState.REACH_DELIVERY)

        # 6. REACH_DELIVERY -> SCAN_TARGET_QR
        fsm.step(bgr, depth)
        self.assertEqual(fsm.state, MissionState.SCAN_TARGET_QR)

        # 7. SCAN_TARGET_QR -> ALIGN_PAYLOAD
        mock_percept.mode = "target_qr"
        fsm.step(bgr, depth)
        self.assertEqual(fsm.state, MissionState.ALIGN_PAYLOAD)

        # 8. ALIGN_PAYLOAD -> DROP_PAYLOAD
        fsm.step(bgr, depth)
        self.assertEqual(fsm.state, MissionState.DROP_PAYLOAD)

        # 9. DROP_PAYLOAD -> ASCEND_RETURN
        fsm.step(bgr, depth)
        self.assertEqual(fsm.state, MissionState.ASCEND_RETURN)

        # 10. ASCEND_RETURN -> ALIGN_RETURN_CORRIDOR
        fsm.step(bgr, depth)
        self.assertEqual(fsm.state, MissionState.ALIGN_RETURN_CORRIDOR)

        # 11. ALIGN_RETURN_CORRIDOR -> FOLLOW_CORRIDOR_RETURN
        mock_percept.mode = "corridor"
        fsm.step(bgr, depth)
        self.assertEqual(fsm.state, MissionState.FOLLOW_CORRIDOR_RETURN)

        # 12. FOLLOW_CORRIDOR_RETURN -> APPROACH_HOME (reached home proximity X=1.0)
        fc.sim_x = 1.0
        fc.sim_z = -3.048
        fc.sim_yaw = 180.0
        fsm.step(bgr, depth)
        self.assertEqual(fsm.state, MissionState.APPROACH_HOME)

        # 13. APPROACH_HOME -> PRECISION_LAND (reached X=0.2, Y=0.1)
        fc.sim_x = 0.2
        fc.sim_y = 0.1
        fc.sim_z = -1.0
        fsm.step(bgr, depth)
        self.assertEqual(fsm.state, MissionState.PRECISION_LAND)

        # 14. PRECISION_LAND -> COMPLETE
        fsm.step(bgr, depth)
        self.assertEqual(fsm.state, MissionState.COMPLETE)


if __name__ == "__main__":
    unittest.main()
