"""
Unit tests for Dual Coordinate System (Global NED + Local Section + Trajectory Storage).
100% drone-free, fast, deterministic.
"""

from __future__ import annotations

import unittest

from aerothon.config import MissionState
from aerothon.coordinates import (
    CoordinateManager,
    GlobalCoordinate,
)


class TestCoordinates(unittest.TestCase):
    def test_global_coordinate_altitude(self) -> None:
        g = GlobalCoordinate(x=10.0, y=-5.0, z=-5.0, yaw_deg=90.0)
        self.assertEqual(g.altitude, 5.0)
        self.assertEqual(g.x, 10.0)
        self.assertEqual(g.y, -5.0)
        self.assertEqual(g.yaw_deg, 90.0)

    def test_coordinate_manager_update_from_fc(self) -> None:
        mgr = CoordinateManager()
        mgr.update_from_fc(x=12.5, y=3.2, z=-10.0, yaw=180.0)
        self.assertEqual(mgr.global_pos.x, 12.5)
        self.assertEqual(mgr.global_pos.y, 3.2)
        self.assertEqual(mgr.global_pos.altitude, 10.0)
        self.assertEqual(mgr.global_pos.yaw_deg, 180.0)

    def test_trajectory_path_storage_and_reversal(self) -> None:
        mgr = CoordinateManager()
        # Record forward points
        mgr.record_path_point(MissionState.TAKEOFF, 0.0, 0.0, -5.0, 0.0)
        mgr.record_path_point(MissionState.FOLLOW_CORRIDOR, 5.0, 0.0, -3.0, 0.0)
        mgr.record_path_point(MissionState.DROP_PAYLOAD, 20.0, 0.0, -5.0, 0.0)

        # Record return points
        mgr.record_path_point(MissionState.ASCEND_RETURN, 20.0, 0.0, -10.0, 180.0)
        mgr.record_path_point(MissionState.PRECISION_LAND, 0.0, 0.0, 0.0, 180.0)

        self.assertEqual(len(mgr.forward_path), 3)
        self.assertEqual(len(mgr.return_path), 2)
        self.assertEqual(len(mgr.full_trajectory), 5)

        # Verify breadcrumb reversal
        rev_pts = mgr.get_reverse_return_waypoints()
        self.assertEqual(len(rev_pts), 3)
        self.assertEqual(rev_pts[0], (20.0, 0.0, -5.0))
        self.assertEqual(rev_pts[-1], (0.0, 0.0, -5.0))

    def test_local_coordinates_start_zone(self) -> None:
        mgr = CoordinateManager()
        mgr.update_from_fc(x=0.5, y=0.0, z=-5.0, yaw=0.0)
        lpos = mgr.compute_local_coordinates(MissionState.SCAN_START_QR)

        self.assertEqual(lpos.section_name, "START_ZONE")
        self.assertEqual(lpos.x_rel, 0.5)
        self.assertAlmostEqual(lpos.z_rel, 0.0, places=3)
        self.assertAlmostEqual(lpos.distance_to_goal, 0.5, places=2)

    def test_local_coordinates_corridor_forward(self) -> None:
        mgr = CoordinateManager()
        mgr.update_from_fc(x=8.0, y=0.2, z=-3.048, yaw=0.0)
        lpos = mgr.compute_local_coordinates(
            MissionState.FOLLOW_CORRIDOR,
            d_left_wall=1.55,
            d_right_wall=1.95,
            fwd_dist=10.0,
        )

        self.assertEqual(lpos.section_name, "CORRIDOR_FWD")
        self.assertAlmostEqual(lpos.x_rel, 5.0, places=2)
        self.assertAlmostEqual(lpos.y_rel, 0.20, places=2)
        self.assertAlmostEqual(lpos.z_rel, 0.0, places=2)
        self.assertAlmostEqual(lpos.distance_to_goal, 10.0, places=2)

    def test_local_coordinates_delivery_zone(self) -> None:
        mgr = CoordinateManager()
        mgr.update_from_fc(x=20.0, y=1.5, z=-10.0, yaw=0.0)
        lpos = mgr.compute_local_coordinates(
            MissionState.SCAN_TARGET_QR,
            target_qr_box_dist=2.4,
        )

        self.assertEqual(lpos.section_name, "DELIVERY_ZONE")
        self.assertAlmostEqual(lpos.x_rel, 2.0, places=2)
        self.assertAlmostEqual(lpos.y_rel, 1.5, places=2)
        self.assertAlmostEqual(lpos.distance_to_goal, 2.4, places=2)

    def test_local_coordinates_corridor_return(self) -> None:
        mgr = CoordinateManager()
        mgr.update_from_fc(x=10.0, y=0.0, z=-3.048, yaw=180.0)
        lpos = mgr.compute_local_coordinates(
            MissionState.FOLLOW_CORRIDOR_RETURN,
            d_left_wall=1.75,
            d_right_wall=1.75,
        )

        self.assertEqual(lpos.section_name, "CORRIDOR_RETURN")
        self.assertAlmostEqual(lpos.x_rel, 8.0, places=2)
        self.assertAlmostEqual(lpos.y_rel, 0.0, places=2)
        self.assertAlmostEqual(lpos.distance_to_goal, 10.0, places=2)

    def test_local_coordinates_home_approach(self) -> None:
        mgr = CoordinateManager()
        mgr.update_from_fc(x=0.3, y=-0.4, z=-1.5, yaw=0.0)
        lpos = mgr.compute_local_coordinates(MissionState.APPROACH_HOME)

        self.assertEqual(lpos.section_name, "HOME_APPROACH")
        self.assertAlmostEqual(lpos.distance_to_goal, 0.5, places=2)


if __name__ == "__main__":
    unittest.main()
