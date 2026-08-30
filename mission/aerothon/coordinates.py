"""
AeroTHON 2026 — Dual Coordinate System & Full Flight Path Trajectory Storage
================================================================================
Author: Bhakti Revankar
Tracks and Stores:
  1. Global Position (World NED Frame from launch 0,0,0)
  2. Local Position (Relative to active mission landmarks: corridor, banner, delivery, QR)
  3. Complete Flight Path (Chronological breadcrumbs from start to finish for both forward and backward paths)
"""

from __future__ import annotations

import dataclasses
import math
import time
from typing import Dict, List, Optional, Tuple

from aerothon.config import DEFAULT_CONFIG, MissionConfig, MissionState


@dataclasses.dataclass
class GlobalCoordinate:
    """Global Position in North-East-Down (NED) World Frame from Launch Pad (0, 0, 0)."""
    x: float = 0.0        # +X: North / Forward (meters)
    y: float = 0.0        # +Y: East / Right (meters)
    z: float = 0.0        # +Z: Down (meters) -> Altitude = -z (when z <= 0)
    yaw_deg: float = 0.0  # Yaw angle in degrees (0° = Initial launch forward heading)

    @property
    def altitude(self) -> float:
        return -self.z if self.z < 0 else self.z


@dataclasses.dataclass
class LocalSectionCoordinate:
    """
    Local Coordinate relative to the active mission section/landmark:
      - section_name: Current operational section (e.g. CORRIDOR_FWD, DELIVERY_ZONE)
      - x_rel: Longitudinal progress along active section (meters)
      - y_rel: Lateral centerline offset (meters, + is right of center)
      - z_rel: Altitude delta relative to target section altitude (meters)
      - distance_to_goal: Estimated distance remaining to section exit/target (meters)
      - extra_info: Descriptive telemetry string for HUD / logs
    """
    section_name: str = "START_PAD"
    x_rel: float = 0.0
    y_rel: float = 0.0
    z_rel: float = 0.0
    distance_to_goal: float = 0.0
    extra_info: str = ""


@dataclasses.dataclass
class TrajectoryPoint:
    """A timestamped 3D waypoint on the stored flight trajectory."""
    timestamp: float
    x: float
    y: float
    z: float
    yaw_deg: float
    state_name: str


class CoordinateManager:
    """
    Manages Dual Coordinate Tracking, Landmark Memory, and Complete Trajectory Storage
    (Both Forward Pass and Backward Return Pass from start to finish).
    """

    def __init__(self, config: MissionConfig = DEFAULT_CONFIG) -> None:
        self.config = config
        self.global_pos = GlobalCoordinate()
        self.local_pos = LocalSectionCoordinate()

        # Landmark World Coordinate Dictionary
        self.landmarks: Dict[str, Tuple[float, float, float]] = {
            "HOME": (0.0, 0.0, 0.0),
            "START_QR": (1.0, 0.0, -self.config.TAKEOFF_ALT_M),
            "CORRIDOR_ENTRY": (3.0, 0.0, -self.config.CORRIDOR_ALT_M),
            "CORRIDOR_EXIT": (3.0 + self.config.CORRIDOR_LENGTH_EST_M, 0.0, -self.config.CORRIDOR_ALT_M),
            "DELIVERY_CENTER": (3.0 + self.config.CORRIDOR_LENGTH_EST_M + 5.0, 0.0, -self.config.DELIVERY_SEARCH_ALT_M),
            "TARGET_QR": (0.0, 0.0, 0.0),
        }

        # Trajectory Storage (Both Forward and Backward Paths from Start to Finish)
        self.forward_path: List[TrajectoryPoint] = []
        self.return_path: List[TrajectoryPoint] = []
        self.full_trajectory: List[TrajectoryPoint] = []

    def update_from_fc(self, x: float, y: float, z: float, yaw: float) -> None:
        """Update global state from Flight Controller telemetry."""
        self.global_pos.x = float(x)
        self.global_pos.y = float(y)
        self.global_pos.z = float(z)
        self.global_pos.yaw_deg = float(yaw)

    def record_path_point(self, state: MissionState, x: float, y: float, z: float, yaw: float) -> None:
        """
        Records every single coordinate along the flight path from start to finish.
        Categorizes points into forward_path or return_path based on mission state.
        """
        pt = TrajectoryPoint(
            timestamp=time.time(),
            x=float(x),
            y=float(y),
            z=float(z),
            yaw_deg=float(yaw),
            state_name=state.name,
        )
        self.full_trajectory.append(pt)

        forward_states = {
            MissionState.INIT,
            MissionState.TAKEOFF,
            MissionState.SCAN_START_QR,
            MissionState.FIND_GREEN_BANNER,
            MissionState.ENTER_CORRIDOR,
            MissionState.FOLLOW_CORRIDOR,
            MissionState.REACH_DELIVERY,
            MissionState.SCAN_TARGET_QR,
            MissionState.ALIGN_PAYLOAD,
            MissionState.DROP_PAYLOAD,
        }
        if state in forward_states:
            self.forward_path.append(pt)
        else:
            self.return_path.append(pt)

    def get_reverse_return_waypoints(self) -> List[Tuple[float, float, float]]:
        """Returns the recorded forward path in reverse order as 3D (x, y, z) waypoints for return navigation."""
        return [(pt.x, pt.y, pt.z) for pt in reversed(self.forward_path)]

    def register_landmark(self, name: str, x_global: float, y_global: float, z_global: float) -> None:
        """Saves observed landmark coordinate into the world coordinate map."""
        self.landmarks[name] = (float(x_global), float(y_global), float(z_global))

    def compute_local_coordinates(
        self,
        state: MissionState,
        d_left_wall: float = 0.0,
        d_right_wall: float = 0.0,
        fwd_dist: float = float("inf"),
        target_qr_box_dist: Optional[float] = None,
    ) -> LocalSectionCoordinate:
        """
        Calculates real-time local coordinates relative to current section geometry.
        """
        gx, gy, gz = self.global_pos.x, self.global_pos.y, self.global_pos.z
        alt = self.global_pos.altitude

        if state in (MissionState.INIT, MissionState.TAKEOFF, MissionState.SCAN_START_QR):
            sec = "START_ZONE"
            x_rel = gx
            y_rel = gy
            z_rel = alt - self.config.TAKEOFF_ALT_M
            d_goal = max(0.0, 1.0 - gx)
            info = f"Alt: {alt:.2f}m / Goal: {self.config.TAKEOFF_ALT_M}m"

        elif state == MissionState.FIND_GREEN_BANNER:
            sec = "BANNER_APPROACH"
            entry_x = self.landmarks["CORRIDOR_ENTRY"][0]
            x_rel = gx
            y_rel = gy
            z_rel = alt - self.config.TAKEOFF_ALT_M
            d_goal = fwd_dist if not math.isinf(fwd_dist) else max(0.0, entry_x - gx)
            info = f"Banner Dist: {d_goal:.2f}m"

        elif state in (MissionState.ENTER_CORRIDOR, MissionState.FOLLOW_CORRIDOR):
            sec = "CORRIDOR_FWD"
            if d_left_wall > 0 and d_right_wall > 0:
                y_center_err = (d_right_wall - d_left_wall) / 2.0
            else:
                y_center_err = gy
            entry_x = self.landmarks["CORRIDOR_ENTRY"][0]
            x_corr = max(0.0, gx - entry_x)
            x_rel = x_corr
            y_rel = y_center_err
            z_rel = alt - self.config.CORRIDOR_ALT_M
            d_goal = (
                fwd_dist
                if (fwd_dist < 15.0 and not math.isinf(fwd_dist))
                else max(0.0, self.config.CORRIDOR_LENGTH_EST_M - x_corr)
            )
            info = f"Corridor progress: {x_corr:.1f}m/{self.config.CORRIDOR_LENGTH_EST_M:.1f}m | LatErr: {y_center_err:+.2f}m"

        elif state in (
            MissionState.REACH_DELIVERY,
            MissionState.SCAN_TARGET_QR,
            MissionState.ALIGN_PAYLOAD,
            MissionState.DROP_PAYLOAD,
        ):
            sec = "DELIVERY_ZONE"
            deliv_entry = self.landmarks["CORRIDOR_EXIT"]
            x_rel = gx - deliv_entry[0]
            y_rel = gy - deliv_entry[1]
            target_alt = (
                self.config.PAYLOAD_DROP_ALT_M
                if state in (MissionState.ALIGN_PAYLOAD, MissionState.DROP_PAYLOAD)
                else self.config.DELIVERY_SEARCH_ALT_M
            )
            z_rel = alt - target_alt
            d_goal = target_qr_box_dist if target_qr_box_dist is not None else 0.0
            info = f"Delivery Arena (X:{x_rel:+.2f}m, Y:{y_rel:+.2f}m) | QR Dist: {d_goal:.2f}m"

        elif state in (
            MissionState.ASCEND_RETURN,
            MissionState.ALIGN_RETURN_CORRIDOR,
            MissionState.FOLLOW_CORRIDOR_RETURN,
            MissionState.EXIT_CORRIDOR,
        ):
            sec = "CORRIDOR_RETURN"
            exit_x = self.landmarks["CORRIDOR_EXIT"][0]
            ret_progress = max(0.0, exit_x - gx)
            if d_left_wall > 0 and d_right_wall > 0:
                y_center_err = (d_right_wall - d_left_wall) / 2.0
            else:
                y_center_err = gy
            x_rel = ret_progress
            y_rel = y_center_err
            z_rel = alt - self.config.RETURN_CORRIDOR_ALT_M
            d_goal = max(0.0, gx)  # Remaining distance to Home Pad
            info = f"Return Progress: {ret_progress:.1f}m | To Home: {d_goal:.1f}m"

        elif state in (MissionState.APPROACH_HOME, MissionState.PRECISION_LAND):
            sec = "HOME_APPROACH"
            x_rel = gx
            y_rel = gy
            z_rel = alt
            d_goal = math.sqrt(gx**2 + gy**2)
            info = f"Dist to Home Pad: {d_goal:.2f}m | Alt: {alt:.2f}m"

        else:
            sec = state.name
            x_rel, y_rel, z_rel, d_goal = gx, gy, gz, 0.0
            info = ""

        self.local_pos = LocalSectionCoordinate(
            section_name=sec,
            x_rel=x_rel,
            y_rel=y_rel,
            z_rel=z_rel,
            distance_to_goal=d_goal,
            extra_info=info,
        )
        return self.local_pos
