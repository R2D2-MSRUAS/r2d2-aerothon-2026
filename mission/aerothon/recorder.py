"""
AeroTHON 2026 — Flight Data Recorder
================================================================================
Records real-time flight telemetry, dual-coordinate states, and perception logs
to CSV format in accordance with SAEINDIA AeroTHON 2026 rulebook requirements.
"""

from __future__ import annotations

import csv
import time
from typing import Any

from aerothon.config import MissionState
from aerothon.coordinates import GlobalCoordinate, LocalSectionCoordinate


class FlightDataRecorder:
    """Records mission events and telemetry to CSV."""

    def __init__(self, log_path: str = "flight_data_log.csv") -> None:
        self.filepath = log_path
        self.file = open(self.filepath, mode="w", newline="")  # noqa: SIM115
        self.writer = csv.writer(self.file)
        self.writer.writerow(
            [
                "timestamp",
                "state",
                "global_x",
                "global_y",
                "global_z",
                "altitude",
                "yaw_deg",
                "section",
                "local_x",
                "local_y",
                "local_z",
                "dist_to_goal",
                "fwd_depth_m",
                "left_wall_m",
                "right_wall_m",
                "target_id",
                "steering",
                "num_obstacles",
                "num_humans",
            ]
        )

    def record(
        self,
        state: MissionState,
        gpos: GlobalCoordinate,
        lpos: LocalSectionCoordinate,
        percept: dict[str, Any],
        target_id: str = "",
    ) -> None:
        self.writer.writerow(
            [
                f"{time.time():.3f}",
                state.name,
                f"{gpos.x:.3f}",
                f"{gpos.y:.3f}",
                f"{gpos.z:.3f}",
                f"{gpos.altitude:.3f}",
                f"{gpos.yaw_deg:.1f}",
                lpos.section_name,
                f"{lpos.x_rel:.3f}",
                f"{lpos.y_rel:.3f}",
                f"{lpos.z_rel:.3f}",
                f"{lpos.distance_to_goal:.3f}",
                f"{percept.get('fwd_dist_m', 0.0):.2f}",
                f"{percept.get('d_left_wall', 0.0):.2f}",
                f"{percept.get('d_right_wall', 0.0):.2f}",
                target_id,
                percept.get("steering", "FORWARD"),
                len(percept.get("obstacles", [])),
                len(percept.get("humans", [])),
            ]
        )
        self.file.flush()

    def close(self) -> None:
        if self.file and not self.file.closed:
            self.file.close()
