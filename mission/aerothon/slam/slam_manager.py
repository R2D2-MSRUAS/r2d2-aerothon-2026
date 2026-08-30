"""
SLAM Manager Module
Handles SLAM pipeline lifecycle, tracking status, and re-localization fallbacks.
"""

import logging

from aerothon.slam.pose_estimator import PoseEstimate, PoseEstimator

log = logging.getLogger("AeroTHON2026.SLAM")


class SLAMManager:
    """Manages Visual-Inertial Odometry / SLAM session health and tracking verification."""

    def __init__(self):
        self.estimator = PoseEstimator()
        self.tracking_lost_count = 0
        self.max_lost_frames = 15

    def process_frame(
        self, x: float, y: float, z: float, yaw: float, valid_features: int = 50
    ) -> PoseEstimate:
        """Processes visual feature count and updates estimated pose."""
        tracking_ok = valid_features >= 10
        if not tracking_ok:
            self.tracking_lost_count += 1
            log.warning(
                f"SLAM feature tracking low ({valid_features} features). Lost count: {self.tracking_lost_count}"
            )
        else:
            self.tracking_lost_count = 0

        pose = self.estimator.update_pose(
            x,
            y,
            z,
            yaw=yaw,
            tracking_ok=(self.tracking_lost_count < self.max_lost_frames),
        )
        return pose

    def is_tracking_healthy(self) -> bool:
        """Returns True if VIO/SLAM tracking is healthy."""
        return self.estimator.current_pose.status == "TRACKING_OK"
