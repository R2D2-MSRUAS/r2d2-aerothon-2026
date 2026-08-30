"""
Pose Estimator & PoseEstimate Data Structure
R2D2 System Architecture Compliance Module
"""

import time
from dataclasses import dataclass
from typing import Tuple


@dataclass
class PoseEstimate:
    """
    3D Pose Estimate published by Visual-Inertial Odometry / SLAM pipeline.
    Passed to Navigation and MAVLink ExternalNav interfaces.
    """
    position: Tuple[float, float, float] = (0.0, 0.0, 0.0)      # (x, y, z) in meters
    orientation: Tuple[float, float, float] = (0.0, 0.0, 0.0)   # (roll, pitch, yaw) in deg
    timestamp: float = 0.0
    status: str = "TRACKING_OK"                                # TRACKING_OK or TRACKING_BROKEN


class PoseEstimator:
    """Tracks position and orientation from OAK-D IMU and visual odometry features."""

    def __init__(self):
        self.current_pose = PoseEstimate(timestamp=time.time())

    def update_pose(
        self,
        x: float,
        y: float,
        z: float,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
        tracking_ok: bool = True
    ) -> PoseEstimate:
        """Updates and returns current pose estimate."""
        status = "TRACKING_OK" if tracking_ok else "TRACKING_BROKEN"
        self.current_pose = PoseEstimate(
            position=(x, y, z),
            orientation=(roll, pitch, yaw),
            timestamp=time.time(),
            status=status
        )
        return self.current_pose
