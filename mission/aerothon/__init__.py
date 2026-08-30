"""
AeroTHON 2026 — Track 1: Rotorcraft Systems Challenge (UAS)
Theme: Aerial Surveillance and Rapid Delivery
R2D2 System Architecture Compliance Package
"""

from aerothon.config import MissionConfig, MissionState
from aerothon.coordinates import (
    CoordinateManager,
    GlobalCoordinate,
    LocalSectionCoordinate,
)
from aerothon.mavlink.interface import MAVLinkInterface
from aerothon.mission.fsm import MissionFSM, R2D2MissionState
from aerothon.payload.winch_control import WinchController
from aerothon.perception.manager import PerceptionManager, PerceptionResult
from aerothon.slam.pose_estimator import PoseEstimate, PoseEstimator
from aerothon.slam.slam_manager import SLAMManager

__version__ = "2.0.0"
__all__ = [
    "CoordinateManager",
    "GlobalCoordinate",
    "LocalSectionCoordinate",
    "MAVLinkInterface",
    "MissionConfig",
    "MissionFSM",
    "MissionState",
    "PerceptionManager",
    "PerceptionResult",
    "PoseEstimate",
    "PoseEstimator",
    "R2D2MissionState",
    "SLAMManager",
    "WinchController",
]
