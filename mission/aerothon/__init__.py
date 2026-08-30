"""
AeroTHON 2026 — Track 1: Rotorcraft Systems Challenge (UAS)
Theme: Aerial Surveillance and Rapid Delivery
R2D2 System Architecture Compliance Package
"""

from aerothon.config import MissionConfig, MissionState
from aerothon.coordinates import CoordinateManager, GlobalCoordinate, LocalSectionCoordinate
from aerothon.perception.manager import PerceptionManager, PerceptionResult
from aerothon.slam.pose_estimator import PoseEstimate, PoseEstimator
from aerothon.slam.slam_manager import SLAMManager
from aerothon.mission.fsm import MissionFSM, R2D2MissionState
from aerothon.mavlink.interface import MAVLinkInterface
from aerothon.payload.winch_control import WinchController

__version__ = "2.0.0"
__all__ = [
    "MissionConfig",
    "MissionState",
    "CoordinateManager",
    "GlobalCoordinate",
    "LocalSectionCoordinate",
    "PerceptionManager",
    "PerceptionResult",
    "PoseEstimate",
    "PoseEstimator",
    "SLAMManager",
    "MissionFSM",
    "R2D2MissionState",
    "MAVLinkInterface",
    "WinchController",
]
