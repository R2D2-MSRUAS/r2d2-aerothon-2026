"""
Mission FSM (Finite State Machine)
R2D2 System Architecture Compliance Module
"""

from enum import Enum, auto
import logging
import time
from typing import Optional

from aerothon.perception.manager import PerceptionResult
from aerothon.slam.pose_estimator import PoseEstimate

log = logging.getLogger("AeroTHON2026.FSM")


class R2D2MissionState(Enum):
    """Standardized R2D2 FSM State Machine States."""
    INIT             = auto()
    TAKEOFF          = auto()
    SCAN_START_QR    = auto()
    FIND_BANNER      = auto()
    ENTER_CORRIDOR   = auto()
    FOLLOW_CORRIDOR  = auto()
    SCAN_TARGET_QR   = auto()
    ALIGN_FOR_DROP   = auto()
    DROP_PAYLOAD     = auto()
    RETURN           = auto()
    LAND             = auto()
    COMPLETE         = auto()


class MissionFSM:
    """Manages sequential mission transitions and state-dependent perception goals."""

    def __init__(self, target_qr_id: Optional[str] = None):
        self.state = R2D2MissionState.INIT
        self.target_qr_id = target_qr_id
        self.state_start_time = time.time()

    def transition_to(self, new_state: R2D2MissionState) -> None:
        """Triggers clean state transition with timing log."""
        log.info(f"FSM State Transition: {self.state.name} -> {new_state.name}")
        self.state = new_state
        self.state_start_time = time.time()

    def update(self, perception: PerceptionResult, pose: PoseEstimate) -> R2D2MissionState:
        """Evaluates perception results and current pose to transition states."""
        elapsed = time.time() - self.state_start_time

        if self.state == R2D2MissionState.INIT:
            self.transition_to(R2D2MissionState.TAKEOFF)

        elif self.state == R2D2MissionState.TAKEOFF:
            # Transition when reached 5m altitude
            if abs(pose.position[2] - 5.0) < 0.3 or elapsed > 8.0:
                self.transition_to(R2D2MissionState.SCAN_START_QR)

        elif self.state == R2D2MissionState.SCAN_START_QR:
            if perception.qr_codes:
                self.target_qr_id = perception.qr_codes[0]["id"]
                log.info(f"Start QR Decoded. Target ID set to: {self.target_qr_id}")
                self.transition_to(R2D2MissionState.FIND_BANNER)
            elif elapsed > 10.0:
                log.warning("Start QR scan timeout. Proceeding to FIND_BANNER.")
                self.transition_to(R2D2MissionState.FIND_BANNER)

        elif self.state == R2D2MissionState.FIND_BANNER:
            if perception.banner_detected or elapsed > 5.0:
                self.transition_to(R2D2MissionState.ENTER_CORRIDOR)

        elif self.state == R2D2MissionState.ENTER_CORRIDOR:
            if elapsed > 3.0:
                self.transition_to(R2D2MissionState.FOLLOW_CORRIDOR)

        elif self.state == R2D2MissionState.FOLLOW_CORRIDOR:
            # Traversed corridor (~15m forward displacement)
            if pose.position[0] >= 15.0 or elapsed > 15.0:
                self.transition_to(R2D2MissionState.SCAN_TARGET_QR)

        elif self.state == R2D2MissionState.SCAN_TARGET_QR:
            for qr in perception.qr_codes:
                if self.target_qr_id is None or qr["id"] == self.target_qr_id:
                    log.info(f"Target QR Verified: {qr['id']}")
                    self.transition_to(R2D2MissionState.ALIGN_FOR_DROP)
                    break
            if elapsed > 12.0:
                self.transition_to(R2D2MissionState.ALIGN_FOR_DROP)

        elif self.state == R2D2MissionState.ALIGN_FOR_DROP:
            if elapsed > 4.0:
                self.transition_to(R2D2MissionState.DROP_PAYLOAD)

        elif self.state == R2D2MissionState.DROP_PAYLOAD:
            if elapsed > 8.0:  # Full winch lowering + release + reel sequence
                self.transition_to(R2D2MissionState.RETURN)

        elif self.state == R2D2MissionState.RETURN:
            if pose.position[0] <= 1.0 or elapsed > 15.0:
                self.transition_to(R2D2MissionState.LAND)

        elif self.state == R2D2MissionState.LAND:
            if abs(pose.position[2]) < 0.2 or elapsed > 10.0:
                self.transition_to(R2D2MissionState.COMPLETE)

        return self.state
