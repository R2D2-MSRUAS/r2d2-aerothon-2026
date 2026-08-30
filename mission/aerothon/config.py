"""
AeroTHON 2026 — Configuration & Mission Parameters
================================================================================
Defines all flight parameters, competition dimensions, timeout limits, and FSM states.
Hardware Stack: NVIDIA Jetson Orin Nano Super, Holybro Pixhawk 6X Mini, 3-Camera Payload,
Pololu DRV8838 + N20 100RPM Winch Driver, GEPRC 2806.5 Motors on 6S LiPo.
"""

from __future__ import annotations

import dataclasses
from enum import Enum, auto


class MissionState(Enum):
    """Complete Mission State Machine including Forward & Backward passes."""
    INIT                    = auto()
    TAKEOFF                 = auto()
    SCAN_START_QR           = auto()
    FIND_GREEN_BANNER       = auto()
    ENTER_CORRIDOR          = auto()
    FOLLOW_CORRIDOR         = auto()
    REACH_DELIVERY          = auto()
    SCAN_TARGET_QR          = auto()
    ALIGN_PAYLOAD           = auto()
    DROP_PAYLOAD            = auto()
    ASCEND_RETURN           = auto()
    ALIGN_RETURN_CORRIDOR   = auto()
    FOLLOW_CORRIDOR_RETURN  = auto()
    EXIT_CORRIDOR           = auto()
    APPROACH_HOME           = auto()
    PRECISION_LAND          = auto()
    AVOID_OBSTACLES         = auto()
    ABORT_RTL               = auto()
    COMPLETE                = auto()


@dataclasses.dataclass(frozen=True)
class MissionConfig:
    """Mission Parameters as mandated by SAEINDIA AeroTHON 2026 Rulebook."""
    # Hardware Profile
    COMPANION_COMPUTER      : str   = "NVIDIA Jetson Orin Nano Super Developer Kit (R199100)"
    AUTOPILOT               : str   = "Holybro Pixhawk 6X Mini (R110841)"
    CAMERA_SETUP            : str   = "3-Camera System (OAK-D Lite + Arducam IMX477 HQ + Skydroid FPV)"

    # Altitudes (meters above ground)
    TAKEOFF_ALT_M           : float = 5.0    # 5.0m takeoff & initial QR scan
    CORRIDOR_ALT_M          : float = 3.048  # 10 ft (~3.05m) corridor navigation altitude
    DELIVERY_SEARCH_ALT_M   : float = 10.0   # 10.0m delivery zone QR search altitude
    PAYLOAD_DROP_ALT_M      : float = 5.0    # 5.0m precision winch drop altitude
    RETURN_CRUISE_ALT_M     : float = 10.0   # 10.0m return climb altitude
    RETURN_CORRIDOR_ALT_M   : float = 3.048  # 10 ft return corridor altitude

    # Velocities (m/s)
    CRUISE_SPEED_MPS        : float = 1.8    # Standard cruise speed
    CORRIDOR_SPEED_MPS      : float = 1.5    # Controlled corridor traverse speed
    SLOW_SPEED_MPS          : float = 0.8    # Approach and slow maneuver speed
    MAX_SPEED_MPS           : float = 2.5    # Safety ceiling for body velocity commands
    YAW_RATE_DPS            : float = 25.0   # Maximum yaw turn rate (deg/s)

    # Obstacle & Proximity Distances (meters)
    OBS_SAFE_DIST_M         : float = 2.0    # > 2.0m: Full cruise speed
    OBS_SLOW_DIST_M         : float = 1.2    # 1.2m - 2.0m: Half speed & lateral steer
    OBS_CRITICAL_STOP_M     : float = 0.8    # < 0.8m: Immediate emergency stop & lateral evasion
    CORRIDOR_WIDTH_M        : float = 3.5    # Specified arena corridor width
    CORRIDOR_LENGTH_EST_M   : float = 15.0   # Nominal corridor length

    # Tolerances
    ALTITUDE_TOLERANCE_M    : float = 0.30   # Altitude convergence threshold
    POSITION_TOLERANCE_M    : float = 0.35   # Waypoint convergence radius
    HEADING_TOLERANCE_DEG   : float = 5.0    # Yaw alignment tolerance

    # Loop & Flight Timeouts (seconds) — Prevents infinite hangs in hardware loops
    ARM_TIMEOUT_S           : float = 10.0   # Max seconds to wait for FC arming
    TAKEOFF_TIMEOUT_S       : float = 25.0   # Max seconds for takeoff altitude climb
    ALTITUDE_CHANGE_TIMEOUT_S: float = 18.0  # Max seconds for in-flight altitude changes
    LANDING_TIMEOUT_S       : float = 30.0   # Max seconds for touchdown descent
    FC_CONNECTION_TIMEOUT_S : float = 30.0   # MAVLink heartbeat connection timeout
    MAX_MISSION_TIME_S      : float = 900.0  # 15 minutes competition time limit (rulebook §4.2.4)

    # Winch & Payload Drop Mechanism (Pololu DRV8838 Driver + N20 100RPM Motor)
    WINCH_SERVO_CHANNEL     : int   = 9      # MAVLink AUX servo channel (Pixhawk Pin 9)
    WINCH_LOWER_PWM         : int   = 1700   # Lower hoist PWM (µs)
    WINCH_NEUTRAL_PWM       : int   = 1500   # Stop/Hold PWM (µs)
    WINCH_RAISE_PWM         : int   = 1300   # Retract cable PWM (µs)
    WINCH_RELEASE_PWM       : int   = 1900   # Payload detach trigger PWM (µs)
    WINCH_LOWER_TIME_S      : float = 4.5    # Lowering duration
    WINCH_HOLD_TIME_S       : float = 1.5    # Ground contact settle duration
    WINCH_REEL_TIME_S       : float = 4.5    # Cable retraction duration

    # Mass & Center of Gravity (CoG) Compensation
    PAYLOAD_MASS_KG         : float = 0.100  # 100g specified payload mass
    DRONE_EMPTY_MASS_KG     : float = 1.850  # Dry weight of UAS (< 2.0kg MTOW)
    PAYLOAD_COG_OFFSET_X_M  : float = 0.12   # Longitudinal distance from CoG to payload mount

    # Vision & 3-Camera Parameters
    QR_CONFIRM_COUNT        : int   = 3      # Consecutive frames to confirm delivery QR
    QR_SCAN_TIMEOUT_S       : float = 10.0   # Time before starting yaw scan pattern


DEFAULT_CONFIG = MissionConfig()
