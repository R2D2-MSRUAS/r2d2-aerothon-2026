"""Immutable, normalized flight-controller state."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Mapping, Optional, Tuple

from .flight_command import Vector3


@dataclass(frozen=True)
class PositionState:
    latitude_deg: Optional[float] = None
    longitude_deg: Optional[float] = None
    absolute_altitude_m: Optional[float] = None
    relative_altitude_m: Optional[float] = None
    local_ned_m: Optional[Vector3] = None


@dataclass(frozen=True)
class AttitudeState:
    roll_rad: Optional[float] = None
    pitch_rad: Optional[float] = None
    yaw_rad: Optional[float] = None
    roll_rate_rps: Optional[float] = None
    pitch_rate_rps: Optional[float] = None
    yaw_rate_rps: Optional[float] = None


@dataclass(frozen=True)
class BatteryState:
    voltage_v: Optional[float] = None
    current_a: Optional[float] = None
    remaining_percent: Optional[int] = None


@dataclass(frozen=True)
class GpsState:
    fix_type: Optional[int] = None
    satellites_visible: Optional[int] = None
    eph_m: Optional[float] = None
    epv_m: Optional[float] = None


@dataclass(frozen=True)
class EkfState:
    flags: Optional[int] = None


@dataclass(frozen=True)
class VehicleState:
    observed_at: float
    source_system: Optional[int] = None
    source_component: Optional[int] = None
    connected: bool = False
    armed: Optional[bool] = None
    mode: Optional[str] = None
    position: Optional[PositionState] = None
    velocity_ned_mps: Optional[Vector3] = None
    attitude: Optional[AttitudeState] = None
    battery: Optional[BatteryState] = None
    gps: Optional[GpsState] = None
    ekf: Optional[EkfState] = None
    field_timestamps: Mapping[str, float] = field(default_factory=dict)
    statustext: Tuple[str, ...] = field(default_factory=tuple)
