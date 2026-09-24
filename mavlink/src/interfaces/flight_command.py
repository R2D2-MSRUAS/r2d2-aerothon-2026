"""Navigation-facing flight-command contracts.

v0.1 validates and stores these commands only. It does not send setpoints.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from math import isfinite
from typing import Optional


class CoordinateFrame(str, Enum):
    BODY_NED = "BODY_NED"
    LOCAL_NED = "LOCAL_NED"


class FlightCommandKind(str, Enum):
    HOLD = "HOLD"
    VELOCITY = "VELOCITY"
    POSITION = "POSITION"
    ATTITUDE = "ATTITUDE"


@dataclass(frozen=True)
class Vector3:
    """A vector whose physical unit and coordinate frame are defined by its parent field."""

    x: float
    y: float
    z: float

    def is_finite(self) -> bool:
        return all(isfinite(value) for value in (self.x, self.y, self.z))


@dataclass(frozen=True)
class FlightCommand:
    command_id: str
    created_at: float
    valid_until: float
    kind: FlightCommandKind
    frame: CoordinateFrame
    position_m: Optional[Vector3] = None
    velocity_mps: Optional[Vector3] = None
    acceleration_mps2: Optional[Vector3] = None
    yaw_rad: Optional[float] = None
    yaw_rate_rps: Optional[float] = None
    source: str = "navigation"

    def validate(self, now: float) -> Optional[str]:
        """Return a human-readable validation error, otherwise ``None``."""

        if not self.command_id:
            return "command_id is required"
        if self.valid_until <= self.created_at:
            return "valid_until must be later than created_at"
        if now > self.valid_until:
            return "flight command has expired"
        if self.kind == FlightCommandKind.VELOCITY and self.velocity_mps is None:
            return "velocity commands require velocity_mps"
        if self.kind == FlightCommandKind.POSITION and self.position_m is None:
            return "position commands require position_m"
        for name, value in (
            ("position_m", self.position_m),
            ("velocity_mps", self.velocity_mps),
            ("acceleration_mps2", self.acceleration_mps2),
        ):
            if value is not None and not value.is_finite():
                return f"{name} must contain finite values"
        for name, value in (("yaw_rad", self.yaw_rad), ("yaw_rate_rps", self.yaw_rate_rps)):
            if value is not None and not isfinite(value):
                return f"{name} must be finite"
        return None
