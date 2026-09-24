"""Connection and telemetry freshness contract."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, Tuple


class HealthStatus(str, Enum):
    CONNECTING = "CONNECTING"
    CONNECTED = "CONNECTED"
    DEGRADED = "DEGRADED"
    STALE = "STALE"
    DISCONNECTED = "DISCONNECTED"


@dataclass(frozen=True)
class MavlinkHealth:
    status: HealthStatus
    observed_at: float
    transport_connected: bool
    heartbeat_age_s: Optional[float]
    telemetry_age_s: Optional[float]
    setpoint_age_s: Optional[float]
    last_error: Optional[str] = None
    reasons: Tuple[str, ...] = field(default_factory=tuple)
