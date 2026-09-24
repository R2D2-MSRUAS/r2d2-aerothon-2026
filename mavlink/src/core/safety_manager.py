"""Safety-policy shell. It observes health but takes no autonomous action in v0.1."""

from __future__ import annotations

from interfaces.mavlink_health import HealthStatus, MavlinkHealth


class SafetyManager:
    def __init__(self, automatic_actions_enabled: bool = False) -> None:
        self.automatic_actions_enabled = automatic_actions_enabled

    def summary(self, health: MavlinkHealth) -> str | None:
        if health.status in {HealthStatus.STALE, HealthStatus.DISCONNECTED}:
            return "; ".join(health.reasons)
        return None
