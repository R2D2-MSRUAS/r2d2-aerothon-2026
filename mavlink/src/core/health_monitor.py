"""Freshness-based health evaluation. No safety action is issued here."""

from __future__ import annotations

from typing import Optional

from interfaces.mavlink_health import HealthStatus, MavlinkHealth


class HealthMonitor:
    def __init__(self, heartbeat_stale_s: float, telemetry_stale_s: float) -> None:
        self._heartbeat_stale_s = heartbeat_stale_s
        self._telemetry_stale_s = telemetry_stale_s
        self._transport_connected = False
        self._last_heartbeat_at: Optional[float] = None
        self._last_telemetry_at: Optional[float] = None
        self._last_setpoint_at: Optional[float] = None
        self._last_command_ack_at: Optional[float] = None
        self._last_command_result: Optional[int] = None
        self._last_error: Optional[str] = None

    def mark_transport(self, connected: bool, error: Optional[str] = None) -> None:
        self._transport_connected = connected
        self._last_error = error

    def mark_heartbeat(self, timestamp: float) -> None:
        self._last_heartbeat_at = timestamp

    def mark_telemetry(self, timestamp: float) -> None:
        self._last_telemetry_at = timestamp

    def mark_setpoint(self, timestamp: float) -> None:
        self._last_setpoint_at = timestamp

    def mark_command_ack(self, timestamp: float, result: int) -> None:
        self._last_command_ack_at = timestamp
        self._last_command_result = result

    def evaluate(self, now: float) -> MavlinkHealth:
        heartbeat_age = _age(now, self._last_heartbeat_at)
        telemetry_age = _age(now, self._last_telemetry_at)
        setpoint_age = _age(now, self._last_setpoint_at)
        reasons = []
        
        if not self._transport_connected:
            status = HealthStatus.DISCONNECTED
            reasons.append("transport is disconnected")
        elif heartbeat_age is None:
            status = HealthStatus.CONNECTING
            reasons.append("waiting for flight-controller heartbeat")
        elif heartbeat_age > self._heartbeat_stale_s:
            status = HealthStatus.STALE
            reasons.append("flight-controller heartbeat is stale")
        elif telemetry_age is None:
            status = HealthStatus.DEGRADED
            reasons.append("waiting for non-heartbeat telemetry")
        elif telemetry_age > self._telemetry_stale_s:
            status = HealthStatus.STALE
            reasons.append("required telemetry is stale")
        else:
            status = HealthStatus.CONNECTED
        return MavlinkHealth(
            status=status,
            observed_at=now,
            transport_connected=self._transport_connected,
            heartbeat_age_s=heartbeat_age,
            telemetry_age_s=telemetry_age,
            setpoint_age_s=setpoint_age,
            last_error=self._last_error,
            reasons=tuple(reasons),
        )


def _age(now: float, timestamp: Optional[float]) -> Optional[float]:
    return None if timestamp is None else max(0.0, now - timestamp)
