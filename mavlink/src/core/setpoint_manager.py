"""Validated, expiry-bounded velocity-setpoint streaming."""

from __future__ import annotations

from time import monotonic
from typing import Optional

from interfaces.flight_command import FlightCommand


class SetpointManager:
    def __init__(self, output_rate_hz: float = 5.0, max_speed_mps: float = 2.5) -> None:
        self._latest: Optional[FlightCommand] = None
        self._last_rejection: Optional[str] = None
        self._output_period_s = 1.0 / output_rate_hz
        self._max_speed_mps = max_speed_mps
        self._last_sent_at: Optional[float] = None

    @property
    def latest(self) -> Optional[FlightCommand]:
        return self._latest

    @property
    def last_rejection(self) -> Optional[str]:
        return self._last_rejection

    def submit(self, command: FlightCommand, now: Optional[float] = None) -> bool:
        error = command.validate(monotonic() if now is None else now)
        if error is None and command.velocity_mps is not None:
            if any(abs(value) > self._max_speed_mps for value in (command.velocity_mps.x, command.velocity_mps.y, command.velocity_mps.z)):
                error = f"velocity exceeds configured {self._max_speed_mps:g} m/s limit"
        if error is not None:
            self._last_rejection = error
            return False
        self._latest = command
        self._last_rejection = None
        return True

    def expire_if_needed(self, now: Optional[float] = None) -> Optional[FlightCommand]:
        if self._latest is None:
            return None
        if (monotonic() if now is None else now) > self._latest.valid_until:
            self._latest = None
        return self._latest

    def stream_if_due(self, adapter, target, now: Optional[float] = None) -> bool:
        """Transmit only a current velocity target; stale input is never replayed."""

        timestamp = monotonic() if now is None else now
        command = self.expire_if_needed(timestamp)
        if command is None or target is None:
            return False
        if self._last_sent_at is not None and timestamp - self._last_sent_at < self._output_period_s:
            return False
        if command.kind == FlightCommandKind.HOLD:
            velocity = Vector3(0.0, 0.0, 0.0)
        elif command.kind == FlightCommandKind.VELOCITY and command.velocity_mps is not None:
            velocity = command.velocity_mps
        else:
            self._last_rejection = f"{command.kind.value} streaming is not implemented"
            return False
        adapter.send_velocity_setpoint(target, command.frame.value, velocity)
        self._last_sent_at = timestamp
        return True
