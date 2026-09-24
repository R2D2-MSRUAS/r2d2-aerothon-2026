"""Single-writer, immutable vehicle-state snapshot manager."""

from __future__ import annotations

from dataclasses import replace
from typing import Any, Optional

from interfaces.vehicle_state import VehicleState


class StateManager:
    def __init__(self, initial_state: VehicleState) -> None:
        self._state = initial_state

    @property
    def state(self) -> VehicleState:
        return self._state

    def update(self, timestamp: float, group: str, **changes: Any) -> VehicleState:
        timestamps = dict(self._state.field_timestamps)
        timestamps[group] = timestamp
        self._state = replace(
            self._state,
            observed_at=timestamp,
            field_timestamps=timestamps,
            **changes,
        )
        return self._state

    def append_statustext(self, timestamp: float, text: str, max_entries: int = 20) -> VehicleState:
        messages = (*self._state.statustext, text)[-max_entries:]
        return self.update(timestamp, "statustext", statustext=messages)

    def set_connected(self, timestamp: float, connected: bool, error: Optional[str] = None) -> VehicleState:
        return self.update(timestamp, "connection", connected=connected)
