"""pymavlink UDP transport used for ArduPilot SITL."""

from __future__ import annotations

from typing import Any, Optional


class UdpTransport:
    def __init__(self, endpoint: str, source_system_id: int, source_component_id: int) -> None:
        self.endpoint = endpoint
        self.source_system_id = source_system_id
        self.source_component_id = source_component_id
        self._connection: Optional[Any] = None

    @property
    def connected(self) -> bool:
        return self._connection is not None

    def open(self) -> None:
        if self._connection is not None:
            return
        from pymavlink import mavutil

        self._connection = mavutil.mavlink_connection(
            self.endpoint,
            source_system=self.source_system_id,
            source_component=self.source_component_id,
            autoreconnect=True,
        )

    def receive(self) -> Optional[Any]:
        if self._connection is None:
            raise RuntimeError("UDP transport is not open")
        return self._connection.recv_match(blocking=False)

    @property
    def connection(self) -> Any:
        if self._connection is None:
            raise RuntimeError("UDP transport is not open")
        return self._connection

    def close(self) -> None:
        if self._connection is not None:
            self._connection.close()
            self._connection = None
