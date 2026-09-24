"""Thin boundary around pymavlink.

This class deliberately exposes decoded messages only to the telemetry manager;
the rest of the autonomy code never imports pymavlink.
"""

from __future__ import annotations

from typing import Any, Optional, Tuple

from transport.udp_transport import UdpTransport


class PymavlinkAdapter:
    def __init__(self, transport: UdpTransport) -> None:
        self._transport = transport

    @property
    def connected(self) -> bool:
        return self._transport.connected

    def open(self) -> None:
        self._transport.open()

    def receive(self) -> Optional[Any]:
        return self._transport.receive()

    def close(self) -> None:
        self._transport.close()

    def send_command_long(self, target: Tuple[int, int], command: int, parameters: tuple[float, ...]) -> None:
        """Send one MAVLink COMMAND_LONG after the FC identity has been bound."""

        if len(parameters) != 7:
            raise ValueError("COMMAND_LONG requires exactly seven parameters")
        target_system, target_component = target
        self._transport.connection.mav.command_long_send(
            target_system, target_component, command, 0, *parameters
        )

    def send_mode(self, target: Tuple[int, int], custom_mode: int) -> None:
        from pymavlink import mavutil

        self._transport.connection.mav.set_mode_send(
            target[0], mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, custom_mode
        )

    def send_velocity_setpoint(self, target: Tuple[int, int], frame: str, velocity: Any) -> None:
        """Send a velocity-only SET_POSITION_TARGET_LOCAL_NED command.

        ArduPilot accepts this in GUIDED mode.  BODY_NED velocities are forward,
        right, down; LOCAL_NED velocities are north, east, down.
        """

        from pymavlink import mavutil

        frame_map = {
            "BODY_NED": mavutil.mavlink.MAV_FRAME_BODY_NED,
            "LOCAL_NED": mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        }
        try:
            mav_frame = frame_map[frame]
        except KeyError as error:
            raise ValueError(f"unsupported setpoint frame: {frame}") from error
        self._transport.connection.mav.set_position_target_local_ned_send(
            0,
            target[0],
            target[1],
            mav_frame,
            0x0DC7,  # velocity only; position, acceleration, yaw and yaw rate ignored
            0.0,
            0.0,
            0.0,
            velocity.x,
            velocity.y,
            velocity.z,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )
