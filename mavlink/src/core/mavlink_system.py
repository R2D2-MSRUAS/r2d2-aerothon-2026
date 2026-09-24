"""Top-level heartbeat → telemetry → state orchestration for v0.1."""

from __future__ import annotations

import logging
from time import monotonic, sleep
from typing import Any, Optional, Tuple

from adapters.pymavlink_adapter import PymavlinkAdapter
from interfaces.mavlink_health import MavlinkHealth
from interfaces.command_request import CommandRequest
from interfaces.command_result import CommandResult
from interfaces.flight_command import FlightCommand
from interfaces.vehicle_state import VehicleState
from transport.udp_transport import UdpTransport

from .command_manager import CommandManager
from .config import MavlinkConfig
from .health_monitor import HealthMonitor
from .safety_manager import SafetyManager
from .setpoint_manager import SetpointManager
from .state_manager import StateManager
from .telemetry_manager import TelemetryManager

LOGGER = logging.getLogger(__name__)


class MavlinkSystem:
    def __init__(self, config: MavlinkConfig) -> None:
        if config.connection.transport != "udp":
            raise NotImplementedError("v0.1 is SITL/UDP only; use a UDP profile")
        self.config = config
        transport = UdpTransport(
            endpoint=config.connection.endpoint,
            source_system_id=config.connection.source_system_id,
            source_component_id=config.connection.source_component_id,
        )
        self.adapter = PymavlinkAdapter(transport)
        self.state_manager = StateManager(VehicleState(observed_at=monotonic()))
        self.health_monitor = HealthMonitor(
            heartbeat_stale_s=config.telemetry.heartbeat_stale_s,
            telemetry_stale_s=config.telemetry.telemetry_stale_s,
        )
        self.telemetry_manager = TelemetryManager(self.state_manager, self.health_monitor)
        self.command_manager = CommandManager(config.safety.commands_enabled)
        self.setpoint_manager = SetpointManager(
            output_rate_hz=config.safety.setpoint_output_rate_hz,
            max_speed_mps=config.safety.max_speed_mps,
        )
        self.safety_manager = SafetyManager(config.safety.automatic_actions_enabled)
        self._target: Optional[Tuple[int, int]] = None

    @property
    def vehicle_state(self) -> VehicleState:
        return self.state_manager.state

    @property
    def health(self) -> MavlinkHealth:
        return self.health_monitor.evaluate(monotonic())

    @property
    def target_identity(self) -> Optional[Tuple[int, int]]:
        return self._target

    def connect(self) -> None:
        self.adapter.open()
        self.health_monitor.mark_transport(True)
        LOGGER.info("CONNECTION_OPENED endpoint=%s", self.config.connection.endpoint)

    def close(self) -> None:
        self.adapter.close()
        now = monotonic()
        self.health_monitor.mark_transport(False)
        self.state_manager.set_connected(now, False)
        LOGGER.info("CONNECTION_LOST reason=closed_by_client")

    def poll_once(self) -> bool:
        """Process at most one incoming MAVLink message without blocking."""

        message = self.adapter.receive()
        if message is None:
            return False
        if not self._accept(message):
            return False
        received_at = monotonic()
        if message.get_type() == "COMMAND_ACK":
            ack = self.command_manager.handle_ack(
                getattr(message, "command", -1), getattr(message, "result", -1), received_at
            )
            self.health_monitor.mark_telemetry(received_at)
            if ack is not None:
                LOGGER.info("COMMAND_ACK request_id=%s status=%s result=%s", ack.request_id, ack.status.value, ack.mav_result)
            return ack is not None
        consumed = self.telemetry_manager.handle(message, received_at)
        if consumed:
            LOGGER.debug("TELEMETRY_RECEIVED type=%s", message.get_type())
        return consumed

    def submit_command(self, request: CommandRequest) -> CommandResult:
        return self.command_manager.submit(request, self.adapter, self._target)

    def submit_flight_command(self, command: FlightCommand, now: Optional[float] = None) -> bool:
        accepted = self.setpoint_manager.submit(command, now)
        if accepted:
            self.health_monitor.mark_setpoint(monotonic() if now is None else now)
        return accepted

    def service_setpoint(self, now: Optional[float] = None) -> bool:
        if not self.config.safety.commands_enabled:
            return False
        return self.setpoint_manager.stream_if_due(self.adapter, self._target, now)

    def wait_for_connection(self, timeout_s: float) -> bool:
        deadline = monotonic() + timeout_s
        while monotonic() < deadline:
            self.poll_once()
            if self.target_identity is not None and self.health.status.value in {"CONNECTED", "DEGRADED"}:
                return True
            sleep(0.02)
        return False

    def _accept(self, message: Any) -> bool:
        message_type = message.get_type()
        source_system = _source(message, "get_srcSystem")
        source_component = _source(message, "get_srcComponent")
        if message_type == "HEARTBEAT" and self._target is None:
            if source_system is None or source_component is None:
                LOGGER.warning("HEARTBEAT_REJECTED reason=missing_source_identity")
                return False
            expected_system = self.config.connection.expected_system_id
            expected_component = self.config.connection.expected_component_id
            if (expected_system is not None and source_system != expected_system) or (
                expected_component is not None and source_component != expected_component
            ):
                LOGGER.warning("HEARTBEAT_REJECTED source=%s:%s", source_system, source_component)
                return False
            self._target = (source_system, source_component)
            LOGGER.info("TARGET_BOUND system=%s component=%s", source_system, source_component)
        if self._target is None:
            return False
        return source_system == self._target[0] and source_component == self._target[1]


def _source(message: Any, method_name: str) -> Optional[int]:
    method = getattr(message, method_name, None)
    return method() if callable(method) else None
