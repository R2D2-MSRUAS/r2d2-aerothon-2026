"""Acknowledged command mapping for the Windows SITL profile."""

from __future__ import annotations

from time import monotonic
from typing import Dict, Optional, Tuple

from interfaces.command_request import CommandRequest
from interfaces.command_result import CommandResult, CommandStatus


class CommandManager:
    def __init__(self, commands_enabled: bool = False) -> None:
        self._commands_enabled = commands_enabled
        self._pending: Dict[int, str] = {}
        self._results: Dict[str, CommandResult] = {}

    def result_for(self, request_id: str) -> Optional[CommandResult]:
        return self._results.get(request_id)

    def handle_ack(self, mav_command: int, mav_result: int, observed_at: Optional[float] = None) -> Optional[CommandResult]:
        """Correlate a COMMAND_ACK with the most recent matching request.

        ACK acceptance proves only that ArduPilot accepted the command.  The
        mission runner still waits for state telemetry (armed, altitude, or
        disarm) before considering a physical action complete.
        """

        request_id = self._pending.pop(mav_command, None)
        if request_id is None:
            return None
        timestamp = monotonic() if observed_at is None else observed_at
        status = CommandStatus.SUCCEEDED if mav_result == 0 else CommandStatus.IN_PROGRESS if mav_result == 5 else CommandStatus.FAILED
        result = CommandResult(request_id, status, timestamp, mav_result=mav_result)
        self._results[request_id] = result
        return result

    _COPTER_MODES = {"GUIDED": 4, "LOITER": 5, "RTL": 6, "LAND": 9}

    def submit(self, request: CommandRequest, adapter=None, target: Optional[Tuple[int, int]] = None) -> CommandResult:
        if not self._commands_enabled:
            result = CommandResult(
                request_id=request.request_id,
                status=CommandStatus.REJECTED,
                observed_at=monotonic(),
                detail="commands are disabled by this MAVLink profile",
            )
            self._results[request.request_id] = result
            return result
        if adapter is None or target is None:
            result = CommandResult(
                request_id=request.request_id,
                status=CommandStatus.REJECTED,
                observed_at=monotonic(),
                detail="a bound flight-controller identity is required",
            )
            self._results[request.request_id] = result
            return result
        try:
            mav_command = self._send(request, adapter, target)
        except (TypeError, ValueError) as error:
            result = CommandResult(request.request_id, CommandStatus.REJECTED, monotonic(), detail=str(error))
            self._results[request.request_id] = result
            return result
        if mav_command is not None and request.requires_ack:
            self._pending[mav_command] = request.request_id
        result = CommandResult(
            request_id=request.request_id,
            status=CommandStatus.ACCEPTED,
            observed_at=monotonic(),
            detail="sent; wait for telemetry before treating the action as complete",
        )
        self._results[request.request_id] = result
        return result

    def _send(self, request: CommandRequest, adapter, target: Tuple[int, int]) -> Optional[int]:
        from pymavlink import mavutil

        action = request.action
        parameters = request.parameters
        if action is CommandAction.SET_MODE:
            name = str(parameters.get("mode", "")).upper()
            if name not in self._COPTER_MODES:
                raise ValueError(f"mode must be one of {', '.join(self._COPTER_MODES)}")
            adapter.send_mode(target, self._COPTER_MODES[name])
            return None
        elif action is CommandAction.ARM:
            adapter.send_command_long(target, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, (1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
            return mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
        elif action is CommandAction.DISARM:
            adapter.send_command_long(target, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
            return mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
        elif action is CommandAction.TAKEOFF:
            altitude = float(parameters["altitude_m"])
            if not 0.1 <= altitude <= 30.0:
                raise ValueError("takeoff altitude must be between 0.1 and 30 m in SITL")
            adapter.send_command_long(target, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, altitude))
            return mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
        elif action is CommandAction.LAND:
            adapter.send_command_long(target, mavutil.mavlink.MAV_CMD_NAV_LAND, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
            return mavutil.mavlink.MAV_CMD_NAV_LAND
        elif action is CommandAction.RTL:
            adapter.send_command_long(target, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
            return mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH
        elif action is CommandAction.YAW:
            angle_deg = float(parameters.get("angle_deg", 180.0))
            rate_dps = float(parameters.get("rate_dps", 25.0))
            if not 0.0 < abs(angle_deg) <= 360.0 or not 0.0 < rate_dps <= 90.0:
                raise ValueError("yaw angle must be within 0-360 degrees and rate within 0-90 deg/s")
            adapter.send_command_long(
                target,
                mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                (abs(angle_deg), rate_dps, 1.0 if angle_deg >= 0 else -1.0, 1.0, 0.0, 0.0, 0.0),
            )
            return mavutil.mavlink.MAV_CMD_CONDITION_YAW
        elif action is CommandAction.WINCH:
            raise ValueError("winch actuation is intentionally unavailable in the SITL mission runner")
        else:  # pragma: no cover - protects future enum extensions
            raise ValueError(f"unsupported command action: {action}")
