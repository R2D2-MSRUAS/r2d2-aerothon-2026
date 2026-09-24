"""MAVLink-message normalization into ``VehicleState``."""

from __future__ import annotations

from dataclasses import replace
from typing import Any, Optional

from interfaces.flight_command import Vector3
from interfaces.vehicle_state import AttitudeState, BatteryState, EkfState, GpsState, PositionState

from .health_monitor import HealthMonitor
from .state_manager import StateManager

MAV_MODE_FLAG_SAFETY_ARMED = 128


class TelemetryManager:
    def __init__(self, state_manager: StateManager, health_monitor: HealthMonitor) -> None:
        self._state_manager = state_manager
        self._health_monitor = health_monitor

    def handle(self, message: Any, received_at: float) -> bool:
        """Normalize one decoded pymavlink message. Returns whether it was consumed."""

        message_type = message.get_type()
        if message_type == "BAD_DATA":
            return False
        source_system = _source_id(message, "get_srcSystem")
        source_component = _source_id(message, "get_srcComponent")
        if message_type == "HEARTBEAT":
            self._health_monitor.mark_heartbeat(received_at)
            base_mode = _value(message, "base_mode", 0)
            custom_mode = _value(message, "custom_mode")
            self._state_manager.update(
                received_at,
                "heartbeat",
                connected=True,
                source_system=source_system,
                source_component=source_component,
                armed=bool(base_mode & MAV_MODE_FLAG_SAFETY_ARMED),
                mode=None if custom_mode is None else f"custom:{custom_mode}",
            )
            return True

        if message_type == "GLOBAL_POSITION_INT":
            current = self._state_manager.state.position or PositionState()
            position = replace(
                current,
                latitude_deg=_scaled(_value(message, "lat"), 1e-7),
                longitude_deg=_scaled(_value(message, "lon"), 1e-7),
                absolute_altitude_m=_scaled(_value(message, "alt"), 1e-3),
                relative_altitude_m=_scaled(_value(message, "relative_alt"), 1e-3),
            )
            self._update_telemetry(received_at, "position", position=position)
            return True

        if message_type == "LOCAL_POSITION_NED":
            current = self._state_manager.state.position or PositionState()
            position = replace(current, local_ned_m=Vector3(_value(message, "x", 0.0), _value(message, "y", 0.0), _value(message, "z", 0.0)))
            velocity = Vector3(_value(message, "vx", 0.0), _value(message, "vy", 0.0), _value(message, "vz", 0.0))
            self._update_telemetry(received_at, "local_position", position=position, velocity_ned_mps=velocity)
            return True

        if message_type == "ATTITUDE":
            attitude = AttitudeState(
                roll_rad=_value(message, "roll"),
                pitch_rad=_value(message, "pitch"),
                yaw_rad=_value(message, "yaw"),
                roll_rate_rps=_value(message, "rollspeed"),
                pitch_rate_rps=_value(message, "pitchspeed"),
                yaw_rate_rps=_value(message, "yawspeed"),
            )
            self._update_telemetry(received_at, "attitude", attitude=attitude)
            return True

        if message_type in {"SYS_STATUS", "BATTERY_STATUS"}:
            current = self._state_manager.state.battery or BatteryState()
            battery = _battery_from(message, current)
            self._update_telemetry(received_at, "battery", battery=battery)
            return True

        if message_type == "GPS_RAW_INT":
            gps = GpsState(
                fix_type=_value(message, "fix_type"),
                satellites_visible=_value(message, "satellites_visible"),
                eph_m=_valid_scaled(_value(message, "eph"), 1e-2),
                epv_m=_valid_scaled(_value(message, "epv"), 1e-2),
            )
            self._update_telemetry(received_at, "gps", gps=gps)
            return True

        if message_type == "EKF_STATUS_REPORT":
            self._update_telemetry(received_at, "ekf", ekf=EkfState(flags=_value(message, "flags")))
            return True

        if message_type == "STATUSTEXT":
            raw_text = _value(message, "text", "")
            text = raw_text.decode(errors="replace").rstrip("\x00") if isinstance(raw_text, bytes) else str(raw_text)
            self._health_monitor.mark_telemetry(received_at)
            self._state_manager.append_statustext(received_at, text)
            return True

        return False

    def _update_telemetry(self, timestamp: float, group: str, **changes: Any) -> None:
        self._health_monitor.mark_telemetry(timestamp)
        self._state_manager.update(timestamp, group, **changes)


def _source_id(message: Any, method_name: str) -> Optional[int]:
    method = getattr(message, method_name, None)
    return method() if callable(method) else None


def _value(message: Any, name: str, default: Any = None) -> Any:
    return getattr(message, name, default)


def _scaled(value: Any, multiplier: float) -> Optional[float]:
    return None if value is None else float(value) * multiplier


def _valid_scaled(value: Any, multiplier: float) -> Optional[float]:
    return None if value in (None, 65535, 65534) else float(value) * multiplier


def _battery_from(message: Any, current: BatteryState) -> BatteryState:
    if message.get_type() == "SYS_STATUS":
        voltage = _valid_scaled(_value(message, "voltage_battery"), 1e-3)
        current_a = _valid_scaled(_value(message, "current_battery"), 1e-2)
        remaining = _value(message, "battery_remaining")
    else:
        voltages = _value(message, "voltages", ()) or ()
        valid_voltages = [value for value in voltages if value not in (None, 0, 65535)]
        voltage = sum(valid_voltages) * 1e-3 if valid_voltages else current.voltage_v
        current_a = _valid_scaled(_value(message, "current_battery"), 1e-2)
        remaining = _value(message, "battery_remaining")
    return BatteryState(
        voltage_v=voltage if voltage is not None else current.voltage_v,
        current_a=current_a if current_a is not None else current.current_a,
        remaining_percent=None if remaining in (None, -1, 255) else int(remaining),
    )
