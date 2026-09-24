"""Validated runtime configuration.

PyYAML is imported only when configuration is loaded so pure contract tests do
not need a MAVLink or YAML runtime.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional


class ConfigError(ValueError):
    pass


@dataclass(frozen=True)
class ConnectionConfig:
    transport: str
    endpoint: str
    source_system_id: int = 245
    source_component_id: int = 190
    expected_system_id: Optional[int] = None
    expected_component_id: Optional[int] = None


@dataclass(frozen=True)
class TelemetryConfig:
    heartbeat_stale_s: float = 3.0
    telemetry_stale_s: float = 5.0


@dataclass(frozen=True)
class SafetyConfig:
    setpoint_expiry_s: float = 0.5
    setpoint_output_rate_hz: float = 5.0
    max_speed_mps: float = 2.5
    commands_enabled: bool = False
    automatic_actions_enabled: bool = False


@dataclass(frozen=True)
class MissionConfig:
    """Mission-2 values used only by the Windows SITL runner.

    This profile is deliberately separate from an aircraft configuration.  It is
    a repeatable test scenario, not a substitute for perception or a flight
    release configuration.
    """

    enabled: bool = False
    simulation_only: bool = True
    max_mission_time_s: float = 900.0
    takeoff_alt_m: float = 5.0
    corridor_alt_m: float = 3.048
    delivery_search_alt_m: float = 10.0
    payload_drop_alt_m: float = 5.0
    return_cruise_alt_m: float = 10.0
    corridor_length_m: float = 15.0
    cruise_speed_mps: float = 1.8
    corridor_speed_mps: float = 1.5
    slow_speed_mps: float = 0.8
    yaw_rate_dps: float = 25.0
    takeoff_timeout_s: float = 25.0
    altitude_tolerance_m: float = 0.3


@dataclass(frozen=True)
class MavlinkConfig:
    connection: ConnectionConfig
    telemetry: TelemetryConfig = TelemetryConfig()
    safety: SafetyConfig = SafetyConfig()
    mission: MissionConfig = MissionConfig()


def _require_mapping(value: Any, label: str) -> Dict[str, Any]:
    if not isinstance(value, dict):
        raise ConfigError(f"{label} must be a mapping")
    return value


def _positive_id(value: Any, label: str) -> Optional[int]:
    if value is None:
        return None
    if not isinstance(value, int) or not 1 <= value <= 255:
        raise ConfigError(f"{label} must be an integer from 1 to 255")
    return value


def load_config(path: str | Path) -> MavlinkConfig:
    """Load one complete YAML profile, suitable for SITL or hardware."""

    profile_path = Path(path)
    raw = _require_mapping(_load_yaml(profile_path), "configuration")

    connection = _require_mapping(raw.get("connection"), "connection")
    transport = connection.get("transport")
    endpoint = connection.get("endpoint")
    if transport not in {"udp", "serial"}:
        raise ConfigError("connection.transport must be 'udp' or 'serial'")
    if not isinstance(endpoint, str) or not endpoint:
        raise ConfigError("connection.endpoint is required")

    telemetry = _require_mapping(raw.get("telemetry", {}), "telemetry")
    safety = _require_mapping(raw.get("safety", {}), "safety")
    mission = _require_mapping(raw.get("mission", {}), "mission")
    result = MavlinkConfig(
        connection=ConnectionConfig(
            transport=transport,
            endpoint=endpoint,
            source_system_id=_positive_id(connection.get("source_system_id", 245), "source_system_id") or 245,
            source_component_id=_positive_id(connection.get("source_component_id", 190), "source_component_id") or 190,
            expected_system_id=_positive_id(connection.get("expected_system_id"), "expected_system_id"),
            expected_component_id=_positive_id(connection.get("expected_component_id"), "expected_component_id"),
        ),
        telemetry=TelemetryConfig(
            heartbeat_stale_s=float(telemetry.get("heartbeat_stale_s", 3.0)),
            telemetry_stale_s=float(telemetry.get("telemetry_stale_s", 5.0)),
        ),
        safety=SafetyConfig(
            setpoint_expiry_s=float(safety.get("setpoint_expiry_s", 0.5)),
            setpoint_output_rate_hz=float(safety.get("setpoint_output_rate_hz", 5.0)),
            max_speed_mps=float(safety.get("max_speed_mps", 2.5)),
            commands_enabled=bool(safety.get("commands_enabled", False)),
            automatic_actions_enabled=bool(safety.get("automatic_actions_enabled", False)),
        ),
        mission=MissionConfig(
            enabled=bool(mission.get("enabled", False)),
            simulation_only=bool(mission.get("simulation_only", True)),
            max_mission_time_s=float(mission.get("max_mission_time_s", 900.0)),
            takeoff_alt_m=float(mission.get("takeoff_alt_m", 5.0)),
            corridor_alt_m=float(mission.get("corridor_alt_m", 3.048)),
            delivery_search_alt_m=float(mission.get("delivery_search_alt_m", 10.0)),
            payload_drop_alt_m=float(mission.get("payload_drop_alt_m", 5.0)),
            return_cruise_alt_m=float(mission.get("return_cruise_alt_m", 10.0)),
            corridor_length_m=float(mission.get("corridor_length_m", 15.0)),
            cruise_speed_mps=float(mission.get("cruise_speed_mps", 1.8)),
            corridor_speed_mps=float(mission.get("corridor_speed_mps", 1.5)),
            slow_speed_mps=float(mission.get("slow_speed_mps", 0.8)),
            yaw_rate_dps=float(mission.get("yaw_rate_dps", 25.0)),
            takeoff_timeout_s=float(mission.get("takeoff_timeout_s", 25.0)),
            altitude_tolerance_m=float(mission.get("altitude_tolerance_m", 0.3)),
        ),
    )
    if result.telemetry.heartbeat_stale_s <= 0 or result.telemetry.telemetry_stale_s <= 0:
        raise ConfigError("telemetry stale thresholds must be positive")
    if result.safety.setpoint_expiry_s <= 0:
        raise ConfigError("safety.setpoint_expiry_s must be positive")
    if result.safety.setpoint_output_rate_hz <= 0 or result.safety.max_speed_mps <= 0:
        raise ConfigError("setpoint output rate and max speed must be positive")
    if result.mission.enabled and not result.mission.simulation_only:
        raise ConfigError("the included Mission 2 runner is limited to simulation_only profiles")
    if result.mission.max_mission_time_s <= 0 or result.mission.corridor_length_m <= 0:
        raise ConfigError("mission time and corridor length must be positive")
    return result


def _load_yaml(path: Path) -> Any:
    """Load YAML with PyYAML when installed, or a small v0.1-profile fallback.

    The fallback handles mappings and scalar values used by the committed sample
    profiles. Production deployments should install the pinned dependencies.
    """

    try:
        import yaml
    except ImportError:
        return _parse_simple_yaml(path.read_text(encoding="utf-8"))
    with path.open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def _parse_simple_yaml(text: str) -> Dict[str, Any]:
    root: Dict[str, Any] = {}
    stack: list[tuple[int, Dict[str, Any]]] = [(-1, root)]
    for number, raw_line in enumerate(text.splitlines(), start=1):
        line = raw_line.split("#", 1)[0].rstrip()
        if not line.strip():
            continue
        indentation = len(line) - len(line.lstrip(" "))
        if "\t" in raw_line[:indentation] or ":" not in line:
            raise ConfigError(f"unsupported YAML syntax at line {number}")
        key, value = (part.strip() for part in line.split(":", 1))
        while indentation <= stack[-1][0]:
            stack.pop()
        parent = stack[-1][1]
        if not value:
            child: Dict[str, Any] = {}
            parent[key] = child
            stack.append((indentation, child))
        else:
            parent[key] = _scalar(value)
    return root


def _scalar(value: str) -> Any:
    if value in {"null", "Null", "NULL", "~"}:
        return None
    if value.lower() in {"true", "false"}:
        return value.lower() == "true"
    try:
        return float(value) if "." in value else int(value)
    except ValueError:
        return value.strip("\"'")
