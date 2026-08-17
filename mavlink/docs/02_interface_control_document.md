# 02 - Interface control document

## Scope

This document defines the Python-facing contracts between Navigation and the MAVLink subsystem. It is an internal abstraction: Navigation must not import `pymavlink` types or construct MAVLink messages directly.

These are **provisional framework contracts**. They establish names, units, validity, and ownership before final command behavior is selected.

## Shared conventions

| Concern | Contract |
| --- | --- |
| Time | UTC monotonic timestamp in seconds for freshness; optional wall-clock timestamp for logs |
| Distance | metres |
| Velocity | metres/second |
| Acceleration | metres/second squared |
| Angles | radians internally; convert only at UI boundaries |
| Position frames | explicit MAVLink frame enum; never infer from field names |
| Missing data | `None`, never a magic numeric value |
| Validity | every outgoing command carries `created_at` and `valid_until` |

## Navigation → MAVLink

### `FlightCommand`

```python
@dataclass(frozen=True)
class FlightCommand:
    command_id: str
    created_at: float
    valid_until: float
    kind: FlightCommandKind       # VELOCITY, POSITION, HOLD, ATTITUDE; supported set is TBD
    frame: CoordinateFrame        # e.g. BODY_NED or LOCAL_NED
    position_m: Vector3 | None = None
    velocity_mps: Vector3 | None = None
    acceleration_mps2: Vector3 | None = None
    yaw_rad: float | None = None
    yaw_rate_rps: float | None = None
    source: str = "navigation"
```

Rules:

- A `FlightCommand` represents one target, not a MAVLink message.
- `kind` determines which fields are required and which fields are ignored.
- The producer must set a coordinate frame; the setpoint manager rejects an absent or unsupported frame.
- The command is rejected if expired, non-finite, outside configured limits, or incompatible with the current vehicle mode.
- Navigation provides replacements at the configured setpoint-input cadence; the manager resends the last valid command at its configured output cadence.

### `CommandRequest`

```python
@dataclass(frozen=True)
class CommandRequest:
    request_id: str
    created_at: float
    action: CommandAction         # ARM, DISARM, TAKEOFF, LAND, RTL, SET_MODE, WINCH
    parameters: Mapping[str, Any]
    source: str                   # navigation, fsm, safety_manager, operator_gateway
    requires_ack: bool = True
```

Discrete commands are not a substitute for continuous setpoints. Every action has defined preconditions in [04_command_interface.md](04_command_interface.md).

## MAVLink → Navigation / FSM

### `VehicleState`

`VehicleState` is the sole normal flight-controller snapshot consumed by the autonomy stack. It is immutable, timestamped, and may contain stale individual fields. Its complete shape is specified in [06_vehicle_state.md](06_vehicle_state.md).

### `MavlinkHealth`

```python
@dataclass(frozen=True)
class MavlinkHealth:
    status: HealthStatus          # CONNECTING, CONNECTED, DEGRADED, STALE, DISCONNECTED
    observed_at: float
    transport_connected: bool
    heartbeat_age_s: float | None
    telemetry_age_s: float | None
    setpoint_age_s: float | None
    last_error: str | None
    reasons: tuple[str, ...]
```

### `CommandResult`

```python
@dataclass(frozen=True)
class CommandResult:
    request_id: str
    status: CommandStatus         # ACCEPTED, IN_PROGRESS, SUCCEEDED, REJECTED, FAILED, TIMED_OUT
    observed_at: float
    mav_result: int | None
    detail: str | None
```

## Command/result delivery

- State and health are published as latest-value snapshots; consumers should not block telemetry ingestion.
- Command results are correlated by `request_id`; a MAVLink ACK alone is not proof that a physical action completed.
- A command that cannot be correlated with an expected ACK is reported as `TIMED_OUT`, never assumed successful.
- All interfaces are in-process initially. A future ROS, ZeroMQ, or HTTP boundary may adapt these contracts without exposing MAVLink types.

## Open interface decisions

| Decision | Current framework position | Owner before flight |
| --- | --- | --- |
| Primary guided setpoint type | Velocity is expected; position/attitude support remains optional | Navigation + flight test |
| Body vs local frame | Explicit per command; default is not assumed | Navigation |
| Max command/setpoint rates | Configurable | Integration test |
| Winch command path | Pixhawk auxiliary output via MAVLink is anticipated | Avionics + payload team |
| Completion evidence | Per-action and telemetry based | Command manager design review |
