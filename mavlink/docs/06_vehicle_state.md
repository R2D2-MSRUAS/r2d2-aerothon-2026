# 06 - Vehicle state

## Purpose

`VehicleState` is the current normalized view of the Pixhawk for autonomy consumers. It prevents Navigation and the FSM from coupling to individual MAVLink message types, payload scaling, or message timing.

## Provisional schema

```python
@dataclass(frozen=True)
class VehicleState:
    observed_at: float
    source_system: int | None
    source_component: int | None
    connected: bool
    armed: bool | None
    mode: str | None
    position: PositionState | None
    velocity: Vector3 | None
    attitude: AttitudeState | None
    battery: BatteryState | None
    gps: GpsState | None
    ekf: EkfState | None
    field_timestamps: Mapping[str, float]
```

| Group | Required fields | Principal source | Notes |
| --- | --- | --- | --- |
| Connection | connected, FC identity, observed time | heartbeat + transport | connection is not a safety guarantee |
| Mode/arming | armed, mode | heartbeat | mode mapping is adapter owned |
| Position | latitude, longitude, absolute/relative altitude, optional local NED | global/local position | frames and validity are explicit |
| Velocity | NED or body vector with frame label | local position / selected source | never mix frames without conversion |
| Attitude | roll, pitch, yaw; optional rates | attitude | radians internally |
| Battery | voltage, current, remaining | battery/system status | unavailable values remain `None` |
| GPS | fix type, satellites, accuracy if reported | GPS raw | health policy chooses minimum quality |
| EKF | flags / healthy interpretation | EKF report | raw flags retained for logs |

## Coherency and updates

- Updates are atomic: consumers receive a complete immutable snapshot, never a partially mutated object.
- Each group has an independent timestamp in `field_timestamps`.
- The state manager does not estimate or invent values absent from telemetry.
- Adapters convert MAVLink-scaled integers and angular units before publishing.
- A raw-message trace may be logged separately for diagnosis but is not part of this contract.

## Consumer rules

| Consumer | May rely on |
| --- | --- |
| FSM | armed/mode, mission-relevant position, GPS/EKF/battery health, `MavlinkHealth` |
| Navigation | position/velocity/attitude only when required groups are fresh |
| Safety manager | all freshness, battery, GPS/EKF, armed/mode, link health |
| Operator logging | all available fields plus raw diagnostic events |

The exact definition of `ekf.healthy`, a valid GPS fix, and required position source is safety configuration, to be validated against the competition mission and firmware parameters.
