# 05 - Telemetry interface

## Principle

Telemetry is decoded into `VehicleState`, not leaked as raw MAVLink messages to Navigation or the FSM. Message selection and requested rates are configurable because they depend on the final ArduPilot setup and available UART bandwidth.

## Baseline telemetry

| MAVLink message | Normalized fields | Consumers | Provisional requested rate | Freshness purpose |
| --- | --- | --- | --- | --- |
| `HEARTBEAT` | connected, armed, mode, autopilot type | all | 1 Hz or FC default | link liveness |
| `SYS_STATUS` | voltage, current, sensor-health flags | safety, FSM | 1-2 Hz | aircraft health |
| `BATTERY_STATUS` | remaining %, voltage/current detail | safety, FSM | 1-2 Hz | battery policy |
| `GLOBAL_POSITION_INT` | latitude, longitude, relative/absolute altitude | Navigation, FSM | 5-10 Hz | global position |
| `LOCAL_POSITION_NED` | local position and velocity | Navigation | 10-20 Hz | guided-state feedback |
| `ATTITUDE` | roll, pitch, yaw, body rates if available | Navigation, safety | 10-20 Hz | attitude feedback |
| `VFR_HUD` | air/ground speed, climb, heading | display, fallback state | 2-5 Hz | pilot-readable status |
| `GPS_RAW_INT` | fix type, satellites, GPS accuracy | safety, FSM | 1-5 Hz | GPS health |
| `EKF_STATUS_REPORT` | estimator flags | safety | 1-2 Hz | estimator health |
| `STATUSTEXT` | FC diagnostics | logs, operator display | event-driven | diagnosis |
| `COMMAND_ACK` | command outcome | command manager | event-driven | transaction tracking |

Rates above are design targets, not committed vehicle parameters. They must be budgeted against the actual baud rate and every active MAVLink stream before hardware flight.

## Telemetry processing

```text
incoming MAVLink frame
  → source identity validation
  → adapter decode
  → message-specific normalization
  → StateManager update with per-field timestamp
  → HealthMonitor freshness evaluation
  → publish latest VehicleState / MavlinkHealth
```

Invalid values, unknown source identities, or undecodable frames are logged and dropped; they do not overwrite valid vehicle state.

## Staleness model

Each field group retains the timestamp of the message that updated it. A snapshot can therefore be connected while a specific field such as GPS or EKF status is stale. Consumers must use field freshness rather than assuming that a recently received heartbeat makes all state current.

Final stale thresholds belong in `safety.yaml`. Initial values must be chosen only during SITL/bench validation, taking message rate and normal transport jitter into account.

## Stream control

The telemetry manager may request message intervals only for messages required by this subsystem. It must:

- avoid changing unrelated GCS/RC telemetry behavior;
- log each successful or rejected interval request;
- tolerate an FC that supplies a different rate than requested;
- derive health from received timestamps, not requested rates alone.
