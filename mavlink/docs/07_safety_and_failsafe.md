# 07 - Safety and failsafe

## Safety boundary

The companion safety manager observes the MAVLink link and can request an approved safety action. It does not supersede ArduPilot. ArduPilot must independently retain its configured RC, battery, geofence, EKF, and link-loss behavior if the Orin process freezes, loses power, or is disconnected.

```text
Companion problem → SafetyManager detects → optional MAVLink safety request → ArduPilot decides/acts
Companion unavailable → no request possible → ArduPilot's independent failsafes remain active
```

## Health states

| State | Meaning | Example condition | MAVLink-layer behavior |
| --- | --- | --- | --- |
| `CONNECTING` | Transport exists but FC not yet accepted | no valid heartbeat | do not accept flight commands |
| `CONNECTED` | Required link/telemetry fresh | heartbeat and required streams fresh | normal operation |
| `DEGRADED` | Link works but noncritical condition is impaired | optional telemetry absent | publish warning and constrain per policy |
| `STALE` | Required heartbeat/telemetry/setpoint exceeded threshold | incoming state too old | stop accepting new dependent actions; safety policy evaluates |
| `DISCONNECTED` | Transport closed or recovery exhausted | serial/UDP connection lost | publish immediately; no command success claims |

## Events and policy inputs

| Event | Detection input | Candidate action | Final decision required |
| --- | --- | --- | --- |
| FC heartbeat lost | heartbeat age | no new setpoints; report link loss | timeout and operator behavior |
| Required telemetry stale | per-group age | hold/abort request if allowed | message groups and thresholds |
| Navigation setpoint stale | `FlightCommand.valid_until` | stop stream or send configured safe setpoint | stop/hold/RTL behavior |
| Unsupported/invalid command | validation error | reject and log | none |
| FC EKF/GPS/battery unhealthy | `VehicleState` | notify FSM; optionally request mode | ArduPilot parameter alignment |
| Winch anomaly | payload telemetry / command result | brake/abort/RTL request if supported | exact hardware integration |

## Required invariants

- A missing input must fail closed: no stale velocity/position command may continue indefinitely.
- Safety actions require explicit policy enablement and audit logging.
- `RTL` is a request, not a guarantee; confirm its mode/state from telemetry.
- During a connection loss, do not repeatedly queue commands that cannot be delivered.
- Any safety behavior added to the companion must be tested with the corresponding ArduPilot parameter configuration.

## Configuration needed before flight

- Required telemetry groups and per-group stale thresholds.
- Setpoint expiry threshold and safe response.
- Allowed automatic actions, authorizing source, and inhibition conditions.
- ArduPilot failsafe parameters for companion, RC, GCS/3DR, battery, geofence, GPS, and EKF conditions.
- Recovery/reconnection behavior and a clear operator takeover procedure.
