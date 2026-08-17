# 09 - Logging

## Objectives

Logs must allow the team to reconstruct connection state, commands, health decisions, and relevant aircraft telemetry after a test. They must not make the control loop block on disk I/O.

## Event schema

```text
timestamp | level | event | session_id | source | target_system | target_component | detail
```

Structured JSON lines are preferred for machine analysis. Human-readable console output may be emitted in parallel.

## Required events

| Event | Minimum details |
| --- | --- |
| `CONNECTION_OPENED` / `CONNECTION_LOST` | transport type, endpoint, error/reason |
| `HEARTBEAT_RECEIVED` | source identity, mode, armed state |
| `TARGET_BOUND` | accepted FC system/component |
| `TELEMETRY_STALE` / `TELEMETRY_RECOVERED` | affected field group, age, threshold |
| `COMMAND_SENT` / `COMMAND_ACK` / `COMMAND_RESULT` | request ID, action, ACK result, confirmation result |
| `SETPOINT_ACCEPTED` / `SETPOINT_REJECTED` | command ID, kind, frame, reason |
| `SETPOINT_STREAM_STARTED` / `SETPOINT_EXPIRED` | rate, last input age, policy action |
| `HEALTH_CHANGED` | old/new health state and reasons |
| `SAFETY_ACTION_REQUESTED` | trigger, action, authorization result |
| `CONFIG_LOADED` | profile name and non-secret config fingerprint |

## Raw MAVLink capture

Raw frame/message capture is optional and intended for SITL and short bench sessions. It should be rate-limited or sampled in flight to avoid storage pressure and must be correlated with the structured event session ID.

## Retention

The final retention location, rotation size, and export workflow are TBD. At minimum, preserve logs from every SITL, props-off, and flight test together with the exact configuration profile and ArduPilot parameter file used.
