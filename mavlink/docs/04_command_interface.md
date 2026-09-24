# 04 - Command interface

## Purpose

The command manager handles finite, acknowledged actions. It is separate from the setpoint manager, which handles continuous navigation targets.

## Current Windows SITL implementation

The `aerothon_sitl.yaml` profile maps `ARM`, `DISARM`, `TAKEOFF`, `LAND`, `RTL`, `SET_MODE`, and `YAW` through pymavlink after a configured FC heartbeat is accepted. `COMMAND_ACK` messages are correlated with the request; an accepted ACK is still only transport acceptance, and mission progression waits for the relevant `VehicleState` evidence. `WINCH` remains rejected because SITL cannot validate the real payload mechanism.

This implementation is limited to `simulation_only: true` profiles and the `main.py mission --confirm-sitl --simulate-perception` entry point. It is not a hardware release.

## Supported action framework

| Action | Intent | Likely MAVLink mechanism | Minimum preconditions | Completion evidence |
| --- | --- | --- | --- | --- |
| `ARM` | Arm vehicle | `MAV_CMD_COMPONENT_ARM_DISARM` | connected, safety gate passes, operator policy permits | ACK plus `VehicleState.armed` |
| `DISARM` | Disarm vehicle | `MAV_CMD_COMPONENT_ARM_DISARM` | policy allows it | ACK plus `armed == false` |
| `SET_MODE` | Change FC mode | `SET_MODE` / ArduPilot mode mapping | requested mode is allow-listed | mode telemetry confirms |
| `TAKEOFF` | Start an ArduPilot-managed takeoff | `MAV_CMD_NAV_TAKEOFF` | armed, approved mode, valid altitude | ACK plus defined state/altitude evidence |
| `LAND` | Request landing | ArduPilot mode/command, final mapping TBD | connected | ACK plus mode telemetry |
| `RTL` | Request return-to-launch | ArduPilot mode/command, final mapping TBD | connected | ACK plus RTL mode telemetry |
| `WINCH` | Control configured auxiliary output | `MAV_CMD_DO_SET_SERVO` or configured alternative | payload policy and channel configuration pass | ACK plus optional winch telemetry |

The exact command number, parameter mapping, and ArduPilot mode mapping are adapter implementation details verified against the installed firmware. They are not exposed to Navigation.

## Request lifecycle

```text
request → precondition validation → MAVLink send → ACK wait → telemetry confirmation → result
```

1. Reject duplicate `request_id` values.
2. Validate source authorization, connection health, command-specific parameters, and safety policy.
3. Send the message with configured retry/timeout policy.
4. Match an ACK when the command type supports one.
5. For state-changing actions, wait for configured telemetry confirmation.
6. Publish `CommandResult` and structured log events at each transition.

## Guardrails

- `ARM`, `TAKEOFF`, `LAND`, `RTL`, and winch control must be disabled by default until their configuration has been bench-tested.
- The command manager must never arm, change mode, or trigger a winch merely because the process reconnects.
- An ACK result of accepted is not physical completion; the result remains `IN_PROGRESS` until corroborating telemetry or a timeout.
- Operator-originated safety commands require a separately authorized ingress; arbitrary GCS/MAVLink traffic is not accepted as an internal request.

## Open decisions

- Authoritative flight mode names and allowed transitions.
- Takeoff altitude, land behavior, and RTL policy.
- Winch auxiliary channel, PWM bounds, safe state, and telemetry source.
- Whether commands may be retried and which are intentionally one-shot.
