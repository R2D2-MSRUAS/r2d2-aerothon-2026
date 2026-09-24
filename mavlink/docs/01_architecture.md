# 01 - MAVLink architecture

## Purpose

The MAVLink subsystem is the communication and abstraction layer between the Jetson Orin Nano autonomy stack and the Pixhawk flight controller running ArduPilot. The Python implementation uses `pymavlink` and supports both UART hardware and UDP SITL transports.

```text
OAK-D → Perception → FSM → Navigation → MAVLink subsystem → Pixhawk / ArduPilot
                                  ▲              │
                                  └── VehicleState + MavlinkHealth ──┘
```

The flow is deliberately not `FSM → MAVLink`: the FSM selects a mission phase, Navigation creates motion intent, and MAVLink translates that intent without making mission decisions.

## System boundary

| Layer | Input | Processing | Output |
| --- | --- | --- | --- |
| FSM | `PerceptionResult`, `VehicleState`, health | Mission transition logic | `MissionCommand` |
| Navigation | `MissionCommand`, perception, `VehicleState` | Guidance / control law | `FlightCommand` |
| MAVLink subsystem | `FlightCommand`, discrete requests | Encode/decode, stream, observe, validate | MAVLink frames; state and health |
| ArduPilot | MAVLink and onboard sensors | EKF, stabilization, failsafes, motor mixing | Actuator outputs and telemetry |

## Internal components

```text
                    MAVLINK CORE
                         │
       ┌─────────────────┼──────────────────┐
       ▼                 ▼                  ▼
 CommandManager   SetpointManager   TelemetryManager
       │                 │                  │
       └─────────────┬───┴──────────┬───────┘
                     ▼              ▼
                SafetyManager   StateManager
                     │              │
                     └──────┬───────┘
                            ▼
                     HealthMonitor
                            │
                            ▼
                    pymavlink Adapter
                            │
                    Serial or UDP Transport
```

| Component | Input | Processing | Output |
| --- | --- | --- | --- |
| Transport | bytes / connection configuration | Open, close, send, receive, reconnect | byte/message stream, transport status |
| pymavlink adapter | typed internal requests, MAVLink frames | `pymavlink` encoding and decoding | typed message events, encoded MAVLink |
| Command manager | `CommandRequest` | Send and track discrete commands | `CommandResult` |
| Setpoint manager | `FlightCommand` | Validate, translate, repeat latest valid setpoint | setpoint frames, setpoint freshness |
| Telemetry manager | decoded MAVLink messages | Filter and normalize telemetry | `TelemetryUpdate` |
| State manager | telemetry updates | Maintain latest coherent snapshot | `VehicleState` |
| Health monitor | timestamps, heartbeat, ACKs, transport state | Determine link health | `MavlinkHealth` |
| Safety manager | health, vehicle state, safety configuration | Recommend/issue permitted safety request | `SafetyEvent` / `CommandRequest` |

## Ownership rules

- Navigation owns trajectory choice and produces frame-aware `FlightCommand` objects.
- MAVLink owns MAVLink message syntax, target IDs, ACK tracking, stream rates, and retry policy.
- MAVLink never silently converts an invalid, stale, or unsupported command into a flight action.
- ArduPilot remains the final authority for flight stabilization, geofence, battery, RC, and companion-link failsafes.
- The 3DR radio is a separate Pixhawk MAVLink link for GCS monitoring and recovery. It is not a transport fallback that the Orin controls.

## Lifecycle

1. Load configuration and open a serial or UDP transport.
2. Wait for an acceptable flight-controller heartbeat and identify its system/component IDs.
3. Start telemetry ingestion and publish `VehicleState` plus `MavlinkHealth`.
4. Accept discrete commands only after connection and precondition checks pass.
5. Stream the newest valid navigation setpoint while its validity window remains open.
6. On stale input, loss of telemetry, or transport failure, stop/hold/escalate according to the safety policy and record the event.

Exact flight modes, rates, timeouts, and failsafe actions are configuration decisions, not architecture constants.
