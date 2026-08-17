# 10 - Testing and validation

## Test progression

```text
contract/unit → SITL → Orin-to-SITL → Pixhawk props-off → avionics bench → integrated autonomy → tethered/controlled flight
```

No stage replaces the previous one. A flight-control integration change returns to the earliest affected stage.

## Level 1 - Unit tests

No Pixhawk or network required.

| Subject | Example assertion |
| --- | --- |
| Contract validation | expired or frame-less `FlightCommand` is rejected |
| Setpoint mapping | body-frame forward velocity selects the configured setpoint message/frame/mask |
| Command transaction | ACK and telemetry confirmation produce a correlated result |
| State normalization | MAVLink scaling maps into documented SI units |
| Health monitor | fake timestamps produce correct health transitions |
| Safety policy | stale setpoint selects the configured, audited response |

## Level 2 - ArduPilot SITL

```text
Navigation mock → MAVLink subsystem → UDP → ArduPilot SITL → MAVProxy / Mission Planner
```

Verify heartbeat discovery, telemetry parsing, guided-mode preconditions, approved setpoint behavior, command ACK handling, reconnect behavior, and logging. Keep SITL parameters/configuration alongside the test evidence.

## Level 3 - Jetson Orin to SITL

Run the production Python environment and configuration on the Orin, connected over UDP to SITL. Verify dependency installation, service startup/restart, CPU/load behavior, timestamp handling, logs, and the exact pymavlink version.

## Level 4 - Pixhawk hardware, props off

Verify UART settings, heartbeat, identity binding, telemetry, mode observation, command inhibition, arm/disarm procedure, 3DR coexistence, link-loss detection, and reconnect behavior. Do not test free-flight setpoints or winch behavior without an approved bench fixture and safety procedure.

## Level 5 - Integrated autonomy

Use mock and then real Perception/FSM/Navigation inputs to verify the contracts:

```text
Perception → FSM → Navigation → FlightCommand → MAVLink → VehicleState/MavlinkHealth → FSM
```

## Evidence for each test

- source revision and dependency lock/version;
- MAVLink configuration profile and complete ArduPilot parameter file;
- test case and expected outcome;
- event logs and, when appropriate, raw MAVLink trace;
- observed result, anomalies, and explicit pass/fail decision.
