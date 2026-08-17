# MAVLink subsystem

This directory contains the companion-computer MAVLink boundary for the R2D2 Aerothon 2026 aircraft.

The target deployment is **Jetson Orin Nano → UART → Pixhawk running ArduPilot**, implemented in Python with **pymavlink**. The MAVLink subsystem accepts autonomy-level requests, translates them to MAVLink, maintains a normalized vehicle snapshot, and reports connection/safety health back to Navigation and the mission FSM.

It does not implement perception, mission decisions, navigation control law, motor control, or replace ArduPilot failsafes.

## Contract boundary

```text
FSM -- MissionCommand --> Navigation -- FlightCommand --> MAVLink
MAVLink -- VehicleState + MavlinkHealth --> Navigation / FSM
```

Navigation owns *what the aircraft should do*. MAVLink owns *how that intent is exchanged with the flight controller*. ArduPilot owns stabilization, mixing, and final onboard failsafes.

## Documentation

| Document | Purpose |
| --- | --- |
| [01_architecture.md](docs/01_architecture.md) | Component responsibilities and data flow |
| [02_interface_control_document.md](docs/02_interface_control_document.md) | Provisional Python-facing input/output contracts |
| [03_mavlink_network.md](docs/03_mavlink_network.md) | Physical links, routing, IDs, and link ownership |
| [04_command_interface.md](docs/04_command_interface.md) | Discrete command behavior |
| [05_telemetry_interface.md](docs/05_telemetry_interface.md) | Required telemetry and freshness rules |
| [06_vehicle_state.md](docs/06_vehicle_state.md) | Normalized vehicle-state model |
| [07_safety_and_failsafe.md](docs/07_safety_and_failsafe.md) | Companion safety responsibilities and escalation |
| [08_configuration.md](docs/08_configuration.md) | Runtime configuration schema |
| [09_logging.md](docs/09_logging.md) | Structured event and message logging |
| [10_testing_and_validation.md](docs/10_testing_and_validation.md) | Unit, SITL, bench, and integrated-test plan |
| [11_integration_checklist.md](docs/11_integration_checklist.md) | Bring-up and handover checklist |

## Status of this framework

The interfaces in these documents are intentionally provisional. Values marked **TBD** must be confirmed on the actual Pixhawk/ArduPilot configuration before flight. They are centralized in configuration rather than embedded in Navigation or MAVLink-library calls.
