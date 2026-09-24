# R2D2 MAVLink - Windows SITL

This is the Windows-only, Python/pymavlink interface between the R2D2 autonomy stack and ArduPilot. It has no ROS or Ubuntu dependency.

The completed runnable target is an **AeroTHON Mission 2 SITL rehearsal** in Mission Planner. It confirms the flight-control integration and the mission sequence, but deliberately simulates perception and payload mechanics. It must never be used as a hardware-flight command program.

## Run the Mission Planner rehearsal

1. Install Python 3.11 for Windows and create a local environment:

   ```powershell
   cd mavlink
   py -3.11 -m venv .venv
   .\.venv\Scripts\python -m pip install -r requirements.txt
   ```

2. Open Mission Planner, select **SIMULATION**, choose **Copter / quad** and start SITL. Use a known home location and wait for stable GPS/EKF status.

3. In Mission Planner, open **SETUP > Advanced > MAVLink Mirror**. Configure a **UDP Client** to `127.0.0.1:14550`, enable write access, and connect it. This mirror is the only link the runner uses; Mission Planner remains the operator display and recovery GCS.

4. Start with the read-only monitor:

   ```powershell
   .\.venv\Scripts\python main.py monitor
   ```

5. Run the deterministic Mission 2 rehearsal:

   ```powershell
   .\.venv\Scripts\python main.py mission --confirm-sitl --simulate-perception
   ```

The two flags are intentional gates. They prevent accidental use of the mission sequencer without an explicit acknowledgement that it is a local simulation using synthetic perception confirmations.

See [12_windows_mission_planner_sitl.md](docs/12_windows_mission_planner_sitl.md) for the complete operator workflow, evidence checklist, and traceability.

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

## Safety boundary

```text
Perception/FSM -> Navigation -> FlightCommand -> MAVLink -> ArduPilot SITL
                                 ^                         |
                                 +--- VehicleState/Health --+
```

The runner owns only SITL orchestration. ArduPilot owns stabilization and failsafes. The present code does not include a real camera pipeline, field geofence, obstacle avoidance controller, telemetry-confirmed winch mechanism, Jetson serial transport, or authorization for flight hardware. Those remain separate acceptance items.
