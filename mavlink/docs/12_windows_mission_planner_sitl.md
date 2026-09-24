# 12 - Windows Mission Planner SITL workflow

## Purpose and boundary

`main.py mission` is the one-command integration rehearsal for AeroTHON 2026 Track 1 Mission 2. It runs entirely on Windows with Python and Mission Planner SITL; it does not use ROS, WSL, Ubuntu, or a Pixhawk.

The program sends only the command types listed below after its explicit `--confirm-sitl` gate and after the `aerothon_sitl.yaml` profile confirms `simulation_only: true`. It will not start with the read-only `sitl.yaml` profile.

| Layer | Responsibility in this repository | Explicitly excluded from the SITL rehearsal |
| --- | --- | --- |
| Mission sequencer | Mission phase order, altitude/distance timing, command gates | Field approval or autonomous hardware operation |
| Navigation/MAVLink | Velocity setpoints, arm/mode/takeoff/land/yaw messages, state normalization | Position estimation, obstacle avoidance policy, flight stabilization |
| Perception boundary | QR/banner/red-zone events | Real OAK-D capture and QR/red-zone validation |
| Payload boundary | Delivery timing event | Servo/winch output and payload confirmation |
| ArduPilot SITL | Flight dynamics, GUIDED control, flight-controller telemetry | Competition field geometry and sensor realism |

## Mission traceability

The included [rulebook](../../docs/AEROTHON-2026-Track-1-Rulebook_Rev2.pdf) specifies this Mission 2 flow. Configuration values are recorded in `config/aerothon_sitl.yaml` and originate from `docs/PARAMETERS.md`.

| Requirement | Runner phase | Simulation evidence |
| --- | --- | --- |
| Autonomous takeoff and start-QR scan at 5 m | `TAKEOFF`, `START_QR` | altitude telemetry and phase log |
| Green-banner corridor entry, 3.5 m corridor at 10 ft / 3.048 m | `OUTBOUND_BANNER`, `DESCEND_CORRIDOR`, `OUTBOUND_CORRIDOR` | synthetic banner confirmation; bounded 1.5 m/s body-frame velocity stream |
| Search delivery QR at 10 m; avoid red zones | `ASCEND_DELIVERY`, `TARGET_QR` | synthetic confirmation only; real perception is required later |
| Deliver from 5 m | `DESCEND_DROP`, `PAYLOAD_SIMULATION` | 4.5 + 1.5 + 4.5 second timing event; no servo output |
| Return at 10 m, traverse corridor, land | `ASCEND_RETURN`, `TURN_HOME`, `RETURN_BANNER`, `DESCEND_RETURN_CORRIDOR`, `INBOUND_CORRIDOR`, `LAND` | yaw command, synthetic banner confirmation, 3.048 m return corridor, disarm telemetry |
| Finish within 15 minutes | every phase | 900 second mission deadline triggers LAND and failure |

## Windows setup

1. Install Mission Planner and Python 3.11 on the Windows machine. In PowerShell, install the isolated Python requirements from the `mavlink` directory:

   ```powershell
   py -3.11 -m venv .venv
   .\.venv\Scripts\python -m pip install -r requirements.txt
   ```

2. Start **Mission Planner > SIMULATION**. Select **Copter** and a matching quad model, then click the vehicle to start SITL. Select a fixed home coordinate if repeatable logs are needed.

3. After GPS/EKF has settled, configure **SETUP > Advanced > MAVLink Mirror** as a UDP Client to `127.0.0.1:14550`, with write access enabled. Start the mirror. Do not attach a real flight controller.

4. Verify the mirror before sending a command:

   ```powershell
   .\.venv\Scripts\python main.py monitor --duration 20
   ```

   The console must report the bound vehicle identity, fresh health, and live mode/arming telemetry.

5. Start the sequence:

   ```powershell
   .\.venv\Scripts\python main.py mission --confirm-sitl --simulate-perception
   ```

6. Observe the complete sequence in Mission Planner. On the DATA screen, verify GUIDED during the active legs and LAND at the end. Preserve the Mission Planner flight log and the JSONL file reported by the script.

## MAVLink contract used

| Intent | MAVLink exchange | Preconditions |
| --- | --- | --- |
| Flight mode | `SET_MODE` with ArduCopter `GUIDED` / `LAND` | accepted heartbeat from system/component 1:1 |
| Arm and takeoff | `MAV_CMD_COMPONENT_ARM_DISARM`, `MAV_CMD_NAV_TAKEOFF` | SITL profile, bound FC, explicit launch gate |
| Corridor/altitude movement | `SET_POSITION_TARGET_LOCAL_NED`, velocity-only `0x0DC7`, `MAV_FRAME_BODY_NED` | setpoint expires after 0.5 s; component resends at 5 Hz |
| Return turn | `MAV_CMD_CONDITION_YAW` | 180 degrees at configured 25 deg/s |
| Landing/abort | `MAV_CMD_NAV_LAND` | mission timeout, action rejection, or planned final phase |

ArduPilot requires a velocity target to be refreshed periodically and accepts these local-NED setpoints only in GUIDED control. The runner updates every 0.2 seconds and bounds any axis to 2.5 m/s.

## Acceptance criteria

- The monitor receives the expected SITL identity and fresh position/attitude/battery telemetry.
- The mission reaches `COMPLETE`, and Mission Planner shows a safe landing/disarm.
- No phase exceeds its timeout; no velocity is higher than 2.5 m/s.
- The JSONL application log and Mission Planner flight log are saved with the exact `aerothon_sitl.yaml` used.
- QR, banner, red-zone and payload events are recorded as **simulated**, never claimed as vision/winch acceptance.

## Before enabling hardware

The following work is intentionally outside this release: integrate the real OAK-D perception producer, validate field geofence coordinates, add obstacle avoidance with test cases, implement the UART transport, bench-test the physical winch with feedback, and complete the props-off then tethered-flight stages in the integration checklist. A hardware profile must keep commands disabled until those stages are approved.

## References

- [AeroTHON 2026 Track 1 Rulebook, sections 2.0 and 4.2.4](../../docs/AEROTHON-2026-Track-1-Rulebook_Rev2.pdf)
- [Mission Planner Simulation](https://ardupilot.org/planner/docs/mission-planner-simulation.html)
- [ArduPilot Copter GUIDED-mode commands](https://en.ardupilot.org/dev/docs/copter-commands-in-guided-mode.html)
- [ArduPilot SITL overview](https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html)
