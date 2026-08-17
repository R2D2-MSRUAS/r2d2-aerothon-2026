# 11 - Integration checklist

## Architecture and ownership

- [ ] FSM emits `MissionCommand`, not MAVLink messages.
- [ ] Navigation is the only autonomy producer of `FlightCommand`.
- [ ] Navigation/FSM consume `VehicleState` and `MavlinkHealth`, not raw pymavlink objects.
- [ ] MAVLink layer uses `pymavlink` only behind its adapter.
- [ ] ArduPilot remains configured as final failsafe authority.

## Configuration

- [ ] Jetson Orin Nano serial device, baud rate, and flow-control setting are confirmed.
- [ ] Pixhawk `SERIALx_*` parameters match the companion link and are recorded.
- [ ] Companion source IDs and Pixhawk target IDs are unique/verified.
- [ ] SITL and hardware profiles are separate.
- [ ] Required telemetry, stream-rate budget, and freshness thresholds are approved.
- [ ] Mode allow-list, command enables, setpoint limits, and automatic safety actions are approved.

## Telemetry and safety

- [ ] Heartbeat, mode/arming, position, attitude, battery, GPS, and EKF telemetry are observed and normalized as required.
- [ ] Per-field freshness is visible in `VehicleState`.
- [ ] Connection, stale telemetry, stale setpoint, and recovery health transitions are logged.
- [ ] ArduPilot companion/RC/GCS/battery/geofence/EKF failsafes are documented and bench-tested.
- [ ] 3DR GCS monitoring/recovery remains independent of the Orin companion link.

## Commands and payload

- [ ] Arm/disarm, mode, takeoff, land, RTL, and winch interfaces each have a documented precondition and completion signal.
- [ ] Commands are disabled until their test approval is recorded.
- [ ] Winch channel, PWM range, safe output, brake behavior, and confirmation telemetry are verified on a bench fixture.
- [ ] A command ACK is never treated as action completion without the designated telemetry evidence.

## Validation and release

- [ ] Contract/unit tests pass.
- [ ] SITL tests pass using the current configuration.
- [ ] Orin-to-SITL test passes with the target Python/pymavlink environment.
- [ ] Props-off Pixhawk UART/telemetry test passes.
- [ ] Test evidence includes logs, config profile, and ArduPilot parameter snapshot.
- [ ] Any change to serial, command, setpoint, safety, or flight-mode configuration has a recorded regression test.
