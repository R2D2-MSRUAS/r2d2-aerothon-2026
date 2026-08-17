# 03 - MAVLink network

## Network intent

The aircraft contains more than one link to the Pixhawk. The Orin link is the autonomous companion interface; the 3DR link is independent GCS telemetry/recovery; the Skydroid link provides RC and its own telemetry path. These links must be configured so that one link's failure does not silently disable the Pixhawk's independent safety behavior.

| Link | Endpoints | Physical interface | Payload | Ownership |
| --- | --- | --- | --- | --- |
| Companion | Jetson Orin Nano ↔ Pixhawk | UART | MAVLink commands, setpoints, telemetry | MAVLink software |
| Backup telemetry | Pixhawk ↔ 3DR air radio ↔ GCS | UART + RF | MAVLink telemetry and GCS recovery commands | Avionics / GCS integration |
| RC / primary pilot | Pixhawk ↔ Skydroid | FC-side telemetry/RC interface, exact protocol TBD | Pilot control, telemetry, FPV path | Avionics |
| SITL | Local process ↔ ArduPilot SITL | UDP | MAVLink test traffic | Development |

## Companion UART contract

| Property | Framework rule | Final value |
| --- | --- | --- |
| Device | Set in `connection.yaml`; never hard-coded | TBD on Orin |
| Baud rate | Set in `connection.yaml`; must match ArduPilot serial parameter | TBD after bench setup |
| Protocol | MAVLink 2 preferred, confirmed from heartbeat | TBD / verified |
| Flow control | Declared explicitly | TBD |
| Target system/component | Learned from accepted FC heartbeat, then pinned for session | Verified at connection |

The process may send only after a valid heartbeat establishes a target. It must log the source system/component ID of every newly observed vehicle and reject unexpected vehicle IDs once a session is bound.

## IDs and routing

| Identifier | Framework behavior |
| --- | --- |
| Orin source system/component | Configurable, unique on the aircraft network |
| Pixhawk target system/component | Discovered from heartbeat then configured/validated |
| GCS source system/component | Independent of Orin; must not reuse the companion identity |
| MAVLink forwarding | Configured on the Pixhawk, not assumed by the companion process |

The companion process does not proxy the 3DR or Skydroid links. It should tolerate telemetry being visible from multiple MAVLink components, but `VehicleState` is derived only from the accepted Pixhawk flight-controller identity.

## Network bring-up

1. Verify the selected Pixhawk UART is electrically connected and ArduPilot serial parameters match its intended MAVLink protocol/rate.
2. Start the companion process with a distinct source identity.
3. Confirm one Pixhawk heartbeat is received, its identity is recorded, and telemetry is fresh.
4. Confirm Mission Planner/MAVProxy via 3DR still sees telemetry independently.
5. Disconnect the companion link and verify configured ArduPilot behavior; repeat for GCS/RC links per the safety plan.

No link-loss conclusion is valid until it has been verified on the installed ArduPilot parameters and hardware.
