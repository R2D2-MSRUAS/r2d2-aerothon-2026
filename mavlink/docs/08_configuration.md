# 08 - Configuration

## Principle

All environment- and aircraft-specific values are configuration, not Python constants. A configuration validator must reject incomplete production configuration and make a SITL profile explicit.

## Files

| File | Contents |
| --- | --- |
| `config/connection.yaml` | serial/UDP endpoint, source identity, reconnect behavior |
| `config/vehicle.yaml` | target identity validation, allowed modes, vehicle limits |
| `config/telemetry.yaml` | message enablement, requested intervals, freshness requirements |
| `config/safety.yaml` | timeouts, permitted automatic actions, safety gates |

## Example shape

```yaml
# connection.yaml
transport: serial                 # serial for aircraft; udp for SITL
serial:
  device: /dev/ttyTHS1            # TBD: verify on Jetson Orin Nano
  baud: null                      # TBD: must match Pixhawk SERIALx_BAUD
  flow_control: null              # TBD
udp:
  endpoint: udp:127.0.0.1:14550   # SITL profile only
source:
  system_id: null                 # TBD: unique companion identity
  component_id: null              # TBD
target:
  expected_system_id: null        # optionally pin after first bench discovery
  expected_component_id: null
```

```yaml
# safety.yaml
setpoints:
  output_rate_hz: null            # TBD and tested
  input_expiry_s: null            # TBD and tested
  on_expiry: report_only          # hold / stop_stream / request_rtl only after validation
telemetry:
  heartbeat_stale_s: null
  required_groups: [connection, mode_arming]
automatic_actions:
  request_rtl: false              # explicitly enabled only after test approval
commands:
  arm_enabled: false
  takeoff_enabled: false
  winch_enabled: false
```

## Validation rules

- Hardware and SITL profiles must be distinct files or named profiles.
- A hardware profile may not contain a loopback UDP endpoint.
- No `null` TBD value is allowed for an enabled command or selected transport.
- IDs must be positive valid MAVLink IDs and the source identity must be unique on the aircraft network.
- Requested telemetry rate and setpoint rate must be positive and included in a link-bandwidth review.
- Mode names and automatic safety actions must be allow-listed.

Configuration changes affecting commands, setpoints, serial links, or safety behavior require SITL and props-off regression tests before use on the aircraft.
