# R2D2 Command & Middleware Layer

Sits between the Mission 2 FSM and the hardware: camera failover/switching
on one side, MAVLink command dispatch to the Pixhawk 6X Mini on the other.
Pure Python, runs on the Jetson Orin Nano; also runs on a laptop with fakes
for bench testing (no hardware imports at module load).

## Files

| File | What it does |
|---|---|
| `config.py` | All tunables: serial endpoints, camera thresholds, failsafe timings |
| `cameras.py` | Camera sources: OAK-D Lite (DepthAI), Arducam B0251 CSI (nvarguscamerasrc), generic OpenCV for bench work |
| `camera_manager.py` | Priority failover, health watchdog, recovery probing, hot switching |
| `mavlink_bridge.py` | pymavlink connection (primary UART, 3DR backup), heartbeat watchdog, ACK-verified commands, winch servo, STATUSTEXT |
| `commander.py` | The single API the FSM calls. Failsafe policy lives here |
| `test_bench.py` | Hardware-free tests of every failover/failsafe path |

## Camera policy

- Priority order: **OAK-D Lite (45° tilt) → Arducam CSI (bottom-facing)**.
- Active source dies (no frames for 1 s, or 5 consecutive read errors) →
  automatic failover down the list, GCS gets `CAM FAILOVER` STATUSTEXT.
- A dead primary is re-probed every 5 s. On recovery it is held **warm on
  standby** — the manager deliberately does *not* auto-switch back, because
  the two mounts have different geometry and a silent swap mid-alignment
  would corrupt the zone PID. The FSM decides via `cmd.use_primary_camera()`.
- `read_frame()` returns `(frame, source_name)` so the vision pipeline
  always knows which mount geometry it is looking through.
- Delivery phase: call `cmd.use_bottom_camera()` for the drop view, switch
  back after winch retract.

## Failsafe policy (companion side)

Pixhawk keeps its own RC/battery/geofence failsafes as final authority.
On top of that, this layer does:

| Trigger | Action |
|---|---|
| Camera failover | Keep flying, notify GCS |
| All cameras lost | `LOITER` immediately; `RTL` after 20 s without recovery |
| Winch release unconfirmed after 10 s | Brake winch + `RTL` (onboard mirror of the GCS 3DR trigger) |
| FC heartbeat lost > 3 s | Logged + surfaced; nothing commandable without a link |

## Usage in the FSM

```python
from middleware import (Config, CameraManager, MavlinkBridge, Commander,
                        OakDLiteSource, ArducamCSISource)

cfg = Config()
cams = CameraManager([OakDLiteSource(cfg.camera),
                      ArducamCSISource(cfg.camera)], cfg.camera)
bridge = MavlinkBridge(cfg.link)          # tries /dev/ttyTHS1, then 3DR USB
cmd = Commander(bridge, cams, cfg)

bridge.connect()
cams.start()

cmd.arm_and_takeoff(15.0)
while mission_running:
    frame, source = cmd.read_frame()      # source = which camera/geometry
    # ... vision + FSM logic (qr_pipeline modes plug in here) ...
    cmd.tick()                            # runs time-based failsafe checks
```

Winch sequence:

```python
cmd.use_bottom_camera()
cmd.winch_release()
# when vision / DRV8838 current confirms the drop:
cmd.confirm_payload_released()            # else brake+RTL fires at 10 s
```

## Testing

Bench (no hardware): `python -m middleware.test_bench` — exercises startup,
failover, standby recovery, all-lost escalation, winch timeout, manual
switching. All green as of Aug 2026.

SITL: run ArduCopter SITL, then

```python
bridge = MavlinkBridge(cfg.link, endpoint=cfg.link.sitl_endpoint)
```

Laptop cameras: substitute `OpenCVSource(cfg.camera, 0, name="oak-d-lite")`
and a video file as the "backup" to rehearse switching with real frames.

## Jetson deployment notes

- `pip install pymavlink depthai opencv-python` (or use the JetPack OpenCV
  build with GStreamer support — required for `nvarguscamerasrc`).
- Give the user serial access: `sudo usermod -aG dialout $USER`, and disable
  nvgetty on the Jetson UART: `sudo systemctl disable nvgetty`.
- Endpoints in `config.py`: carrier UART is `/dev/ttyTHS1` @ 921600 to the
  Pixhawk TELEM port; the 3DR air radio enumerates as `/dev/ttyUSB0` @ 57600.
- Winch: Pixhawk AUX1 = servo output 9 (`winch_servo_channel`); set
  `SERVO9_FUNCTION=0` (disabled/GPIO-style passthrough for DO_SET_SERVO).
