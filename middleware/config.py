"""Central configuration for the R2D2 command & middleware layer.

Values match the Mission 2 (SkyScan) architecture doc and the Robu hardware
list (Aug 2026): Jetson Orin Nano Super + Pixhawk 6X Mini, OAK-D Lite primary
camera, Arducam B0251 (IMX477, CSI) backup, Skydroid T12 primary datalink,
3DR 433 MHz backup telemetry.
"""
from dataclasses import dataclass, field


# ---------------------------------------------------------------- MAVLink ---
@dataclass
class LinkConfig:
    # Primary: Pixhawk 6X Mini TELEM port wired to the Jetson carrier UART.
    primary_endpoint: str = "/dev/ttyTHS1"
    primary_baud: int = 921_600
    # Backup: 3DR SiK radio on USB (onboard end). GCS holds the other radio.
    backup_endpoint: str = "/dev/ttyUSB0"
    backup_baud: int = 57_600
    # Bench testing: SITL / mavlink-router, e.g. "udpin:0.0.0.0:14550".
    sitl_endpoint: str = "udpin:0.0.0.0:14550"

    source_system: int = 191          # companion-computer system id convention
    source_component: int = 191

    heartbeat_timeout_s: float = 3.0  # no FC heartbeat for this long -> link down
    heartbeat_send_hz: float = 1.0
    command_ack_timeout_s: float = 1.5
    command_retries: int = 3
    reconnect_backoff_s: float = 2.0


# ---------------------------------------------------------------- Cameras ---
@dataclass
class CameraConfig:
    # OAK-D Lite (USB3, 45 deg tilt) — primary vision.
    oak_rgb_width: int = 1920
    oak_rgb_height: int = 1080
    oak_fps: int = 30

    # Arducam B0251 IMX477 on CSI, bottom-facing, 10 deg incline — backup.
    csi_sensor_id: int = 0
    csi_width: int = 1920
    csi_height: int = 1080
    csi_fps: int = 30

    # Health / failover thresholds
    frame_timeout_s: float = 1.0        # no frame for this long -> source stale
    max_consecutive_errors: int = 5     # read errors before declaring dead
    open_retries: int = 2               # re-open attempts before failover
    recovery_probe_period_s: float = 5.0  # how often to re-probe a dead primary
    watchdog_period_s: float = 0.25


# --------------------------------------------------------------- Failsafe ---
@dataclass
class FailsafeConfig:
    # Camera failures
    camera_switch_notice: bool = True       # STATUSTEXT to GCS on failover
    all_cameras_lost_action: str = "LOITER"  # hold position, keep trying
    all_cameras_lost_rtl_after_s: float = 20.0  # then give up and RTL

    # Link failures (companion-side view; Pixhawk has its own failsafes)
    link_lost_action: str = "RTL"
    link_lost_grace_s: float = 5.0

    # Winch / payload (mirrors the 3DR GCS-side triggers)
    winch_release_confirm_timeout_s: float = 10.0
    winch_servo_channel: int = 9            # Pixhawk AUX1 = servo output 9
    winch_pwm_release: int = 1900
    winch_pwm_hold: int = 1100
    winch_pwm_brake: int = 1500


@dataclass
class Config:
    link: LinkConfig = field(default_factory=LinkConfig)
    camera: CameraConfig = field(default_factory=CameraConfig)
    failsafe: FailsafeConfig = field(default_factory=FailsafeConfig)


DEFAULT = Config()
