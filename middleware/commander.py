"""Commander — glue between the mission FSM, the CameraManager, and the
MavlinkBridge, plus the onboard failsafe policy.

The FSM talks only to this class. It never touches pymavlink or cameras
directly, so the flight code stays testable with mocks.

Failsafe policy (companion side — the Pixhawk keeps its own geofence/RC/
battery failsafes as the final authority):

  camera failover        -> keep flying, STATUSTEXT the GCS, tell the FSM
                            which mount geometry is now active.
  all cameras lost       -> LOITER immediately; if nothing recovers within
                            `all_cameras_lost_rtl_after_s`, request RTL.
  FC link lost           -> nothing we can command (no link) — log, and on
                            link restore re-assert the FSM's desired state.
  winch release overdue  -> brake winch + RTL (mirror of the GCS-side 3DR
                            trigger, so either end can catch it).
"""
import logging
import threading
import time

from .config import Config, DEFAULT

log = logging.getLogger("middleware.commander")


class Commander:
    def __init__(self, bridge, camera_manager, cfg: Config = DEFAULT,
                 on_notice=None):
        """bridge: MavlinkBridge (or mock). camera_manager: CameraManager
        (or mock). on_notice: optional callable(str) — mirrored to the FSM
        / logs in addition to GCS STATUSTEXT."""
        self.bridge = bridge
        self.cams = camera_manager
        self.cfg = cfg
        self.on_notice = on_notice or (lambda s: None)

        self._lock = threading.Lock()
        self._all_lost_since = None
        self._rtl_requested = False
        self._winch_active_since = None
        self._winch_released = False

        # wire events
        self.cams.on_event = self._on_camera_event
        self.bridge.on_event = self._on_link_event

    # -------------------------------------------------------------- notices
    def _notify(self, text, severity=6):
        log.info("NOTICE: %s", text)
        self.on_notice(text)
        if self.bridge.link_ok:
            self.bridge.statustext(text, severity)

    # ------------------------------------------------------- camera events
    def _on_camera_event(self, event, detail):
        if event == "failover":
            self._notify(f"CAM FAILOVER {detail['from']}>{detail['to']}",
                         severity=4)
            self._all_lost_since = None
        elif event == "switched":
            self._notify(f"CAM SWITCH {detail['from']}>{detail['to']}")
        elif event == "recovered":
            self._all_lost_since = None
            if detail.get("active"):
                self._notify(f"CAM RESTORED {detail['name']}")
            else:
                # standby recovery — FSM may call switch_primary() when safe
                self._notify(f"CAM STANDBY {detail['name']}")
        elif event == "all_lost":
            self._handle_all_cameras_lost()

    def _handle_all_cameras_lost(self):
        now = time.monotonic()
        with self._lock:
            first = self._all_lost_since is None
            if first:
                self._all_lost_since = now
            elapsed = now - self._all_lost_since
            escalate = (not self._rtl_requested and
                        elapsed >= self.cfg.failsafe.all_cameras_lost_rtl_after_s)
            if escalate:
                self._rtl_requested = True
        if first:
            self._notify("ALL CAMS LOST - LOITER", severity=2)
            if self.cfg.failsafe.all_cameras_lost_action == "LOITER":
                self.bridge.set_mode("LOITER")
        elif escalate:
            self._notify("CAMS STILL LOST - RTL", severity=2)
            self.request_rtl("all cameras lost")

    # --------------------------------------------------------- link events
    def _on_link_event(self, event, detail):
        if event == "link_lost":
            log.error("FC link lost — companion cannot command; FC failsafes "
                      "in charge")
            self.on_notice("FC LINK LOST")
        elif event == "link_up":
            self._notify(f"FC LINK UP {detail.get('endpoint', '')}")

    # ------------------------------------------------------ camera control
    def read_frame(self):
        """(frame, source_name) — source_name tells the vision pipeline
        which mount geometry to use (oak-d-lite = 45 deg / arducam-csi =
        bottom-facing)."""
        return self.cams.read()

    def switch_camera(self, name: str) -> bool:
        return self.cams.switch_to(name)

    def use_bottom_camera(self) -> bool:
        """Winch/delivery phase: bottom-facing Arducam for the drop view."""
        return self.cams.switch_to("arducam-csi")

    def use_primary_camera(self) -> bool:
        return self.cams.switch_to("oak-d-lite")

    # ------------------------------------------------------ flight commands
    def arm_and_takeoff(self, alt_m: float) -> bool:
        if not self.bridge.set_mode("GUIDED").accepted:
            return False
        if not self.bridge.arm().accepted:
            return False
        r = self.bridge.takeoff(alt_m)
        if r.accepted:
            self._notify(f"TAKEOFF {alt_m:.0f}M")
        return r.accepted

    def goto(self, lat, lon, alt_rel_m, ground_speed=-1.0) -> bool:
        return self.bridge.goto(lat, lon, alt_rel_m, ground_speed).accepted

    def loiter(self) -> bool:
        return self.bridge.set_mode("LOITER").accepted

    def land(self) -> bool:
        return self.bridge.set_mode("LAND").accepted

    def request_rtl(self, reason: str) -> bool:
        self._notify(f"RTL: {reason[:40]}", severity=2)
        r = self.bridge.rtl()
        if not r.accepted:            # belt and braces: mode change fallback
            r = self.bridge.set_mode("RTL")
        return r.accepted

    # -------------------------------------------------------------- winch
    def winch_release(self) -> bool:
        fs = self.cfg.failsafe
        r = self.bridge.set_servo(fs.winch_servo_channel, fs.winch_pwm_release)
        if r.accepted:
            with self._lock:
                self._winch_active_since = time.monotonic()
                self._winch_released = False
            self._notify("WINCH RELEASE CMD")
        return r.accepted

    def winch_brake(self) -> bool:
        r = self.bridge.set_servo(self.cfg.failsafe.winch_servo_channel,
                                  self.cfg.failsafe.winch_pwm_brake)
        if r.accepted:
            self._notify("WINCH BRAKE")
        return r.accepted

    def confirm_payload_released(self):
        """FSM calls this when the release is visually/electrically
        confirmed (vision drop detect or DRV8838 current signature)."""
        with self._lock:
            self._winch_released = True
            self._winch_active_since = None
        self._notify("PAYLOAD RELEASED OK")

    # ------------------------------------------------------------ tick
    def tick(self):
        """Call from the FSM main loop (~30 ms cycle). Runs time-based
        failsafe checks that aren't event-driven."""
        fs = self.cfg.failsafe
        with self._lock:
            winch_t0 = self._winch_active_since
            released = self._winch_released
        if (winch_t0 is not None and not released and
                time.monotonic() - winch_t0
                >= fs.winch_release_confirm_timeout_s):
            with self._lock:
                self._winch_active_since = None
            self._notify("WINCH TIMEOUT - BRAKE+RTL", severity=2)
            self.winch_brake()
            self.request_rtl("winch release unconfirmed")

    # ------------------------------------------------------------ health
    def health(self) -> dict:
        return {
            "link_ok": self.bridge.link_ok,
            "link_endpoint": self.bridge.active_endpoint,
            "fc": self.bridge.get_state(),
            "cameras": self.cams.status(),
            "rtl_requested": self._rtl_requested,
        }
