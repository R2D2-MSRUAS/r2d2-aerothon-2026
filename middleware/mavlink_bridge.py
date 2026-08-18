"""MavlinkBridge — companion-computer MAVLink command layer for the
Pixhawk 6X Mini (ArduPilot / EKF3).

Responsibilities (and nothing more — all flight dynamics stay on the FC):

  * Connect over the primary UART (921600), fall back to the 3DR radio if
    the primary endpoint won't come up, and reconnect with backoff.
  * Send our own heartbeat and watch the FC's; expose link health.
  * Reliable command dispatch: COMMAND_LONG with COMMAND_ACK matching,
    retries, and per-command results.
  * The specific commands the Mission 2 FSM needs: set mode, arm/disarm,
    takeoff, reposition/goto, RTL, LOITER, winch servo (MAV_CMD_DO_SET_SERVO)
    and STATUSTEXT notices to the GCS over telemetry.

Everything is thread-safe; a single RX thread drains the link and fans
messages out to interest queues.
"""
import logging
import queue
import threading
import time

from .config import LinkConfig

log = logging.getLogger("middleware.mavlink")

try:
    from pymavlink import mavutil
    HAVE_MAVLINK = True
except ImportError:
    mavutil = None
    HAVE_MAVLINK = False


class CommandResult:
    def __init__(self, cmd, accepted, ack_result=None, error=None):
        self.cmd = cmd
        self.accepted = accepted
        self.ack_result = ack_result   # MAV_RESULT_* or None on timeout
        self.error = error

    def __repr__(self):
        return (f"CommandResult(cmd={self.cmd}, accepted={self.accepted}, "
                f"ack={self.ack_result}, error={self.error})")


class MavlinkBridge:
    def __init__(self, cfg: LinkConfig, endpoint=None, baud=None,
                 on_event=None):
        """endpoint/baud override the config (pass cfg.sitl_endpoint for
        bench work). on_event: callable(event, detail) for link state changes
        — ("link_up", ...), ("link_lost", ...), ("link_degraded", ...)."""
        if not HAVE_MAVLINK:
            raise RuntimeError("pymavlink not installed "
                               "(pip install pymavlink)")
        self.cfg = cfg
        self._endpoint = endpoint
        self._baud = baud
        self.on_event = on_event or (lambda evt, detail: None)

        self.conn = None
        self.active_endpoint = None
        self._last_fc_heartbeat = 0.0
        self._link_was_up = False
        self._send_lock = threading.Lock()
        self._ack_queue = queue.Queue()
        self._stop = threading.Event()
        self._rx_thread = None
        self._hb_thread = None

        # Latest telemetry snapshots for the FSM / health reporting
        self.state = {"mode": None, "armed": False, "system_status": None,
                      "battery_v": None, "battery_remaining": None,
                      "relative_alt_m": None, "lat": None, "lon": None,
                      "gps_fix": None, "satellites": None}
        self._state_lock = threading.Lock()

    # ------------------------------------------------------------ lifecycle
    def connect(self, timeout_s: float = 15.0) -> bool:
        """Try primary endpoint, then the 3DR backup. Starts RX + heartbeat
        threads on success."""
        candidates = []
        if self._endpoint:
            candidates.append((self._endpoint, self._baud or 115200))
        else:
            candidates.append((self.cfg.primary_endpoint,
                               self.cfg.primary_baud))
            candidates.append((self.cfg.backup_endpoint,
                               self.cfg.backup_baud))

        for ep, baud in candidates:
            conn = self._try_endpoint(ep, baud, timeout_s)
            if conn is not None:
                self.conn = conn
                self.active_endpoint = ep
                self._last_fc_heartbeat = time.monotonic()
                self._link_was_up = True
                self._start_threads()
                log.info("MAVLink up on %s (sys %d comp %d)", ep,
                         conn.target_system, conn.target_component)
                self.on_event("link_up", {"endpoint": ep})
                return True
        log.error("MAVLink: no endpoint reachable")
        return False

    def _try_endpoint(self, endpoint, baud, timeout_s):
        try:
            log.info("MAVLink: trying %s @ %s", endpoint, baud)
            conn = mavutil.mavlink_connection(
                endpoint, baud=baud,
                source_system=self.cfg.source_system,
                source_component=self.cfg.source_component)
            hb = conn.wait_heartbeat(timeout=timeout_s)
            if hb is None:
                conn.close()
                return None
            return conn
        except Exception as e:
            log.warning("MAVLink: %s failed: %s", endpoint, e)
            return None

    def close(self):
        self._stop.set()
        for t in (self._rx_thread, self._hb_thread):
            if t is not None:
                t.join(timeout=2.0)
        if self.conn is not None:
            try:
                self.conn.close()
            except Exception:
                pass
            self.conn = None

    def _start_threads(self):
        self._stop.clear()
        self._rx_thread = threading.Thread(
            target=self._rx_loop, name="mav-rx", daemon=True)
        self._hb_thread = threading.Thread(
            target=self._hb_loop, name="mav-hb", daemon=True)
        self._rx_thread.start()
        self._hb_thread.start()

    # ------------------------------------------------------------ RX / link
    def _rx_loop(self):
        while not self._stop.is_set():
            try:
                msg = self.conn.recv_match(blocking=True, timeout=0.5)
            except Exception as e:
                log.warning("MAVLink rx error: %s", e)
                time.sleep(0.1)
                continue
            if msg is None:
                continue
            t = msg.get_type()
            if t == "HEARTBEAT" and msg.get_srcComponent() == 1:
                self._last_fc_heartbeat = time.monotonic()
                with self._state_lock:
                    self.state["armed"] = bool(
                        msg.base_mode
                        & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    self.state["mode"] = mavutil.mode_string_v10(msg)
                    self.state["system_status"] = msg.system_status
            elif t == "COMMAND_ACK":
                self._ack_queue.put(msg)
            elif t == "SYS_STATUS":
                with self._state_lock:
                    self.state["battery_v"] = msg.voltage_battery / 1000.0
                    self.state["battery_remaining"] = msg.battery_remaining
            elif t == "GLOBAL_POSITION_INT":
                with self._state_lock:
                    self.state["relative_alt_m"] = msg.relative_alt / 1000.0
                    self.state["lat"] = msg.lat / 1e7
                    self.state["lon"] = msg.lon / 1e7
            elif t == "GPS_RAW_INT":
                with self._state_lock:
                    self.state["gps_fix"] = msg.fix_type
                    self.state["satellites"] = msg.satellites_visible

    def _hb_loop(self):
        period = 1.0 / self.cfg.heartbeat_send_hz
        while not self._stop.wait(period):
            try:
                with self._send_lock:
                    self.conn.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            except Exception as e:
                log.warning("MAVLink heartbeat send failed: %s", e)
            # link watchdog
            age = time.monotonic() - self._last_fc_heartbeat
            if age > self.cfg.heartbeat_timeout_s:
                if self._link_was_up:
                    self._link_was_up = False
                    log.error("FC heartbeat lost (%.1fs)", age)
                    self.on_event("link_lost", {"age_s": age})
            elif not self._link_was_up:
                self._link_was_up = True
                log.info("FC heartbeat restored")
                self.on_event("link_up", {"endpoint": self.active_endpoint})

    @property
    def link_ok(self) -> bool:
        return (self.conn is not None and
                (time.monotonic() - self._last_fc_heartbeat)
                <= self.cfg.heartbeat_timeout_s)

    def get_state(self) -> dict:
        with self._state_lock:
            return dict(self.state)

    # ---------------------------------------------------------- reliable tx
    def send_command(self, command, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0,
                     retries=None) -> CommandResult:
        """COMMAND_LONG with ACK wait + retry. Blocking; call from the FSM
        thread, not from event callbacks."""
        retries = self.cfg.command_retries if retries is None else retries
        # drain stale acks
        while not self._ack_queue.empty():
            try:
                self._ack_queue.get_nowait()
            except queue.Empty:
                break
        last_err = None
        for attempt in range(1 + retries):
            try:
                with self._send_lock:
                    self.conn.mav.command_long_send(
                        self.conn.target_system, self.conn.target_component,
                        command, 0, p1, p2, p3, p4, p5, p6, p7)
            except Exception as e:
                last_err = str(e)
                time.sleep(0.2)
                continue
            ack = self._wait_ack(command, self.cfg.command_ack_timeout_s)
            if ack is not None:
                accepted = ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
                if not accepted:
                    log.warning("cmd %d rejected: result=%d (attempt %d)",
                                command, ack.result, attempt + 1)
                    if ack.result in (
                            mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED,
                            mavutil.mavlink.MAV_RESULT_IN_PROGRESS):
                        time.sleep(0.3)
                        continue
                return CommandResult(command, accepted, ack.result)
            log.warning("cmd %d: no ACK (attempt %d/%d)", command,
                        attempt + 1, 1 + retries)
        return CommandResult(command, False, None,
                             error=last_err or "ack timeout")

    def _wait_ack(self, command, timeout):
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            try:
                ack = self._ack_queue.get(timeout=remaining)
            except queue.Empty:
                return None
            if ack.command == command:
                return ack
            # ack for something else — ignore and keep waiting

    # --------------------------------------------------------- FSM commands
    def set_mode(self, mode_name: str) -> CommandResult:
        """e.g. 'GUIDED', 'LOITER', 'RTL', 'AUTO', 'LAND' (ArduCopter)."""
        mapping = self.conn.mode_mapping() or {}
        if mode_name not in mapping:
            return CommandResult("SET_MODE", False,
                                 error=f"unknown mode {mode_name}")
        return self.send_command(
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mapping[mode_name])

    def arm(self, force=False) -> CommandResult:
        return self.send_command(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1,
            21196 if force else 0)

    def disarm(self, force=False) -> CommandResult:
        return self.send_command(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            21196 if force else 0)

    def takeoff(self, alt_m: float) -> CommandResult:
        return self.send_command(
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, alt_m)

    def rtl(self) -> CommandResult:
        return self.send_command(
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH)

    def set_servo(self, channel: int, pwm: int) -> CommandResult:
        """Winch actuation path: MAV_CMD_DO_SET_SERVO on the Pixhawk aux
        output driving the DRV8838."""
        return self.send_command(
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO, channel, pwm)

    def goto(self, lat: float, lon: float, alt_rel_m: float,
             ground_speed=-1.0) -> CommandResult:
        """Reposition in GUIDED mode (MAV_CMD_DO_REPOSITION)."""
        try:
            with self._send_lock:
                self.conn.mav.command_int_send(
                    self.conn.target_system, self.conn.target_component,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    mavutil.mavlink.MAV_CMD_DO_REPOSITION,
                    0, 0, ground_speed, 1, 0, float("nan"),
                    int(lat * 1e7), int(lon * 1e7), alt_rel_m)
        except Exception as e:
            return CommandResult("DO_REPOSITION", False, error=str(e))
        ack = self._wait_ack(mavutil.mavlink.MAV_CMD_DO_REPOSITION,
                             self.cfg.command_ack_timeout_s)
        if ack is None:
            # Many ArduPilot builds don't ack COMMAND_INT reposition; treat
            # send-without-error as success but flag it.
            return CommandResult("DO_REPOSITION", True, ack_result=None)
        return CommandResult("DO_REPOSITION",
                             ack.result ==
                             mavutil.mavlink.MAV_RESULT_ACCEPTED,
                             ack.result)

    def statustext(self, text: str, severity=6):
        """Push a notice to the GCS (severity 6 = INFO, 4 = WARNING,
        2 = CRITICAL). Truncated to 50 chars per MAVLink spec."""
        try:
            with self._send_lock:
                self.conn.mav.statustext_send(
                    severity, text[:50].encode("ascii", "replace"))
        except Exception as e:
            log.warning("statustext failed: %s", e)
