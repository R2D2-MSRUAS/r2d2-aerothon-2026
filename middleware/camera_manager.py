"""CameraManager — priority failover and hot switching between sources.

Design (matches the Mission 2 architecture doc):

  * Sources are held in priority order — [OAK-D Lite, Arducam CSI] in flight.
  * A watchdog thread checks the active source every ``watchdog_period_s``.
    If it goes unhealthy (no frames / repeated read errors / device gone),
    the manager fails over to the next source down the list that opens.
  * A dead higher-priority source is re-probed every
    ``recovery_probe_period_s``. Recovery is *reported* via callback but the
    manager does NOT auto-switch back mid-flight — the FSM decides, because
    the two cameras have different mounts (45° tilt vs bottom-facing) and a
    silent swap would corrupt the alignment PID. Call ``switch_to()`` to act.
  * ``read()`` always serves frames from whatever is active, so the vision
    pipeline never has to know which physical camera is behind it.

Events emitted to the callback: ("failover", ...), ("recovered", ...),
("switched", ...), ("all_lost", ...). The commander turns these into
STATUSTEXT / failsafe actions.
"""
import logging
import threading
import time

from .cameras import CameraSource
from .config import CameraConfig

log = logging.getLogger("middleware.camera_manager")


class CameraManager:
    def __init__(self, sources, cfg: CameraConfig, on_event=None):
        """sources: list[CameraSource] in priority order (index 0 = primary).
        on_event: callable(event: str, detail: dict) — called from the
        watchdog thread; keep it fast and thread-safe."""
        if not sources:
            raise ValueError("need at least one camera source")
        self.sources = sources
        self.cfg = cfg
        self.on_event = on_event or (lambda evt, detail: None)

        self._active_idx = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._watchdog = None
        self._last_probe_ts = 0.0
        self._all_lost_since = None

    # ------------------------------------------------------------ lifecycle
    def start(self) -> bool:
        """Open the highest-priority source that works and start the
        watchdog. Returns False only if *no* source opens."""
        ok = self._activate_best(start_from=0)
        self._watchdog = threading.Thread(
            target=self._watchdog_loop, name="cam-watchdog", daemon=True)
        self._watchdog.start()
        return ok

    def stop(self):
        self._stop.set()
        if self._watchdog is not None:
            self._watchdog.join(timeout=2.0)
        with self._lock:
            for s in self.sources:
                s.close()
            self._active_idx = None

    # ------------------------------------------------------------ frame API
    def read(self):
        """Latest frame from the active source, or None. Also returns the
        source name so downstream can adjust geometry (tilt differs)."""
        with self._lock:
            src = self._active_source()
        if src is None:
            return None, None
        return src.read(), src.name

    @property
    def active_name(self):
        with self._lock:
            src = self._active_source()
        return src.name if src else None

    def status(self) -> dict:
        with self._lock:
            return {
                "active": self._active_source().name
                          if self._active_idx is not None else None,
                "sources": {
                    s.name: {"open": s.is_open, "healthy": s.healthy,
                             "errors": s.consecutive_errors}
                    for s in self.sources
                },
            }

    # ------------------------------------------------------------ switching
    def switch_to(self, name: str) -> bool:
        """Manual/FSM-commanded switch to a named source (e.g. switch back
        to a recovered primary, or force the bottom camera for the winch
        phase). Keeps the old source open until the new one delivers."""
        with self._lock:
            idx = self._index_of(name)
            if idx is None:
                log.error("switch_to: unknown source %r", name)
                return False
            if idx == self._active_idx:
                return True
            target = self.sources[idx]
            if not target.is_open and not self._open_with_retries(target):
                log.error("switch_to: %s failed to open", name)
                return False
            old = self._active_source()
            self._active_idx = idx
        if old is not None and old is not target:
            old.close()
        log.info("switched to %s", name)
        self.on_event("switched", {"to": name,
                                   "from": old.name if old else None})
        return True

    # ------------------------------------------------------------ internals
    def _active_source(self):
        return (self.sources[self._active_idx]
                if self._active_idx is not None else None)

    def _index_of(self, name):
        for i, s in enumerate(self.sources):
            if s.name == name:
                return i
        return None

    def _open_with_retries(self, src: CameraSource) -> bool:
        for attempt in range(1 + self.cfg.open_retries):
            if src.open():
                return True
            time.sleep(0.2 * (attempt + 1))
        return False

    def _activate_best(self, start_from: int) -> bool:
        """Try sources from `start_from` down the priority list; activate the
        first that opens."""
        for i in range(start_from, len(self.sources)):
            if self._open_with_retries(self.sources[i]):
                with self._lock:
                    self._active_idx = i
                self._all_lost_since = None
                return True
        with self._lock:
            self._active_idx = None
        if self._all_lost_since is None:
            self._all_lost_since = time.monotonic()
        return False

    def _watchdog_loop(self):
        while not self._stop.wait(self.cfg.watchdog_period_s):
            with self._lock:
                idx = self._active_idx
                src = self._active_source()

            # -- no camera at all: keep retrying the whole list -------------
            if src is None:
                if self._activate_best(0):
                    name = self.active_name
                    log.warning("camera restored: %s", name)
                    self.on_event("recovered", {"name": name, "active": True})
                else:
                    self.on_event("all_lost", {
                        "since_s": time.monotonic() - self._all_lost_since})
                continue

            # -- active source failed: fail over down the list --------------
            if not src.healthy:
                log.warning("%s unhealthy (errors=%d, last frame %.1fs ago)",
                            src.name, src.consecutive_errors,
                            time.monotonic() - src.last_frame_ts
                            if src.last_frame_ts else -1)
                src.close()
                if self._activate_best(idx + 1):
                    new = self.active_name
                    log.warning("failover: %s -> %s", src.name, new)
                    self.on_event("failover", {"from": src.name, "to": new})
                else:
                    log.error("all cameras lost")
                    self.on_event("all_lost", {"since_s": 0.0})
                continue

            # -- healthy: periodically re-probe dead higher-priority sources
            now = time.monotonic()
            if idx > 0 and (now - self._last_probe_ts
                            >= self.cfg.recovery_probe_period_s):
                self._last_probe_ts = now
                for j in range(idx):
                    probe = self.sources[j]
                    if probe.is_open:
                        continue
                    if probe.open():
                        # Report recovery; leave it open and warm so a
                        # switch_to() from the FSM is instant.
                        log.info("%s recovered (standby)", probe.name)
                        self.on_event("recovered",
                                      {"name": probe.name, "active": False})
                        break
