"""Bench tests for the middleware — no hardware needed.

Uses fake camera sources and a mock MAVLink bridge to exercise:
  1. normal startup on the primary camera
  2. primary death -> automatic failover to backup
  3. primary recovery -> standby notice, FSM-commanded switch back
  4. all cameras lost -> LOITER, then RTL after the timeout
  5. winch release unconfirmed -> brake + RTL
  6. manual camera switching (delivery-phase bottom cam)

Run:  python -m middleware.test_bench
"""
import logging
import time

import numpy as np

from .cameras import CameraSource
from .camera_manager import CameraManager
from .commander import Commander
from .config import Config

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s %(name)s %(levelname)s %(message)s")


# ------------------------------------------------------------------ fakes --
class FakeCamera(CameraSource):
    def __init__(self, cfg, name):
        super().__init__(cfg)
        self.name = name
        self.alive = True          # flip to False to simulate device death
        self.can_open = True

    def open(self):
        if not self.can_open:
            return False
        self._open = True
        self.consecutive_errors = 0
        self.last_frame_ts = 0.0
        return True

    def _grab(self):
        if not self.alive:
            raise IOError("device gone")
        return np.zeros((8, 8, 3), np.uint8)


class FakeResult:
    accepted = True
    ack_result = 0
    error = None


class FakeBridge:
    """Records every command; always accepts."""
    def __init__(self):
        self.calls = []
        self.link_ok = True
        self.active_endpoint = "fake"
        self.on_event = lambda e, d: None

    def _rec(self, *a):
        self.calls.append(a)
        return FakeResult()

    def set_mode(self, m):        return self._rec("set_mode", m)
    def arm(self, force=False):   return self._rec("arm")
    def disarm(self, force=False): return self._rec("disarm")
    def takeoff(self, alt):       return self._rec("takeoff", alt)
    def rtl(self):                return self._rec("rtl")
    def set_servo(self, ch, pwm): return self._rec("set_servo", ch, pwm)
    def goto(self, *a):           return self._rec("goto", *a)
    def statustext(self, t, severity=6): self.calls.append(("statustext", t))
    def get_state(self):          return {}


def wait_for(pred, timeout=5.0, msg="condition"):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if pred():
            return
        time.sleep(0.05)
    raise AssertionError(f"timeout waiting for {msg}")


# ------------------------------------------------------------------ tests --
def main():
    cfg = Config()
    # fast thresholds so the test runs in seconds
    cfg.camera.frame_timeout_s = 0.4
    cfg.camera.max_consecutive_errors = 2
    cfg.camera.watchdog_period_s = 0.1
    cfg.camera.recovery_probe_period_s = 0.3
    cfg.camera.open_retries = 0
    cfg.failsafe.all_cameras_lost_rtl_after_s = 1.0
    cfg.failsafe.winch_release_confirm_timeout_s = 0.5

    oak = FakeCamera(cfg.camera, "oak-d-lite")
    csi = FakeCamera(cfg.camera, "arducam-csi")
    mgr = CameraManager([oak, csi], cfg.camera)
    bridge = FakeBridge()
    cmd = Commander(bridge, mgr, cfg)

    assert mgr.start(), "startup failed"
    frame, src = cmd.read_frame()
    assert src == "oak-d-lite" and frame is not None
    print("PASS 1: startup on primary")

    # 2. kill primary -> failover
    oak.alive = False
    oak.can_open = False
    wait_for(lambda: cmd.read_frame()[1] == "arducam-csi", msg="failover")
    assert any("CAM FAILOVER" in c[1] for c in bridge.calls
               if c[0] == "statustext")
    print("PASS 2: automatic failover to backup")

    # 3. primary recovers -> standby notice, then FSM switches back
    oak.alive = True
    oak.can_open = True
    wait_for(lambda: any("CAM STANDBY" in c[1] for c in bridge.calls
                         if c[0] == "statustext"), msg="standby recovery")
    assert mgr.active_name == "arducam-csi", "must NOT auto-switch back"
    assert cmd.use_primary_camera()
    assert cmd.read_frame()[1] == "oak-d-lite"
    print("PASS 3: standby recovery + commanded switch-back")

    # 4. kill both -> LOITER then RTL
    bridge.calls.clear()
    oak.alive = csi.alive = False
    oak.can_open = csi.can_open = False
    wait_for(lambda: ("set_mode", "LOITER") in bridge.calls,
             msg="LOITER on all-lost")
    wait_for(lambda: ("rtl",) in bridge.calls, timeout=5.0,
             msg="RTL escalation")
    print("PASS 4: all-cameras-lost -> LOITER -> RTL")

    # 5. winch timeout -> brake + RTL
    bridge.calls.clear()
    assert cmd.winch_release()
    t0 = time.monotonic()
    while time.monotonic() - t0 < 2.0:
        cmd.tick()
        time.sleep(0.05)
    assert ("set_servo", cfg.failsafe.winch_servo_channel,
            cfg.failsafe.winch_pwm_brake) in bridge.calls
    assert ("rtl",) in bridge.calls
    print("PASS 5: winch unconfirmed -> brake + RTL")

    # 5b. confirmed release does NOT trigger failsafe
    bridge.calls.clear()
    assert cmd.winch_release()
    cmd.confirm_payload_released()
    time.sleep(0.7)
    cmd.tick()
    assert ("rtl",) not in bridge.calls
    print("PASS 5b: confirmed release -> no failsafe")

    # 6. manual switching both ways (cameras back alive)
    oak.alive = csi.alive = True
    oak.can_open = csi.can_open = True
    wait_for(lambda: mgr.active_name is not None, msg="restore")
    assert cmd.use_bottom_camera() and mgr.active_name == "arducam-csi"
    assert cmd.use_primary_camera() and mgr.active_name == "oak-d-lite"
    print("PASS 6: manual delivery-phase switching")

    mgr.stop()
    print("\nALL BENCH TESTS PASSED")


if __name__ == "__main__":
    main()
