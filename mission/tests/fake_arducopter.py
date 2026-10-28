"""
A small stand-in for a dronekit Vehicle connected to ArduCopter in GUIDED,
plus a virtual clock, so MAVLinkFlightController can be tested end to end
without SITL.

It models the behaviour that matters for the FSM:
  * SET_POSITION_TARGET_LOCAL_NED velocity / position setpoints in
    LOCAL_NED or BODY_NED (body = rotated by the current heading)
  * yaw: if a setpoint carries a yaw target the nose turns to it; if yaw is
    ignored, ArduCopter's default WP_YAW_BEHAVIOR turns the nose toward the
    direction of travel and cancels any CONDITION_YAW
  * MAV_CMD_CONDITION_YAW, takeoff, LAND (disarms on touchdown)
  * setpoints expire after 3 s (GUIDED velocity timeout)

It is a kinematic model, not a flight simulator. SITL is still the real test.
"""

from __future__ import annotations

import math
import types

YAW_IGNORE = 1024
VEL_IGNORE = 8 | 16 | 32
POS_IGNORE = 1 | 2 | 4
FRAME_LOCAL_NED = 1
FRAME_BODY_NED = 8
CMD_CONDITION_YAW = 115


class VirtualClock:
    def __init__(self, vehicle: "FakeArduCopter") -> None:
        self.t = 1000.0
        self.vehicle = vehicle

    def time(self) -> float:
        return self.t

    def sleep(self, dt: float) -> None:
        steps = max(1, int(round(dt / 0.02)))
        for _ in range(steps):
            self.t += dt / steps
            self.vehicle.physics(dt / steps)

    def __getattr__(self, name):  # perf_counter, monotonic, etc.
        import time as _t

        return getattr(_t, name)


class _Factory:
    def set_position_target_local_ned_encode(self, *a):
        return ("SPT",) + a

    def command_long_encode(self, *a):
        return ("CMD",) + a


class FakeArduCopter:
    def __init__(self, heading_deg: float = 0.0) -> None:
        self.n = self.e = self.alt = 0.0
        self.vn = self.ve = self.vd = 0.0
        self.heading = math.radians(heading_deg)
        self.yaw_target: float | None = None
        self.follow_velocity_yaw = False
        self.cmd_v = (0.0, 0.0, 0.0)
        self.cmd_pos: tuple[float, float, float] | None = None
        self.last_sp = -1e9
        self.armed = False
        self.mode = types.SimpleNamespace(name="STABILIZE")
        self.takeoff_alt: float | None = None
        self.message_factory = _Factory()
        self.version = "ArduCopter-fake"
        self.clock = VirtualClock(self)
        self.heading_trace: list[float] = []

    # ------------------------------------------------ dronekit-ish surface
    @property
    def attitude(self):
        return types.SimpleNamespace(yaw=math.atan2(math.sin(self.heading), math.cos(self.heading)))

    @property
    def location(self):
        return types.SimpleNamespace(
            local_frame=types.SimpleNamespace(north=self.n, east=self.e, down=-self.alt),
            global_relative_frame=types.SimpleNamespace(lat=0.0, lon=0.0, alt=self.alt),
        )

    def simple_takeoff(self, alt: float) -> None:
        self.takeoff_alt = alt

    def close(self) -> None:
        pass

    def __setattr__(self, k, v):
        if k == "mode" and hasattr(self, "mode") and not isinstance(v, types.SimpleNamespace):
            v = types.SimpleNamespace(name=getattr(v, "name", str(v)))
        object.__setattr__(self, k, v)

    def send_mavlink(self, msg) -> None:
        if msg[0] == "CMD":
            cmd = msg[3]
            if cmd == CMD_CONDITION_YAW:
                ang, _rate, _dirn, rel = msg[5], msg[6], msg[7], msg[8]
                base = self.heading if rel else 0.0
                self.yaw_target = base + math.radians(ang)
                self.follow_velocity_yaw = False
            return
        _, _t, _ts, _tc, frame, mask, x, y, z, vx, vy, vz, _ax, _ay, _az, yaw, _yr = msg
        self.last_sp = self.clock.t
        if frame == FRAME_BODY_NED:
            c, s = math.cos(self.heading), math.sin(self.heading)
            vx, vy = vx * c - vy * s, vx * s + vy * c
        if not (mask & VEL_IGNORE) == VEL_IGNORE:
            self.cmd_v, self.cmd_pos = (vx, vy, vz), None
        elif not (mask & POS_IGNORE) == POS_IGNORE:
            self.cmd_pos, self.cmd_v = (x, y, z), (0.0, 0.0, 0.0)
        if mask & YAW_IGNORE:
            self.yaw_target, self.follow_velocity_yaw = None, True
        else:
            self.yaw_target, self.follow_velocity_yaw = yaw, False

    # ------------------------------------------------------------- physics
    def physics(self, dt: float) -> None:
        if self.mode.name == "LAND":
            self.vn = self.ve = 0.0
            self.alt = max(0.0, self.alt - 0.7 * dt)
            if self.alt <= 0.0:
                self.armed = False
            return
        if not self.armed:
            return
        if self.mode.name == "GUIDED" and self.takeoff_alt is not None:
            if self.alt < self.takeoff_alt - 0.05:
                self.alt = min(self.takeoff_alt, self.alt + 1.5 * dt)
                return
            self.takeoff_alt = None  # takeoff complete
        if self.clock.t - self.last_sp > 3.0:  # GUIDED setpoint timeout
            self.cmd_v, self.cmd_pos = (0.0, 0.0, 0.0), None
        if self.cmd_pos is not None:
            pn, pe, pd = self.cmd_pos
            tv = (0.8 * (pn - self.n), 0.8 * (pe - self.e), 0.8 * (pd + self.alt))
            tv = tuple(max(-2.0, min(2.0, v)) for v in tv)
        else:
            tv = self.cmd_v
        k = min(1.0, dt / 0.3)  # ~0.3 s velocity response
        self.vn += (tv[0] - self.vn) * k
        self.ve += (tv[1] - self.ve) * k
        self.vd += (tv[2] - self.vd) * k
        self.n += self.vn * dt
        self.e += self.ve * dt
        self.alt = max(0.0, self.alt - self.vd * dt)

        target = self.yaw_target
        if target is None and self.follow_velocity_yaw and math.hypot(self.vn, self.ve) > 0.25:
            target = math.atan2(self.ve, self.vn)
        if target is not None:
            err = math.atan2(math.sin(target - self.heading), math.cos(target - self.heading))
            step = math.radians(90.0) * dt
            self.heading += max(-step, min(step, err))
        self.heading_trace.append(self.heading)
