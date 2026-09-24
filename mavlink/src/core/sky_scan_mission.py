"""Windows-SITL execution harness for AeroTHON Mission 2.

The class intentionally orchestrates flight intent only.  QR, banner, red-zone
and obstacle decisions belong to the perception/navigation layer.  The bundled
runner has an explicit ``simulated_perception`` mode so the complete MAVLink
sequence can be rehearsed before that layer is connected.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from time import monotonic
from typing import Optional
from uuid import uuid4

from interfaces.command_request import CommandAction, CommandRequest
from interfaces.flight_command import CoordinateFrame, FlightCommand, FlightCommandKind, Vector3


class MissionPhase(str, Enum):
    GUIDED = "GUIDED"
    ARM = "ARM"
    TAKEOFF = "TAKEOFF"
    START_QR = "START_QR"
    OUTBOUND_BANNER = "OUTBOUND_BANNER"
    DESCEND_CORRIDOR = "DESCEND_CORRIDOR"
    OUTBOUND_CORRIDOR = "OUTBOUND_CORRIDOR"
    ASCEND_DELIVERY = "ASCEND_DELIVERY"
    TARGET_QR = "TARGET_QR"
    DESCEND_DROP = "DESCEND_DROP"
    PAYLOAD_SIMULATION = "PAYLOAD_SIMULATION"
    ASCEND_RETURN = "ASCEND_RETURN"
    TURN_HOME = "TURN_HOME"
    RETURN_BANNER = "RETURN_BANNER"
    DESCEND_RETURN_CORRIDOR = "DESCEND_RETURN_CORRIDOR"
    INBOUND_CORRIDOR = "INBOUND_CORRIDOR"
    LAND = "LAND"
    COMPLETE = "COMPLETE"
    ABORTED = "ABORTED"


@dataclass(frozen=True)
class MissionEvent:
    phase: MissionPhase
    detail: str


class SkyScanMission:
    """A conservative mission sequencer driven by normalized ``VehicleState``."""

    def __init__(self, system, simulated_perception: bool = False) -> None:
        self.system = system
        self.profile = system.config.mission
        self.simulated_perception = simulated_perception
        self.phase = MissionPhase.GUIDED
        self.started_at = monotonic()
        self.phase_started_at = self.started_at
        self.events: list[MissionEvent] = []
        self._sent_current_action = False
        self._emit("requesting GUIDED mode")

    @property
    def finished(self) -> bool:
        return self.phase in {MissionPhase.COMPLETE, MissionPhase.ABORTED}

    @property
    def success(self) -> bool:
        return self.phase is MissionPhase.COMPLETE

    def step(self, now: Optional[float] = None) -> None:
        now = monotonic() if now is None else now
        if self.finished:
            return
        if now - self.started_at > self.profile.max_mission_time_s:
            self._abort("mission time limit exceeded")
            return
        if self.system.health.status.value not in {"CONNECTED", "DEGRADED"}:
            return

        if self.phase is MissionPhase.GUIDED:
            self._command_once(CommandAction.SET_MODE, {"mode": "GUIDED"})
            if self._phase_age(now) >= 1.0:
                self._advance(MissionPhase.ARM, "GUIDED request sent; requesting arm")
        elif self.phase is MissionPhase.ARM:
            self._command_once(CommandAction.ARM)
            if self.system.vehicle_state.armed:
                self._advance(MissionPhase.TAKEOFF, "armed; requesting autonomous takeoff")
            elif self._phase_age(now) > 10.0:
                self._abort("arming timeout")
        elif self.phase is MissionPhase.TAKEOFF:
            self._command_once(CommandAction.TAKEOFF, {"altitude_m": self.profile.takeoff_alt_m})
            if self._at_altitude(self.profile.takeoff_alt_m):
                self._advance(MissionPhase.START_QR, "takeoff altitude reached; scanning start QR")
            elif self._phase_age(now) > self.profile.takeoff_timeout_s:
                self._abort("takeoff altitude timeout")
        elif self.phase is MissionPhase.START_QR:
            if self._perception_ready(now, "start QR decoded (simulated)"):
                self._advance(MissionPhase.OUTBOUND_BANNER, "start QR confirmed; locating outbound corridor banner")
        elif self.phase is MissionPhase.OUTBOUND_BANNER:
            if self._perception_ready(now, "outbound green corridor banner confirmed (simulated)"):
                self._advance(MissionPhase.DESCEND_CORRIDOR, "outbound corridor aligned; descending to corridor altitude")
        elif self.phase is MissionPhase.DESCEND_CORRIDOR:
            self._vertical_to(self.profile.corridor_alt_m, now)
            if self._at_altitude(self.profile.corridor_alt_m):
                self._advance(MissionPhase.OUTBOUND_CORRIDOR, "corridor altitude reached; transiting outbound")
        elif self.phase is MissionPhase.OUTBOUND_CORRIDOR:
            self._velocity(self.profile.corridor_speed_mps, 0.0, 0.0, now)
            if self._phase_age(now) >= self.profile.corridor_length_m / self.profile.corridor_speed_mps:
                self._advance(MissionPhase.ASCEND_DELIVERY, "outbound corridor distance complete; ascending for target search")
        elif self.phase is MissionPhase.ASCEND_DELIVERY:
            self._vertical_to(self.profile.delivery_search_alt_m, now)
            if self._at_altitude(self.profile.delivery_search_alt_m):
                self._advance(MissionPhase.TARGET_QR, "delivery-search altitude reached; locating target QR")
        elif self.phase is MissionPhase.TARGET_QR:
            if self._perception_ready(now, "delivery QR verified outside red zone (simulated)"):
                self._advance(MissionPhase.DESCEND_DROP, "target QR confirmed; descending for delivery")
        elif self.phase is MissionPhase.DESCEND_DROP:
            self._vertical_to(self.profile.payload_drop_alt_m, now)
            if self._at_altitude(self.profile.payload_drop_alt_m):
                self._advance(MissionPhase.PAYLOAD_SIMULATION, "drop altitude reached; running payload timing simulation")
        elif self.phase is MissionPhase.PAYLOAD_SIMULATION:
            # Real winch output is explicitly excluded: SITL has no physical payload feedback.
            self._hold(now)
            if self._phase_age(now) >= 10.5:  # 4.5 s lower + 1.5 s settle + 4.5 s reel
                self._advance(MissionPhase.ASCEND_RETURN, "payload timing complete; ascending for return")
        elif self.phase is MissionPhase.ASCEND_RETURN:
            self._vertical_to(self.profile.return_cruise_alt_m, now)
            if self._at_altitude(self.profile.return_cruise_alt_m):
                self._advance(MissionPhase.TURN_HOME, "return altitude reached; commanding 180-degree turn")
        elif self.phase is MissionPhase.TURN_HOME:
            self._command_once(CommandAction.YAW, {"angle_deg": 180.0, "rate_dps": self.profile.yaw_rate_dps})
            if self._phase_age(now) >= 180.0 / self.profile.yaw_rate_dps + 1.0:
                self._advance(MissionPhase.RETURN_BANNER, "turn time complete; locating return corridor banner")
        elif self.phase is MissionPhase.RETURN_BANNER:
            if self._perception_ready(now, "return green corridor banner confirmed (simulated)"):
                self._advance(MissionPhase.DESCEND_RETURN_CORRIDOR, "return corridor aligned; descending to corridor altitude")
        elif self.phase is MissionPhase.DESCEND_RETURN_CORRIDOR:
            self._vertical_to(self.profile.corridor_alt_m, now)
            if self._at_altitude(self.profile.corridor_alt_m):
                self._advance(MissionPhase.INBOUND_CORRIDOR, "return corridor altitude reached; transiting inbound")
        elif self.phase is MissionPhase.INBOUND_CORRIDOR:
            self._velocity(self.profile.corridor_speed_mps, 0.0, 0.0, now)
            if self._phase_age(now) >= self.profile.corridor_length_m / self.profile.corridor_speed_mps:
                self._advance(MissionPhase.LAND, "inbound corridor distance complete; requesting land")
        elif self.phase is MissionPhase.LAND:
            self._command_once(CommandAction.LAND)
            if self.system.vehicle_state.armed is False:
                self._advance(MissionPhase.COMPLETE, "vehicle disarmed after landing")
            elif self._phase_age(now) > 45.0:
                self._abort("landing timeout")

    def _vertical_to(self, target_altitude_m: float, now: float) -> None:
        altitude = self._relative_altitude()
        if altitude is None:
            self._hold(now)
            return
        vertical_speed = self.profile.slow_speed_mps
        down_mps = -vertical_speed if altitude < target_altitude_m else vertical_speed
        self._velocity(0.0, 0.0, down_mps, now)

    def _velocity(self, forward_mps: float, right_mps: float, down_mps: float, now: float) -> None:
        command = FlightCommand(
            command_id=str(uuid4()),
            created_at=now,
            valid_until=now + self.system.config.safety.setpoint_expiry_s,
            kind=FlightCommandKind.VELOCITY,
            frame=CoordinateFrame.BODY_NED,
            velocity_mps=Vector3(forward_mps, right_mps, down_mps),
            source="sky_scan_sitl",
        )
        self.system.submit_flight_command(command, now)
        self.system.service_setpoint(now)

    def _hold(self, now: float) -> None:
        command = FlightCommand(
            command_id=str(uuid4()),
            created_at=now,
            valid_until=now + self.system.config.safety.setpoint_expiry_s,
            kind=FlightCommandKind.HOLD,
            frame=CoordinateFrame.BODY_NED,
            source="sky_scan_sitl",
        )
        self.system.submit_flight_command(command, now)
        self.system.service_setpoint(now)

    def _command_once(self, action: CommandAction, parameters: Optional[dict] = None) -> None:
        if self._sent_current_action:
            return
        result = self.system.submit_command(
            CommandRequest(str(uuid4()), monotonic(), action, parameters or {}, source="sky_scan_sitl")
        )
        self._sent_current_action = True
        if result.status.value == "REJECTED":
            self._abort(f"{action.value} rejected: {result.detail}")

    def _perception_ready(self, now: float, simulation_detail: str) -> bool:
        if self.simulated_perception and self._phase_age(now) >= 1.0:
            self._emit(simulation_detail)
            return True
        return False

    def _at_altitude(self, target_altitude_m: float) -> bool:
        altitude = self._relative_altitude()
        return altitude is not None and abs(altitude - target_altitude_m) <= self.profile.altitude_tolerance_m

    def _relative_altitude(self) -> Optional[float]:
        position = self.system.vehicle_state.position
        return None if position is None else position.relative_altitude_m

    def _phase_age(self, now: float) -> float:
        return now - self.phase_started_at

    def _advance(self, phase: MissionPhase, detail: str) -> None:
        self.phase = phase
        self.phase_started_at = monotonic()
        self._sent_current_action = False
        self._emit(detail)

    def _abort(self, detail: str) -> None:
        if self.phase is not MissionPhase.LAND and self.system.target_identity is not None:
            self.system.submit_command(CommandRequest(str(uuid4()), monotonic(), CommandAction.LAND, source="sky_scan_sitl"))
        self.phase = MissionPhase.ABORTED
        self._emit(detail)

    def _emit(self, detail: str) -> None:
        event = MissionEvent(self.phase, detail)
        self.events.append(event)
        print(f"[{event.phase.value}] {event.detail}")
