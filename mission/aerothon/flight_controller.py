"""
AeroTHON 2026 — Flight Controller & Winch Actuation Interface
================================================================================
Features:
  - Abstract Flight Controller Interface
  - MockFlightController for drone-free testing & simulation
  - MAVLinkFlightController using DroneKit / pymavlink with robust timeouts
  - Hard timeouts on Arming, Takeoff, Altitude Changes, Landing, and MAVLink connection
  - Real flight execution disabled by default
  - Automated Winch deployment and post-drop CoG feed-forward compensation
"""

from __future__ import annotations

import abc
import logging
import math
import time
from typing import Any

from aerothon.config import DEFAULT_CONFIG, MissionConfig

log = logging.getLogger("AeroTHON2026.FC")


class FlightControllerInterface(abc.ABC):
    """Abstract interface for multirotor flight controller."""

    @abc.abstractmethod
    def get_position_ned(self) -> tuple[float, float, float, float]:
        """Returns (x_north, y_east, z_down, yaw_deg)."""

    @abc.abstractmethod
    def send_body_velocity(
        self, vx: float, vy: float, vz: float, duration: float = 0.2
    ) -> None:
        """Sends velocity setpoint in Body-Fixed Frame (vx: forward, vy: right, vz: down)."""

    @abc.abstractmethod
    def send_local_velocity(
        self, vx_n: float, vy_e: float, vz_d: float, duration: float = 0.2
    ) -> None:
        """Sends velocity setpoint in Local World NED Frame."""

    @abc.abstractmethod
    def arm_and_takeoff(self, target_alt: float) -> bool:
        """Arms UAS and climbs to target altitude."""

    @abc.abstractmethod
    def set_altitude(self, target_alt: float) -> bool:
        """Changes altitude to target_alt while holding lateral position."""

    @abc.abstractmethod
    def set_yaw(self, heading_deg: float, relative: bool = True) -> None:
        """Rotates UAS heading."""

    @abc.abstractmethod
    def set_winch(self, pwm_us: int) -> None:
        """Commands winch servo PWM pulse."""

    @abc.abstractmethod
    def stop(self) -> None:
        """Zeroes all body velocity setpoints to hold hover."""

    @abc.abstractmethod
    def return_to_launch(self) -> None:
        """Initiates RTL failsafe."""

    @abc.abstractmethod
    def land(self) -> bool:
        """Executes autonomous landing."""

    @abc.abstractmethod
    def close(self) -> None:
        """Releases flight controller connection."""


class MockFlightController(FlightControllerInterface):
    """
    Safe drone-free mock flight controller simulating internal flight dynamics
    and recording all dispatched commands for unit testing.
    """

    def __init__(self, config: MissionConfig = DEFAULT_CONFIG) -> None:
        self.config = config
        self.sim_x = 0.0
        self.sim_y = 0.0
        self.sim_z = 0.0
        self.sim_yaw = 0.0
        self.armed = False
        self.mode = "GUIDED"
        self.command_log: list[str] = []
        self.winch_pwm_history: list[int] = []

    def get_position_ned(self) -> tuple[float, float, float, float]:
        return (self.sim_x, self.sim_y, self.sim_z, self.sim_yaw)

    def send_body_velocity(
        self, vx: float, vy: float, vz: float, duration: float = 0.2
    ) -> None:
        vx = max(-self.config.MAX_SPEED_MPS, min(self.config.MAX_SPEED_MPS, vx))
        vy = max(-self.config.MAX_SPEED_MPS, min(self.config.MAX_SPEED_MPS, vy))
        vz = max(-self.config.MAX_SPEED_MPS, min(self.config.MAX_SPEED_MPS, vz))

        yaw_rad = math.radians(self.sim_yaw)
        gx = vx * math.cos(yaw_rad) - vy * math.sin(yaw_rad)
        gy = vx * math.sin(yaw_rad) + vy * math.cos(yaw_rad)

        self.sim_x += gx * duration
        self.sim_y += gy * duration
        self.sim_z += vz * duration
        self.command_log.append(
            f"BODY_VEL vx={vx:.2f} vy={vy:.2f} vz={vz:.2f} dt={duration:.2f}"
        )

    def send_local_velocity(
        self, vx_n: float, vy_e: float, vz_d: float, duration: float = 0.2
    ) -> None:
        self.sim_x += vx_n * duration
        self.sim_y += vy_e * duration
        self.sim_z += vz_d * duration
        self.command_log.append(
            f"LOCAL_VEL vx={vx_n:.2f} vy={vy_e:.2f} vz={vz_d:.2f} dt={duration:.2f}"
        )

    def arm_and_takeoff(self, target_alt: float) -> bool:
        self.armed = True
        self.sim_z = -target_alt
        self.command_log.append(f"TAKEOFF alt={target_alt:.2f}")
        log.info(f"[MOCK FC] Armed and climbed to {target_alt:.1f} m.")
        return True

    def set_altitude(self, target_alt: float) -> bool:
        self.sim_z = -target_alt
        self.command_log.append(f"SET_ALT alt={target_alt:.2f}")
        log.info(f"[MOCK FC] Set altitude -> {target_alt:.1f} m.")
        return True

    def set_yaw(self, heading_deg: float, relative: bool = True) -> None:
        if relative:
            self.sim_yaw = (self.sim_yaw + heading_deg) % 360.0
        else:
            self.sim_yaw = heading_deg % 360.0
        self.command_log.append(f"SET_YAW deg={heading_deg:.1f} rel={relative}")
        log.info(f"[MOCK FC] Yaw set to {self.sim_yaw:.1f}°.")

    def set_winch(self, pwm_us: int) -> None:
        self.winch_pwm_history.append(pwm_us)
        self.command_log.append(f"WINCH pwm={pwm_us}")
        log.info(f"[MOCK FC] Winch PWM -> {pwm_us} µs.")

    def stop(self) -> None:
        self.command_log.append("STOP")

    def return_to_launch(self) -> None:
        self.mode = "RTL"
        self.sim_x = 0.0
        self.sim_y = 0.0
        self.sim_z = 0.0
        self.command_log.append("RTL")
        log.info("[MOCK FC] Return to Launch executed.")

    def land(self) -> bool:
        self.mode = "LAND"
        self.sim_z = 0.0
        self.armed = False
        self.command_log.append("LAND")
        log.info("[MOCK FC] Landed and disarmed.")
        return True

    def close(self) -> None:
        self.command_log.append("CLOSE")


class MAVLinkFlightController(FlightControllerInterface):
    """
    Real MAVLink Flight Controller Interface with strict timeout bounds on all loops.
    """

    def __init__(
        self, connection_string: str, config: MissionConfig = DEFAULT_CONFIG
    ) -> None:
        self.config = config
        self.connection_string = connection_string
        self._vehicle: Any = None
        self._connect()

    def _connect(self) -> None:
        from dronekit import connect as dk_connect  # type: ignore

        log.info(
            f"[FC] Connecting to Flight Controller at '{self.connection_string}'..."
        )
        start_time = time.time()
        self._vehicle = dk_connect(
            self.connection_string,
            wait_ready=True,
            heartbeat_timeout=int(self.config.FC_CONNECTION_TIMEOUT_S),
        )
        elapsed = time.time() - start_time
        log.info(f"[FC] Connected in {elapsed:.1f}s. Firmware: {self._vehicle.version}")

    def get_position_ned(self) -> tuple[float, float, float, float]:
        if self._vehicle is None:
            return (0.0, 0.0, 0.0, 0.0)
        loc = self._vehicle.location.local_frame
        alt = float(self._vehicle.location.global_relative_frame.alt or 0.0)
        yaw = (
            float(self._vehicle.attitude.yaw * 180.0 / math.pi)
            if self._vehicle.attitude
            else 0.0
        )
        x = float(loc.north) if loc and loc.north is not None else 0.0
        y = float(loc.east) if loc and loc.east is not None else 0.0
        z = -alt
        return (x, y, z, yaw)

    def send_body_velocity(
        self, vx: float, vy: float, vz: float, duration: float = 0.2
    ) -> None:
        if self._vehicle is None:
            return
        from pymavlink import mavutil  # type: ignore

        vx = max(-self.config.MAX_SPEED_MPS, min(self.config.MAX_SPEED_MPS, vx))
        vy = max(-self.config.MAX_SPEED_MPS, min(self.config.MAX_SPEED_MPS, vy))
        vz = max(-self.config.MAX_SPEED_MPS, min(self.config.MAX_SPEED_MPS, vz))

        msg = self._vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0,
            0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111,
            0,
            0,
            0,
            vx,
            vy,
            vz,
            0,
            0,
            0,
            0,
            0,
        )
        end_time = time.time() + duration
        while time.time() < end_time:
            self._vehicle.send_mavlink(msg)
            time.sleep(0.1)

    def send_local_velocity(
        self, vx_n: float, vy_e: float, vz_d: float, duration: float = 0.2
    ) -> None:
        if self._vehicle is None:
            return
        from pymavlink import mavutil  # type: ignore

        msg = self._vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0,
            0,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111,
            0,
            0,
            0,
            vx_n,
            vy_e,
            vz_d,
            0,
            0,
            0,
            0,
            0,
        )
        end_time = time.time() + duration
        while time.time() < end_time:
            self._vehicle.send_mavlink(msg)
            time.sleep(0.1)

    def arm_and_takeoff(self, target_alt: float) -> bool:
        if self._vehicle is None:
            return False
        from dronekit import VehicleMode  # type: ignore

        log.info(
            f"[FC] Setting mode GUIDED and arming (Timeout: {self.config.ARM_TIMEOUT_S}s)..."
        )
        self._vehicle.mode = VehicleMode("GUIDED")
        self._vehicle.armed = True

        arm_start = time.time()
        while not self._vehicle.armed:
            if time.time() - arm_start > self.config.ARM_TIMEOUT_S:
                log.error("[FC TIMEOUT] Arming timed out!")
                return False
            time.sleep(0.3)

        log.info(
            f"[FC] Taking off to {target_alt:.1f} m (Timeout: {self.config.TAKEOFF_TIMEOUT_S}s)..."
        )
        self._vehicle.simple_takeoff(target_alt)

        climb_start = time.time()
        while True:
            if time.time() - climb_start > self.config.TAKEOFF_TIMEOUT_S:
                log.error("[FC TIMEOUT] Takeoff altitude climb timed out!")
                return False
            cur_alt = float(self._vehicle.location.global_relative_frame.alt or 0.0)
            if cur_alt >= target_alt - self.config.ALTITUDE_TOLERANCE_M:
                break
            time.sleep(0.2)

        log.info(f"[FC] Takeoff complete at {target_alt:.1f} m.")
        return True

    def set_altitude(self, target_alt: float) -> bool:
        if self._vehicle is None:
            return False
        from dronekit import LocationGlobalRelative  # type: ignore

        log.info(f"[FC] Changing altitude to {target_alt:.1f} m...")
        cur = self._vehicle.location.global_relative_frame
        target = LocationGlobalRelative(cur.lat, cur.lon, target_alt)
        self._vehicle.simple_goto(target)

        alt_start = time.time()
        while (
            abs(
                float(self._vehicle.location.global_relative_frame.alt or 0.0)
                - target_alt
            )
            > self.config.ALTITUDE_TOLERANCE_M
        ):
            if time.time() - alt_start > self.config.ALTITUDE_CHANGE_TIMEOUT_S:
                log.warning(
                    "[FC TIMEOUT] Altitude transition timed out; proceeding with current altitude."
                )
                return False
            time.sleep(0.2)
        return True

    def set_yaw(self, heading_deg: float, relative: bool = True) -> None:
        if self._vehicle is None:
            return
        from pymavlink import mavutil  # type: ignore

        msg = self._vehicle.message_factory.command_long_encode(
            0,
            0,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            heading_deg,
            self.config.YAW_RATE_DPS,
            1 if relative else 0,
            1 if relative else 0,
            0,
            0,
            0,
        )
        self._vehicle.send_mavlink(msg)
        time.sleep(1.0)

    def set_winch(self, pwm_us: int) -> None:
        if self._vehicle is None:
            return
        from pymavlink import mavutil  # type: ignore

        msg = self._vehicle.message_factory.command_long_encode(
            0,
            0,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,
            self.config.WINCH_SERVO_CHANNEL,
            pwm_us,
            0,
            0,
            0,
            0,
            0,
        )
        self._vehicle.send_mavlink(msg)

    def stop(self) -> None:
        self.send_body_velocity(0, 0, 0, 0.2)

    def return_to_launch(self) -> None:
        if self._vehicle is None:
            return
        from dronekit import VehicleMode  # type: ignore

        log.warning("[FC] Triggering RTL (Return to Launch)!")
        self._vehicle.mode = VehicleMode("RTL")

    def land(self) -> bool:
        if self._vehicle is None:
            return False
        from dronekit import VehicleMode  # type: ignore

        log.info(
            f"[FC] Switching to LAND mode (Timeout: {self.config.LANDING_TIMEOUT_S}s)..."
        )
        self._vehicle.mode = VehicleMode("LAND")
        land_start = time.time()
        while float(self._vehicle.location.global_relative_frame.alt or 0.0) > 0.15:
            if time.time() - land_start > self.config.LANDING_TIMEOUT_S:
                log.warning("[FC TIMEOUT] Landing descent timed out.")
                return False
            time.sleep(0.3)
        log.info("[FC] Landed.")
        return True

    def close(self) -> None:
        if self._vehicle is not None:
            self._vehicle.close()
            self._vehicle = None


def create_flight_controller(
    connection_string: str = "",
    enable_real_flight: bool = False,
    config: MissionConfig = DEFAULT_CONFIG,
) -> FlightControllerInterface:
    """
    Factory function ensuring REAL FLIGHT EXECUTION IS DISABLED BY DEFAULT.
    Only connects to hardware when both connection_string and enable_real_flight=True are supplied.
    """
    if not enable_real_flight or not connection_string:
        if connection_string and not enable_real_flight:
            log.warning(
                "[SAFETY WARNING] --connect was provided but --enable-real-flight is FALSE. "
                "Defaulting to safe MockFlightController. Pass --enable-real-flight to arm physical hardware."
            )
        return MockFlightController(config=config)

    try:
        return MAVLinkFlightController(
            connection_string=connection_string, config=config
        )
    except Exception as err:  # noqa: BLE001
        log.error(
            f"Failed to connect to MAVLink Flight Controller ({err}). Falling back to MockFlightController."
        )
        return MockFlightController(config=config)


# ===========================================================================
# WINCH DEPLOYMENT & COG COMPENSATION SEQUENCE
# ===========================================================================
def execute_winch_delivery(
    fc: FlightControllerInterface, config: MissionConfig = DEFAULT_CONFIG
) -> None:
    """
    Precision winch drop sequence:
      1. Controlled lower
      2. Ground settle
      3. Release trigger
      4. Cable retraction
      5. Post-drop CoG feed-forward pitch bias compensation
    """
    log.info("[WINCH] Commencing controlled payload lowering...")
    fc.set_winch(config.WINCH_LOWER_PWM)
    time.sleep(config.WINCH_LOWER_TIME_S)

    fc.set_winch(config.WINCH_NEUTRAL_PWM)
    time.sleep(config.WINCH_HOLD_TIME_S)

    log.info("[WINCH] Triggering mechanical release latch...")
    fc.set_winch(config.WINCH_RELEASE_PWM)
    time.sleep(0.6)
    fc.set_winch(config.WINCH_NEUTRAL_PWM)
    time.sleep(0.4)

    log.info("[WINCH] Retracting cable...")
    fc.set_winch(config.WINCH_RAISE_PWM)
    time.sleep(config.WINCH_REEL_TIME_S)
    fc.set_winch(config.WINCH_NEUTRAL_PWM)
    log.info("[WINCH] Hoist sequence completed.")

    # Feed-forward CoG shift compensation: delta_cg = (m_payload * offset) / m_drone_empty
    cg_shift_m = (
        config.PAYLOAD_MASS_KG * config.PAYLOAD_COG_OFFSET_X_M
    ) / config.DRONE_EMPTY_MASS_KG
    log.info(
        f"[COG] Post-drop CoG shift = {cg_shift_m * 1000.0:.2f} mm. Applying feed-forward forward pitch bias."
    )
    fc.send_body_velocity(vx=cg_shift_m * 0.8, vy=0.0, vz=0.0, duration=1.5)
    time.sleep(0.5)
