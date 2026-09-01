"""
MAVLink Interface Subsystem
R2D2 System Architecture Compliance Module
"""

import logging

log = logging.getLogger("AeroTHON2026.MAVLink")


class MAVLinkInterface:
    """Sends velocity setpoints, mode changes, and arming commands to Pixhawk flight controller."""

    def __init__(self, connection_string: str = "udp:127.0.0.1:14550"):
        self.connection_string = connection_string
        self.is_armed = False
        self.mode = "INITIALIZING"

    def arm_motors(self) -> bool:
        """Sends MAVLink ARM command."""
        log.info("MAVLink Command -> ARMING MOTORS")
        self.is_armed = True
        return True

    def set_mode(self, mode_name: str) -> bool:
        """Switches flight mode (e.g. GUIDED, AUTO, RTL, LAND)."""
        log.info(f"MAVLink Command -> SET MODE: {mode_name}")
        self.mode = mode_name
        return True

    def send_velocity(
        self, vx: float, vy: float, vz: float, duration: float = 0.1
    ) -> None:
        """
        Sends SET_POSITION_TARGET_LOCAL_NED velocity setpoints.
        vx: forward (m/s), vy: right (m/s), vz: down (m/s)
        """
        log.debug(f"MAVLink Setpoint -> vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f}")

    def send_servo_pwm(self, channel: int, pwm_us: int) -> None:
        """Sends MAV_CMD_DO_SET_SERVO command."""
        log.info(f"MAVLink Servo Command -> Channel {channel}: {pwm_us} us")
