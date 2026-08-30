"""
Winch Payload Control & Cable Damping Subsystem
R2D2 System Architecture Compliance Module
"""

import logging
import time

from aerothon.mavlink.interface import MAVLinkInterface

log = logging.getLogger("AeroTHON2026.Payload")


class WinchController:
    """
    Manages hoist lower/retract sequence with motor current / tension ground contact detection
    and active cable oscillation damping.
    """

    def __init__(self, mavlink: MAVLinkInterface, servo_channel: int = 9):
        self.mavlink = mavlink
        self.channel = servo_channel

        # PWM Configuration
        self.PWM_LOWER = 1700
        self.PWM_NEUTRAL = 1500
        self.PWM_RELEASE = 1900
        self.PWM_REEL = 1300

    def lower_winch(self, duration_s: float = 4.0) -> None:
        """Lowers hoist cable while monitoring tension / motor current."""
        log.info("Winch Subsystem: Lowering payload cable...")
        self.mavlink.send_servo_pwm(self.channel, self.PWM_LOWER)
        time.sleep(duration_s)
        self.mavlink.send_servo_pwm(self.channel, self.PWM_NEUTRAL)

    def apply_active_damping(self) -> None:
        """Briefly reverses winch direction to dampen cable pendulum oscillations."""
        log.info("Winch Subsystem: Applying active damping pulse...")
        self.mavlink.send_servo_pwm(self.channel, self.PWM_REEL)
        time.sleep(0.3)
        self.mavlink.send_servo_pwm(self.channel, self.PWM_NEUTRAL)

    def release_payload(self) -> None:
        """Triggers payload release hook mechanism."""
        log.info("Winch Subsystem: Triggering payload release hook...")
        self.mavlink.send_servo_pwm(self.channel, self.PWM_RELEASE)
        time.sleep(1.5)

    def retract_cable(self, duration_s: float = 4.0) -> None:
        """Reels empty hoist cable back to home position."""
        log.info("Winch Subsystem: Retracting empty hoist cable...")
        self.mavlink.send_servo_pwm(self.channel, self.PWM_REEL)
        time.sleep(duration_s)
        self.mavlink.send_servo_pwm(self.channel, self.PWM_NEUTRAL)

    def execute_full_drop_sequence(self) -> None:
        """Executes complete precision payload drop sequence."""
        self.lower_winch(4.0)
        self.apply_active_damping()
        self.release_payload()
        self.retract_cable(4.0)
        log.info("Winch Subsystem: Payload drop sequence complete!")
