"""Exercise the real pymavlink decoder over UDP without ArduPilot.

This validates the local transport boundary. The separate SITL test remains the
acceptance test for ArduPilot itself.
"""

from __future__ import annotations

import io
from pathlib import Path
import socket
import sys
from time import monotonic, sleep
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

try:
    from pymavlink.dialects.v20 import common as mavlink2
except ImportError:  # pragma: no cover - dependency test environment
    mavlink2 = None

from core.config import ConnectionConfig, MavlinkConfig
from core.mavlink_system import MavlinkSystem


@unittest.skipIf(mavlink2 is None, "pymavlink is not installed")
class UdpPymavlinkTests(unittest.TestCase):
    def test_real_udp_mavlink_frames_produce_vehicle_state(self):
        port_reservation = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        port_reservation.bind(("127.0.0.1", 0))
        port = port_reservation.getsockname()[1]
        port_reservation.close()
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        system = MavlinkSystem(MavlinkConfig(connection=ConnectionConfig("udp", f"udpin:127.0.0.1:{port}")))
        try:
            system.connect()
            payload = _frames()
            sender.sendto(payload, ("127.0.0.1", port))
            deadline = monotonic() + 2.0
            while monotonic() < deadline and system.vehicle_state.battery is None:
                system.poll_once()
                sleep(0.01)
            self.assertEqual(system.target_identity, (1, 1))
            self.assertEqual(system.vehicle_state.position.latitude_deg, 12.5)
            self.assertEqual(system.vehicle_state.battery.remaining_percent, 77)
        finally:
            system.close()
            sender.close()


def _frames() -> bytes:
    stream = io.BytesIO()
    mav = mavlink2.MAVLink(stream, srcSystem=1, srcComponent=1)
    mav.heartbeat_send(mavlink2.MAV_TYPE_QUADROTOR, mavlink2.MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 4, 3)
    mav.global_position_int_send(0, 125000000, 775000000, 880000, 30000, 0, 0, 0, 0)
    mav.sys_status_send(0, 0, 0, 0, 22200, 340, 77, 0, 0, 0, 0, 0, 0)
    return stream.getvalue()
