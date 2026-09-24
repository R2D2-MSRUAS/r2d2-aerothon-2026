"""Optional integration test; it runs only when a live SITL endpoint is supplied."""

import os
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from core.config import ConnectionConfig, MavlinkConfig
from core.mavlink_system import MavlinkSystem


@unittest.skipUnless(os.getenv("MAVLINK_SITL_ENDPOINT"), "set MAVLINK_SITL_ENDPOINT to run against live SITL")
class SitlConnectionTests(unittest.TestCase):
    def test_receives_heartbeat_and_telemetry(self):
        endpoint = os.environ["MAVLINK_SITL_ENDPOINT"]
        system = MavlinkSystem(MavlinkConfig(connection=ConnectionConfig("udp", endpoint)))
        try:
            system.connect()
            self.assertTrue(system.wait_for_connection(timeout_s=15.0))
            self.assertIsNotNone(system.target_identity)
        finally:
            system.close()
