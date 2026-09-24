from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "mocks"))

from core.config import ConnectionConfig, MavlinkConfig
from core.mavlink_system import MavlinkSystem
from fake_message import FakeMessage


class SystemTests(unittest.TestCase):
    def test_first_accepted_heartbeat_binds_identity(self):
        system = MavlinkSystem(MavlinkConfig(connection=ConnectionConfig("udp", "udpin:0.0.0.0:14550")))
        self.assertTrue(system._accept(FakeMessage("HEARTBEAT", source_system=1, source_component=1)))
        self.assertEqual(system.target_identity, (1, 1))
        self.assertFalse(system._accept(FakeMessage("ATTITUDE", source_system=2, source_component=1)))

    def test_expected_identity_rejects_other_vehicle(self):
        config = MavlinkConfig(connection=ConnectionConfig("udp", "udpin:0.0.0.0:14550", expected_system_id=1, expected_component_id=1))
        system = MavlinkSystem(config)
        self.assertFalse(system._accept(FakeMessage("HEARTBEAT", source_system=2, source_component=1)))
        self.assertIsNone(system.target_identity)


if __name__ == "__main__":
    unittest.main()
