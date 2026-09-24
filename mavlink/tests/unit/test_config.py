from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from core.config import load_config


class ConfigTests(unittest.TestCase):
    def test_sitl_profile_loads_with_safe_defaults(self):
        root = Path(__file__).resolve().parents[2]
        config = load_config(root / "config" / "sitl.yaml")
        self.assertEqual(config.connection.transport, "udp")
        self.assertEqual(config.connection.endpoint, "udpin:0.0.0.0:14550")
        self.assertFalse(config.safety.commands_enabled)
        self.assertFalse(config.safety.automatic_actions_enabled)

    def test_aerothon_profile_is_explicitly_simulation_only(self):
        root = Path(__file__).resolve().parents[2]
        config = load_config(root / "config" / "aerothon_sitl.yaml")
        self.assertTrue(config.mission.enabled)
        self.assertTrue(config.mission.simulation_only)
        self.assertTrue(config.safety.commands_enabled)
        self.assertEqual(config.mission.corridor_alt_m, 3.048)


if __name__ == "__main__":
    unittest.main()
