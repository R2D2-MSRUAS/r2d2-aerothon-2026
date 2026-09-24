from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "mocks"))

from core.health_monitor import HealthMonitor
from core.state_manager import StateManager
from core.telemetry_manager import TelemetryManager
from interfaces.mavlink_health import HealthStatus
from interfaces.vehicle_state import VehicleState
from fake_message import FakeMessage


class TelemetryTests(unittest.TestCase):
    def setUp(self):
        self.state = StateManager(VehicleState(observed_at=0.0))
        self.health = HealthMonitor(heartbeat_stale_s=3.0, telemetry_stale_s=5.0)
        self.health.mark_transport(True)
        self.manager = TelemetryManager(self.state, self.health)

    def test_heartbeat_binds_armed_and_mode_information(self):
        self.assertTrue(self.manager.handle(FakeMessage("HEARTBEAT", base_mode=128, custom_mode=4), 1.0))
        vehicle = self.state.state
        self.assertTrue(vehicle.connected)
        self.assertTrue(vehicle.armed)
        self.assertEqual(vehicle.mode, "custom:4")
        self.assertEqual(self.health.evaluate(1.1).status, HealthStatus.DEGRADED)

    def test_position_attitude_and_battery_are_normalized(self):
        self.manager.handle(FakeMessage("HEARTBEAT", base_mode=0, custom_mode=0), 1.0)
        self.manager.handle(FakeMessage("GLOBAL_POSITION_INT", lat=125000000, lon=775000000, alt=880000, relative_alt=30000), 1.1)
        self.manager.handle(FakeMessage("ATTITUDE", roll=0.1, pitch=0.2, yaw=0.3, rollspeed=0.01, pitchspeed=0.02, yawspeed=0.03), 1.2)
        self.manager.handle(FakeMessage("SYS_STATUS", voltage_battery=22200, current_battery=340, battery_remaining=77), 1.3)
        vehicle = self.state.state
        self.assertEqual(vehicle.position.latitude_deg, 12.5)
        self.assertEqual(vehicle.position.longitude_deg, 77.5)
        self.assertAlmostEqual(vehicle.position.absolute_altitude_m, 880.0)
        self.assertAlmostEqual(vehicle.attitude.yaw_rad, 0.3)
        self.assertAlmostEqual(vehicle.battery.voltage_v, 22.2)
        self.assertAlmostEqual(vehicle.battery.current_a, 3.4)
        self.assertEqual(vehicle.battery.remaining_percent, 77)
        self.assertEqual(self.health.evaluate(1.4).status, HealthStatus.CONNECTED)

    def test_stale_heartbeat_is_reported(self):
        self.manager.handle(FakeMessage("HEARTBEAT"), 1.0)
        self.assertEqual(self.health.evaluate(4.1).status, HealthStatus.STALE)


if __name__ == "__main__":
    unittest.main()
