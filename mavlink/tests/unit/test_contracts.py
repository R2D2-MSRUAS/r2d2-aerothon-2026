from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from core.command_manager import CommandManager
from core.setpoint_manager import SetpointManager
from interfaces.command_request import CommandAction, CommandRequest
from interfaces.command_result import CommandStatus
from interfaces.flight_command import CoordinateFrame, FlightCommand, FlightCommandKind, Vector3


class ContractTests(unittest.TestCase):
    def test_velocity_requires_velocity_vector(self):
        command = FlightCommand("one", 10.0, 11.0, FlightCommandKind.VELOCITY, CoordinateFrame.BODY_NED)
        self.assertEqual(command.validate(10.5), "velocity commands require velocity_mps")

    def test_setpoint_manager_accepts_valid_hold_without_sending(self):
        manager = SetpointManager()
        command = FlightCommand("hold", 10.0, 11.0, FlightCommandKind.HOLD, CoordinateFrame.LOCAL_NED)
        self.assertTrue(manager.submit(command, now=10.1))
        self.assertIs(manager.latest, command)

    def test_setpoint_manager_rejects_expired_command(self):
        manager = SetpointManager()
        command = FlightCommand("expired", 10.0, 10.1, FlightCommandKind.VELOCITY, CoordinateFrame.BODY_NED, velocity_mps=Vector3(1, 0, 0))
        self.assertFalse(manager.submit(command, now=10.2))
        self.assertEqual(manager.last_rejection, "flight command has expired")

    def test_commands_are_disabled_by_default(self):
        result = CommandManager().submit(CommandRequest("arm", 1.0, CommandAction.ARM))
        self.assertEqual(result.status, CommandStatus.REJECTED)
        self.assertIn("disabled", result.detail)

    def test_enabled_commands_still_require_a_bound_vehicle_identity(self):
        result = CommandManager(commands_enabled=True).submit(CommandRequest("arm", 1.0, CommandAction.ARM))
        self.assertEqual(result.status, CommandStatus.REJECTED)
        self.assertIn("bound flight-controller", result.detail)

    def test_velocity_cannot_exceed_configured_limit(self):
        manager = SetpointManager(max_speed_mps=2.5)
        command = FlightCommand(
            "fast", 10.0, 11.0, FlightCommandKind.VELOCITY, CoordinateFrame.BODY_NED,
            velocity_mps=Vector3(2.6, 0.0, 0.0),
        )
        self.assertFalse(manager.submit(command, now=10.1))
        self.assertIn("2.5", manager.last_rejection)

    def test_command_ack_is_correlated_to_pending_command(self):
        manager = CommandManager(commands_enabled=True)
        manager._pending[400] = "arm-request"
        result = manager.handle_ack(400, 0, observed_at=2.0)
        self.assertEqual(result.status, CommandStatus.SUCCEEDED)
        self.assertEqual(manager.result_for("arm-request"), result)


if __name__ == "__main__":
    unittest.main()
