"""
Velocity Generator Module
Converts high-level goals into body velocity vector setpoints.
"""

from aerothon.navigation.corridor_controller import CorridorController
from aerothon.navigation.obstacle_avoidance import ObstacleAvoidance
from aerothon.perception.manager import PerceptionResult


class VelocityGenerator:
    """Generates (vx, vy, vz) velocity setpoint vectors for MAVLink command sending."""

    def __init__(self):
        self.corridor_ctrl = CorridorController()
        self.avoidance = ObstacleAvoidance()

    def generate_corridor_velocity(
        self, perception: PerceptionResult, base_speed: float = 1.5
    ) -> tuple[float, float, float]:
        """
        Computes (vx, vy, vz) for corridor traversal.
        """
        status = self.avoidance.evaluate_clearance(perception)
        vy = self.corridor_ctrl.compute_lateral_correction(perception)

        if status == "STOP":
            return 0.0, vy, 0.0
        elif status == "SLOW":
            return base_speed * 0.5, vy, 0.0
        else:
            return base_speed, vy, 0.0
