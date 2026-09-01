"""
Obstacle Avoidance & Emergency Stop Handler
R2D2 Navigation & Control Subsystem
"""

from aerothon.perception.manager import PerceptionResult


class ObstacleAvoidance:
    """Checks forward clearance and red zone safety flags to prevent collisions."""

    def __init__(self, safe_dist_m: float = 2.0, critical_stop_m: float = 0.8):
        self.safe_dist_m = safe_dist_m
        self.critical_stop_m = critical_stop_m

    def evaluate_clearance(self, perception: PerceptionResult) -> str:
        """
        Returns status string: 'CLEAR', 'SLOW', or 'STOP'.
        """
        if perception.red_zone_detected:
            return "STOP"

        fwd_dist = perception.obstacles.get("forward", float("inf"))

        if fwd_dist < self.critical_stop_m:
            return "STOP"
        elif fwd_dist < self.safe_dist_m:
            return "SLOW"
        else:
            return "CLEAR"
