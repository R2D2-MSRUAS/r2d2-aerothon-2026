"""
Corridor Controller & Wall-Centering Engine
R2D2 Navigation & Control Subsystem
"""

from aerothon.perception.manager import PerceptionResult


class CorridorController:
    """Computes lateral velocity corrections to keep drone centered in corridor."""

    def __init__(self, safe_thresh_m: float = 1.0, k_p: float = 0.5):
        self.safe_thresh_m = safe_thresh_m
        self.k_p = k_p

    def compute_lateral_correction(self, perception: PerceptionResult) -> float:
        """
        Returns vy (lateral velocity in m/s).
        vy > 0 steers right, vy < 0 steers left.
        """
        left_dist = perception.obstacles.get("left", float("inf"))
        right_dist = perception.obstacles.get("right", float("inf"))

        if left_dist == float("inf") and right_dist == float("inf"):
            return 0.0

        # Calculate imbalance: positive means closer to left wall
        imbalance = right_dist - left_dist

        # Apply proportional gain for smooth centering
        vy = self.k_p * imbalance

        # Clamp maximum lateral velocity to 0.6 m/s for safety
        vy = max(-0.6, min(0.6, vy))
        return vy
