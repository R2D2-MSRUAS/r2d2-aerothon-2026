"""
Perception Manager & PerceptionResult Data Structure
R2D2 System Architecture Compliance Module
"""

from dataclasses import dataclass, field


@dataclass
class PerceptionResult:
    """
    Unified Perception Output aggregated from vision sub-detectors.
    Passed directly to Mission FSM and Navigation Controller.
    """
    banner_detected: bool = False
    banner_center: tuple[int, int] | None = None
    banner_distance: float | None = None
    qr_codes: list[dict[str, float]] = field(default_factory=list)
    obstacles: dict[str, float] = field(default_factory=lambda: {"forward": float("inf"), "left": float("inf"), "right": float("inf")})
    red_zones: list[dict[str, float]] = field(default_factory=list)
    red_zone_detected: bool = False
    corridor_offset: float = 0.0
    corridor_width: float = 3.5


class PerceptionManager:
    """Aggregates raw camera, depth, and vision sub-module outputs into PerceptionResult."""

    def __init__(self):
        self.latest_result = PerceptionResult()

    def update(
        self,
        banner_center: tuple[int, int] | None = None,
        banner_distance: float | None = None,
        qr_codes: list[dict[str, float]] | None = None,
        obstacles: dict[str, float] | None = None,
        red_zones: list[dict[str, float]] | None = None,
        corridor_offset: float = 0.0,
    ) -> PerceptionResult:
        """Composes and returns the updated PerceptionResult object."""
        result = PerceptionResult()
        
        # Banner Detection
        if banner_center is not None:
            result.banner_detected = True
            result.banner_center = banner_center
            result.banner_distance = banner_distance
            
        # QR Codes
        if qr_codes:
            result.qr_codes = qr_codes
            
        # Obstacles (3-column depth analysis)
        if obstacles:
            result.obstacles = obstacles
            
        # Red Zones
        if red_zones:
            result.red_zones = red_zones
            result.red_zone_detected = len(red_zones) > 0
            
        # Corridor Offset
        result.corridor_offset = corridor_offset
        
        self.latest_result = result
        return result
