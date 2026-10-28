"""
AeroTHON 2026 Track 1 — field geometry from the organisers' field document
("AeroTHON 2026_Trak1_Feild_Properties_Dimensions.pdf", received Oct 2026).

Every competition dimension the code depends on lives here, so the changes
the Technical Committee announces on 29 Oct (event registration) only need
editing in this one file.

Units: metres unless the name says otherwise.
"""

from __future__ import annotations

import dataclasses

FT = 0.3048
INCH = 0.0254


@dataclasses.dataclass(frozen=True)
class FieldSpec:
    # ---------------------------------------------------------- Corridor ---
    # 4 walls with white screens, 8 ft high, 22 ft long. Two walls placed
    # 11.5 ft apart form one corridor (forward lap), the other two form the
    # exit/return corridor (with static obstacles).
    CORRIDOR_WALL_HEIGHT_M: float = 8.0 * FT  # 2.44 m
    CORRIDOR_LENGTH_M: float = 22.0 * FT  # 6.71 m
    CORRIDOR_WIDTH_M: float = 11.5 * FT  # 3.51 m between walls

    # ------------------------------------------- Entry / exit banners ------
    # Entry (forward) corridor: GREEN #00B140. Exit corridor: BLUE #0000FF.
    # Each corridor has one banner on its arch and one on the ground at the
    # entrance. All banners have a 1-inch white border.
    BANNER_LENGTH_M: float = 11.5 * FT  # 3.51 m
    ARCH_BANNER_HEIGHT_M: float = 1.5 * FT  # 0.46 m
    GROUND_BANNER_WIDTH_M: float = 1.0 * FT  # 0.30 m
    BANNER_BORDER_M: float = 1.0 * INCH
    GREEN_BANNER_RGB: tuple[int, int, int] = (0x00, 0xB1, 0x40)
    BLUE_BANNER_RGB: tuple[int, int, int] = (0x00, 0x00, 0xFF)

    # ------------------------------------- Static obstacles (exit corridor) --
    OBSTACLE_COUNT: int = 3
    OBSTACLE_WIDTH_M: float = 3.5 * FT  # 1.07 m
    OBSTACLE_HEIGHT_M: float = 6.0 * FT  # 1.83 m

    # ------------------------------------------- Take-off / landing pad -----
    PAD_SIZE_M: float = 4.0 * FT  # 1.22 m square
    PAD_HEIGHT_M: float = 0.5 * FT  # 0.15 m (6 in) raised podium

    # -------------------------------------------------- Delivery zone ------
    DELIVERY_ZONE_LENGTH_M: float = 40.0
    DELIVERY_ZONE_WIDTH_M: float = 30.0
    QR_SIZE_M: float = 1.0  # 1 m x 1 m print, NO white border
    QR_COUNT_DELIVERY: int = 5
    RED_ZONE_SIZE_M: float = 10.0 * FT  # 3.05 m square, #FF0000, 1 in border
    PAYLOAD_LOWER_HEIGHT_M: float = 5.0  # lowered by pulley from 5 m

    # --------------------------------------------------------- Layout ------
    # NOT given in the document (organisers place things on the day).
    # Measured in the local frame: origin = centre of the take-off pad,
    # x = forward (towards the corridors), y = right. Re-measure on 29 Oct.
    START_QR_X_M: float = 2.0
    CORRIDOR_ENTRY_X_M: float = 4.0  # where the forward corridor begins
    FORWARD_CORRIDOR_Y_M: float = 0.0  # centreline of the forward corridor
    RETURN_CORRIDOR_Y_M: float = 4.5  # centreline of the exit/return corridor
    DELIVERY_ZONE_GAP_M: float = 2.0  # corridor end -> delivery zone edge

    # Compass heading of the x axis (pad -> corridors). None = take it from
    # the way the drone faces at arming: PLACE THE DRONE ON THE PAD FACING
    # THE CORRIDOR ENTRANCE. Set a number only if that is not possible.
    COURSE_HEADING_DEG: float | None = None

    # ---------------------------------------------------- Flight choices ---
    # Fly mid-height inside the 2.44 m walls: clears the ground, stays well
    # below the wall tops and the arch banner.
    CORRIDOR_FLIGHT_ALT_M: float = 1.5

    @property
    def corridor_exit_x(self) -> float:
        return self.CORRIDOR_ENTRY_X_M + self.CORRIDOR_LENGTH_M

    @property
    def corridor_half_width(self) -> float:
        return self.CORRIDOR_WIDTH_M / 2.0


FIELD = FieldSpec()
