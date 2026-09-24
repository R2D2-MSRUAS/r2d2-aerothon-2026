"""Tiny Navigation stand-in for contract-level integration tests.

It produces HOLD intent only; it does not request velocity or position movement.
"""

from __future__ import annotations

from time import monotonic
from uuid import uuid4

from interfaces.flight_command import CoordinateFrame, FlightCommand, FlightCommandKind


def hold_command(valid_for_s: float = 0.5) -> FlightCommand:
    now = monotonic()
    return FlightCommand(
        command_id=str(uuid4()),
        created_at=now,
        valid_until=now + valid_for_s,
        kind=FlightCommandKind.HOLD,
        frame=CoordinateFrame.LOCAL_NED,
        source="mock_navigation",
    )
