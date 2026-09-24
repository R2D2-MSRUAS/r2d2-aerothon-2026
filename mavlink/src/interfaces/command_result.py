"""Results for finite MAVLink command transactions."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional


class CommandStatus(str, Enum):
    ACCEPTED = "ACCEPTED"
    IN_PROGRESS = "IN_PROGRESS"
    SUCCEEDED = "SUCCEEDED"
    REJECTED = "REJECTED"
    FAILED = "FAILED"
    TIMED_OUT = "TIMED_OUT"


@dataclass(frozen=True)
class CommandResult:
    request_id: str
    status: CommandStatus
    observed_at: float
    mav_result: Optional[int] = None
    detail: Optional[str] = None
