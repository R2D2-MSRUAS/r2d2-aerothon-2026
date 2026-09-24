"""Discrete command contracts. v0.1 deliberately keeps them disabled by default."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Mapping


class CommandAction(str, Enum):
    ARM = "ARM"
    DISARM = "DISARM"
    TAKEOFF = "TAKEOFF"
    LAND = "LAND"
    RTL = "RTL"
    YAW = "YAW"
    SET_MODE = "SET_MODE"
    WINCH = "WINCH"


@dataclass(frozen=True)
class CommandRequest:
    request_id: str
    created_at: float
    action: CommandAction
    parameters: Mapping[str, Any] = field(default_factory=dict)
    source: str = "fsm"
    requires_ack: bool = True
