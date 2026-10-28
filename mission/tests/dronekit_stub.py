"""Minimal `dronekit` stand-in so tests run without dronekit installed."""

import sys
import types

if "dronekit" not in sys.modules:
    try:
        import dronekit  # noqa: F401
    except Exception:  # noqa: BLE001
        m = types.ModuleType("dronekit")

        class VehicleMode:
            def __init__(self, name: str) -> None:
                self.name = name

        m.VehicleMode = VehicleMode
        sys.modules["dronekit"] = m
