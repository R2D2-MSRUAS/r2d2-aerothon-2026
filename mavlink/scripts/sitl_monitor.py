"""Run the v0.1 heartbeat/telemetry monitor against ArduPilot SITL.

This program does not arm, set mode, or transmit setpoints. It only receives
MAVLink telemetry and prints normalized VehicleState plus health changes.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import asdict
from pathlib import Path
from time import monotonic, sleep

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from core.config import load_config
from core.logging_setup import configure_logging
from core.mavlink_system import MavlinkSystem


def main() -> int:
    parser = argparse.ArgumentParser(description="Receive ArduPilot SITL MAVLink telemetry safely.")
    parser.add_argument("--config", type=Path, default=PROJECT_ROOT / "config" / "sitl.yaml")
    parser.add_argument("--timeout", type=float, default=15.0, help="seconds to wait for a heartbeat")
    parser.add_argument("--duration", type=float, default=30.0, help="seconds to monitor after connection")
    args = parser.parse_args()

    log_path = configure_logging(PROJECT_ROOT / "logs")
    config = load_config(args.config)
    system = MavlinkSystem(config)
    try:
        system.connect()
        print(f"Waiting for ArduPilot heartbeat on {config.connection.endpoint} ...")
        if not system.wait_for_connection(args.timeout):
            print("No accepted heartbeat received. Check the SITL UDP --out endpoint and profile.", file=sys.stderr)
            return 2
        print(f"Connected to FC identity {system.target_identity}; log: {log_path}")
        deadline = monotonic() + args.duration
        previous_health = None
        while monotonic() < deadline:
            system.poll_once()
            health = system.health
            if health.status != previous_health:
                print(f"health={health.status.value} reasons={health.reasons}")
                previous_health = health.status
            if system.vehicle_state.field_timestamps:
                print(asdict(system.vehicle_state))
            sleep(0.05)
        return 0
    finally:
        system.close()


if __name__ == "__main__":
    raise SystemExit(main())
