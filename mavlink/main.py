"""Windows entry point for MAVLink monitoring and Mission-2 SITL rehearsal."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from time import monotonic, sleep

PROJECT_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from core.config import load_config
from core.logging_setup import configure_logging
from core.mavlink_system import MavlinkSystem
from core.sky_scan_mission import SkyScanMission


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="R2D2 MAVLink monitor and Windows Mission Planner SITL runner.")
    subcommands = parser.add_subparsers(dest="command", required=True)
    monitor = subcommands.add_parser("monitor", help="receive and display telemetry; never sends flight commands")
    monitor.add_argument("--config", type=Path, default=PROJECT_ROOT / "config" / "sitl.yaml")
    monitor.add_argument("--duration", type=float, default=30.0)
    mission = subcommands.add_parser("mission", help="run the Mission-2 sequence against a local Windows SITL only")
    mission.add_argument("--config", type=Path, default=PROJECT_ROOT / "config" / "aerothon_sitl.yaml")
    mission.add_argument("--confirm-sitl", action="store_true", help="required acknowledgement that this is a local SITL run")
    mission.add_argument("--simulate-perception", action="store_true", help="use deterministic QR/banner/target confirmations; no real vision")
    mission.add_argument("--connect-timeout", type=float, default=20.0)
    return parser


def main() -> int:
    args = _parser().parse_args()
    config = load_config(args.config)
    log_path = configure_logging(PROJECT_ROOT / "logs")
    system = MavlinkSystem(config)
    try:
        system.connect()
        if not system.wait_for_connection(getattr(args, "connect_timeout", 20.0)):
            print("No FC heartbeat. In Mission Planner, enable MAVLink Mirror as UDP Client to 127.0.0.1:14550 with write access.", file=sys.stderr)
            return 2
        print(f"Connected to {system.target_identity}; log: {log_path}")
        if args.command == "monitor":
            return _monitor(system, args.duration)
        if not args.confirm_sitl or not args.simulate_perception:
            print("Mission runner requires --confirm-sitl and --simulate-perception. It is not a hardware-flight executable.", file=sys.stderr)
            return 2
        if not config.mission.enabled or not config.mission.simulation_only or not config.safety.commands_enabled:
            print("The selected profile is not an enabled simulation-only command profile.", file=sys.stderr)
            return 2
        return _mission(system)
    finally:
        system.close()


def _monitor(system: MavlinkSystem, duration_s: float) -> int:
    deadline = monotonic() + duration_s
    while monotonic() < deadline:
        system.poll_once()
        print(f"health={system.health.status.value} armed={system.vehicle_state.armed} mode={system.vehicle_state.mode}")
        sleep(1.0)
    return 0


def _mission(system: MavlinkSystem) -> int:
    mission = SkyScanMission(system, simulated_perception=True)
    while not mission.finished:
        system.poll_once()
        mission.step()
        sleep(0.05)
    return 0 if mission.success else 3


if __name__ == "__main__":
    raise SystemExit(main())
