#!/usr/bin/env bash
# ArduPilot SITL: start SITL first. Default address tcp:127.0.0.1:5762
cd "$(dirname "$0")" && python3 drone_fsm_prototype.py --connect "${1:-tcp:127.0.0.1:5762}" --enable-real-flight --camera sim
