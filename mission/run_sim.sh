#!/usr/bin/env bash
# Software-only sim (macOS / Linux)
cd "$(dirname "$0")" && python3 drone_fsm_prototype.py --sim "$@"
