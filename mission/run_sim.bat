@echo off
REM Software-only sim: no Mission Planner needed.
cd /d "%~dp0"
python drone_fsm_prototype.py --sim
pause
