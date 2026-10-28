@echo off
REM ArduPilot SITL (Mission Planner: Simulation tab, Multirotor). Start SITL first.
REM SITL serves TCP 5760 (Mission Planner uses it), 5762 and 5763. We use 5762 so Mission Planner can stay connected.
cd /d "%~dp0"
set CONN=%~1
if "%CONN%"=="" set CONN=tcp:127.0.0.1:5762
python drone_fsm_prototype.py --connect %CONN% --enable-real-flight --camera sim
pause
