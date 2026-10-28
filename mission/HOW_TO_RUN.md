# How to run (R2D2 Mission 2)

All commands are run from this `mission/` folder.

## 1. One-time setup
Windows: double-click `setup.bat`.
Mac / Linux:
```
pip uninstall -y opencv-python opencv-python-headless
pip install numpy "opencv-contrib-python==4.11.0.86" pymavlink dronekit
```
Check: `python -c "import cv2; print(cv2.__version__, hasattr(cv2, 'wechat_qrcode_WeChatQRCode'))"` prints `4.11.0 True`.

## 2. Software sim (no Mission Planner)
Windows: `run_sim.bat` · Mac/Linux: `./run_sim.sh`

## 3. ArduPilot SITL (Mission Planner)
1. Mission Planner -> Simulation -> Multirotor, start SITL. Mission Planner
   connects on TCP 5760; SITL also serves 5762 and 5763 for a second client.
2. Windows: `run_sitl.bat` (uses `tcp:127.0.0.1:5762`; pass another address
   as the first argument). Mac/Linux: `./run_sitl.sh`
3. Keep Mission Planner open to watch the drone on the map.

`--camera sim` draws the AeroTHON field from the SITL drone's position, so the
banner, corridor and QR codes are visible in SITL.

## On the real field
- Put the drone on the pad **facing the corridor entrance**: that heading
  becomes the course x axis (`COURSE_HEADING_DEG` in `aerothon/field_spec.py`).
- Re-measure the layout values in `aerothon/field_spec.py` on 29 Oct.
