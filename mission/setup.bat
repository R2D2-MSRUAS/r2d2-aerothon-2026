@echo off
REM One-time setup: installs what the mission code needs to run.
cd /d "%~dp0"
python -m pip install --upgrade pip
python -m pip uninstall -y opencv-python opencv-python-headless
python -m pip install numpy "opencv-contrib-python==4.11.0.86" pymavlink dronekit
python -c "import cv2; print(cv2.__version__, hasattr(cv2, wechat_qrcode_WeChatQRCode))"
echo Setup done. You should see: 4.11.0 True
pause
