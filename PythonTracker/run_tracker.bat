@echo off
title Intel RealSense D455C + MediaPipe 3D Arm Tracker
echo ============================================================================
echo   Intel RealSense D455C + MediaPipe True-Metric 3D Arm Tracker for Unity VR
echo ============================================================================
echo.

cd /d "%~dp0"

echo [1/2] Checking Python packages (pyrealsense2, mediapipe, opencv-python, numpy)...
python -c "import pyrealsense2, mediapipe, cv2, numpy" 2>nul
if %errorlevel% neq 0 (
    echo Installing required packages...
    python -m pip install --upgrade pip
    python -m pip install pyrealsense2 mediapipe opencv-python numpy
)

echo.
echo [2/2] Starting D455C + MediaPipe 60Hz UDP Tracker on 127.0.0.1:5055 ...
python realsense_d455_arm_tracker.py
pause

