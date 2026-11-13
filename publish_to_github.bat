@echo off
setlocal EnableDelayedExpansion
title Initialize & Push to GitHub

cd /d "%~dp0"

echo ============================================================================
echo   Pimax + RealSense D455C VR Arm Tracker - GitHub Publisher
echo ============================================================================
echo.

REM 1. Ensure Git is initialized and all latest files are committed
if not exist ".git" (
    git init
    git branch -M main
)

git add .
git commit -m "Release v5.0: Pimax Crystal + RealSense D455C + MediaPipe VR Arm & 21-Finger Tracker" 2>nul

REM 2. Check if origin remote is already configured
git remote get-url origin >nul 2>&1
if %errorlevel% neq 0 (
    echo [Step 1] Create a new empty repository on GitHub:
    echo          https://github.com/new
    echo          ^(Leave "Add a README file" UNCHECKED^)
    echo.
    echo [Step 2] Paste your new GitHub repository URL below
    echo          ^(e.g. https://github.com/YourUsername/Pimax_RealSense_ArmVR.git^):
    set /p REPO_URL="GitHub URL: "
    if "!REPO_URL!"=="" (
        echo No URL entered. Exiting.
        pause
        exit /b 1
    )
    git remote add origin "!REPO_URL!"
)

echo.
echo Pushing to GitHub (branch: main)...
git push -u origin main

echo.
echo [DONE] Your repository is now live on GitHub!
pause

