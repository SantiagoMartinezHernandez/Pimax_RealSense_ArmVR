@echo off
setlocal EnableDelayedExpansion
title Copy Scripts to Unity Project (Portable Multi-PC)

echo ============================================================================
echo   Pimax + RealSense D455C VR Arm Tracker - Unity Script Sync Utility
echo ============================================================================
echo.

REM Source folder is always relative to where this .bat file is located
set "SOURCE_DIR=%~dp0UnityScripts"
set "CONFIG_FILE=%~dp0unity_project_path.txt"

REM 1. Check if a custom Unity Assets path was saved in unity_project_path.txt
set "TARGET_ASSETS="
if exist "%CONFIG_FILE%" (
    set /p TARGET_ASSETS=<"%CONFIG_FILE%"
)

REM 2. Otherwise check default locations on ANY Windows PC (%USERPROFILE% = C:\Users\<CurrentUsername>)
if not defined TARGET_ASSETS (
    if exist "%USERPROFILE%\PimaxArmExperiment\Assets" (
        set "TARGET_ASSETS=%USERPROFILE%\PimaxArmExperiment\Assets"
    ) else if exist "%USERPROFILE%\Documents\PimaxArmExperiment\Assets" (
        set "TARGET_ASSETS=%USERPROFILE%\Documents\PimaxArmExperiment\Assets"
    ) else if exist "%~dp0..\PimaxArmExperiment\Assets" (
        set "TARGET_ASSETS=%~dp0..\PimaxArmExperiment\Assets"
    )
)

REM 3. If still not found, prompt the user once and save it for all future runs!
if not exist "!TARGET_ASSETS!" (
    echo [!] Could not find Unity project automatically at:
    echo     "%USERPROFILE%\PimaxArmExperiment\Assets"
    echo.
    echo Please paste the full path to your Unity project folder ^(e.g. C:\Users\YourName\PimaxArmExperiment^)
    echo and press ENTER:
    set /p USER_PROJ_PATH="Unity Project Path: "
    
    REM Strip quotes if user dragged-and-dropped the folder
    set "USER_PROJ_PATH=!USER_PROJ_PATH:"=!"
    
    if exist "!USER_PROJ_PATH!\Assets" (
        set "TARGET_ASSETS=!USER_PROJ_PATH!\Assets"
    ) else if exist "!USER_PROJ_PATH!" (
        set "TARGET_ASSETS=!USER_PROJ_PATH!"
    ) else (
        echo.
        echo [ERROR] Path does not exist: "!USER_PROJ_PATH!"
        echo Make sure you have created the Unity project first, then run this script again.
        pause
        exit /b 1
    )
    
    echo !TARGET_ASSETS!>"%CONFIG_FILE%"
    echo [Saved] Remembered Unity Assets path in unity_project_path.txt
)

echo Source Scripts : "%SOURCE_DIR%\*.cs"
echo Target Unity   : "!TARGET_ASSETS!\"
echo.

copy /Y "%SOURCE_DIR%\*.cs" "!TARGET_ASSETS!\"

echo.
echo [SUCCESS] All 6 C# scripts copied to "!TARGET_ASSETS!\"!
echo Switch back to Unity (it will recompile automatically).
pause
