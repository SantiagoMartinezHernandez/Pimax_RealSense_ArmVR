# Pimax Crystal + Intel RealSense D455C VR Arm & 21-Finger Illusion System (v6.4)

Real-time, jitter-free 3D upper-limb and 21-DOF finger embodiment in **Unity VR** using the **Pimax Crystal HMD**, **Intel RealSense D455C Stereo Depth Camera**, and **Google MediaPipe**. Engineered for general seated psychophysics experiments (free reaching, strength-machine force illusions, and proprioceptive drift) with continuous 90/120 Hz Kalman prediction, Critically Damped Second-Order (CDSO) PD joint control, background torso auto-alignment, **User-Paced 4-Stop Move & Hold 6-DOF Calibration**, and an in-headset 3D VR controller/gaze-dwell dashboard.

---

## 📁 Repository Structure

```text
Pimax_RealSense_ArmVR/
├── PythonTracker/
│   ├── realsense_d455_arm_tracker.py   # v6.0 Rigid-Link FK + 1D Torso-Depth Kalman D455C streamer
│   ├── requirements.txt                # Python dependencies (pyrealsense2, mediapipe, opencv-python, numpy)
│   └── run_tracker.bat                 # 1-Click Windows launcher (auto-installs packages & models)
├── UnityScripts/
│   ├── OneEuroFilter.cs                # Continuous 3D Kalman Filter, CDSO PD Springs & Sigmoid Math
│   ├── RealSenseArmReceiver.cs         # 90/120Hz Kalman Predictor + Background Torso Auto-Alignment
│   ├── ArmAvatarController.cs          # CDSO PD Trigonometric IK, Multi-Experiment Presets & HUD
│   ├── VRDashboardAndController.cs     # In-Headset 3D World-Space VR Menu & Pimax Controller Raycast
│   ├── ProceduralHandMeshBuilder.cs    # Continuous 3D Skinned Hand & Arm Mesh generator (MRTK/Skin)
│   └── SetupSceneMenu.cs               # 1-Click Unity Editor Menu ("Psychophysics VR -> Auto-Setup")
├── copy_to_unity.bat                   # Portable Multi-PC script sync utility (auto-detects Unity path)
└── TECHNICAL_SPECIFICATIONS_BRIEF.md   # Full mathematical & architectural documentation
```

---

## 🚀 Step-by-Step Installation Guide (Setting Up a New PC)

### Step 1: Install Prerequisites on the New PC
1. **Python 3.10 or 3.11 (64-bit):**
   - Download from [python.org/downloads](https://www.python.org/downloads/).
   - ⚠️ **CRITICAL:** During installation, check the box **"Add python.exe to PATH"** at the bottom of the installer window!
2. **Pimax Play:**
   - Install **Pimax Play**, connect your **Pimax Crystal**, and verify under *General Settings* that **Pimax OpenXR** is set as the active OpenXR runtime.
3. **Unity Hub & Unity Editor:**
   - Download **Unity Hub** from [unity.com/download](https://unity.com/download).
   - Install **Unity 6 (6000.0 LTS)** or **Unity 2022.3 LTS** (standard Windows Build Support is sufficient).
4. **Git (Optional, for cloning):**
   - Install from [git-scm.com](https://git-scm.com/) (or download this repository as a `.zip` from GitHub and extract it).

---

### Step 2: Clone or Extract This Repository
Open **Command Prompt** or **PowerShell** on the new PC and run:
```powershell
cd $HOME
git clone https://github.com/SantiagoMartinezHernandez/Pimax_RealSense_ArmVR.git
```
*(Or download the ZIP from https://github.com/SantiagoMartinezHernandez/Pimax_RealSense_ArmVR and extract it to `C:\Users\<YourUsername>\Pimax_RealSense_ArmVR`).*

---

### Step 3: Create the New Unity Project & Connect Paths for `copy_to_unity.bat`
1. Open **Unity Hub** and click **"New project"** (top right).
2. Select the **"Universal 3D (URP)"** or **"3D (Built-In Render Pipeline)"** template.
3. Set the **Project Name** and **Location**:
   - **Recommended Default:** Name it **`PimaxArmExperiment`** and set Location to your user folder (`C:\Users\<YourUsername>`).
   - **How `copy_to_unity.bat` Connects Automatically:**
     - `copy_to_unity.bat` uses `%USERPROFILE%\PimaxArmExperiment\Assets` instead of a hardcoded username! So if you create `PimaxArmExperiment` in your user folder (`C:\Users\<YourUsername>\PimaxArmExperiment`) or `Documents\PimaxArmExperiment`, **`copy_to_unity.bat` finds it automatically with zero configuration!**
     - **What if you save the Unity project on another drive (e.g., `D:\VRProjects\MyArmProject`)?**
       - Simply double-click `copy_to_unity.bat`. If it doesn't see the default folder, it will prompt you **once**:
         `Please paste the full path to your Unity project folder:`
       - You can paste the path (or drag-and-drop your Unity project folder into the terminal window) and press `Enter`. It saves that path inside `unity_project_path.txt` so every future double-click of `copy_to_unity.bat` is instant!

---

### Step 4: Configure OpenXR in Unity (4 Clicks)
Once your new Unity project opens:
1. Go to **`Edit` -> `Project Settings...`** in the top menu bar.
2. Scroll down the left sidebar to **`XR Plug-in Management`** and click **"Install XR Plugin Management"**.
3. Once installed, check the box next to **`OpenXR`**.
   - *(If a yellow/red warning icon appears next to OpenXR, click it and press **"Fix All"**).*
4. Close Project Settings.

---

### Step 5: Sync Scripts (`copy_to_unity.bat`) & Run 1-Click Auto-Setup
1. Double-click **`copy_to_unity.bat`** in your `Pimax_RealSense_ArmVR` folder.
   - It will copy all 6 C# scripts (`OneEuroFilter.cs`, `RealSenseArmReceiver.cs`, `ArmAvatarController.cs`, `VRDashboardAndController.cs`, `ProceduralHandMeshBuilder.cs`, and `SetupSceneMenu.cs`) directly into your Unity project's `Assets/` folder.
2. Click back into the **Unity Editor** window (wait ~3 seconds for it to compile the scripts).
3. Look at the **very top menu bar** of Unity (next to `File`, `Edit`, `Assets`, `GameObject`, `Component`) and click:
   **`Psychophysics VR` -> `Auto-Setup Arm Tracker Scene`**
   - A confirmation dialog will pop up confirming that `RightShoulderAnchor`, `LeftShoulderAnchor`, `RealSenseCameraOrientation`, and `ArmTrackingManager` have been created and linked automatically!
4. Press **`Ctrl + S`** to save your Unity scene.

---

### Step 6: Launch the D455C Tracker & Press Play!
1. Plug your **Intel RealSense D455C** into a blue **USB 3.0 / USB-C SS port** on the new PC.
2. Go to `Pimax_RealSense_ArmVR\PythonTracker\` and double-click **`run_tracker.bat`**.
   - On the first run, it automatically installs `pyrealsense2`, `mediapipe`, `opencv-python`, and `numpy`, downloads `pose_landmarker_lite.task` and `hand_landmarker.task`, and opens the 60 FPS live tracking window.
3. Click the **Play (`▶`)** button at the top of Unity!

---

## 🎮 Live VR Controller, 3D Headset Menu & Keyboard Controls

| Input (VR Controller / Keyboard) | Function |
| :--- | :--- |
| **`GUIDED 4-STOP HOLD CALIBRATION (All Modes)`** (VR/HUD Button) | Spawns glowing 3D guide orb(s) for Right, Left, or Both arms across 4 stationary non-coplanar stops inside the Pimax camera sweet spot. Hold controller(s) inside each orb for `1.2s` (or press `Trigger`/`A`/`X`/`Space`) to record zero-velocity 3D anchors. Solves **3D Camera Yaw/Pitch/Roll**, **isotropic arm scale**, **independent Left/Right shoulder sockets**, and **6-DOF IMU wrist offset** with zero velocity lag! |
| **3-Second Gaze Hold** (Inside VR) | Hands-free menu click: hold your head-gaze cursor on any button on the floating 3D VR Dashboard for **3 seconds** to auto-click! |
| **`A` / `X` Button** or **`Tab`** | **Show / Hide & Recenter 3D VR Dashboard** directly inside your Pimax headset! |
| **Point + `Trigger`** | **Click any button** on the floating 3D VR Dashboard inside the headset. |
| **`Grip + Trigger`** | **Quick Match VR Controller:** Single-point snapshot using the Pimax controller's exact 3D coordinate + IMU rotation. |
| **`B` / `Y` Button** or **`M`** | **Cycle 3D Hand/Arm Model:** Switches live between **`MicrosoftContinuousMesh`**, **`HumanSkinContinuousMesh`**, and **`BubbleJoints`**. |
| **`Grip` Button** or **`R`** | **Cycle Active Arms:** Switches live between **`RightArmOnly`** ➔ **`LeftArmOnly`** ➔ **`BothArms`** (synced automatically between Python and Unity). |
| **Controller `Thumbstick`** | **Live Adjustments:** Up/Down adjusts Shoulder Height (`Y`); Left/Right adjusts Force/Movement Gain (hold `Grip` + Left/Right to rotate Camera Yaw). |
| **`C`** or **`Space`** (Keyboard) | **Manual Calibration Snapshot:** Forces immediate camera yaw + scale snapshot and saves to persistent memory (`PlayerPrefs`). *(Note: Continuous Background Auto-Alignment is ON by default so manual calibration is optional!)* |
| **`V`** / **`Q`** (Python Window) | **`V`**: Toggle OpenCV video overlay drawing (saves ~2 ms/frame). **`Q`**: Quit tracker cleanly. |

> **OpenXR Controller Note:** To see Pimax Crystal controllers inside Unity, open **`Edit` -> `Project Settings` -> `XR Plug-in Management` -> `OpenXR` -> `Interaction Profiles`**, click **`+`**, and add **`Oculus Touch Controller Profile`**.

