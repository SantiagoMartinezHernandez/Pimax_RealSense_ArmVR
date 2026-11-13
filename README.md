# Pimax Crystal + Intel RealSense D455C VR Arm & 21-Finger Illusion System (v5.0)

Real-time, low-latency (<20 ms perceived delay) 3D upper-limb and 21-DOF finger embodiment in **Unity VR** using the **Pimax Crystal HMD**, **Intel RealSense D455C Stereo Depth Camera**, and **Google MediaPipe**. Engineered for seated psychophysics experiments on motor agency, proprioceptive drift, and pseudo-haptic force perception.

---

## 📁 Repository Structure

```text
Pimax_RealSense_ArmVR/
├── PythonTracker/
│   ├── realsense_d455_arm_tracker.py   # D455C + MediaPipe Dual-Arm & 21-Finger UDP streamer (60 Hz)
│   ├── requirements.txt                # Python dependencies (pyrealsense2, mediapipe, opencv-python, numpy)
│   └── run_tracker.bat                 # 1-Click Windows launcher (auto-installs packages & models)
├── UnityScripts/
│   ├── OneEuroFilter.cs                # State-informed 1€ Filter with 12ms velocity lead prediction
│   ├── RealSenseArmReceiver.cs         # Zero-backlog 660-byte UDP receiver & coordinate transformer
│   ├── ArmAvatarController.cs          # Trigonometric IK, Impossible-Pose Filter, Perturbations & HUD
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
git clone <YOUR_GITHUB_REPO_URL> Pimax_RealSense_ArmVR
```
*(Or extract the downloaded ZIP directly to `C:\Users\<YourUsername>\Pimax_RealSense_ArmVR`).*

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
   - It will copy all 5 C# scripts (`OneEuroFilter.cs`, `RealSenseArmReceiver.cs`, `ArmAvatarController.cs`, `ProceduralHandMeshBuilder.cs`, and `SetupSceneMenu.cs`) directly into your Unity project's `Assets/` folder.
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

## 🎮 Live Keyboard Shortcuts & HUD Controls

| Key / Button | Function |
| :--- | :--- |
| **`C`** or **`Space`** (in Unity) | **1-Click Calibration & Auto-Save:** Scales VR arm to your real D455C arm length, aligns camera horizontal angle (`Yaw`) using your chest axis, places the green handle marker, and saves to persistent memory (`PlayerPrefs`). |
| **`M`** (in Unity) | **Cycle 3D Hand/Arm Model:** Switches live between **`MicrosoftContinuousMesh`** (Holo-White unified 3D mesh), **`HumanSkinContinuousMesh`** (Skin-toned unified 3D mesh), and **`BubbleJoints`** (High-contrast spheres + capsules). |
| **`R`** (in Unity or Python) | **Cycle Active Arms:** Switches live between **`RightArmOnly`** ➔ **`LeftArmOnly`** ➔ **`BothArms`** (synced automatically between Python and Unity). |
| **`V`** (in Python Window) | **Toggle Video Overlay Drawing:** Hides the OpenCV preview rendering to save an extra ~2 ms of CPU time during experimental trials. |
| **`Q`** (in Python Window) | Cleanly stops the RealSense D455C pipeline and exits. |

