# Pimax Crystal + Intel RealSense D455C + MediaPipe 3D Arm VR Prototype

All files for your prototype are located in:
`C:\Users\ASant\Pimax_RealSense_ArmVR`

## Folder Structure
- `PythonTracker/`
  - `realsense_d455_arm_tracker.py` — Streams synchronized 848x480 @ 60 FPS RGB + Stereo Depth from the Intel RealSense D455C, fuses MediaPipe 2D/3D pose landmarks with hardware stereo depth (`rs2_deproject_pixel_to_point`), and sends a 68-byte binary UDP packet to Unity at `127.0.0.1:5055`.
  - `run_tracker.bat` — One-click launcher on Windows (auto-installs `pyrealsense2`, `mediapipe`, `opencv-python`, `numpy` and downloads `pose_landmarker_lite.task` if missing).
- `UnityScripts/`
  - `OneEuroFilter.cs` — Adaptive 1-Euro Filter (Casiez et al., CHI 2012) that removes depth/landmark micro-jitter at rest while keeping latency < 2 ms during fast movements.
  - `RealSenseArmReceiver.cs` — Receives the 68-byte UDP packet in Unity, transforms from RealSense metric coordinates into Unity VR world space anchored at the Avatar's shoulder, and exposes live telemetry.
  - `ArmAvatarController.cs` — Reconstructs the 3D arm using Analytic Two-Bone IK (plus an **Auto-Spawned Prototype Arm** if you haven't rigged a 3D character yet!) and provides real-time **Psychophysics Perturbation sliders** (`movementGain`, `pseudoHapticDrag`, `elbowFlexionOffsetDeg`, `artificialDelayMs`).

---

## Step 1: Physical Camera Placement & Running the D455C Python Tracker

1. **Plug the Intel RealSense D455C into a USB 3.0 / USB-C port** (blue/SS port is required for 848x480 @ 60 FPS).
2. **Position the D455C:**
   - Place the D455C on a tripod **0.6 m to 1.5 m away** from the participant seated at the strength machine, angled roughly **30°–60° to the front-side** of the active arm so the shoulder, elbow, and wrist remain clearly visible throughout the pull/push stroke.
3. **Run the Tracker:**
   - Double-click `C:\Users\ASant\Pimax_RealSense_ArmVR\PythonTracker\run_tracker.bat` (or run `python realsense_d455_arm_tracker.py` from terminal).
   - A debug window (`D455C True-Metric 3D Arm Tracker`) will open showing your live framerate (~60 FPS) and the true metric `(X, Y, Z)` coordinates in meters on your **S**houlder, **E**lbow, and **W**rist.
   - **Green circles** mean pure D455C hardware depth is locked; **Orange circles** mean the joint fell back to anatomical depth continuity.
   - Press **`r`** in the Python window to switch between Right Arm and Left Arm.

---

## Step 2: Unity Scene Setup with Pimax Crystal (Under 3 Minutes)

1. **Copy Scripts into Unity:**
   - Open your Unity project (configured with **OpenXR / Pimax XR** so your Pimax Crystal headset tracks head movement).
   - Drag the 3 scripts from `C:\Users\ASant\Pimax_RealSense_ArmVR\UnityScripts\` (`OneEuroFilter.cs`, `RealSenseArmReceiver.cs`, `ArmAvatarController.cs`) into your Unity project's `Assets/Scripts` folder.
2. **Create the Shoulder Anchor under your VR Camera:**
   - In your Unity Hierarchy, expand your **XR Origin / Camera Offset / Main Camera** (the Pimax HMD camera).
   - Create an Empty GameObject as a child of `Camera Offset` (or at a fixed point relative to the seated chair) and name it **`RightShoulderAnchor`**.
   - Position `RightShoulderAnchor` roughly where the participant's right shoulder sits relative to their head: e.g., `Position = (0.18, -0.22, -0.05)` relative to the HMD when seated upright.
3. **Create the Camera Orientation Reference:**
   - Create an Empty GameObject named **`RealSenseCameraOrientation`** in the scene.
   - If the D455C is directly in front of the user facing them, leave `Rotation = (0, 0, 0)` and keep `Mirror X = true` checked on `RealSenseArmReceiver`.
   - If the D455C is mounted at a 45° angle to the user's right side, set `RealSenseCameraOrientation` Y-rotation to `-45` (or adjust it live in Play Mode while looking at the Scene view until your forward reach moves the virtual arm straight forward).
4. **Create the Arm Manager:**
   - Create an Empty GameObject named **`ArmTrackingManager`**.
   - Add Component -> **`RealSenseArmReceiver`**
     - Drag `RightShoulderAnchor` into **`Avatar Shoulder Anchor`**.
     - Drag `RealSenseCameraOrientation` into **`Camera Orientation Reference`**.
   - Add Component -> **`ArmAvatarController`**
     - Leave `Upper Arm Bone`, `Forearm Bone`, and `Hand Bone` **EMPTY** for your first test! Because `Auto Spawn Prototype Arm If Missing` is checked by default, Unity will automatically build an articulated 3D VR arm (UpperArm, Forearm, Palm) as soon as you press Play.
     - *(Later, when you drop in a rigged Humanoid FBX/Avatar, simply drag its RightUpperArm, RightForeArm, and RightHand bones into those 3 slots.)*

---

## Step 3: One-Key Calibration & Running Psychophysics Perturbations

1. **Press Play in Unity** while `realsense_d455_arm_tracker.py` is running.
2. Have the participant sit in the strength test machine holding the starting handle position while wearing the Pimax Crystal.
3. **Press `C` on the keyboard**:
   - `ArmAvatarController` immediately measures the participant's real metric arm length from the D455C (`measuredRealArmLengthMeters`), computes the exact `armScale` ratio to match the VR arm, and spawns `CalibratedHandleOrigin` at their exact starting hand position.
4. **Test Illusions Live in the Inspector (`ArmAvatarController`):**
   - **Perceived Force / Heavy Load Illusion:** Lower `Movement Gain` to `0.65` and increase `Pseudo Haptic Drag` to `4.0`. When the user pulls the strength machine, the visual VR arm moves slower and shorter than their proprioceptive arm—inducing the classic pseudo-haptic illusion of heavier weight.
   - **Speed / Leichtigkeit Amplification:** Increase `Movement Gain` to `1.35`.
   - **Proprioceptive Drift:** Adjust `Elbow Flexion Offset Deg` between `-20°` and `+20°`.
   - **Sense of Agency Breakdown:** Increase `Artificial Delay Ms` from `0 ms` to `150 ms` to measure when the participant notices the visual latency.

