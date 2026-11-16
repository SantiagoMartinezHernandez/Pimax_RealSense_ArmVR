using System.Collections.Generic;
using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

namespace PsychophysicsVR.Tracking
{
    public enum ActiveArmMode
    {
        RightArmOnly = 0,
        LeftArmOnly = 1,
        BothArms = 2
    }

    public enum HandVisualStyle
    {
        BubbleJoints = 0,
        MicrosoftContinuousMesh = 1,
        HumanSkinContinuousMesh = 2,
        CustomRiggedPrefab = 3
    }

    public enum ExperimentPreset
    {
        GeneralFreeMovement = 0,
        StrengthMachineTask = 1,
        ProprioceptiveDriftTask = 2
    }

    public struct TrajectoryCalibSample
    {
        public float Timestamp;
        public Vector3 HeadPos;
        public Quaternion HeadYawRot;

        public bool HasRight;
        public Vector3 RightCamRelWrist;
        public Vector3 RightCtrlWorldPos;
        public Quaternion RightCtrlWorldRot;
        public float RightConfidence;

        public bool HasLeft;
        public Vector3 LeftCamRelWrist;
        public Vector3 LeftCtrlWorldPos;
        public Quaternion LeftCtrlWorldRot;
        public float LeftConfidence;
    }

    /// <summary>
    /// v6.0 Dual-Arm + 21-Finger Articulated Hand Controller for Pimax VR Psychophysics.
    /// Upgrades in v6.0:
    ///   1. Critically Damped Second-Order (CDSO) PD Joint Controller + C-infinity Soft-Sigmoid
    ///      Anatomical Constraints (absorbs 10cm movement jitter with zero overshoot or boundary pops).
    ///   2. General Multi-Experiment Architecture (GeneralFreeMovement, StrengthMachineTask, ProprioceptiveDriftTask).
    ///   3. Background Torso Auto-Alignment + Pimax VR Controller Cross-Calibration (`CalibrateToVRController`).
    ///   4. Auto-attaches `VRDashboardAndController` so a 3D World-Space UI & Controller Laser Pointer
    ///      are immediately available inside the Pimax Crystal headset.
    /// </summary>
    public class ArmAvatarController : MonoBehaviour
    {
        private const string PrefPrefix = "PimaxArmVR_v6_";

        [Header("Tracking Source")]
        public RealSenseArmReceiver receiver;

        [Header("Experiment Configuration & Visuals")]
        public ExperimentPreset experimentPreset = ExperimentPreset.GeneralFreeMovement;
        public ActiveArmMode activeArmMode = ActiveArmMode.RightArmOnly;
        public HandVisualStyle handVisualStyle = HandVisualStyle.MicrosoftContinuousMesh;

        [Tooltip("Optional: Drag a rigged Microsoft MRTK / OpenXR Hand FBX/Prefab here if using CustomRiggedPrefab mode.")]
        public Transform customRightHandPrefab;
        public Transform customLeftHandPrefab;

        [Header("VR Shoulder Positioning (Relative to Pimax Headset)")]
        public bool lockShoulderToHeadsetYaw = true;

        [Tooltip("Right Shoulder offset from VR Headset (Eyes) in meters: X = Right (+0.18m), Y = Down (-0.15m), Z = Forward (0.00m).")]
        public Vector3 shoulderOffsetFromHead = new Vector3(0.18f, -0.15f, 0.00f);

        [Tooltip("When true (set automatically by Guided Both-Arms Path Calibration), Left Shoulder uses its own calibrated 3D offset instead of mirroring Right Shoulder.")]
        public bool hasIndependentLeftShoulder = false;
        public Vector3 leftShoulderIndependentOffset = new Vector3(-0.18f, -0.15f, 0.00f);

        [Header("Arm Dimensions & CDSO PD Joint Controller")]
        public float prototypeUpperArmLength = 0.28f;
        public float prototypeForearmLength = 0.25f;
        public float handSizeScale = 1.05f;

        [Tooltip("Critically Damped Second-Order PD Controller natural frequency (Hz). Lower = silkier smooth, Higher = faster snap (4.8 Hz recommended).")]
        [Range(1.5f, 12.0f)]
        public float pdNaturalFrequencyHz = 4.8f;

        [Header("Persistent Calibration & Desktop HUD")]
        public bool autoLoadSavedCalibration = true;
        public bool triggerCalibrationNow = false;
        public bool showOnScreenCalibrationHUD = true;
        [Range(1.0f, 3.0f)]
        public float hudScale = 1.55f;

        [Header("Psychophysics Perturbation Controls")]
        [Range(0.2f, 2.5f)]
        public float movementGain = 1.0f;
        public Transform machineHandleOrigin;

        [Range(-45f, 45f)]
        public float elbowFlexionOffsetDeg = 0f;

        [Range(0f, 300f)]
        public float artificialDelayMs = 0f;

        [Range(0f, 20f)]
        public float pseudoHapticDrag = 0f;

        [HideInInspector] public Transform upperArmBone;
        [HideInInspector] public Transform forearmBone;
        [HideInInspector] public Transform handBone;
        [HideInInspector] public bool autoSpawnPrototypeArmIfMissing = true;

        private Camera _vrCamera;
        private ProceduralArmRig _rightArmRig;
        private ProceduralArmRig _leftArmRig;
        private int _lastSyncedPythonArmMode = -1;
        public string CalibrationStatusMessage { get; private set; } = "Auto-Align Active! Use VR Controller [A/X] for In-Headset Menu.";
        private float _calibrationFlashTimer = 0f;

        private static readonly int[,] FingerBonePairs = new int[20, 2]
        {
            {0, 1}, {1, 2}, {2, 3}, {3, 4},       // Thumb
            {0, 5}, {5, 6}, {6, 7}, {7, 8},       // Index
            {0, 9}, {9, 10}, {10, 11}, {11, 12},  // Middle
            {0, 13}, {13, 14}, {14, 15}, {15, 16},// Ring
            {0, 17}, {17, 18}, {18, 19}, {19, 20} // Pinky
        };

        private class ProceduralArmRig
        {
            public readonly bool IsRight;
            public GameObject RootGo;
            public GameObject BubblePrimitivesGroup;
            public ContinuousHandArmMesh ContinuousSkinnedMesh;

            public Transform ShoulderAnchor;
            public Transform UpperArmBone;
            public Transform ForearmBone;
            public Transform HandBone;

            public Transform UpperArmMesh;
            public Transform ForearmMesh;
            public Transform ElbowSphere;
            public Transform PalmPlateMesh;

            public readonly Transform[] FingerJointSpheres = new Transform[21];
            public readonly Transform[] FingerBoneCapsules = new Transform[20];
            public readonly Vector3[] DefaultFingerRelPos = new Vector3[21];
            public readonly Vector3[] SmoothedFingerWorldPos = new Vector3[21];
            public bool FingersInitialized;

            public Transform CustomHandInstance;
            public readonly Transform[] CustomMappedBones = new Transform[21];

            // CDSO Critically Damped Second-Order PD Controllers
            public readonly CriticallyDampedSpring3D WristSpring = new CriticallyDampedSpring3D();
            public readonly CriticallyDampedSpring3D ElbowPerpSpring = new CriticallyDampedSpring3D();
            public readonly CriticallyDampedSpring3D HandFwdSpring = new CriticallyDampedSpring3D();
            public readonly CriticallyDampedSpring3D PalmNormSpring = new CriticallyDampedSpring3D();
            public readonly CriticallyDampedQuaternion UpperArmRotPD = new CriticallyDampedQuaternion();
            public readonly CriticallyDampedQuaternion ForearmRotPD = new CriticallyDampedQuaternion();
            public readonly CriticallyDampedQuaternion HandRotPD = new CriticallyDampedQuaternion();

            public Vector3 SmoothedElbowPerpDir;
            public Vector3 DraggedWristPos;
            public Quaternion CalibratedHandRotOffset = Quaternion.identity;
            public bool PoseInitialized;
            public bool DragInitialized;

            public struct DelayedSample
            {
                public float Timestamp;
                public Vector3 Shoulder, Elbow, Wrist, Index, Pinky;
            }
            public readonly Queue<DelayedSample> DelayQueue = new Queue<DelayedSample>(128);

            public ProceduralArmRig(bool isRight)
            {
                IsRight = isRight;
                SmoothedElbowPerpDir = new Vector3(isRight ? 0.35f : -0.35f, -0.92f, -0.15f).normalized;
                InitDefaultFingers();
            }

            private void InitDefaultFingers()
            {
                float s = IsRight ? 1f : -1f;
                float[] fingerX = { -0.040f * s, -0.025f * s, 0.0f, 0.022f * s, 0.042f * s };
                float[] segLens = { 0.032f, 0.028f, 0.020f, 0.016f };
                DefaultFingerRelPos[0] = Vector3.zero;

                for (int f = 0; f < 5; f++)
                {
                    Vector3 cur = new Vector3(fingerX[f], f == 0 ? 0.015f : 0.035f, 0f);
                    for (int seg = 0; seg < 4; seg++)
                    {
                        int idx = 1 + f * 4 + seg;
                        float bendZ = (seg >= 1) ? -0.006f * seg : 0f;
                        cur += new Vector3(f == 0 && seg == 0 ? -0.015f * s : 0f, segLens[seg], bendZ);
                        DefaultFingerRelPos[idx] = cur;
                    }
                }
            }
        }

        private void Start()
        {
            if (receiver == null)
                receiver = GetComponent<RealSenseArmReceiver>();

            _vrCamera = Camera.main != null ? Camera.main : Object.FindFirstObjectByType<Camera>();

            EnsureShoulderAnchors();

            _rightArmRig = BuildArmRig(true, receiver.avatarShoulderAnchor);
            _leftArmRig = BuildArmRig(false, receiver.leftAvatarShoulderAnchor);

            upperArmBone = _rightArmRig.UpperArmBone;
            forearmBone = _rightArmRig.ForearmBone;
            handBone = _rightArmRig.HandBone;

            if (receiver != null)
            {
                receiver.avatarTargetTotalArmLength = prototypeUpperArmLength + prototypeForearmLength;
            }

            if (autoLoadSavedCalibration)
            {
                LoadPersistentSettings();
            }

            ApplyVisualStyleToBothArms();
            UpdateActiveArmVisibility();
            UpdateHandleMarkerVisibility();

            // Auto-attach In-Headset 3D VR Dashboard & Pimax OpenXR Controller Integration
            if (GetComponent<VRDashboardAndController>() == null)
            {
                gameObject.AddComponent<VRDashboardAndController>();
            }
        }

        private void EnsureShoulderAnchors()
        {
            Transform parent = _vrCamera != null ? _vrCamera.transform.parent : null;

            if (receiver.avatarShoulderAnchor == null)
            {
                GameObject rGo = GameObject.Find("RightShoulderAnchor") ?? new GameObject("RightShoulderAnchor");
                receiver.avatarShoulderAnchor = rGo.transform;
            }
            if (_vrCamera != null && receiver.avatarShoulderAnchor.parent == _vrCamera.transform)
            {
                receiver.avatarShoulderAnchor.SetParent(parent, true);
            }

            if (receiver.leftAvatarShoulderAnchor == null)
            {
                GameObject lGo = GameObject.Find("LeftShoulderAnchor") ?? new GameObject("LeftShoulderAnchor");
                lGo.transform.SetParent(parent, true);
                receiver.leftAvatarShoulderAnchor = lGo.transform;
            }
        }

        private void Update()
        {
            if (_calibrationFlashTimer > 0f)
                _calibrationFlashTimer -= Time.unscaledDeltaTime;

            UpdateShoulderAnchorsFromHeadset();

            if (triggerCalibrationNow || WasKeyPressedThisFrame(KeyCode.C) || WasKeyPressedThisFrame(KeyCode.Space))
            {
                triggerCalibrationNow = false;
                PerformOneClickCalibration();
            }

            if (receiver != null && receiver.remotePythonArmMode >= 0 && receiver.remotePythonArmMode <= 2)
            {
                if (_lastSyncedPythonArmMode != receiver.remotePythonArmMode)
                {
                    if (_lastSyncedPythonArmMode != -1)
                    {
                        activeArmMode = (ActiveArmMode)receiver.remotePythonArmMode;
                        UpdateActiveArmVisibility();
                        SavePersistentSettings();
                        CalibrationStatusMessage = $"Synced Arm Mode from Python: {activeArmMode}";
                    }
                    _lastSyncedPythonArmMode = receiver.remotePythonArmMode;
                }
            }

            if (WasKeyPressedThisFrame(KeyCode.R))
            {
                CycleArmMode();
            }

            if (WasKeyPressedThisFrame(KeyCode.M))
            {
                CycleVisualStyle();
            }
        }

        public void CycleArmMode()
        {
            activeArmMode = (ActiveArmMode)(((int)activeArmMode + 1) % 3);
            UpdateActiveArmVisibility();
            SavePersistentSettings();
            CalibrationStatusMessage = $"Arm Mode: {activeArmMode}";
        }

        public void CycleVisualStyle()
        {
            int maxModes = (customRightHandPrefab != null || customLeftHandPrefab != null) ? 4 : 3;
            handVisualStyle = (HandVisualStyle)(((int)handVisualStyle + 1) % maxModes);
            ApplyVisualStyleToBothArms();
            SavePersistentSettings();
            CalibrationStatusMessage = $"3D Model: {handVisualStyle}";
        }

        public void CycleExperimentPreset()
        {
            experimentPreset = (ExperimentPreset)(((int)experimentPreset + 1) % 3);
            UpdateHandleMarkerVisibility();
            SavePersistentSettings();
            CalibrationStatusMessage = $"Experiment Task: {experimentPreset}";
        }

        public void ToggleContinuousAutoCalibration()
        {
            if (receiver == null) return;
            receiver.continuousAutoCalibration = !receiver.continuousAutoCalibration;
            if (receiver.continuousAutoCalibration)
                receiver.ResetAutoCalibrationLock();
            SavePersistentSettings();
            CalibrationStatusMessage = $"Auto-Align Chest: {(receiver.continuousAutoCalibration ? "ON" : "OFF")}";
        }

        private void UpdateActiveArmVisibility()
        {
            if (_rightArmRig?.RootGo != null)
                _rightArmRig.RootGo.SetActive(activeArmMode == ActiveArmMode.RightArmOnly || activeArmMode == ActiveArmMode.BothArms);
            if (_leftArmRig?.RootGo != null)
                _leftArmRig.RootGo.SetActive(activeArmMode == ActiveArmMode.LeftArmOnly || activeArmMode == ActiveArmMode.BothArms);
        }

        private void UpdateHandleMarkerVisibility()
        {
            if (machineHandleOrigin != null)
            {
                machineHandleOrigin.gameObject.SetActive(experimentPreset == ExperimentPreset.StrengthMachineTask);
            }
        }

        private void UpdateShoulderAnchorsFromHeadset()
        {
            if (!lockShoulderToHeadsetYaw || receiver == null)
                return;

            if (_vrCamera == null)
                _vrCamera = Camera.main;
            if (_vrCamera == null)
                return;

            float headYaw = _vrCamera.transform.eulerAngles.y;
            Quaternion yawOnlyRot = Quaternion.Euler(0f, headYaw, 0f);

            if (receiver.avatarShoulderAnchor != null)
            {
                receiver.avatarShoulderAnchor.position = _vrCamera.transform.position + yawOnlyRot * shoulderOffsetFromHead;
                receiver.avatarShoulderAnchor.rotation = yawOnlyRot;
            }

            if (receiver.leftAvatarShoulderAnchor != null)
            {
                Vector3 leftOffset = hasIndependentLeftShoulder
                    ? leftShoulderIndependentOffset
                    : new Vector3(-shoulderOffsetFromHead.x, shoulderOffsetFromHead.y, shoulderOffsetFromHead.z);
                receiver.leftAvatarShoulderAnchor.position = _vrCamera.transform.position + yawOnlyRot * leftOffset;
                receiver.leftAvatarShoulderAnchor.rotation = yawOnlyRot;
            }
        }

        private static bool WasKeyPressedThisFrame(KeyCode legacyKey)
        {
#if ENABLE_INPUT_SYSTEM
            if (Keyboard.current != null)
            {
                if (legacyKey == KeyCode.C && Keyboard.current.cKey.wasPressedThisFrame) return true;
                if (legacyKey == KeyCode.Space && Keyboard.current.spaceKey.wasPressedThisFrame) return true;
                if (legacyKey == KeyCode.R && Keyboard.current.rKey.wasPressedThisFrame) return true;
                if (legacyKey == KeyCode.M && Keyboard.current.mKey.wasPressedThisFrame) return true;
            }
#endif
#if ENABLE_LEGACY_INPUT_MANAGER
            try
            {
                if (Input.GetKeyDown(legacyKey)) return true;
            }
            catch { }
#endif
            return false;
        }

        [ContextMenu("Calibrate Arm Now")]
        public void PerformOneClickCalibration()
        {
            if (receiver == null || !receiver.HasValidTracking)
            {
                CalibrationStatusMessage = "Waiting for D455C tracking data before calibrating!";
                return;
            }

            bool useLeft = (activeArmMode == ActiveArmMode.LeftArmOnly);
            Vector3 localReach = useLeft ? receiver.LocalLeftShoulderToWrist : receiver.LocalShoulderToWrist;
            if (localReach.y > 0.08f && new Vector2(localReach.x, localReach.z).magnitude < 0.25f)
            {
                CalibrationStatusMessage = "Hand appears raised near VR Headset! Lower hand & press [C].";
                Debug.LogWarning("[ArmAvatarController] " + CalibrationStatusMessage);
                return;
            }

            float totalAvatarArm = prototypeUpperArmLength + prototypeForearmLength;
            receiver.CalibrateArmScaleToAvatar(totalAvatarArm);

            Vector3 chestRightXZ = new Vector3(receiver.LocalOppositeToActiveShoulder.x, 0f, receiver.LocalOppositeToActiveShoulder.z);
            if (_vrCamera != null && chestRightXZ.magnitude > 0.12f)
            {
                Vector3 vrRightXZ = new Vector3(_vrCamera.transform.right.x, 0f, _vrCamera.transform.right.z).normalized;
                float rawChestAngle = Mathf.Atan2(chestRightXZ.x, chestRightXZ.z) * Mathf.Rad2Deg;
                float targetRightAngle = Mathf.Atan2(vrRightXZ.x, vrRightXZ.z) * Mathf.Rad2Deg;
                receiver.cameraYawOffsetDeg = Mathf.DeltaAngle(rawChestAngle, targetRightAngle);
            }
            else if (_vrCamera != null)
            {
                Vector3 localReachXZ = new Vector3(localReach.x, 0f, localReach.z);
                if (localReachXZ.magnitude > 0.12f)
                {
                    float headYaw = _vrCamera.transform.eulerAngles.y;
                    Quaternion headYawRot = Quaternion.Euler(0f, headYaw, 0f);
                    float inwardX = useLeft ? shoulderOffsetFromHead.x : -shoulderOffsetFromHead.x;
                    Vector3 expectedVec = headYawRot * new Vector3(inwardX, 0f, localReachXZ.magnitude);
                    float rawAngle = Mathf.Atan2(localReachXZ.x, localReachXZ.z) * Mathf.Rad2Deg;
                    float targetAngle = Mathf.Atan2(expectedVec.x, expectedVec.z) * Mathf.Rad2Deg;
                    receiver.cameraYawOffsetDeg = Mathf.DeltaAngle(rawAngle, targetAngle);
                }
            }

            ResetArmElbowPoles();

            if (experimentPreset == ExperimentPreset.StrengthMachineTask)
            {
                EnsureHandleMarker();
                Quaternion yawRot = Quaternion.Euler(0f, receiver.cameraYawOffsetDeg, 0f);
                Transform anchor = useLeft ? receiver.leftAvatarShoulderAnchor : receiver.avatarShoulderAnchor;
                machineHandleOrigin.position = anchor.position + yawRot * (localReach * receiver.armScale);
                machineHandleOrigin.gameObject.SetActive(true);
            }

            SavePersistentSettings();

            _calibrationFlashTimer = 3.0f;
            CalibrationStatusMessage = $"Calibrated! Scale={receiver.armScale:F2}x | CamYaw={receiver.cameraYawOffsetDeg:F0} deg";
            Debug.Log($"[ArmAvatarController] {CalibrationStatusMessage}");
        }

        /// <summary>
        /// Uses the Pimax OpenXR Controller's sub-millimeter 3D position as a golden reference
        /// to align camera yaw, arm scale, and shoulder offset simultaneously!
        /// </summary>
        public void CalibrateToVRController(Vector3 controllerWorldPos, Quaternion controllerWorldRot, bool isLeftController)
        {
            if (receiver == null || !receiver.HasValidTracking)
            {
                CalibrationStatusMessage = "Cannot match VR controller: waiting for D455C camera stream!";
                return;
            }

            UpdateShoulderAnchorsFromHeadset();

            Transform anchor = isLeftController ? receiver.leftAvatarShoulderAnchor : receiver.avatarShoulderAnchor;
            Vector3 localReach = isLeftController ? receiver.LocalLeftShoulderToWrist : receiver.LocalShoulderToWrist;
            if (anchor == null || localReach.sqrMagnitude < 0.01f) return;

            // 1. Align horizontal Yaw between tracked shoulder->wrist vector and shoulder->VRController vector
            Vector3 desiredVec = controllerWorldPos - anchor.position;
            Vector3 desiredXZ = new Vector3(desiredVec.x, 0f, desiredVec.z);
            Vector3 localXZ = new Vector3(localReach.x, 0f, localReach.z);
            if (desiredXZ.magnitude > 0.10f && localXZ.magnitude > 0.10f)
            {
                float rawAngle = Mathf.Atan2(localXZ.x, localXZ.z) * Mathf.Rad2Deg;
                float targetAngle = Mathf.Atan2(desiredXZ.x, desiredXZ.z) * Mathf.Rad2Deg;
                receiver.cameraYawOffsetDeg = Mathf.DeltaAngle(rawAngle, targetAngle);
            }

            // 2. Match arm scale to controller reach distance (within anatomical limits)
            float totalAvatarArm = prototypeUpperArmLength + prototypeForearmLength;
            receiver.CalibrateArmScaleToAvatar(totalAvatarArm);

            // 3. Shift shoulderOffsetFromHead gently so the mapped wrist lands on the exact controller 3D point
            Quaternion baseRot = receiver.cameraOrientationReference != null ? receiver.cameraOrientationReference.rotation : Quaternion.identity;
            Quaternion yawRot = Quaternion.Euler(0f, receiver.cameraYawOffsetDeg, 0f) * baseRot;
            Vector3 mappedWrist = anchor.position + yawRot * (localReach * receiver.armScale);
            Vector3 worldErr = controllerWorldPos - mappedWrist;

            if (_vrCamera != null)
            {
                Quaternion invHeadYaw = Quaternion.Inverse(Quaternion.Euler(0f, _vrCamera.transform.eulerAngles.y, 0f));
                Vector3 localErr = invHeadYaw * worldErr;
                if (isLeftController) localErr.x = -localErr.x;

                shoulderOffsetFromHead += localErr;
                // Keep shoulder inside plausible anatomical bounds relative to the HMD eyes
                shoulderOffsetFromHead.x = Mathf.Clamp(shoulderOffsetFromHead.x, 0.08f, 0.30f);
                shoulderOffsetFromHead.y = Mathf.Clamp(shoulderOffsetFromHead.y, -0.30f, -0.04f);
                shoulderOffsetFromHead.z = Mathf.Clamp(shoulderOffsetFromHead.z, -0.18f, 0.18f);
                UpdateShoulderAnchorsFromHeadset();
            }

            if (experimentPreset == ExperimentPreset.StrengthMachineTask)
            {
                EnsureHandleMarker();
                machineHandleOrigin.position = controllerWorldPos;
                machineHandleOrigin.gameObject.SetActive(true);
            }

            // 4. Use 6-DOF IMU orientation (`controllerWorldRot`) to refine wrist roll/pitch alignment
            ProceduralArmRig targetRig = isLeftController ? _leftArmRig : _rightArmRig;
            if (targetRig != null && targetRig.HandBone != null)
            {
                // Controller's local +Z forward points along fingers; align HandBone's up-axis (+Y) toward controller +Z
                Quaternion desiredHandWorldRot = controllerWorldRot * Quaternion.Euler(90f, 0f, 0f);
                Quaternion baseHandRot = targetRig.HandBone.rotation * Quaternion.Inverse(targetRig.CalibratedHandRotOffset);
                Quaternion localDelta = Quaternion.Inverse(baseHandRot) * desiredHandWorldRot;
                // Clamp rotational correction to +-30 degrees for anatomical safety
                targetRig.CalibratedHandRotOffset = Quaternion.RotateTowards(Quaternion.identity, localDelta, 30f);
            }

            ResetArmElbowPoles();
            SavePersistentSettings();

            _calibrationFlashTimer = 3.0f;
            CalibrationStatusMessage = $"Matched 6-DOF Controller! Yaw={receiver.cameraYawOffsetDeg:F0} deg, ShY={shoulderOffsetFromHead.y:F2}m";
            Debug.Log($"[ArmAvatarController] {CalibrationStatusMessage}");
        }

        private void ResetArmElbowPoles()
        {
            if (_rightArmRig != null)
            {
                Vector3 p = new Vector3(0.35f, -0.92f, -0.15f).normalized;
                _rightArmRig.SmoothedElbowPerpDir = p;
                _rightArmRig.ElbowPerpSpring.Snap(p);
            }
            if (_leftArmRig != null)
            {
                Vector3 p = new Vector3(-0.35f, -0.92f, -0.15f).normalized;
                _leftArmRig.SmoothedElbowPerpDir = p;
                _leftArmRig.ElbowPerpSpring.Snap(p);
            }
        }

        private void EnsureHandleMarker()
        {
            if (machineHandleOrigin == null)
            {
                GameObject existing = GameObject.Find("CalibratedHandleOrigin");
                if (existing != null)
                {
                    machineHandleOrigin = existing.transform;
                }
                else
                {
                    GameObject handleGo = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                    handleGo.name = "CalibratedHandleOrigin";
                    Destroy(handleGo.GetComponent<Collider>());
                    handleGo.transform.localScale = Vector3.one * 0.042f;
                    ApplyColor(handleGo, new Color(0.1f, 0.95f, 0.3f));
                    machineHandleOrigin = handleGo.transform;
                }
            }
        }

        public void SavePersistentSettings()
        {
            if (receiver == null) return;

            PlayerPrefs.SetInt(PrefPrefix + "HasSaved", 1);
            PlayerPrefs.SetFloat(PrefPrefix + "ArmScale", receiver.armScale);
            PlayerPrefs.SetFloat(PrefPrefix + "CamYaw", receiver.cameraYawOffsetDeg);
            PlayerPrefs.SetFloat(PrefPrefix + "CamPitch", receiver.cameraPitchOffsetDeg);
            PlayerPrefs.SetFloat(PrefPrefix + "CamRoll", receiver.cameraRollOffsetDeg);
            PlayerPrefs.SetInt(PrefPrefix + "AutoCalib", receiver.continuousAutoCalibration ? 1 : 0);
            PlayerPrefs.SetFloat(PrefPrefix + "ShoulderX", shoulderOffsetFromHead.x);
            PlayerPrefs.SetFloat(PrefPrefix + "ShoulderY", shoulderOffsetFromHead.y);
            PlayerPrefs.SetFloat(PrefPrefix + "ShoulderZ", shoulderOffsetFromHead.z);
            PlayerPrefs.SetInt(PrefPrefix + "HasIndepLeft", hasIndependentLeftShoulder ? 1 : 0);
            PlayerPrefs.SetFloat(PrefPrefix + "LeftShoulderX", leftShoulderIndependentOffset.x);
            PlayerPrefs.SetFloat(PrefPrefix + "LeftShoulderY", leftShoulderIndependentOffset.y);
            PlayerPrefs.SetFloat(PrefPrefix + "LeftShoulderZ", leftShoulderIndependentOffset.z);
            PlayerPrefs.SetFloat(PrefPrefix + "PdFreqHz", pdNaturalFrequencyHz);
            PlayerPrefs.SetFloat(PrefPrefix + "KfVelNoise", receiver.kalmanProcessNoiseVel);
            PlayerPrefs.SetFloat(PrefPrefix + "HudScale", hudScale);
            PlayerPrefs.SetInt(PrefPrefix + "ExpPreset", (int)experimentPreset);
            PlayerPrefs.SetInt(PrefPrefix + "ArmMode", (int)activeArmMode);
            PlayerPrefs.SetInt(PrefPrefix + "VisualStyle", (int)handVisualStyle);

            if (machineHandleOrigin != null)
            {
                PlayerPrefs.SetInt(PrefPrefix + "HasHandle", 1);
                PlayerPrefs.SetFloat(PrefPrefix + "HandleX", machineHandleOrigin.position.x);
                PlayerPrefs.SetFloat(PrefPrefix + "HandleY", machineHandleOrigin.position.y);
                PlayerPrefs.SetFloat(PrefPrefix + "HandleZ", machineHandleOrigin.position.z);
            }
            PlayerPrefs.Save();
        }

        public void LoadPersistentSettings()
        {
            if (receiver == null || PlayerPrefs.GetInt(PrefPrefix + "HasSaved", 0) == 0)
                return;

            receiver.armScale = PlayerPrefs.GetFloat(PrefPrefix + "ArmScale", receiver.armScale);
            receiver.cameraYawOffsetDeg = PlayerPrefs.GetFloat(PrefPrefix + "CamYaw", receiver.cameraYawOffsetDeg);
            receiver.cameraPitchOffsetDeg = PlayerPrefs.GetFloat(PrefPrefix + "CamPitch", receiver.cameraPitchOffsetDeg);
            receiver.cameraRollOffsetDeg = PlayerPrefs.GetFloat(PrefPrefix + "CamRoll", receiver.cameraRollOffsetDeg);
            receiver.continuousAutoCalibration = PlayerPrefs.GetInt(PrefPrefix + "AutoCalib", 1) == 1;
            shoulderOffsetFromHead = new Vector3(
                PlayerPrefs.GetFloat(PrefPrefix + "ShoulderX", shoulderOffsetFromHead.x),
                PlayerPrefs.GetFloat(PrefPrefix + "ShoulderY", shoulderOffsetFromHead.y),
                PlayerPrefs.GetFloat(PrefPrefix + "ShoulderZ", shoulderOffsetFromHead.z)
            );
            hasIndependentLeftShoulder = PlayerPrefs.GetInt(PrefPrefix + "HasIndepLeft", 0) == 1;
            leftShoulderIndependentOffset = new Vector3(
                PlayerPrefs.GetFloat(PrefPrefix + "LeftShoulderX", leftShoulderIndependentOffset.x),
                PlayerPrefs.GetFloat(PrefPrefix + "LeftShoulderY", leftShoulderIndependentOffset.y),
                PlayerPrefs.GetFloat(PrefPrefix + "LeftShoulderZ", leftShoulderIndependentOffset.z)
            );
            pdNaturalFrequencyHz = PlayerPrefs.GetFloat(PrefPrefix + "PdFreqHz", pdNaturalFrequencyHz);
            receiver.kalmanProcessNoiseVel = PlayerPrefs.GetFloat(PrefPrefix + "KfVelNoise", receiver.kalmanProcessNoiseVel);
            hudScale = PlayerPrefs.GetFloat(PrefPrefix + "HudScale", hudScale);
            experimentPreset = (ExperimentPreset)PlayerPrefs.GetInt(PrefPrefix + "ExpPreset", (int)experimentPreset);
            activeArmMode = (ActiveArmMode)PlayerPrefs.GetInt(PrefPrefix + "ArmMode", (int)activeArmMode);
            handVisualStyle = (HandVisualStyle)PlayerPrefs.GetInt(PrefPrefix + "VisualStyle", (int)handVisualStyle);

            if (PlayerPrefs.GetInt(PrefPrefix + "HasHandle", 0) == 1 && experimentPreset == ExperimentPreset.StrengthMachineTask)
            {
                EnsureHandleMarker();
                machineHandleOrigin.position = new Vector3(
                    PlayerPrefs.GetFloat(PrefPrefix + "HandleX", 0f),
                    PlayerPrefs.GetFloat(PrefPrefix + "HandleY", 1f),
                    PlayerPrefs.GetFloat(PrefPrefix + "HandleZ", 0.4f)
                );
            }

            CalibrationStatusMessage = $"Restored session (YPR={receiver.cameraYawOffsetDeg:F0}/{receiver.cameraPitchOffsetDeg:F0}/{receiver.cameraRollOffsetDeg:F0}, PD={pdNaturalFrequencyHz:F1}Hz)";
        }

        public void ResetCalibrationToDefaults()
        {
            PlayerPrefs.DeleteAll();
            shoulderOffsetFromHead = new Vector3(0.18f, -0.15f, 0.00f);
            hasIndependentLeftShoulder = false;
            leftShoulderIndependentOffset = new Vector3(-0.18f, -0.15f, 0.00f);
            pdNaturalFrequencyHz = 4.8f;
            movementGain = 1.0f;
            elbowFlexionOffsetDeg = 0f;
            artificialDelayMs = 0f;
            pseudoHapticDrag = 0f;
            if (receiver != null)
            {
                receiver.armScale = 1.0f;
                receiver.cameraYawOffsetDeg = 0f;
                receiver.cameraPitchOffsetDeg = 0f;
                receiver.cameraRollOffsetDeg = 0f;
                receiver.kalmanProcessNoiseVel = 2.8f;
                receiver.continuousAutoCalibration = true;
                receiver.ResetAutoCalibrationLock();
            }
            if (_rightArmRig != null) _rightArmRig.CalibratedHandRotOffset = Quaternion.identity;
            if (_leftArmRig != null) _leftArmRig.CalibratedHandRotOffset = Quaternion.identity;
            CalibrationStatusMessage = "Reset settings to factory defaults.";
        }

        /// <summary>
        /// v6.2 Multi-Point Lag-Compensated 3D Trajectory Solver (Procrustes + Polar Decomposition):
        ///   Solves for:
        ///     1. Temporal pipeline lag (0..4 frames) via rotation-invariant speed profile correlation.
        ///     2. Full 3D Camera Orientation (Yaw, Pitch, Roll) + isotropic 3D Workspace Scale.
        ///     3. Independent Right and Left 3D Shoulder Socket Offsets relative to Pimax Head Yaw.
        ///     4. Averaged 6-DOF IMU Wrist Orientation Offset.
        /// </summary>
        public void SolveGuidedTrajectoryCalibration(List<TrajectoryCalibSample> samples)
        {
            if (receiver == null || samples == null || samples.Count < 25)
            {
                CalibrationStatusMessage = "Guided calibration cancelled: too few samples collected.";
                return;
            }

            // 1. Estimate temporal pipeline lag (camera frames lagging behind 1000Hz controller)
            int bestLag = 0;
            float bestCorr = -1f;
            int n = samples.Count;
            for (int lag = 0; lag <= 4; lag++)
            {
                float dot = 0f, normCtrl = 0f, normCam = 0f;
                for (int i = 0; i < n - 1 - lag; i++)
                {
                    if (samples[i].HasRight && samples[i + 1].HasRight && samples[i + lag].HasRight && samples[i + 1 + lag].HasRight)
                    {
                        float vc = (samples[i + 1].RightCtrlWorldPos - samples[i].RightCtrlWorldPos).magnitude;
                        float vm = (samples[i + 1 + lag].RightCamRelWrist - samples[i + lag].RightCamRelWrist).magnitude;
                        dot += vc * vm;
                        normCtrl += vc * vc;
                        normCam += vm * vm;
                    }
                    if (samples[i].HasLeft && samples[i + 1].HasLeft && samples[i + lag].HasLeft && samples[i + 1 + lag].HasLeft)
                    {
                        float vc = (samples[i + 1].LeftCtrlWorldPos - samples[i].LeftCtrlWorldPos).magnitude;
                        float vm = (samples[i + 1 + lag].LeftCamRelWrist - samples[i + lag].LeftCamRelWrist).magnitude;
                        dot += vc * vm;
                        normCtrl += vc * vc;
                        normCam += vm * vm;
                    }
                }
                if (normCtrl > 1e-5f && normCam > 1e-5f)
                {
                    float corr = dot / Mathf.Sqrt(normCtrl * normCam);
                    if (corr > bestCorr)
                    {
                        bestCorr = corr;
                        bestLag = lag;
                    }
                }
            }

            Quaternion baseRot = receiver.cameraOrientationReference != null
                ? receiver.cameraOrientationReference.rotation
                : Quaternion.identity;

            // 2. Collect time-aligned sample pairs weighted inversely by speed (waypoints anchor the geometry)
            List<Vector3> rCamPts = new List<Vector3>(n);
            List<Vector3> rWorldPts = new List<Vector3>(n);
            List<float> rWeights = new List<float>(n);

            List<Vector3> lCamPts = new List<Vector3>(n);
            List<Vector3> lWorldPts = new List<Vector3>(n);
            List<float> lWeights = new List<float>(n);

            Vector3 rMeanCam = Vector3.zero, rMeanWorld = Vector3.zero;
            float rWeightSum = 0f;
            Vector3 lMeanCam = Vector3.zero, lMeanWorld = Vector3.zero;
            float lWeightSum = 0f;

            for (int i = 0; i < n - bestLag; i++)
            {
                TrajectoryCalibSample ctrlSample = samples[i];
                TrajectoryCalibSample camSample = samples[i + bestLag];
                float dtStep = (i > 0) ? Mathf.Max(0.005f, ctrlSample.Timestamp - samples[i - 1].Timestamp) : 0.016f;

                if (ctrlSample.HasRight && camSample.HasRight && camSample.RightConfidence > 0.35f)
                {
                    float speed = (i > 0 && samples[i - 1].HasRight)
                        ? (ctrlSample.RightCtrlWorldPos - samples[i - 1].RightCtrlWorldPos).magnitude / dtStep
                        : 0f;
                    float w = camSample.RightConfidence / (1.0f + 2.5f * speed * speed);

                    Vector3 u = baseRot * camSample.RightCamRelWrist;
                    // Subtract approximate head-rotated shoulder anchor to isolate torso-relative wrist movement in room space
                    Vector3 approxShoulder = ctrlSample.HeadPos + ctrlSample.HeadYawRot * shoulderOffsetFromHead;
                    Vector3 p = ctrlSample.RightCtrlWorldPos - approxShoulder;

                    rCamPts.Add(u);
                    rWorldPts.Add(p);
                    rWeights.Add(w);
                    rMeanCam += u * w;
                    rMeanWorld += p * w;
                    rWeightSum += w;
                }

                if (ctrlSample.HasLeft && camSample.HasLeft && camSample.LeftConfidence > 0.35f)
                {
                    float speed = (i > 0 && samples[i - 1].HasLeft)
                        ? (ctrlSample.LeftCtrlWorldPos - samples[i - 1].LeftCtrlWorldPos).magnitude / dtStep
                        : 0f;
                    float w = camSample.LeftConfidence / (1.0f + 2.5f * speed * speed);

                    Vector3 u = baseRot * camSample.LeftCamRelWrist;
                    Vector3 approxLeftOffset = hasIndependentLeftShoulder
                        ? leftShoulderIndependentOffset
                        : new Vector3(-shoulderOffsetFromHead.x, shoulderOffsetFromHead.y, shoulderOffsetFromHead.z);
                    Vector3 approxShoulder = ctrlSample.HeadPos + ctrlSample.HeadYawRot * approxLeftOffset;
                    Vector3 p = ctrlSample.LeftCtrlWorldPos - approxShoulder;

                    lCamPts.Add(u);
                    lWorldPts.Add(p);
                    lWeights.Add(w);
                    lMeanCam += u * w;
                    lMeanWorld += p * w;
                    lWeightSum += w;
                }
            }

            if (rWeightSum < 0.1f && lWeightSum < 0.1f)
            {
                CalibrationStatusMessage = "Guided calibration failed: no valid overlapping controller + camera pairs.";
                return;
            }

            if (rWeightSum > 0.1f) { rMeanCam /= rWeightSum; rMeanWorld /= rWeightSum; }
            if (lWeightSum > 0.1f) { lMeanCam /= lWeightSum; lMeanWorld /= lWeightSum; }

            // 3. Solve Optimal Isotropic Scale `s` and 3D Camera Rotation `qCam` (Yaw, Pitch, Roll) via Polar Decomposition
            float sumU2 = 0f, sumW2 = 0f;
            Vector3 c0 = Vector3.zero, c1 = Vector3.zero, c2 = Vector3.zero;

            for (int i = 0; i < rCamPts.Count; i++)
            {
                Vector3 uc = rCamPts[i] - rMeanCam;
                Vector3 wc = rWorldPts[i] - rMeanWorld;
                float w = rWeights[i];
                sumU2 += w * uc.sqrMagnitude;
                sumW2 += w * wc.sqrMagnitude;
                c0 += wc * (w * uc.x);
                c1 += wc * (w * uc.y);
                c2 += wc * (w * uc.z);
            }
            for (int i = 0; i < lCamPts.Count; i++)
            {
                Vector3 uc = lCamPts[i] - lMeanCam;
                Vector3 wc = lWorldPts[i] - lMeanWorld;
                float w = lWeights[i];
                sumU2 += w * uc.sqrMagnitude;
                sumW2 += w * wc.sqrMagnitude;
                c0 += wc * (w * uc.x);
                c1 += wc * (w * uc.y);
                c2 += wc * (w * uc.z);
            }

            float solvedScale = (sumU2 > 0.01f && sumW2 > 0.01f)
                ? Mathf.Clamp(Mathf.Sqrt(sumW2 / sumU2), 0.65f, 1.50f)
                : receiver.armScale;

            Quaternion qCam = Quaternion.Euler(receiver.cameraPitchOffsetDeg, receiver.cameraYawOffsetDeg, receiver.cameraRollOffsetDeg);
            for (int iter = 0; iter < 25; iter++)
            {
                Vector3 r0 = qCam * Vector3.right;
                Vector3 r1 = qCam * Vector3.up;
                Vector3 r2 = qCam * Vector3.forward;

                float denom = Mathf.Abs(Vector3.Dot(r0, c0) + Vector3.Dot(r1, c1) + Vector3.Dot(r2, c2)) + 1e-6f;
                Vector3 omega = (Vector3.Cross(r0, c0) + Vector3.Cross(r1, c1) + Vector3.Cross(r2, c2)) / denom;
                float wAngle = omega.magnitude;
                if (wAngle < 1e-5f) break;
                qCam = (Quaternion.AngleAxis(wAngle * Mathf.Rad2Deg, omega / wAngle) * qCam).normalized;
            }

            Vector3 solvedEuler = qCam.eulerAngles;
            float solvedPitch = Mathf.Clamp(NormalizeSignedAngle(solvedEuler.x), -35f, 35f);
            float solvedYaw = NormalizeSignedAngle(solvedEuler.y);
            float solvedRoll = Mathf.Clamp(NormalizeSignedAngle(solvedEuler.z), -35f, 35f);
            qCam = Quaternion.Euler(solvedPitch, solvedYaw, solvedRoll);

            receiver.armScale = solvedScale;
            receiver.cameraYawOffsetDeg = solvedYaw;
            receiver.cameraPitchOffsetDeg = solvedPitch;
            receiver.cameraRollOffsetDeg = solvedRoll;

            // 4. Solve Exact Right and Left 3D Shoulder Socket Offsets in Head-Yaw Space
            Vector3 rShoulderSum = Vector3.zero, lShoulderSum = Vector3.zero;
            float rShWeight = 0f, lShWeight = 0f;
            float residualSum = 0f, residualWeight = 0f;

            for (int i = 0; i < n - bestLag; i++)
            {
                TrajectoryCalibSample ctrlSample = samples[i];
                TrajectoryCalibSample camSample = samples[i + bestLag];
                Quaternion invHeadYaw = Quaternion.Inverse(ctrlSample.HeadYawRot);

                if (ctrlSample.HasRight && camSample.HasRight && camSample.RightConfidence > 0.35f)
                {
                    Vector3 mappedOffset = qCam * ((baseRot * camSample.RightCamRelWrist) * solvedScale);
                    Vector3 exactShoulderWorld = ctrlSample.RightCtrlWorldPos - mappedOffset;
                    Vector3 exactLocalOffset = invHeadYaw * (exactShoulderWorld - ctrlSample.HeadPos);
                    rShoulderSum += exactLocalOffset * camSample.RightConfidence;
                    rShWeight += camSample.RightConfidence;
                }
                if (ctrlSample.HasLeft && camSample.HasLeft && camSample.LeftConfidence > 0.35f)
                {
                    Vector3 mappedOffset = qCam * ((baseRot * camSample.LeftCamRelWrist) * solvedScale);
                    Vector3 exactShoulderWorld = ctrlSample.LeftCtrlWorldPos - mappedOffset;
                    Vector3 exactLocalOffset = invHeadYaw * (exactShoulderWorld - ctrlSample.HeadPos);
                    lShoulderSum += exactLocalOffset * camSample.LeftConfidence;
                    lShWeight += camSample.LeftConfidence;
                }
            }

            if (rShWeight > 0.1f)
            {
                Vector3 oR = rShoulderSum / rShWeight;
                oR.x = Mathf.Clamp(oR.x, 0.06f, 0.32f);
                oR.y = Mathf.Clamp(oR.y, -0.32f, -0.03f);
                oR.z = Mathf.Clamp(oR.z, -0.20f, 0.20f);
                shoulderOffsetFromHead = oR;
                hasIndependentLeftShoulder = (lShWeight > 0.1f);
            }

            if (lShWeight > 0.1f)
            {
                Vector3 oL = lShoulderSum / lShWeight;
                oL.x = Mathf.Clamp(oL.x, -0.32f, -0.06f);
                oL.y = Mathf.Clamp(oL.y, -0.32f, -0.03f);
                oL.z = Mathf.Clamp(oL.z, -0.20f, 0.20f);
                if (rShWeight <= 0.1f)
                {
                    // Left-arm only mode: store mirrored X into main shoulderOffsetFromHead
                    shoulderOffsetFromHead = new Vector3(-oL.x, oL.y, oL.z);
                    hasIndependentLeftShoulder = false;
                }
                else
                {
                    // Both arms calibrated: lock independent Left Shoulder so bimanual midline matches to the mm!
                    leftShoulderIndependentOffset = oL;
                    hasIndependentLeftShoulder = true;
                }
            }

            UpdateShoulderAnchorsFromHeadset();

            // 5. Compute Residual 3D Error (cm) Across All Recorded Points for Researcher Verification
            for (int i = 0; i < n - bestLag; i++)
            {
                TrajectoryCalibSample ctrlSample = samples[i];
                TrajectoryCalibSample camSample = samples[i + bestLag];
                if (ctrlSample.HasRight && camSample.HasRight)
                {
                    Vector3 predShoulder = ctrlSample.HeadPos + ctrlSample.HeadYawRot * shoulderOffsetFromHead;
                    Vector3 predWrist = predShoulder + qCam * ((baseRot * camSample.RightCamRelWrist) * solvedScale);
                    residualSum += Vector3.Distance(predWrist, ctrlSample.RightCtrlWorldPos);
                    residualWeight += 1f;
                }
                if (ctrlSample.HasLeft && camSample.HasLeft)
                {
                    Vector3 leftOff = hasIndependentLeftShoulder
                        ? leftShoulderIndependentOffset
                        : new Vector3(-shoulderOffsetFromHead.x, shoulderOffsetFromHead.y, shoulderOffsetFromHead.z);
                    Vector3 predShoulder = ctrlSample.HeadPos + ctrlSample.HeadYawRot * leftOff;
                    Vector3 predWrist = predShoulder + qCam * ((baseRot * camSample.LeftCamRelWrist) * solvedScale);
                    residualSum += Vector3.Distance(predWrist, ctrlSample.LeftCtrlWorldPos);
                    residualWeight += 1f;
                }
            }

            float meanErrCm = (residualWeight > 0f) ? (residualSum / residualWeight) * 100f : 0f;

            // 6. Refine 6-DOF IMU Wrist Orientation Offset using final resting controller orientation & lock handle origin
            TrajectoryCalibSample finalSample = samples[n - 1];
            if (finalSample.HasRight && _rightArmRig != null && _rightArmRig.HandBone != null)
            {
                Quaternion desiredHandWorldRot = finalSample.RightCtrlWorldRot * Quaternion.Euler(90f, 0f, 0f);
                Quaternion baseHandRot = _rightArmRig.HandBone.rotation * Quaternion.Inverse(_rightArmRig.CalibratedHandRotOffset);
                Quaternion localDelta = Quaternion.Inverse(baseHandRot) * desiredHandWorldRot;
                _rightArmRig.CalibratedHandRotOffset = Quaternion.RotateTowards(Quaternion.identity, localDelta, 30f);

                if (experimentPreset == ExperimentPreset.StrengthMachineTask)
                {
                    EnsureHandleMarker();
                    machineHandleOrigin.position = finalSample.RightCtrlWorldPos;
                    machineHandleOrigin.gameObject.SetActive(true);
                }
            }
            if (finalSample.HasLeft && _leftArmRig != null && _leftArmRig.HandBone != null)
            {
                Quaternion desiredHandWorldRot = finalSample.LeftCtrlWorldRot * Quaternion.Euler(90f, 0f, 0f);
                Quaternion baseHandRot = _leftArmRig.HandBone.rotation * Quaternion.Inverse(_leftArmRig.CalibratedHandRotOffset);
                Quaternion localDelta = Quaternion.Inverse(baseHandRot) * desiredHandWorldRot;
                _leftArmRig.CalibratedHandRotOffset = Quaternion.RotateTowards(Quaternion.identity, localDelta, 30f);
            }

            // Lock high-precision 3D path fit so background auto-align doesn't drift over it
            receiver.continuousAutoCalibration = false;
            ResetArmElbowPoles();
            SavePersistentSettings();

            _calibrationFlashTimer = 4.0f;
            CalibrationStatusMessage = $"3D Path Calibrated! Err={meanErrCm:F1}cm | Lag={bestLag}f | Scale={solvedScale:F2}x | YPR=({solvedYaw:F0},{solvedPitch:F0},{solvedRoll:F0})";
            Debug.Log($"[ArmAvatarController] {CalibrationStatusMessage}");
        }

        private static float NormalizeSignedAngle(float deg)
        {
            deg %= 360f;
            if (deg > 180f) deg -= 360f;
            if (deg < -180f) deg += 360f;
            return deg;
        }

        private void OnDisable()
        {
            if (Application.isPlaying)
            {
                SavePersistentSettings();
            }
        }

        private void LateUpdate()
        {
            UpdateShoulderAnchorsFromHeadset();

            if (activeArmMode == ActiveArmMode.RightArmOnly || activeArmMode == ActiveArmMode.BothArms)
            {
                UpdateSingleArmRig(
                    _rightArmRig,
                    receiver != null && receiver.trackingConfidence > 0.30f,
                    receiver != null ? receiver.ShoulderPos : Vector3.zero,
                    receiver != null ? receiver.ElbowPos : Vector3.zero,
                    receiver != null ? receiver.WristPos : Vector3.zero,
                    receiver != null ? receiver.IndexPos : Vector3.zero,
                    receiver != null ? receiver.PinkyPos : Vector3.zero,
                    receiver != null && receiver.handFingersTracked,
                    receiver != null ? receiver.HandLandmarksWorld : null
                );
            }

            if (activeArmMode == ActiveArmMode.LeftArmOnly || activeArmMode == ActiveArmMode.BothArms)
            {
                UpdateSingleArmRig(
                    _leftArmRig,
                    receiver != null && receiver.leftTrackingConfidence > 0.30f,
                    receiver != null ? receiver.LeftShoulderPos : Vector3.zero,
                    receiver != null ? receiver.LeftElbowPos : Vector3.zero,
                    receiver != null ? receiver.LeftWristPos : Vector3.zero,
                    receiver != null ? receiver.LeftIndexPos : Vector3.zero,
                    receiver != null ? receiver.LeftPinkyPos : Vector3.zero,
                    receiver != null && receiver.leftHandFingersTracked,
                    receiver != null ? receiver.LeftHandLandmarksWorld : null
                );
            }
        }

        private void UpdateSingleArmRig(
            ProceduralArmRig rig,
            bool hasTracking,
            Vector3 sIn, Vector3 eIn, Vector3 wIn, Vector3 idxIn, Vector3 pkyIn,
            bool fingersTracked, Vector3[] handWorldIn)
        {
            if (rig == null || rig.UpperArmBone == null) return;

            if (rig.ShoulderAnchor != null)
            {
                rig.UpperArmBone.position = rig.ShoulderAnchor.position;
            }

            float sideSign = rig.IsRight ? 1f : -1f;

            if (!hasTracking)
            {
                Vector3 sPos = rig.UpperArmBone.position;
                Quaternion camYaw = _vrCamera != null ? Quaternion.Euler(0f, _vrCamera.transform.eulerAngles.y, 0f) : Quaternion.identity;
                Vector3 restElbow = sPos + camYaw * new Vector3(0.04f * sideSign, -0.20f, 0.14f);
                Vector3 restWrist = restElbow + camYaw * new Vector3(-0.06f * sideSign, 0.03f, 0.22f);
                Vector3 restIndex = restWrist + camYaw * new Vector3(-0.02f * sideSign, 0.02f, 0.08f);
                Vector3 restPinky = restWrist + camYaw * new Vector3(0.02f * sideSign, -0.02f, 0.08f);
                SolveCriticallyDampedTrigonometricIK(rig, sPos, restElbow, restWrist, restIndex, restPinky, restWrist);
                UpdateArticulatedFingersAndMesh(rig, rig.HandBone.position, false, null);
                return;
            }

            float now = Time.unscaledTime;
            rig.DelayQueue.Enqueue(new ProceduralArmRig.DelayedSample
            {
                Timestamp = now,
                Shoulder = sIn,
                Elbow = eIn,
                Wrist = wIn,
                Index = idxIn,
                Pinky = pkyIn
            });

            ProceduralArmRig.DelayedSample sample = GetDelayedSample(rig, now, artificialDelayMs * 0.001f);

            Vector3 targetShoulder = rig.UpperArmBone.position;
            Vector3 targetElbow = sample.Elbow;
            Vector3 targetWrist = sample.Wrist;

            // 1. Movement Gain Perturbation (uses CalibratedHandleOrigin in StrengthMachineTask, or Shoulder in GeneralFreeMovement)
            Vector3 gainPivot = (experimentPreset == ExperimentPreset.StrengthMachineTask && machineHandleOrigin != null)
                ? machineHandleOrigin.position
                : targetShoulder;
            if (Mathf.Abs(movementGain - 1.0f) > 0.001f)
            {
                targetWrist = gainPivot + (targetWrist - gainPivot) * movementGain;
            }

            // 2. Viscous Pseudo-Haptic Drag
            if (pseudoHapticDrag > 0.01f)
            {
                if (!rig.DragInitialized)
                {
                    rig.DraggedWristPos = targetWrist;
                    rig.DragInitialized = true;
                }
                float speed = 1f / (1f + pseudoHapticDrag);
                rig.DraggedWristPos = Vector3.Lerp(rig.DraggedWristPos, targetWrist, Mathf.Clamp01(Time.deltaTime * 60f * speed));
                targetWrist = rig.DraggedWristPos;
            }
            else
            {
                rig.DragInitialized = false;
            }

            // 3. Elbow Flexion Angular Offset Perturbation
            if (Mathf.Abs(elbowFlexionOffsetDeg) > 0.01f)
            {
                Vector3 bendAxis = Vector3.Cross(rig.SmoothedElbowPerpDir, (targetWrist - targetShoulder).normalized).normalized;
                if (bendAxis.sqrMagnitude < 0.001f) bendAxis = Vector3.right;
                Quaternion rotOffset = Quaternion.AngleAxis(elbowFlexionOffsetDeg, bendAxis);
                targetWrist = targetElbow + rotOffset * (targetWrist - targetElbow);
            }

            // 4. Solve CDSO PD Controlled Trigonometric IK with Smooth Sigmoid Anatomical Constraints
            SolveCriticallyDampedTrigonometricIK(rig, targetShoulder, targetElbow, targetWrist, sample.Index, sample.Pinky, wIn);

            // 5. Update 21-Finger Hand & Continuous 3D Skinned Mesh
            UpdateArticulatedFingersAndMesh(rig, rig.HandBone.position, fingersTracked, handWorldIn);
        }

        private static ProceduralArmRig.DelayedSample GetDelayedSample(ProceduralArmRig rig, float now, float delaySeconds)
        {
            if (delaySeconds <= 0.001f)
            {
                ProceduralArmRig.DelayedSample latest = default;
                while (rig.DelayQueue.Count > 0)
                    latest = rig.DelayQueue.Dequeue();
                return latest;
            }

            ProceduralArmRig.DelayedSample chosen = rig.DelayQueue.Peek();
            while (rig.DelayQueue.Count > 1 && (now - rig.DelayQueue.Peek().Timestamp) >= delaySeconds)
            {
                chosen = rig.DelayQueue.Dequeue();
            }
            return chosen;
        }

        /// <summary>
        /// v6.0 Critically Damped Second-Order (CDSO) PD Trigonometric IK Solver:
        ///   - Uses smooth C-infinity SoftMax/SoftMin anatomical constraints (zero hard-boundary teleport pops!).
        ///   - Steps wrist trajectory and elbow swivel via closed-form Critically Damped PD Springs (zeta = 1.0),
        ///     giving finite physical limb inertia that absorbs residual noise with zero overshoot.
        ///   - Solves exact 3D Elbow position via cross-product-free trigonometry (`cos(A)*d_wrist + sin(A)*e_perp`).
        /// </summary>
        private void SolveCriticallyDampedTrigonometricIK(
            ProceduralArmRig rig,
            Vector3 shoulderPos, Vector3 elbowHint, Vector3 wristTarget,
            Vector3 indexPos, Vector3 pinkyPos, Vector3 rawWristForHandRot)
        {
            float a = prototypeUpperArmLength;
            float b = prototypeForearmLength;
            float maxReach = (a + b) - 0.008f;
            float minReach = Mathf.Abs(a - b) + 0.03f;
            float sideSign = rig.IsRight ? 1f : -1f;
            float dt = Mathf.Clamp(Time.unscaledDeltaTime, 0.001f, 0.05f);

            Quaternion headYaw = _vrCamera != null ? Quaternion.Euler(0f, _vrCamera.transform.eulerAngles.y, 0f) : Quaternion.identity;
            Quaternion invHeadYaw = Quaternion.Inverse(headYaw);

            // --- 1. Smooth C-infinity Soft-Sigmoid Wrist Anatomical Bounds ---
            Vector3 localWristRelShoulder = invHeadYaw * (wristTarget - shoulderPos);
            localWristRelShoulder.z = BiomechanicalMath.SoftMax(localWristRelShoulder.z, -0.10f, 30f);

            bool isHoldingHeadset = false;
            if (_vrCamera != null)
            {
                Vector3 wristToHead = wristTarget - _vrCamera.transform.position;
                if (wristToHead.magnitude < 0.28f && localWristRelShoulder.y > -0.05f)
                {
                    isHoldingHeadset = true;
                }
            }
            Vector3 boundedWristTarget = shoulderPos + headYaw * localWristRelShoulder;

            // --- 2. Critically Damped Second-Order (CDSO) PD Spring on Wrist Trajectory ---
            if (!rig.PoseInitialized)
            {
                rig.WristSpring.Snap(boundedWristTarget);
                rig.PoseInitialized = true;
            }
            Vector3 smoothedWrist = rig.WristSpring.Step(boundedWristTarget, dt, pdNaturalFrequencyHz);

            Vector3 toWrist = smoothedWrist - shoulderPos;
            float c = Mathf.Clamp(toWrist.magnitude, minReach, maxReach);
            Vector3 dirToWrist = toWrist.sqrMagnitude > 0.0001f ? toWrist.normalized : (headYaw * Vector3.forward);

            float cosAngleA = Mathf.Clamp((a * a + c * c - b * b) / (2f * a * c), -1f, 1f);
            float sinAngleA = Mathf.Sqrt(Mathf.Max(0f, 1f - cosAngleA * cosAngleA));

            // --- 3. Smooth C-infinity Soft-Sigmoid Elbow Anatomical Constraints ---
            Vector3 anatomicalPoleWorld = (headYaw * new Vector3(0.38f * sideSign, -0.90f, -0.15f)).normalized;

            Vector3 localElbowRelShoulder = invHeadYaw * (elbowHint - shoulderPos);
            // Smoothly prevent upward elbow inversions & medial chest crossings with zero step discontinuities!
            localElbowRelShoulder.y = BiomechanicalMath.SoftMin(localElbowRelShoulder.y, -0.03f, 18f);
            localElbowRelShoulder.x = sideSign * BiomechanicalMath.SoftMax(localElbowRelShoulder.x * sideSign, -0.10f, 22f);

            Vector3 clampedObsElbowDir = (headYaw * localElbowRelShoulder).normalized;

            Vector3 obsPerp = clampedObsElbowDir - Vector3.Dot(clampedObsElbowDir, dirToWrist) * dirToWrist;
            Vector3 polePerp = anatomicalPoleWorld - Vector3.Dot(anatomicalPoleWorld, dirToWrist) * dirToWrist;
            if (polePerp.sqrMagnitude < 0.001f)
                polePerp = headYaw * Vector3.down;
            polePerp.Normalize();

            Vector3 targetElbowPerp;
            if (obsPerp.sqrMagnitude < 0.005f)
            {
                targetElbowPerp = polePerp;
            }
            else
            {
                obsPerp.Normalize();
                // Continuous smooth sigmoid weight toward polePerp when approaching dorsal hyperextension
                float dotPole = Vector3.Dot(obsPerp, polePerp);
                float antiFlipBlend = Mathf.SmoothStep(0f, 0.80f, Mathf.InverseLerp(0.35f, -0.25f, dotPole));
                obsPerp = Vector3.Slerp(obsPerp, polePerp, antiFlipBlend).normalized;

                float reachRatio = c / maxReach;
                float poleWeight = isHoldingHeadset
                    ? 0.80f
                    : Mathf.SmoothStep(0f, 1f, Mathf.InverseLerp(0.75f, 0.96f, reachRatio));

                targetElbowPerp = Vector3.Slerp(obsPerp, polePerp, poleWeight).normalized;
            }

            // CDSO PD Spring on Elbow Swivel Direction
            Vector3 springPerp = rig.ElbowPerpSpring.Step(targetElbowPerp, dt, pdNaturalFrequencyHz * 0.85f);
            springPerp -= Vector3.Dot(springPerp, dirToWrist) * dirToWrist;
            if (springPerp.sqrMagnitude < 0.001f)
                springPerp = polePerp;
            rig.SmoothedElbowPerpDir = springPerp.normalized;

            // --- 4. Direct Trigonometric IK Solution ---
            Vector3 ikElbowDir = (dirToWrist * cosAngleA + rig.SmoothedElbowPerpDir * sinAngleA).normalized;
            Vector3 ikElbowPos = shoulderPos + ikElbowDir * a;
            Vector3 ikForearmDir = (smoothedWrist - ikElbowPos).normalized;
            Vector3 bendPlaneNormal = Vector3.Cross(ikElbowDir, ikForearmDir);
            if (bendPlaneNormal.sqrMagnitude < 0.001f)
                bendPlaneNormal = Vector3.Cross(rig.SmoothedElbowPerpDir, dirToWrist);
            bendPlaneNormal.Normalize();

            Quaternion targetUpperRot = AlignBoneAxes(ikElbowDir, bendPlaneNormal);
            Quaternion targetForeRot = AlignBoneAxes(ikForearmDir, bendPlaneNormal);

            // --- 5. Hand Orientation with CDSO PD Spring & Anatomical Wrist ROM Clamping ---
            Vector3 rawHandFwd = ((indexPos + pinkyPos) * 0.5f - rawWristForHandRot);
            Vector3 targetHandFwd = rawHandFwd.sqrMagnitude > 0.0004f ? rawHandFwd.normalized : ikForearmDir;
            if (Vector3.Angle(ikForearmDir, targetHandFwd) > 70f)
            {
                targetHandFwd = Vector3.RotateTowards(ikForearmDir, targetHandFwd, 70f * Mathf.Deg2Rad, 0f).normalized;
            }
            Vector3 smoothedHandFwd = rig.HandFwdSpring.Step(targetHandFwd, dt, pdNaturalFrequencyHz * 1.4f).normalized;
            if (smoothedHandFwd.sqrMagnitude < 0.001f) smoothedHandFwd = ikForearmDir;

            Vector3 acrossKnuckles = rig.IsRight ? (indexPos - pinkyPos) : (pinkyPos - indexPos);
            Vector3 rawPalmNorm = acrossKnuckles.sqrMagnitude > 0.0002f
                ? Vector3.Cross(smoothedHandFwd, acrossKnuckles.normalized)
                : bendPlaneNormal;
            if (rawPalmNorm.sqrMagnitude < 0.001f) rawPalmNorm = bendPlaneNormal;
            Vector3 smoothedPalmNorm = rig.PalmNormSpring.Step(rawPalmNorm.normalized, dt, pdNaturalFrequencyHz * 1.4f).normalized;
            if (smoothedPalmNorm.sqrMagnitude < 0.001f) smoothedPalmNorm = bendPlaneNormal;

            Quaternion targetHandRot = AlignBoneAxes(smoothedHandFwd, smoothedPalmNorm) * rig.CalibratedHandRotOffset;

            // --- 6. Drive Bone Rotations via Critically Damped Quaternion PD Controllers ---
            float rotFreq = pdNaturalFrequencyHz * 1.35f;
            rig.UpperArmBone.position = shoulderPos;
            rig.UpperArmBone.rotation = rig.UpperArmRotPD.Step(targetUpperRot, dt, rotFreq);

            rig.ForearmBone.position = ikElbowPos;
            rig.ForearmBone.rotation = rig.ForearmRotPD.Step(targetForeRot, dt, rotFreq);

            rig.HandBone.position = ikElbowPos + ikForearmDir * b;
            rig.HandBone.rotation = rig.HandRotPD.Step(targetHandRot, dt, rotFreq);
        }

        private void UpdateArticulatedFingersAndMesh(
            ProceduralArmRig rig, Vector3 ikWristWorldPos, bool hasLiveFingerTracking, Vector3[] handLandmarksWorld)
        {
            Vector3[] targetPts = new Vector3[21];
            targetPts[0] = ikWristWorldPos;

            if (hasLiveFingerTracking && handLandmarksWorld != null)
            {
                Vector3 mpWrist = handLandmarksWorld[0];
                for (int i = 1; i < 21; i++)
                {
                    Vector3 offsetFromWrist = (handLandmarksWorld[i] - mpWrist) * handSizeScale;
                    targetPts[i] = ikWristWorldPos + offsetFromWrist;
                }
            }
            else
            {
                for (int i = 1; i < 21; i++)
                {
                    targetPts[i] = ikWristWorldPos + rig.HandBone.rotation * (rig.DefaultFingerRelPos[i] * handSizeScale);
                }
            }

            rig.SmoothedFingerWorldPos[0] = ikWristWorldPos;
            float fingerLerp = rig.FingersInitialized ? Mathf.Clamp01(Time.deltaTime * 40f) : 1f;
            rig.FingersInitialized = true;

            for (int i = 1; i < 21; i++)
            {
                rig.SmoothedFingerWorldPos[i] = Vector3.Lerp(rig.SmoothedFingerWorldPos[i], targetPts[i], fingerLerp);
            }

            if (rig.ContinuousSkinnedMesh != null && rig.ContinuousSkinnedMesh.MeshObject.activeSelf)
            {
                Vector3 bendNormal = Vector3.Cross(
                    (rig.ForearmBone.position - rig.UpperArmBone.position).normalized,
                    (ikWristWorldPos - rig.ForearmBone.position).normalized
                );
                Vector3 palmNorm = rig.PalmNormSpring.Initialized ? rig.PalmNormSpring.Value.normalized : Vector3.up;
                if (bendNormal.sqrMagnitude < 0.001f) bendNormal = palmNorm;

                rig.ContinuousSkinnedMesh.UpdateMeshPose(
                    rig.UpperArmBone.position,
                    rig.ForearmBone.position,
                    ikWristWorldPos,
                    rig.SmoothedFingerWorldPos,
                    palmNorm,
                    bendNormal.normalized
                );
            }

            if (rig.BubblePrimitivesGroup != null && rig.BubblePrimitivesGroup.activeSelf)
            {
                for (int i = 0; i < 21; i++)
                {
                    if (rig.FingerJointSpheres[i] != null)
                        rig.FingerJointSpheres[i].position = rig.SmoothedFingerWorldPos[i];
                }

                for (int b = 0; b < 20; b++)
                {
                    Transform cap = rig.FingerBoneCapsules[b];
                    if (cap == null) continue;

                    int i0 = FingerBonePairs[b, 0];
                    int i1 = FingerBonePairs[b, 1];
                    Vector3 p0 = rig.SmoothedFingerWorldPos[i0];
                    Vector3 p1 = rig.SmoothedFingerWorldPos[i1];
                    Vector3 diff = p1 - p0;
                    float len = diff.magnitude;

                    cap.position = (p0 + p1) * 0.5f;
                    if (len > 0.001f) cap.up = diff / len;

                    float radius = (i0 == 0) ? 0.0085f : 0.0075f;
                    cap.localScale = new Vector3(radius * 2f, len * 0.5f, radius * 2f);
                }

                if (rig.PalmPlateMesh != null)
                {
                    Vector3[] pts = rig.SmoothedFingerWorldPos;
                    Vector3 knuckleMid = (pts[5] + pts[9] + pts[13] + pts[17]) * 0.25f;
                    Vector3 palmCenter = Vector3.Lerp(pts[0], knuckleMid, 0.52f);
                    Vector3 palmFwd = (knuckleMid - pts[0]).normalized;
                    Vector3 palmRight = (pts[5] - pts[17]).normalized;
                    Vector3 palmUp = Vector3.Cross(palmFwd, palmRight).normalized;

                    rig.PalmPlateMesh.position = palmCenter;
                    if (palmFwd.sqrMagnitude > 0.01f && palmUp.sqrMagnitude > 0.01f)
                        rig.PalmPlateMesh.rotation = Quaternion.LookRotation(palmUp, palmFwd);

                    float palmWidth = Mathf.Clamp(Vector3.Distance(pts[5], pts[17]) * 1.15f, 0.055f, 0.11f);
                    float palmLen = Mathf.Clamp(Vector3.Distance(pts[0], knuckleMid) * 0.95f, 0.055f, 0.11f);
                    rig.PalmPlateMesh.localScale = new Vector3(palmWidth, palmLen, 0.020f);
                }
            }

            if (handVisualStyle == HandVisualStyle.CustomRiggedPrefab && rig.CustomHandInstance != null)
            {
                DriveCustomRiggedHand(rig);
            }
        }

        private void DriveCustomRiggedHand(ProceduralArmRig rig)
        {
            rig.CustomHandInstance.position = rig.SmoothedFingerWorldPos[0];
            rig.CustomHandInstance.rotation = rig.HandBone.rotation;
            Vector3 palmNorm = rig.PalmNormSpring.Initialized ? rig.PalmNormSpring.Value.normalized : Vector3.up;

            for (int b = 0; b < 20; b++)
            {
                int i0 = FingerBonePairs[b, 0];
                int i1 = FingerBonePairs[b, 1];
                Transform bone = rig.CustomMappedBones[i0];
                if (bone == null || i0 == 0) continue;

                Vector3 dir = (rig.SmoothedFingerWorldPos[i1] - rig.SmoothedFingerWorldPos[i0]);
                if (dir.sqrMagnitude > 0.00001f)
                {
                    bone.rotation = Quaternion.LookRotation(dir.normalized, palmNorm);
                }
            }
        }

        public void ApplyVisualStyleToBothArms()
        {
            ApplyVisualStyleToRig(_rightArmRig, customRightHandPrefab);
            ApplyVisualStyleToRig(_leftArmRig, customLeftHandPrefab);
        }

        private void ApplyVisualStyleToRig(ProceduralArmRig rig, Transform customPrefabTemplate)
        {
            if (rig == null) return;

            bool isBubble = (handVisualStyle == HandVisualStyle.BubbleJoints);
            bool isCustom = (handVisualStyle == HandVisualStyle.CustomRiggedPrefab && customPrefabTemplate != null);
            bool isContinuousMesh = !isBubble && !isCustom;

            rig.UpperArmMesh.gameObject.SetActive(isBubble || isCustom);
            rig.ForearmMesh.gameObject.SetActive(isBubble || isCustom);
            rig.ElbowSphere.gameObject.SetActive(isBubble || isCustom);

            if (rig.BubblePrimitivesGroup != null)
                rig.BubblePrimitivesGroup.SetActive(isBubble);

            if (rig.ContinuousSkinnedMesh != null)
            {
                rig.ContinuousSkinnedMesh.MeshObject.SetActive(isContinuousMesh);
                if (handVisualStyle == HandVisualStyle.HumanSkinContinuousMesh)
                {
                    rig.ContinuousSkinnedMesh.SetStyle(new Color(0.91f, 0.73f, 0.61f), false);
                }
                else
                {
                    rig.ContinuousSkinnedMesh.SetStyle(new Color(0.88f, 0.93f, 0.98f), true);
                }
            }

            if (isCustom && rig.CustomHandInstance == null)
            {
                rig.CustomHandInstance = Instantiate(customPrefabTemplate, rig.RootGo.transform);
                rig.CustomHandInstance.name = "Custom_Microsoft_Hand_Rig";
                AutoMapCustomHandBones(rig.CustomHandInstance, rig.CustomMappedBones);
            }
            if (rig.CustomHandInstance != null)
            {
                rig.CustomHandInstance.gameObject.SetActive(isCustom);
            }
        }

        private static void AutoMapCustomHandBones(Transform root, Transform[] targetArray)
        {
            string[][] keywords = new string[21][]
            {
                new[] { "wrist", "root", "hand" },
                new[] { "thumb_metacarpal", "thumb_0", "thumb1" },
                new[] { "thumb_proximal", "thumb_1", "thumb2" },
                new[] { "thumb_distal", "thumb_2", "thumb3" },
                new[] { "thumb_tip", "thumb_3", "thumb4" },
                new[] { "index_metacarpal", "index_0" },
                new[] { "index_proximal", "index_1", "index1" },
                new[] { "index_intermediate", "index_2", "index2" },
                new[] { "index_distal", "index_3", "index3" },
                new[] { "middle_metacarpal", "middle_0" },
                new[] { "middle_proximal", "middle_1", "middle1" },
                new[] { "middle_intermediate", "middle_2", "middle2" },
                new[] { "middle_distal", "middle_3", "middle3" },
                new[] { "ring_metacarpal", "ring_0" },
                new[] { "ring_proximal", "ring_1", "ring1" },
                new[] { "ring_intermediate", "ring_2", "ring2" },
                new[] { "ring_distal", "ring_3", "ring3" },
                new[] { "pinky_metacarpal", "little_metacarpal", "pinky_0" },
                new[] { "pinky_proximal", "little_proximal", "pinky_1", "pinky1" },
                new[] { "pinky_intermediate", "little_intermediate", "pinky_2", "pinky2" },
                new[] { "pinky_distal", "little_distal", "pinky_3", "pinky3" },
            };

            Transform[] allChildren = root.GetComponentsInChildren<Transform>(true);
            for (int i = 0; i < 21; i++)
            {
                foreach (Transform t in allChildren)
                {
                    string lower = t.name.ToLowerInvariant();
                    foreach (string kw in keywords[i])
                    {
                        if (lower.Contains(kw))
                        {
                            targetArray[i] = t;
                            break;
                        }
                    }
                    if (targetArray[i] != null) break;
                }
            }
        }

        private Quaternion AlignBoneAxes(Vector3 targetForward, Vector3 targetUp)
        {
            Quaternion worldRot = Quaternion.LookRotation(targetForward, targetUp);
            Quaternion localOffset = Quaternion.Inverse(Quaternion.LookRotation(Vector3.up, Vector3.forward));
            return worldRot * localOffset;
        }

        private void OnGUI()
        {
            if (!showOnScreenCalibrationHUD || receiver == null) return;

            Matrix4x4 prevMatrix = GUI.matrix;
            float autoDpiScale = Mathf.Max(1.0f, Screen.height / 1080f) * hudScale;
            GUI.matrix = Matrix4x4.TRS(Vector3.zero, Quaternion.identity, new Vector3(autoDpiScale, autoDpiScale, 1f));

            GUIStyle boxStyle = new GUIStyle(GUI.skin.box) { fontSize = 14, alignment = TextAnchor.UpperLeft };
            GUIStyle labelStyle = new GUIStyle(GUI.skin.label) { fontSize = 13, richText = true };
            GUIStyle btnStyle = new GUIStyle(GUI.skin.button) { fontSize = 13, fontStyle = FontStyle.Bold };

            GUILayout.BeginArea(new Rect(12, 12, 540, 470), boxStyle);

            bool fingersOk = (activeArmMode == ActiveArmMode.LeftArmOnly) ? receiver.leftHandFingersTracked : receiver.handFingersTracked;
            string fingerBadge = fingersOk
                ? "<color=#44FF66><b>21 FINGERS</b></color>"
                : "<color=#FFAA33><b>ARM ONLY</b></color>";
            string autoBadge = receiver.continuousAutoCalibration
                ? (receiver.autoCalibrationLocked ? "<color=#66EEFF><b>AUTO-LOCKED</b></color>" : "<color=#FFEE66><b>AUTO-ALIGNING</b></color>")
                : "<color=#CCCCCC><b>MANUAL</b></color>";

            GUILayout.Label($"<b>Pimax + D455C v6.2 (3D Path Procrustes + CDSO PD)</b> | {fingerBadge} | {autoBadge}", labelStyle);
            GUILayout.Label($"Status: <b>{CalibrationStatusMessage}</b>", labelStyle);
            GUILayout.Label("<color=#AADDFF>VR Menu [Tab / A / X]: 3s Gaze Hold Clicks | Ctrl: [B/Y]=Mesh [Grip]=Arm [Grip+Trig]=Match</color>", labelStyle);

            GUILayout.BeginHorizontal();
            GUI.backgroundColor = _calibrationFlashTimer > 0f ? Color.green : new Color(0.15f, 0.85f, 1f);
            if (GUILayout.Button("Calibrate Cam (C)", btnStyle, GUILayout.Height(34)))
            {
                PerformOneClickCalibration();
            }
            GUI.backgroundColor = new Color(0.25f, 0.95f, 0.55f);
            if (GUILayout.Button("Guided 8s Path Calib", btnStyle, GUILayout.Height(34)))
            {
                GetComponent<VRDashboardAndController>()?.StartGuidedPathCalibrationPrompt();
            }
            GUI.backgroundColor = receiver.continuousAutoCalibration ? new Color(0.4f, 0.9f, 0.7f) : new Color(0.7f, 0.7f, 0.7f);
            if (GUILayout.Button($"Auto-Align: {(receiver.continuousAutoCalibration ? "ON" : "OFF")}", btnStyle, GUILayout.Height(34)))
            {
                ToggleContinuousAutoCalibration();
            }
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUI.backgroundColor = new Color(0.95f, 0.78f, 0.25f);
            if (GUILayout.Button($"Arm: {activeArmMode} (R)", btnStyle, GUILayout.Height(30)))
            {
                CycleArmMode();
            }
            GUI.backgroundColor = new Color(0.55f, 0.92f, 0.65f);
            if (GUILayout.Button($"Mesh: {handVisualStyle} (M)", btnStyle, GUILayout.Height(30)))
            {
                CycleVisualStyle();
            }
            GUILayout.EndHorizontal();

            GUI.backgroundColor = new Color(0.85f, 0.75f, 1f);
            if (GUILayout.Button($"Experiment Task: {experimentPreset}", btnStyle, GUILayout.Height(28)))
            {
                CycleExperimentPreset();
            }
            GUI.backgroundColor = Color.white;

            GUILayout.Space(4);
            GUILayout.Label($"<b>PD Controller Freq: {pdNaturalFrequencyHz:F1} Hz</b> | <b>Kalman Vel Noise: {receiver.kalmanProcessNoiseVel:F1}</b>", labelStyle);
            GUILayout.BeginHorizontal();
            pdNaturalFrequencyHz = GUILayout.HorizontalSlider(pdNaturalFrequencyHz, 1.5f, 12.0f);
            receiver.kalmanProcessNoiseVel = GUILayout.HorizontalSlider(receiver.kalmanProcessNoiseVel, 0.3f, 10.0f);
            GUILayout.EndHorizontal();

            GUILayout.Label($"<b>Shoulder Height (Y): {shoulderOffsetFromHead.y:F2}m</b> | <b>Width (X): {shoulderOffsetFromHead.x:F2}m</b>", labelStyle);
            GUILayout.BeginHorizontal();
            shoulderOffsetFromHead.y = GUILayout.HorizontalSlider(shoulderOffsetFromHead.y, -0.35f, 0.05f);
            shoulderOffsetFromHead.x = GUILayout.HorizontalSlider(shoulderOffsetFromHead.x, 0.05f, 0.35f);
            GUILayout.EndHorizontal();

            GUILayout.Label($"<b>Camera Angle (Yaw): {receiver.cameraYawOffsetDeg:F0} deg</b>", labelStyle);
            receiver.cameraYawOffsetDeg = GUILayout.HorizontalSlider(receiver.cameraYawOffsetDeg, -180f, 180f);

            GUILayout.Label($"<b>Psychophysics Gain: {movementGain:F2}x</b> | <b>HUD Scale: {hudScale:F2}x</b>", labelStyle);
            GUILayout.BeginHorizontal();
            movementGain = GUILayout.HorizontalSlider(movementGain, 0.3f, 2.2f);
            hudScale = GUILayout.HorizontalSlider(hudScale, 1.0f, 2.5f);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            if (GUILayout.Button("Save Settings", GUILayout.Height(26)))
            {
                SavePersistentSettings();
                CalibrationStatusMessage = "Saved all current sliders to persistent memory!";
            }
            if (GUILayout.Button("Reset Defaults", GUILayout.Height(26)))
            {
                ResetCalibrationToDefaults();
            }
            GUILayout.EndHorizontal();

            GUILayout.EndArea();
            GUI.matrix = prevMatrix;
        }

        private ProceduralArmRig BuildArmRig(bool isRight, Transform shoulderAnchor)
        {
            ProceduralArmRig rig = new ProceduralArmRig(isRight);
            rig.ShoulderAnchor = shoulderAnchor;

            string prefix = isRight ? "Right" : "Left";
            rig.RootGo = new GameObject($"{prefix}_VR_Arm_Rig");
            rig.RootGo.transform.SetParent(this.transform, false);

            rig.UpperArmBone = new GameObject($"{prefix}UpperArmBone").transform;
            rig.UpperArmBone.SetParent(rig.RootGo.transform, false);
            rig.UpperArmBone.position = shoulderAnchor != null ? shoulderAnchor.position : transform.position;

            rig.ForearmBone = new GameObject($"{prefix}ForearmBone").transform;
            rig.ForearmBone.SetParent(rig.UpperArmBone, false);
            rig.ForearmBone.localPosition = new Vector3(0f, prototypeUpperArmLength, 0f);

            rig.HandBone = new GameObject($"{prefix}HandBone").transform;
            rig.HandBone.SetParent(rig.ForearmBone, false);
            rig.HandBone.localPosition = new Vector3(0f, prototypeForearmLength, 0f);

            rig.ContinuousSkinnedMesh = new ContinuousHandArmMesh(rig.RootGo.transform, $"{prefix}_Continuous_3D_HandArm_Mesh", isRight);

            Color skinColor = new Color(0.93f, 0.76f, 0.64f);
            Color jointColor = new Color(0.98f, 0.84f, 0.45f);
            Color tipColor = new Color(0.96f, 0.62f, 0.52f);

            rig.UpperArmMesh = CreateLimbMesh(rig.UpperArmBone, $"{prefix}UpperArmVisual", prototypeUpperArmLength, 0.040f, new Color(0.22f, 0.52f, 0.82f));
            rig.ForearmMesh = CreateLimbMesh(rig.ForearmBone, $"{prefix}ForearmVisual", prototypeForearmLength, 0.033f, new Color(0.32f, 0.68f, 0.92f));
            rig.ElbowSphere = CreateJointSphere(rig.ForearmBone, $"{prefix}ElbowJointVisual", 0.040f, new Color(0.95f, 0.75f, 0.2f));

            rig.BubblePrimitivesGroup = new GameObject($"{prefix}_BubbleJoint_Hand_Primitives");
            rig.BubblePrimitivesGroup.transform.SetParent(rig.RootGo.transform, false);

            rig.PalmPlateMesh = GameObject.CreatePrimitive(PrimitiveType.Cube).transform;
            rig.PalmPlateMesh.name = $"{prefix}PalmPlate";
            Destroy(rig.PalmPlateMesh.GetComponent<Collider>());
            rig.PalmPlateMesh.SetParent(rig.BubblePrimitivesGroup.transform, false);
            ApplyColor(rig.PalmPlateMesh.gameObject, skinColor);

            for (int i = 0; i < 21; i++)
            {
                GameObject sph = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                sph.name = $"{prefix}HandJoint_{i}";
                Destroy(sph.GetComponent<Collider>());
                sph.transform.SetParent(rig.BubblePrimitivesGroup.transform, false);
                bool isWrist = (i == 0);
                bool isTip = (i == 4 || i == 8 || i == 12 || i == 16 || i == 20);
                float diam = isWrist ? 0.042f : (isTip ? 0.015f : 0.0175f);
                sph.transform.localScale = Vector3.one * diam;
                ApplyColor(sph, isTip ? tipColor : jointColor);
                rig.FingerJointSpheres[i] = sph.transform;
            }

            for (int b = 0; b < 20; b++)
            {
                GameObject cap = GameObject.CreatePrimitive(PrimitiveType.Capsule);
                cap.name = $"{prefix}FingerBone_{b}";
                Destroy(cap.GetComponent<Collider>());
                cap.transform.SetParent(rig.BubblePrimitivesGroup.transform, false);
                ApplyColor(cap, skinColor);
                rig.FingerBoneCapsules[b] = cap.transform;
            }

            return rig;
        }

        private static Transform CreateLimbMesh(Transform parentBone, string name, float length, float radius, Color color)
        {
            GameObject limb = GameObject.CreatePrimitive(PrimitiveType.Capsule);
            limb.name = name;
            Destroy(limb.GetComponent<Collider>());
            limb.transform.SetParent(parentBone, false);
            limb.transform.localPosition = new Vector3(0f, length * 0.5f, 0f);
            limb.transform.localScale = new Vector3(radius * 2f, length * 0.5f, radius * 2f);
            ApplyColor(limb, color);
            return limb.transform;
        }

        private static Transform CreateJointSphere(Transform parentBone, string name, float radius, Color color)
        {
            GameObject joint = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            joint.name = name;
            Destroy(joint.GetComponent<Collider>());
            joint.transform.SetParent(parentBone, false);
            joint.transform.localPosition = Vector3.zero;
            joint.transform.localScale = Vector3.one * (radius * 2f);
            ApplyColor(joint, color);
            return joint.transform;
        }

        private static void ApplyColor(GameObject go, Color color)
        {
            Renderer r = go.GetComponent<Renderer>();
            if (r != null && r.sharedMaterial != null)
            {
                Material mat = new Material(r.sharedMaterial);
                mat.color = color;
                if (mat.HasProperty("_BaseColor"))
                    mat.SetColor("_BaseColor", color);
                r.material = mat;
            }
        }
    }
}
