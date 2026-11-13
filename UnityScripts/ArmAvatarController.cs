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

    /// <summary>
    /// Dual-Arm + 21-Finger Articulated Hand Controller for Pimax VR Psychophysics (v5.0).
    /// Upgrades in v5.0:
    ///   1. TRUE Continuous 3D Hand & Arm Mesh (ContinuousHandArmMesh):
    ///      Press 'M' to toggle between BubbleJoints, MicrosoftContinuousMesh (unified smooth-shaded 3D
    ///      hand & arm mesh with sculpted palm, thumb thenar muscle, and seamless tapered fingers),
    ///      and HumanSkinContinuousMesh!
    ///   2. Anatomical Impossible-Pose Filter & Cross-Product-Free Trigonometric IK:
    ///      Eliminates the 180-degree sign-flip bug when starting with hands near the VR headset,
    ///      and clamps elbow/wrist trajectories to valid human biomechanical ROM cones.
    ///   3. Dual-Arm Support ('R' key) + Persistent Calibration Memory (PlayerPrefs).
    /// </summary>
    public class ArmAvatarController : MonoBehaviour
    {
        private const string PrefPrefix = "PimaxArmVR_v5_";

        [Header("Tracking Source")]
        public RealSenseArmReceiver receiver;

        [Header("Arm Selection & 3D Model Style (Press 'R' and 'M' or use HUD)")]
        public ActiveArmMode activeArmMode = ActiveArmMode.RightArmOnly;
        public HandVisualStyle handVisualStyle = HandVisualStyle.MicrosoftContinuousMesh;

        [Tooltip("Optional: Drag a rigged Microsoft MRTK / OpenXR Hand FBX/Prefab here if using CustomRiggedPrefab mode.")]
        public Transform customRightHandPrefab;
        public Transform customLeftHandPrefab;

        [Header("VR Shoulder Positioning (Relative to Pimax Headset)")]
        public bool lockShoulderToHeadsetYaw = true;

        [Tooltip("Right Shoulder offset from VR Headset (Eyes) in meters: X = Right (+0.18m), Y = Down (-0.15m), Z = Forward (0.00m).")]
        public Vector3 shoulderOffsetFromHead = new Vector3(0.18f, -0.15f, 0.00f);

        [Header("Arm Dimensions & Speed")]
        public float prototypeUpperArmLength = 0.28f;
        public float prototypeForearmLength = 0.25f;
        public float handSizeScale = 1.05f;

        [Range(10f, 80f)]
        public float boneRotationSmoothing = 42f;

        [Header("Persistent Calibration & HUD")]
        public bool autoLoadSavedCalibration = true;
        public bool triggerCalibrationNow = false;
        public bool showOnScreenCalibrationHUD = true;
        [Range(1.0f, 3.0f)]
        public float hudScale = 1.65f;

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
        private string _calibrationStatusMessage = "Ready! Press [C] to Calibrate, [M] to switch 3D Hand Mesh, [R] for Left/Right/Both.";
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

            // Biomechanical state for impossible-pose filtering & trigonometric IK
            public Vector3 PrevWristTarget;
            public Vector3 PrevElbowHint;
            public Vector3 SmoothedElbowPerpDir;
            public Vector3 SmoothedHandForward = Vector3.forward;
            public Vector3 SmoothedPalmNormal = Vector3.up;
            public Vector3 DraggedWristPos;
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
                // Natural anatomical elbow direction: downward (-Y) and slightly outward (+-X)
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

            if (autoLoadSavedCalibration)
            {
                LoadPersistentSettings();
            }

            ApplyVisualStyleToBothArms();
            UpdateActiveArmVisibility();
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
                        _calibrationStatusMessage = $"Synced Arm Mode from Python: {activeArmMode}";
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
            _calibrationStatusMessage = $"Switched Arm Mode to: {activeArmMode}";
        }

        public void CycleVisualStyle()
        {
            int maxModes = (customRightHandPrefab != null || customLeftHandPrefab != null) ? 4 : 3;
            handVisualStyle = (HandVisualStyle)(((int)handVisualStyle + 1) % maxModes);
            ApplyVisualStyleToBothArms();
            SavePersistentSettings();
            _calibrationStatusMessage = $"Switched 3D Model to: {handVisualStyle}";
        }

        private void UpdateActiveArmVisibility()
        {
            if (_rightArmRig?.RootGo != null)
                _rightArmRig.RootGo.SetActive(activeArmMode == ActiveArmMode.RightArmOnly || activeArmMode == ActiveArmMode.BothArms);
            if (_leftArmRig?.RootGo != null)
                _leftArmRig.RootGo.SetActive(activeArmMode == ActiveArmMode.LeftArmOnly || activeArmMode == ActiveArmMode.BothArms);
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
                Vector3 leftOffset = new Vector3(-shoulderOffsetFromHead.x, shoulderOffsetFromHead.y, shoulderOffsetFromHead.z);
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
                _calibrationStatusMessage = "⚠️ Waiting for D455C tracking data before calibrating!";
                return;
            }

            // Guard against calibrating while the user's hand is raised up holding their VR headset!
            bool useLeft = (activeArmMode == ActiveArmMode.LeftArmOnly);
            Vector3 localReach = useLeft ? receiver.LocalLeftShoulderToWrist : receiver.LocalShoulderToWrist;
            if (localReach.y > 0.08f && new Vector2(localReach.x, localReach.z).magnitude < 0.25f)
            {
                _calibrationStatusMessage = "⚠️ Hand appears raised near VR Headset! Lower hand to machine handle & press [C].";
                Debug.LogWarning("[ArmAvatarController] " + _calibrationStatusMessage);
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

            // Reset elbow swivel directions to clean anatomical downward pole on calibration
            if (_rightArmRig != null)
                _rightArmRig.SmoothedElbowPerpDir = new Vector3(0.35f, -0.92f, -0.15f).normalized;
            if (_leftArmRig != null)
                _leftArmRig.SmoothedElbowPerpDir = new Vector3(-0.35f, -0.92f, -0.15f).normalized;

            EnsureHandleMarker();
            Quaternion yawRot = Quaternion.Euler(0f, receiver.cameraYawOffsetDeg, 0f);
            Transform anchor = useLeft ? receiver.leftAvatarShoulderAnchor : receiver.avatarShoulderAnchor;
            machineHandleOrigin.position = anchor.position + yawRot * (localReach * receiver.armScale);

            SavePersistentSettings();

            _calibrationFlashTimer = 3.0f;
            _calibrationStatusMessage = $"✅ CALIBRATED & SAVED! Scale={receiver.armScale:F2}x | CamYaw={receiver.cameraYawOffsetDeg:F0}°";
            Debug.Log($"[ArmAvatarController] {_calibrationStatusMessage}");
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
            PlayerPrefs.SetFloat(PrefPrefix + "ShoulderX", shoulderOffsetFromHead.x);
            PlayerPrefs.SetFloat(PrefPrefix + "ShoulderY", shoulderOffsetFromHead.y);
            PlayerPrefs.SetFloat(PrefPrefix + "ShoulderZ", shoulderOffsetFromHead.z);
            PlayerPrefs.SetFloat(PrefPrefix + "FilterCutoff", receiver.filterMinCutoff);
            PlayerPrefs.SetFloat(PrefPrefix + "PredMs", receiver.motionPredictionMs);
            PlayerPrefs.SetFloat(PrefPrefix + "HudScale", hudScale);
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
            shoulderOffsetFromHead = new Vector3(
                PlayerPrefs.GetFloat(PrefPrefix + "ShoulderX", shoulderOffsetFromHead.x),
                PlayerPrefs.GetFloat(PrefPrefix + "ShoulderY", shoulderOffsetFromHead.y),
                PlayerPrefs.GetFloat(PrefPrefix + "ShoulderZ", shoulderOffsetFromHead.z)
            );
            receiver.filterMinCutoff = PlayerPrefs.GetFloat(PrefPrefix + "FilterCutoff", receiver.filterMinCutoff);
            receiver.motionPredictionMs = PlayerPrefs.GetFloat(PrefPrefix + "PredMs", receiver.motionPredictionMs);
            hudScale = PlayerPrefs.GetFloat(PrefPrefix + "HudScale", hudScale);
            activeArmMode = (ActiveArmMode)PlayerPrefs.GetInt(PrefPrefix + "ArmMode", (int)activeArmMode);
            handVisualStyle = (HandVisualStyle)PlayerPrefs.GetInt(PrefPrefix + "VisualStyle", (int)handVisualStyle);

            if (PlayerPrefs.GetInt(PrefPrefix + "HasHandle", 0) == 1)
            {
                EnsureHandleMarker();
                machineHandleOrigin.position = new Vector3(
                    PlayerPrefs.GetFloat(PrefPrefix + "HandleX", 0f),
                    PlayerPrefs.GetFloat(PrefPrefix + "HandleY", 1f),
                    PlayerPrefs.GetFloat(PrefPrefix + "HandleZ", 0.4f)
                );
            }

            _calibrationStatusMessage = $"Restored saved calibration (Scale={receiver.armScale:F2}x, Yaw={receiver.cameraYawOffsetDeg:F0}°)";
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
                SolveAnatomicalTrigonometricIK(rig, sPos, restElbow, restWrist, restIndex, restPinky, restWrist);
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

            // 1. Movement Gain Perturbation
            Vector3 gainPivot = machineHandleOrigin != null ? machineHandleOrigin.position : targetShoulder;
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

            // 4. Solve Cross-Product-Free Trigonometric IK with Impossible-Pose Filtering
            SolveAnatomicalTrigonometricIK(rig, targetShoulder, targetElbow, targetWrist, sample.Index, sample.Pinky, wIn);

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
        /// Cross-Product-Free Trigonometric Two-Bone IK with Biomechanical Impossible-Pose Filter:
        ///   - Fixes the "elbow protruding backwards/wrong way when holding the VR headset at startup" bug!
        ///   - Instead of rotating around a Cross-Product normal (which can invert 180 degrees when your hand
        ///     is raised to put on the VR headset), we compute the perpendicular elbow direction `elbowPerpDir`
        ///     directly and clamp it to the human anatomical lower-lateral hemisphere!
        /// </summary>
        private void SolveAnatomicalTrigonometricIK(
            ProceduralArmRig rig,
            Vector3 shoulderPos, Vector3 elbowHint, Vector3 wristTarget,
            Vector3 indexPos, Vector3 pinkyPos, Vector3 rawWristForHandRot)
        {
            float a = prototypeUpperArmLength;
            float b = prototypeForearmLength;
            float maxReach = (a + b) - 0.008f;
            float minReach = Mathf.Abs(a - b) + 0.03f;
            float sideSign = rig.IsRight ? 1f : -1f;

            Quaternion headYaw = _vrCamera != null ? Quaternion.Euler(0f, _vrCamera.transform.eulerAngles.y, 0f) : Quaternion.identity;
            Quaternion invHeadYaw = Quaternion.Inverse(headYaw);

            // --- 1. Filter Anatomically Impossible Wrist Positions (e.g. behind torso or inside neck/headset) ---
            Vector3 localWristRelShoulder = invHeadYaw * (wristTarget - shoulderPos);
            // A human seated at a machine cannot put their wrist 15cm behind their shoulder socket
            if (localWristRelShoulder.z < -0.10f)
                localWristRelShoulder.z = -0.10f;
            // Detect if the user is currently holding/adjusting their Pimax VR headset on their face!
            bool isHoldingHeadset = false;
            if (_vrCamera != null)
            {
                Vector3 wristToHead = wristTarget - _vrCamera.transform.position;
                if (wristToHead.magnitude < 0.28f && localWristRelShoulder.y > -0.05f)
                {
                    isHoldingHeadset = true;
                }
            }
            wristTarget = shoulderPos + headYaw * localWristRelShoulder;

            // --- 2. State-Informed Continuity on Wrist & Elbow ---
            if (!rig.PoseInitialized)
            {
                rig.PrevWristTarget = wristTarget;
                rig.PrevElbowHint = elbowHint;
                rig.PoseInitialized = true;
            }
            else
            {
                float maxStep = 4.5f * Mathf.Max(Time.deltaTime, 0.004f);
                Vector3 wDelta = wristTarget - rig.PrevWristTarget;
                if (wDelta.magnitude > maxStep)
                    wristTarget = rig.PrevWristTarget + wDelta.normalized * maxStep;

                float rawDist = Vector3.Distance(shoulderPos, wristTarget);
                float extensionFactor = Mathf.InverseLerp(maxReach * 0.78f, maxReach, rawDist);
                float wristBlend = Mathf.Lerp(0.82f, 0.45f, extensionFactor);
                wristTarget = Vector3.Lerp(rig.PrevWristTarget, wristTarget, wristBlend);
                elbowHint = Vector3.Lerp(rig.PrevElbowHint, elbowHint, 0.65f);

                rig.PrevWristTarget = wristTarget;
                rig.PrevElbowHint = elbowHint;
            }

            Vector3 toWrist = wristTarget - shoulderPos;
            float c = Mathf.Clamp(toWrist.magnitude, minReach, maxReach);
            Vector3 dirToWrist = toWrist.sqrMagnitude > 0.0001f ? toWrist.normalized : (headYaw * Vector3.forward);

            // Law of cosines for interior shoulder angle A (between dirToWrist and UpperArm)
            float cosAngleA = Mathf.Clamp((a * a + c * c - b * b) / (2f * a * c), -1f, 1f);
            float sinAngleA = Mathf.Sqrt(Mathf.Max(0f, 1f - cosAngleA * cosAngleA));

            // --- 3. Filter Anatomically Impossible Elbow Positions ---
            // Natural human elbow pole direction in torso space: downward (-Y), slightly outward (+-X), slightly back (-Z)
            Vector3 anatomicalPoleWorld = (headYaw * new Vector3(0.38f * sideSign, -0.90f, -0.15f)).normalized;

            Vector3 localElbowRelShoulder = invHeadYaw * (elbowHint - shoulderPos);
            // Clamp impossible upward elbow flip (elbow can never point above the shoulder when reaching forward/holding headset)
            if (localElbowRelShoulder.y > -0.02f)
                localElbowRelShoulder.y = -0.15f;
            // Clamp impossible medial chest penetration (right elbow cannot cross 12cm to the left of right shoulder)
            if (localElbowRelShoulder.x * sideSign < -0.10f)
                localElbowRelShoulder.x = -0.10f * sideSign;

            Vector3 clampedObsElbowDir = (headYaw * localElbowRelShoulder).normalized;

            // Compute component of observed elbow direction perpendicular to `dirToWrist`
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
                // If the observed perpendicular elbow direction is > 85 degrees away from the anatomical pole
                // (e.g. inverted upward or inward from camera occlusion while holding VR headset), clamp it toward polePerp!
                if (Vector3.Dot(obsPerp, polePerp) < 0.10f)
                {
                    obsPerp = Vector3.Slerp(obsPerp, polePerp, 0.75f).normalized;
                }

                float reachRatio = c / maxReach;
                float poleWeight = isHoldingHeadset
                    ? 0.80f // When adjusting VR headset on face, keep elbow gracefully tucked downward!
                    : Mathf.InverseLerp(0.76f, 0.96f, reachRatio);

                targetElbowPerp = Vector3.Slerp(obsPerp, polePerp, poleWeight).normalized;
            }

            // Smooth the perpendicular elbow direction & re-orthogonalize against dirToWrist
            rig.SmoothedElbowPerpDir = Vector3.Slerp(
                rig.SmoothedElbowPerpDir,
                targetElbowPerp,
                Mathf.Clamp01(Time.deltaTime * (boneRotationSmoothing * 0.65f))
            );
            rig.SmoothedElbowPerpDir -= Vector3.Dot(rig.SmoothedElbowPerpDir, dirToWrist) * dirToWrist;
            if (rig.SmoothedElbowPerpDir.sqrMagnitude < 0.001f)
                rig.SmoothedElbowPerpDir = polePerp;
            rig.SmoothedElbowPerpDir.Normalize();

            // --- 4. Direct Trigonometric IK Solution (Zero Cross-Product Sign Ambiguity!) ---
            Vector3 ikElbowDir = (dirToWrist * cosAngleA + rig.SmoothedElbowPerpDir * sinAngleA).normalized;
            Vector3 ikElbowPos = shoulderPos + ikElbowDir * a;
            Vector3 ikForearmDir = (wristTarget - ikElbowPos).normalized;
            Vector3 bendPlaneNormal = Vector3.Cross(ikElbowDir, ikForearmDir);
            if (bendPlaneNormal.sqrMagnitude < 0.001f)
                bendPlaneNormal = Vector3.Cross(rig.SmoothedElbowPerpDir, dirToWrist);
            bendPlaneNormal.Normalize();

            Quaternion targetUpperRot = AlignBoneAxes(ikElbowDir, bendPlaneNormal);
            Quaternion targetForeRot = AlignBoneAxes(ikForearmDir, bendPlaneNormal);

            // --- 5. Hand Orientation with Anatomical Wrist ROM Clamping ---
            Vector3 rawHandFwd = ((indexPos + pinkyPos) * 0.5f - rawWristForHandRot);
            Vector3 targetHandFwd = rawHandFwd.sqrMagnitude > 0.0004f ? rawHandFwd.normalized : ikForearmDir;
            if (Vector3.Angle(ikForearmDir, targetHandFwd) > 70f)
            {
                targetHandFwd = Vector3.RotateTowards(ikForearmDir, targetHandFwd, 70f * Mathf.Deg2Rad, 0f).normalized;
            }
            rig.SmoothedHandForward = Vector3.Slerp(rig.SmoothedHandForward, targetHandFwd, Mathf.Clamp01(Time.deltaTime * boneRotationSmoothing)).normalized;

            Vector3 acrossKnuckles = rig.IsRight ? (indexPos - pinkyPos) : (pinkyPos - indexPos);
            Vector3 rawPalmNorm = acrossKnuckles.sqrMagnitude > 0.0002f
                ? Vector3.Cross(rig.SmoothedHandForward, acrossKnuckles.normalized)
                : bendPlaneNormal;
            if (rawPalmNorm.sqrMagnitude < 0.001f) rawPalmNorm = bendPlaneNormal;
            rig.SmoothedPalmNormal = Vector3.Slerp(rig.SmoothedPalmNormal, rawPalmNorm.normalized, Mathf.Clamp01(Time.deltaTime * boneRotationSmoothing)).normalized;

            Quaternion targetHandRot = AlignBoneAxes(rig.SmoothedHandForward, rig.SmoothedPalmNormal);

            float rotAlpha = Mathf.Clamp01(Time.deltaTime * boneRotationSmoothing);

            rig.UpperArmBone.position = shoulderPos;
            rig.UpperArmBone.rotation = Quaternion.Slerp(rig.UpperArmBone.rotation, targetUpperRot, rotAlpha);

            rig.ForearmBone.position = ikElbowPos;
            rig.ForearmBone.rotation = Quaternion.Slerp(rig.ForearmBone.rotation, targetForeRot, rotAlpha);

            rig.HandBone.position = ikElbowPos + ikForearmDir * b;
            rig.HandBone.rotation = Quaternion.Slerp(rig.HandBone.rotation, targetHandRot, rotAlpha);
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
            float fingerLerp = rig.FingersInitialized ? Mathf.Clamp01(Time.deltaTime * 38f) : 1f;
            rig.FingersInitialized = true;

            for (int i = 1; i < 21; i++)
            {
                rig.SmoothedFingerWorldPos[i] = Vector3.Lerp(rig.SmoothedFingerWorldPos[i], targetPts[i], fingerLerp);
            }

            // A) Update Continuous 3D Skinned Hand & Arm Mesh (if in MicrosoftContinuousMesh or HumanSkinContinuousMesh mode)
            if (rig.ContinuousSkinnedMesh != null && rig.ContinuousSkinnedMesh.MeshObject.activeSelf)
            {
                Vector3 bendNormal = Vector3.Cross(
                    (rig.ForearmBone.position - rig.UpperArmBone.position).normalized,
                    (ikWristWorldPos - rig.ForearmBone.position).normalized
                );
                if (bendNormal.sqrMagnitude < 0.001f) bendNormal = rig.SmoothedPalmNormal;

                rig.ContinuousSkinnedMesh.UpdateMeshPose(
                    rig.UpperArmBone.position,
                    rig.ForearmBone.position,
                    ikWristWorldPos,
                    rig.SmoothedFingerWorldPos,
                    rig.SmoothedPalmNormal,
                    bendNormal.normalized
                );
            }

            // B) Update Bubble-Joint Primitives (if in BubbleJoints mode)
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

            // C) Update Custom Imported FBX Hand Rig if active
            if (handVisualStyle == HandVisualStyle.CustomRiggedPrefab && rig.CustomHandInstance != null)
            {
                DriveCustomRiggedHand(rig);
            }
        }

        private void DriveCustomRiggedHand(ProceduralArmRig rig)
        {
            rig.CustomHandInstance.position = rig.SmoothedFingerWorldPos[0];
            rig.CustomHandInstance.rotation = rig.HandBone.rotation;

            for (int b = 0; b < 20; b++)
            {
                int i0 = FingerBonePairs[b, 0];
                int i1 = FingerBonePairs[b, 1];
                Transform bone = rig.CustomMappedBones[i0];
                if (bone == null || i0 == 0) continue;

                Vector3 dir = (rig.SmoothedFingerWorldPos[i1] - rig.SmoothedFingerWorldPos[i0]);
                if (dir.sqrMagnitude > 0.00001f)
                {
                    bone.rotation = Quaternion.LookRotation(dir.normalized, rig.SmoothedPalmNormal);
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

            // Toggle Bubble Primitives vs Continuous 3D Skinned Mesh
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
                    // Sleek Microsoft MRTK / HoloLens Continuous 3D Hand Mesh finish
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
            GUIStyle labelStyle = new GUIStyle(GUI.skin.label) { fontSize = 14, richText = true };
            GUIStyle btnStyle = new GUIStyle(GUI.skin.button) { fontSize = 14, fontStyle = FontStyle.Bold };

            GUILayout.BeginArea(new Rect(12, 12, 520, 425), boxStyle);

            bool fingersOk = (activeArmMode == ActiveArmMode.LeftArmOnly) ? receiver.leftHandFingersTracked : receiver.handFingersTracked;
            string fingerBadge = fingersOk
                ? "<color=#44FF66><b>21 FINGERS LOCKED</b></color>"
                : "<color=#FFAA33><b>ARM LOCKED (Fingers Rest)</b></color>";

            GUILayout.Label($"<b>Pimax + RealSense D455C v5.0</b> | {fingerBadge}", labelStyle);
            GUILayout.Label($"Status: <b>{_calibrationStatusMessage}</b>", labelStyle);

            GUI.backgroundColor = _calibrationFlashTimer > 0f ? Color.green : new Color(0.15f, 0.85f, 1f);
            if (GUILayout.Button("🎯 CALIBRATE & AUTO-SAVE SESSION (Press C or Space)", btnStyle, GUILayout.Height(38)))
            {
                PerformOneClickCalibration();
            }

            GUILayout.BeginHorizontal();
            GUI.backgroundColor = new Color(0.95f, 0.78f, 0.25f);
            if (GUILayout.Button($"🦾 ARM: {activeArmMode} (Press R)", btnStyle, GUILayout.Height(34)))
            {
                CycleArmMode();
            }
            GUI.backgroundColor = new Color(0.55f, 0.92f, 0.65f);
            if (GUILayout.Button($"✋ MESH: {handVisualStyle} (Press M)", btnStyle, GUILayout.Height(34)))
            {
                CycleVisualStyle();
            }
            GUILayout.EndHorizontal();
            GUI.backgroundColor = Color.white;

            GUILayout.Space(4);
            GUILayout.Label($"<b>Shoulder Height (Y): {shoulderOffsetFromHead.y:F2}m</b> | <b>Width (X): {shoulderOffsetFromHead.x:F2}m</b>", labelStyle);
            GUILayout.BeginHorizontal();
            shoulderOffsetFromHead.y = GUILayout.HorizontalSlider(shoulderOffsetFromHead.y, -0.35f, 0.05f);
            shoulderOffsetFromHead.x = GUILayout.HorizontalSlider(shoulderOffsetFromHead.x, 0.05f, 0.35f);
            GUILayout.EndHorizontal();

            GUILayout.Label($"<b>Camera Horizontal Angle (Yaw): {receiver.cameraYawOffsetDeg:F0}°</b>", labelStyle);
            receiver.cameraYawOffsetDeg = GUILayout.HorizontalSlider(receiver.cameraYawOffsetDeg, -180f, 180f);

            GUILayout.Label($"<b>Speed Prediction: {receiver.motionPredictionMs:F0} ms</b> | <b>Filter Cutoff: {receiver.filterMinCutoff:F1} Hz</b>", labelStyle);
            GUILayout.BeginHorizontal();
            receiver.motionPredictionMs = GUILayout.HorizontalSlider(receiver.motionPredictionMs, 0f, 30f);
            receiver.filterMinCutoff = GUILayout.HorizontalSlider(receiver.filterMinCutoff, 0.8f, 5.0f);
            GUILayout.EndHorizontal();

            GUILayout.Label($"<b>Psychophysics Gain: {movementGain:F2}x</b> | <b>HUD Scale: {hudScale:F2}x</b>", labelStyle);
            GUILayout.BeginHorizontal();
            movementGain = GUILayout.HorizontalSlider(movementGain, 0.3f, 2.2f);
            hudScale = GUILayout.HorizontalSlider(hudScale, 1.0f, 2.5f);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            if (GUILayout.Button("💾 Save Settings Now", GUILayout.Height(26)))
            {
                SavePersistentSettings();
                _calibrationStatusMessage = "✅ Saved all current sliders to persistent memory!";
            }
            if (GUILayout.Button("🔄 Reset Defaults", GUILayout.Height(26)))
            {
                PlayerPrefs.DeleteAll();
                shoulderOffsetFromHead = new Vector3(0.18f, -0.15f, 0.00f);
                receiver.armScale = 1.0f;
                receiver.cameraYawOffsetDeg = 0f;
                movementGain = 1.0f;
                _calibrationStatusMessage = "Reset settings to factory defaults.";
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

            // 1. Build Continuous Skinned 3D Mesh (Microsoft MRTK / Smooth Human Skin Mesh)
            rig.ContinuousSkinnedMesh = new ContinuousHandArmMesh(rig.RootGo.transform, $"{prefix}_Continuous_3D_HandArm_Mesh", isRight);

            // 2. Build Classic Bubble-Joint Primitives Group
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
