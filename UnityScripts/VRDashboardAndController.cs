using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
using UnityEngine.InputSystem.Controls;
using UnityEngine.InputSystem.XR;
#endif

namespace PsychophysicsVR.Tracking
{
    /// <summary>
    /// v6.1 In-Headset 3D World-Space VR Dashboard with:
    ///   1. Hands-Free 3-Second Gaze-Dwell Auto-Click (hold your gaze/cursor on any button for 3s to click!).
    ///   2. Instant Click via Controller Trigger, Mouse Left-Click, or Enter Key.
    ///   3. Dual-API Pimax OpenXR Controller Tracking (UnityEngine.XR.InputDevices + InputSystem XRController)
    ///      plus visible 3D controller wands and 6-DOF IMU + Position calibration (Grip + Trigger).
    /// </summary>
    public class VRDashboardAndController : MonoBehaviour
    {
        [Header("References")]
        public ArmAvatarController controller;
        public RealSenseArmReceiver receiver;

        [Header("In-Headset 3D VR Dashboard Settings")]
        public bool showVRDashboard = true;
        public float dashboardDistance = 0.72f;
        public float dashboardSideAngleDeg = -22f;
        public float dashboardHeightOffset = -0.04f;

        [Header("Hands-Free Gaze-Dwell Click")]
        [Tooltip("How many seconds to hold your gaze on a VR button to automatically click it.")]
        [Range(1.0f, 5.0f)]
        public float gazeDwellSeconds = 3.0f;
        public bool enableGazeDwellClick = true;

        private Camera _vrCam;
        private GameObject _boardRoot;
        private TextMesh _headerText;
        private TextMesh _statusText;
        private LineRenderer _laserLine;
        private Transform _cursorDot;
        private Transform _rightCtrlMarker;
        private Transform _leftCtrlMarker;

        private class VRButtonItem
        {
            public Collider HitCollider;
            public Renderer BgRenderer;
            public TextMesh Label;
            public Color NormalColor;
            public Color HoverColor;
            public Action OnClick;
            public Func<string> DynamicLabel;
        }
        private readonly List<VRButtonItem> _buttons = new List<VRButtonItem>();
        private VRButtonItem _hoveredButton;

        // Gaze-dwell state
        private float _dwellTimer;
        private float _dwellCooldownTimer;
        private VRButtonItem _flashButton;
        private float _flashTimer;
        private bool _anyControllerTracked;

        // Edge-detection state for XR controller buttons
        private bool _prevRPrimary, _prevLPrimary;
        private bool _prevRSecondary, _prevLSecondary;
        private bool _prevRGrip, _prevLGrip;
        private bool _prevRTrigger, _prevLTrigger;

        private struct ControllerState
        {
            public bool Tracked;
            public Vector3 WorldPos;
            public Quaternion WorldRot;
            public bool Primary;
            public bool Secondary;
            public bool Grip;
            public bool Trigger;
            public Vector2 Stick;
        }

        private void Start()
        {
            if (controller == null) controller = GetComponent<ArmAvatarController>();
            if (receiver == null) receiver = GetComponent<RealSenseArmReceiver>();
            _vrCam = Camera.main != null ? Camera.main : UnityEngine.Object.FindFirstObjectByType<Camera>();

            BuildWorldSpaceVRDashboard();
            PositionDashboardInFrontOfUser(immediate: true);
            _boardRoot.SetActive(showVRDashboard);
        }

        private void Update()
        {
            if (_vrCam == null)
                _vrCam = Camera.main != null ? Camera.main : UnityEngine.Object.FindFirstObjectByType<Camera>();

            if (_dwellCooldownTimer > 0f)
                _dwellCooldownTimer -= Time.unscaledDeltaTime;

            if (_flashTimer > 0f)
            {
                _flashTimer -= Time.unscaledDeltaTime;
                if (_flashTimer <= 0f && _flashButton != null && _flashButton.BgRenderer != null)
                {
                    Color restore = (_flashButton == _hoveredButton) ? _flashButton.HoverColor : _flashButton.NormalColor;
                    SetMaterialColor(_flashButton.BgRenderer.gameObject, restore);
                    _flashButton = null;
                }
            }

            // Keyboard Tab toggles VR Dashboard on/off & recenters it in front of user
            if (WasTabPressedThisFrame())
            {
                ToggleVRDashboard();
            }

            PollControllersAndUpdatePointer();
            RefreshDynamicButtonLabels();
        }

        public void ToggleVRDashboard()
        {
            showVRDashboard = !showVRDashboard;
            _dwellTimer = 0f;
            if (_boardRoot != null)
            {
                _boardRoot.SetActive(showVRDashboard);
                if (showVRDashboard)
                {
                    PositionDashboardInFrontOfUser(immediate: true);
                }
            }
        }

        private void PositionDashboardInFrontOfUser(bool immediate)
        {
            if (_vrCam == null || _boardRoot == null) return;

            float headYaw = _vrCam.transform.eulerAngles.y + dashboardSideAngleDeg;
            Quaternion yawRot = Quaternion.Euler(0f, headYaw, 0f);
            Vector3 targetPos = _vrCam.transform.position + yawRot * new Vector3(0f, dashboardHeightOffset, dashboardDistance);
            Quaternion targetRot = Quaternion.LookRotation((targetPos - _vrCam.transform.position).normalized, Vector3.up);

            if (immediate)
            {
                _boardRoot.transform.position = targetPos;
                _boardRoot.transform.rotation = targetRot;
            }
            else
            {
                _boardRoot.transform.position = Vector3.Lerp(_boardRoot.transform.position, targetPos, Time.unscaledDeltaTime * 6f);
                _boardRoot.transform.rotation = Quaternion.Slerp(_boardRoot.transform.rotation, targetRot, Time.unscaledDeltaTime * 6f);
            }
        }

        private void PollControllersAndUpdatePointer()
        {
            ControllerState right = ReadControllerState(isLeft: false);
            ControllerState left = ReadControllerState(isLeft: true);

            _anyControllerTracked = right.Tracked || left.Tracked;

            // Show sleek 3D controller wand markers whenever tracked!
            UpdateControllerMarker(_rightCtrlMarker, right.Tracked, right.WorldPos, right.WorldRot);
            UpdateControllerMarker(_leftCtrlMarker, left.Tracked, left.WorldPos, left.WorldRot);

            // 1. Primary Button (A / X) -> Toggle & Recenter In-Headset 3D VR Menu
            if ((right.Primary && !_prevRPrimary) || (left.Primary && !_prevLPrimary))
            {
                ToggleVRDashboard();
            }

            // 2. Secondary Button (B / Y) -> Cycle 3D Hand Mesh Style
            if ((right.Secondary && !_prevRSecondary) || (left.Secondary && !_prevLSecondary))
            {
                controller?.CycleVisualStyle();
            }

            // 3. Grip + Trigger -> Golden-Reference 6-DOF Calibration using Controller Pose + IMU Orientation!
            bool rTrigJustPressed = right.Trigger && !_prevRTrigger;
            bool lTrigJustPressed = left.Trigger && !_prevLTrigger;

            if (right.Grip && rTrigJustPressed && right.Tracked && controller != null)
            {
                controller.CalibrateToVRController(right.WorldPos, right.WorldRot, isLeftController: false);
            }
            else if (left.Grip && lTrigJustPressed && left.Tracked && controller != null)
            {
                controller.CalibrateToVRController(left.WorldPos, left.WorldRot, isLeftController: true);
            }
            else
            {
                if ((right.Grip && !_prevRGrip && !right.Trigger) || (left.Grip && !_prevLGrip && !left.Trigger))
                {
                    controller?.CycleArmMode();
                }
            }

            // 4. Thumbstick Continuous Tuning
            Vector2 stick = right.Stick.sqrMagnitude > left.Stick.sqrMagnitude ? right.Stick : left.Stick;
            if (controller != null && stick.sqrMagnitude > 0.06f)
            {
                float dt = Time.unscaledDeltaTime;
                if (Mathf.Abs(stick.y) > 0.25f)
                {
                    controller.shoulderOffsetFromHead.y = Mathf.Clamp(
                        controller.shoulderOffsetFromHead.y + stick.y * 0.20f * dt, -0.35f, 0.05f);
                }
                if (Mathf.Abs(stick.x) > 0.25f)
                {
                    if ((right.Grip || left.Grip) && receiver != null)
                    {
                        receiver.cameraYawOffsetDeg = Mathf.Clamp(
                            receiver.cameraYawOffsetDeg + stick.x * 45f * dt, -180f, 180f);
                    }
                    else
                    {
                        controller.movementGain = Mathf.Clamp(
                            controller.movementGain + stick.x * 0.60f * dt, 0.2f, 2.5f);
                    }
                }
            }

            // 5. Laser Pointer & 3-Second Gaze-Dwell Raycast against 3D VR Menu Buttons
            bool manualClick = WasManualClickPressedThisFrame();
            UpdateLaserPointerAndClicks(
                right.Tracked, right.WorldPos, right.WorldRot, rTrigJustPressed && !right.Grip,
                left.Tracked, left.WorldPos, left.WorldRot, lTrigJustPressed && !left.Grip,
                manualClick
            );

            _prevRPrimary = right.Primary;
            _prevLPrimary = left.Primary;
            _prevRSecondary = right.Secondary;
            _prevLSecondary = left.Secondary;
            _prevRGrip = right.Grip;
            _prevLGrip = left.Grip;
            _prevRTrigger = right.Trigger;
            _prevLTrigger = left.Trigger;
        }

        private void UpdateLaserPointerAndClicks(
            bool rTracked, Vector3 rPos, Quaternion rRot, bool rClick,
            bool lTracked, Vector3 lPos, Quaternion lRot, bool lClick,
            bool manualDesktopClick)
        {
            if (!showVRDashboard || _laserLine == null)
            {
                if (_laserLine != null) _laserLine.enabled = false;
                if (_cursorDot != null) _cursorDot.gameObject.SetActive(false);
                _dwellTimer = 0f;
                return;
            }

            Vector3 rayOrigin;
            Vector3 rayDir;
            bool clickRequested = manualDesktopClick;

            if (rTracked)
            {
                rayOrigin = rPos;
                rayDir = rRot * Vector3.forward;
                clickRequested |= rClick;
            }
            else if (lTracked)
            {
                rayOrigin = lPos;
                rayDir = lRot * Vector3.forward;
                clickRequested |= lClick;
            }
            else if (_vrCam != null)
            {
                // Head-gaze ray from Pimax HMD eyes center
                rayOrigin = _vrCam.transform.position;
                rayDir = _vrCam.transform.forward;
            }
            else
            {
                _laserLine.enabled = false;
                return;
            }

            Ray ray = new Ray(rayOrigin, rayDir);
            VRButtonItem hitBtn = null;
            Vector3 endPt = rayOrigin + rayDir * 1.4f;

            if (Physics.Raycast(ray, out RaycastHit hit, 3.0f))
            {
                endPt = hit.point;
                for (int i = 0; i < _buttons.Count; i++)
                {
                    if (_buttons[i].HitCollider == hit.collider)
                    {
                        hitBtn = _buttons[i];
                        break;
                    }
                }
            }

            // Reset dwell timer when gaze/pointer moves to a different button
            if (_hoveredButton != hitBtn)
            {
                if (_hoveredButton != null && _hoveredButton.BgRenderer != null && _hoveredButton != _flashButton)
                    SetMaterialColor(_hoveredButton.BgRenderer.gameObject, _hoveredButton.NormalColor);

                _hoveredButton = hitBtn;
                _dwellTimer = 0f;

                if (_hoveredButton != null && _hoveredButton.BgRenderer != null)
                    SetMaterialColor(_hoveredButton.BgRenderer.gameObject, _hoveredButton.HoverColor);
            }

            // Handle 3-Second Gaze-Dwell Charging & Visual Feedback
            float dwellProgress = 0f;
            if (_hoveredButton != null && _hoveredButton.OnClick != null)
            {
                if (enableGazeDwellClick && _dwellCooldownTimer <= 0f)
                {
                    _dwellTimer += Time.unscaledDeltaTime;
                    dwellProgress = Mathf.Clamp01(_dwellTimer / Mathf.Max(0.5f, gazeDwellSeconds));

                    if (_hoveredButton.BgRenderer != null && _hoveredButton != _flashButton)
                    {
                        Color chargingCol = Color.Lerp(_hoveredButton.HoverColor, new Color(0.15f, 0.90f, 0.35f), dwellProgress);
                        SetMaterialColor(_hoveredButton.BgRenderer.gameObject, chargingCol);
                    }

                    if (_dwellTimer >= gazeDwellSeconds)
                    {
                        clickRequested = true;
                        _dwellTimer = 0f;
                        _dwellCooldownTimer = 1.0f;
                    }
                }
            }
            else
            {
                _dwellTimer = 0f;
            }

            if (clickRequested && _hoveredButton != null && _hoveredButton.OnClick != null)
            {
                _hoveredButton.OnClick.Invoke();
                _flashButton = _hoveredButton;
                _flashTimer = 0.35f;
                if (_flashButton.BgRenderer != null)
                    SetMaterialColor(_flashButton.BgRenderer.gameObject, new Color(0.2f, 1.0f, 0.4f));
            }

            // Laser only visible when a VR controller is tracked (keeps head-gaze clean with just the floating reticle dot)
            bool showRay = rTracked || lTracked;
            _laserLine.enabled = showRay;
            if (showRay)
            {
                _laserLine.SetPosition(0, rayOrigin);
                _laserLine.SetPosition(1, endPt);
            }

            if (_cursorDot != null)
            {
                bool showDot = (hitBtn != null) || rTracked || lTracked || _vrCam != null;
                _cursorDot.gameObject.SetActive(showDot);
                // Place dot slightly toward the camera/user so it sits cleanly on the surface of the 3D button
                Vector3 toUser = _vrCam != null ? (_vrCam.transform.position - endPt).normalized : -rayDir;
                _cursorDot.position = (hitBtn != null) ? (endPt + toUser * 0.006f) : (rayOrigin + rayDir * 0.68f);

                float dotScale = Mathf.Lerp(0.011f, 0.024f, dwellProgress);
                _cursorDot.localScale = Vector3.one * dotScale;
                SetMaterialColor(_cursorDot.gameObject, Color.Lerp(Color.yellow, new Color(0.2f, 1.0f, 0.35f), dwellProgress));
            }
        }

        private ControllerState ReadControllerState(bool isLeft)
        {
            ControllerState state = default;
            XRNode node = isLeft ? XRNode.LeftHand : XRNode.RightHand;
            UnityEngine.XR.InputDevice dev = InputDevices.GetDeviceAtXRNode(node);

            if (dev.isValid)
            {
                bool hasTrackedFlag = dev.TryGetFeatureValue(UnityEngine.XR.CommonUsages.isTracked, out bool isTracked);
                bool hasPos = dev.TryGetFeatureValue(UnityEngine.XR.CommonUsages.devicePosition, out Vector3 localPos);
                bool hasRot = dev.TryGetFeatureValue(UnityEngine.XR.CommonUsages.deviceRotation, out Quaternion localRot);

                if (hasPos && (!hasTrackedFlag || isTracked || localPos.sqrMagnitude > 0.0001f))
                {
                    state.Tracked = true;
                    TransformLocalPoseToWorld(localPos, hasRot ? localRot : Quaternion.identity, out state.WorldPos, out state.WorldRot);
                    state.Primary = GetBoolUsage(dev, UnityEngine.XR.CommonUsages.primaryButton);
                    state.Secondary = GetBoolUsage(dev, UnityEngine.XR.CommonUsages.secondaryButton);
                    state.Grip = GetBoolUsage(dev, UnityEngine.XR.CommonUsages.gripButton);
                    state.Trigger = GetBoolUsage(dev, UnityEngine.XR.CommonUsages.triggerButton);
                    state.Stick = GetVec2Usage(dev, UnityEngine.XR.CommonUsages.primary2DAxis);
                    return state;
                }
            }

#if ENABLE_INPUT_SYSTEM
            XRController xrCtrl = isLeft ? XRController.leftHand : XRController.rightHand;
            if (xrCtrl != null && xrCtrl.added)
            {
                bool trackedCtrl = xrCtrl.isTracked != null && xrCtrl.isTracked.isPressed;
                Vector3 localPos = xrCtrl.devicePosition != null ? xrCtrl.devicePosition.ReadValue() : Vector3.zero;
                Quaternion localRot = xrCtrl.deviceRotation != null ? xrCtrl.deviceRotation.ReadValue() : Quaternion.identity;

                if (trackedCtrl || localPos.sqrMagnitude > 0.0001f)
                {
                    state.Tracked = true;
                    TransformLocalPoseToWorld(localPos, localRot, out state.WorldPos, out state.WorldRot);
                    state.Primary = ReadInputSystemButton(xrCtrl, "primaryButton");
                    state.Secondary = ReadInputSystemButton(xrCtrl, "secondaryButton");
                    state.Grip = ReadInputSystemButton(xrCtrl, "gripPressed");
                    state.Trigger = ReadInputSystemButton(xrCtrl, "triggerPressed");

                    StickControl stickCtrl = xrCtrl.TryGetChildControl<StickControl>("thumbstick");
                    if (stickCtrl != null) state.Stick = stickCtrl.ReadValue();
                }
            }
#endif
            return state;
        }

        private void TransformLocalPoseToWorld(Vector3 localPos, Quaternion localRot, out Vector3 worldPos, out Quaternion worldRot)
        {
            Transform xrOrigin = (_vrCam != null && _vrCam.transform.parent != null) ? _vrCam.transform.parent : null;
            if (xrOrigin != null)
            {
                worldPos = xrOrigin.TransformPoint(localPos);
                worldRot = xrOrigin.rotation * localRot;
            }
            else
            {
                worldPos = localPos;
                worldRot = localRot;
            }
        }

#if ENABLE_INPUT_SYSTEM
        private static bool ReadInputSystemButton(XRController ctrl, string controlName)
        {
            ButtonControl btn = ctrl.TryGetChildControl<ButtonControl>(controlName);
            return btn != null && btn.isPressed;
        }
#endif

        private static bool GetBoolUsage(UnityEngine.XR.InputDevice dev, InputFeatureUsage<bool> usage)
        {
            return dev.isValid && dev.TryGetFeatureValue(usage, out bool val) && val;
        }

        private static Vector2 GetVec2Usage(UnityEngine.XR.InputDevice dev, InputFeatureUsage<Vector2> usage)
        {
            return dev.isValid && dev.TryGetFeatureValue(usage, out Vector2 val) ? val : Vector2.zero;
        }

        private static bool WasTabPressedThisFrame()
        {
#if ENABLE_INPUT_SYSTEM
            if (Keyboard.current != null && Keyboard.current.tabKey.wasPressedThisFrame)
                return true;
#endif
#if ENABLE_LEGACY_INPUT_MANAGER
            try { if (Input.GetKeyDown(KeyCode.Tab)) return true; } catch { }
#endif
            return false;
        }

        private static bool WasManualClickPressedThisFrame()
        {
#if ENABLE_INPUT_SYSTEM
            if (Keyboard.current != null && (Keyboard.current.enterKey.wasPressedThisFrame || Keyboard.current.numpadEnterKey.wasPressedThisFrame))
                return true;
            if (Mouse.current != null && Mouse.current.leftButton.wasPressedThisFrame)
                return true;
#endif
#if ENABLE_LEGACY_INPUT_MANAGER
            try
            {
                if (Input.GetKeyDown(KeyCode.Return) || Input.GetKeyDown(KeyCode.KeypadEnter))
                    return true;
            }
            catch { }
#endif
            return false;
        }

        private void UpdateControllerMarker(Transform marker, bool visible, Vector3 pos, Quaternion rot)
        {
            if (marker == null) return;
            marker.gameObject.SetActive(visible);
            if (visible)
            {
                marker.position = pos;
                marker.rotation = rot;
            }
        }

        private void RefreshDynamicButtonLabels()
        {
            if (!showVRDashboard || _boardRoot == null) return;

            if (_headerText != null && controller != null && receiver != null)
            {
                string fStr = receiver.handFingersTracked ? "21-Fingers: ON" : "Arm Tracking";
                string ctrlStr = _anyControllerTracked ? "VR Ctrl: ON" : "Gaze Mode (3s Hold)";
                _headerText.text = $"PIMAX + D455C v6.1  |  {fStr}  |  {ctrlStr}";
            }
            if (_statusText != null && controller != null)
            {
                string hint = _anyControllerTracked
                    ? "Controller: [A/X] Menu | [B/Y] Mesh | [Grip] Arm | [Grip+Trig] 6-DOF Calibrate"
                    : "Tip: Add 'Oculus Touch Profile' in OpenXR Settings for Controllers — OR Hold Gaze 3s to Click!";
                _statusText.text = $"{controller.CalibrationStatusMessage}\n{hint}";
            }

            for (int i = 0; i < _buttons.Count; i++)
            {
                VRButtonItem btn = _buttons[i];
                if (btn.DynamicLabel == null || btn.Label == null) continue;

                string baseText = btn.DynamicLabel();
                if (btn == _hoveredButton && btn.OnClick != null && enableGazeDwellClick && _dwellCooldownTimer <= 0f && _dwellTimer > 0.05f)
                {
                    float remaining = Mathf.Max(0f, gazeDwellSeconds - _dwellTimer);
                    int filled = Mathf.Clamp(Mathf.RoundToInt((_dwellTimer / gazeDwellSeconds) * 5f), 0, 5);
                    string bar = new string('●', filled) + new string('○', 5 - filled);
                    btn.Label.text = $"{baseText}\n[{bar} {remaining:F1}s]";
                }
                else
                {
                    btn.Label.text = baseText;
                }
            }
        }

        private void BuildWorldSpaceVRDashboard()
        {
            _boardRoot = new GameObject("InHeadset_3D_VR_Dashboard");

            GameObject backdrop = GameObject.CreatePrimitive(PrimitiveType.Cube);
            backdrop.name = "DashboardBackdrop";
            Destroy(backdrop.GetComponent<Collider>());
            backdrop.transform.SetParent(_boardRoot.transform, false);
            backdrop.transform.localScale = new Vector3(0.58f, 0.50f, 0.008f);
            SetMaterialColor(backdrop, new Color(0.08f, 0.11f, 0.16f));

            _headerText = CreateWorldText(_boardRoot.transform, "HeaderText", new Vector3(0f, 0.22f, -0.008f), 40, Color.cyan);
            _statusText = CreateWorldText(_boardRoot.transform, "StatusText", new Vector3(0f, 0.172f, -0.008f), 28, new Color(0.85f, 0.92f, 1.0f));

            CreateVRButton(new Vector3(-0.14f, 0.110f, -0.010f), new Vector2(0.26f, 0.048f),
                new Color(0.12f, 0.48f, 0.72f), () => "Calibrate Camera (C)",
                () => controller?.PerformOneClickCalibration());

            CreateVRButton(new Vector3(0.14f, 0.110f, -0.010f), new Vector2(0.26f, 0.048f),
                new Color(0.16f, 0.58f, 0.42f), () => "Match Controller (Grip+Trig)",
                () => TryCalibrateWithActiveController());

            CreateVRButton(new Vector3(-0.14f, 0.048f, -0.010f), new Vector2(0.26f, 0.046f),
                new Color(0.22f, 0.44f, 0.56f),
                () => $"Auto-Align: {(receiver != null && receiver.continuousAutoCalibration ? "ON" : "OFF")}",
                () => controller?.ToggleContinuousAutoCalibration());

            CreateVRButton(new Vector3(0.14f, 0.048f, -0.010f), new Vector2(0.26f, 0.046f),
                new Color(0.42f, 0.32f, 0.62f),
                () => $"Task: {(controller != null ? controller.experimentPreset.ToString() : "General")}",
                () => controller?.CycleExperimentPreset());

            CreateVRButton(new Vector3(-0.14f, -0.014f, -0.010f), new Vector2(0.26f, 0.046f),
                new Color(0.62f, 0.48f, 0.14f),
                () => $"Arm: {(controller != null ? controller.activeArmMode.ToString() : "Right")}",
                () => controller?.CycleArmMode());

            CreateVRButton(new Vector3(0.14f, -0.014f, -0.010f), new Vector2(0.26f, 0.046f),
                new Color(0.20f, 0.54f, 0.36f),
                () => $"Mesh: {(controller != null ? controller.handVisualStyle.ToString() : "Microsoft")}",
                () => controller?.CycleVisualStyle());

            CreateVRButton(new Vector3(-0.20f, -0.076f, -0.010f), new Vector2(0.12f, 0.042f),
                new Color(0.25f, 0.30f, 0.40f), () => "PD -",
                () => { if (controller != null) controller.pdNaturalFrequencyHz = Mathf.Max(1.5f, controller.pdNaturalFrequencyHz - 0.5f); });

            CreateVRButton(new Vector3(0.0f, -0.076f, -0.010f), new Vector2(0.24f, 0.042f),
                new Color(0.16f, 0.20f, 0.28f),
                () => $"PD Smooth: {(controller != null ? controller.pdNaturalFrequencyHz : 4.8f):F1} Hz", null);

            CreateVRButton(new Vector3(0.20f, -0.076f, -0.010f), new Vector2(0.12f, 0.042f),
                new Color(0.25f, 0.30f, 0.40f), () => "PD +",
                () => { if (controller != null) controller.pdNaturalFrequencyHz = Mathf.Min(12.0f, controller.pdNaturalFrequencyHz + 0.5f); });

            CreateVRButton(new Vector3(-0.20f, -0.132f, -0.010f), new Vector2(0.12f, 0.042f),
                new Color(0.25f, 0.30f, 0.40f), () => "Gain -",
                () => { if (controller != null) controller.movementGain = Mathf.Max(0.2f, controller.movementGain - 0.1f); });

            CreateVRButton(new Vector3(0.0f, -0.132f, -0.010f), new Vector2(0.24f, 0.042f),
                new Color(0.16f, 0.20f, 0.28f),
                () => $"Force Gain: {(controller != null ? controller.movementGain : 1.0f):F2}x", null);

            CreateVRButton(new Vector3(0.20f, -0.132f, -0.010f), new Vector2(0.12f, 0.042f),
                new Color(0.25f, 0.30f, 0.40f), () => "Gain +",
                () => { if (controller != null) controller.movementGain = Mathf.Min(2.5f, controller.movementGain + 0.1f); });

            CreateVRButton(new Vector3(-0.20f, -0.188f, -0.010f), new Vector2(0.12f, 0.042f),
                new Color(0.25f, 0.30f, 0.40f), () => "Sh.Y -",
                () => { if (controller != null) controller.shoulderOffsetFromHead.y = Mathf.Max(-0.35f, controller.shoulderOffsetFromHead.y - 0.02f); });

            CreateVRButton(new Vector3(0.0f, -0.188f, -0.010f), new Vector2(0.24f, 0.042f),
                new Color(0.18f, 0.44f, 0.28f),
                () => "Save Session",
                () => controller?.SavePersistentSettings());

            CreateVRButton(new Vector3(0.20f, -0.188f, -0.010f), new Vector2(0.12f, 0.042f),
                new Color(0.25f, 0.30f, 0.40f), () => "Sh.Y +",
                () => { if (controller != null) controller.shoulderOffsetFromHead.y = Mathf.Min(0.05f, controller.shoulderOffsetFromHead.y + 0.02f); });

            CreateVRButton(new Vector3(0.0f, -0.232f, -0.010f), new Vector2(0.38f, 0.036f),
                new Color(0.55f, 0.22f, 0.22f), () => "Hide VR Menu (A/X, Tab, or 3s Gaze)",
                () => ToggleVRDashboard());

            // Laser Pointer LineRenderer + Gaze/Cursor Reticle Dot
            GameObject laserGo = new GameObject("VRLaserPointer");
            laserGo.transform.SetParent(this.transform, false);
            _laserLine = laserGo.AddComponent<LineRenderer>();
            _laserLine.positionCount = 2;
            _laserLine.startWidth = 0.003f;
            _laserLine.endWidth = 0.0015f;
            Shader unlit = Shader.Find("Unlit/Color") ?? Shader.Find("Sprites/Default");
            Material laserMat = new Material(unlit) { color = Color.cyan };
            _laserLine.sharedMaterial = laserMat;
            _laserLine.enabled = false;

            GameObject dotGo = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            dotGo.name = "VRLaserCursorDot";
            Destroy(dotGo.GetComponent<Collider>());
            dotGo.transform.SetParent(this.transform, false);
            dotGo.transform.localScale = Vector3.one * 0.012f;
            SetMaterialColor(dotGo, Color.yellow);
            dotGo.SetActive(false);
            _cursorDot = dotGo.transform;

            _rightCtrlMarker = CreateControllerWand("RightPimaxControllerVisual", new Color(0.2f, 0.9f, 1.0f));
            _leftCtrlMarker = CreateControllerWand("LeftPimaxControllerVisual", new Color(1.0f, 0.7f, 0.2f));
        }

        private void TryCalibrateWithActiveController()
        {
            if (controller == null) return;
            ControllerState right = ReadControllerState(isLeft: false);
            if (right.Tracked)
            {
                controller.CalibrateToVRController(right.WorldPos, right.WorldRot, false);
                return;
            }
            ControllerState left = ReadControllerState(isLeft: true);
            if (left.Tracked)
            {
                controller.CalibrateToVRController(left.WorldPos, left.WorldRot, true);
                return;
            }
            controller.PerformOneClickCalibration();
        }

        private Transform CreateControllerWand(string name, Color col)
        {
            GameObject root = new GameObject(name);
            root.transform.SetParent(this.transform, false);

            GameObject grip = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            grip.name = "Handle";
            Destroy(grip.GetComponent<Collider>());
            grip.transform.SetParent(root.transform, false);
            grip.transform.localPosition = new Vector3(0f, -0.03f, -0.02f);
            grip.transform.localRotation = Quaternion.Euler(35f, 0f, 0f);
            grip.transform.localScale = new Vector3(0.032f, 0.055f, 0.032f);
            SetMaterialColor(grip, new Color(0.18f, 0.20f, 0.25f));

            GameObject ring = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            ring.name = "TipEmitter";
            Destroy(ring.GetComponent<Collider>());
            ring.transform.SetParent(root.transform, false);
            ring.transform.localPosition = new Vector3(0f, 0.01f, 0.04f);
            ring.transform.localScale = new Vector3(0.045f, 0.025f, 0.045f);
            SetMaterialColor(ring, col);

            root.SetActive(false);
            return root.transform;
        }

        private void CreateVRButton(Vector3 localPos, Vector2 size, Color color, Func<string> labelFunc, Action onClick)
        {
            GameObject btnGo = GameObject.CreatePrimitive(PrimitiveType.Cube);
            btnGo.name = "VRBtn_" + labelFunc();
            btnGo.transform.SetParent(_boardRoot.transform, false);
            btnGo.transform.localPosition = localPos;
            btnGo.transform.localScale = new Vector3(size.x, size.y, 0.008f);
            SetMaterialColor(btnGo, color);

            TextMesh txt = CreateWorldText(btnGo.transform, "Label", new Vector3(0f, 0f, -0.6f), 32, Color.white);
            txt.transform.localScale = new Vector3(1f / size.x, 1f / size.y, 1f / 0.008f);
            txt.text = labelFunc();

            _buttons.Add(new VRButtonItem
            {
                HitCollider = btnGo.GetComponent<Collider>(),
                BgRenderer = btnGo.GetComponent<Renderer>(),
                Label = txt,
                NormalColor = color,
                HoverColor = Color.Lerp(color, Color.white, 0.38f),
                OnClick = onClick,
                DynamicLabel = labelFunc
            });
        }

        private static TextMesh CreateWorldText(Transform parent, string name, Vector3 localPos, int fontSize, Color color)
        {
            GameObject tGo = new GameObject(name);
            tGo.transform.SetParent(parent, false);
            tGo.transform.localPosition = localPos;
            TextMesh tm = tGo.AddComponent<TextMesh>();
            tm.fontSize = fontSize;
            tm.characterSize = 0.0040f;
            tm.anchor = TextAnchor.MiddleCenter;
            tm.alignment = TextAlignment.Center;
            tm.color = color;
            return tm;
        }

        private static void SetMaterialColor(GameObject go, Color color)
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
