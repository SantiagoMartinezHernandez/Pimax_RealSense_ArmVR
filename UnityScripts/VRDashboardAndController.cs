using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

namespace PsychophysicsVR.Tracking
{
    /// <summary>
    /// v6.0 In-Headset 3D World-Space VR Dashboard + Pimax OpenXR Controller Integration.
    /// Features:
    ///   - Renders a crisp 3D interactive menu directly inside the Pimax Crystal VR headset
    ///     (zero UI/TMP package dependencies — works in every Unity template).
    ///   - Polls Right & Left Pimax OpenXR controllers via UnityEngine.XR.InputDevices:
    ///       * A / X (Primary Button)   : Show / Hide & Recenter 3D VR Dashboard (also Tab key)
    ///       * B / Y (Secondary Button) : Cycle 3D Hand Mesh (Microsoft / Skin / Bubble)
    ///       * Grip Button              : Cycle Arm Mode (Right / Left / Both)
    ///       * Trigger Button           : Click pointed 3D VR Menu button
    ///       * Grip + Trigger           : Match VR Controller 3D Position (Golden-Reference Calibration)
    ///       * Thumbstick Up/Down       : Adjust Shoulder Height (Y)
    ///       * Thumbstick Left/Right    : Adjust Force/Movement Gain (or CamYaw while holding Grip)
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

        // Edge-detection state for XR controller buttons
        private bool _prevRPrimary, _prevLPrimary;
        private bool _prevRSecondary, _prevLSecondary;
        private bool _prevRGrip, _prevLGrip;
        private bool _prevRTrigger, _prevLTrigger;
        private float _recenterSuppressTimer;

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

            if (_recenterSuppressTimer > 0f)
                _recenterSuppressTimer -= Time.unscaledDeltaTime;

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
            UnityEngine.XR.InputDevice rightDev = InputDevices.GetDeviceAtXRNode(XRNode.RightHand);
            UnityEngine.XR.InputDevice leftDev = InputDevices.GetDeviceAtXRNode(XRNode.LeftHand);

            bool rTracked = ReadControllerPose(rightDev, out Vector3 rPos, out Quaternion rRot);
            bool lTracked = ReadControllerPose(leftDev, out Vector3 lPos, out Quaternion lRot);

            UpdateControllerMarker(_rightCtrlMarker, rTracked && showVRDashboard, rPos, rRot);
            UpdateControllerMarker(_leftCtrlMarker, lTracked && showVRDashboard, lPos, lRot);

            // Read buttons on both controllers
            bool rPrim = GetBoolUsage(rightDev, UnityEngine.XR.CommonUsages.primaryButton);
            bool lPrim = GetBoolUsage(leftDev, UnityEngine.XR.CommonUsages.primaryButton);
            bool rSec = GetBoolUsage(rightDev, UnityEngine.XR.CommonUsages.secondaryButton);
            bool lSec = GetBoolUsage(leftDev, UnityEngine.XR.CommonUsages.secondaryButton);
            bool rGrip = GetBoolUsage(rightDev, UnityEngine.XR.CommonUsages.gripButton);
            bool lGrip = GetBoolUsage(leftDev, UnityEngine.XR.CommonUsages.gripButton);
            bool rTrig = GetBoolUsage(rightDev, UnityEngine.XR.CommonUsages.triggerButton);
            bool lTrig = GetBoolUsage(leftDev, UnityEngine.XR.CommonUsages.triggerButton);

            // 1. Primary Button (A / X) -> Toggle & Recenter In-Headset 3D VR Menu
            if ((rPrim && !_prevRPrimary) || (lPrim && !_prevLPrimary))
            {
                ToggleVRDashboard();
            }

            // 2. Secondary Button (B / Y) -> Cycle 3D Hand Mesh Style
            if ((rSec && !_prevRSecondary) || (lSec && !_prevLSecondary))
            {
                controller?.CycleVisualStyle();
            }

            // 3. Grip + Trigger -> Golden-Reference Calibration using exact 3D Controller Pose!
            bool rTrigJustPressed = rTrig && !_prevRTrigger;
            bool lTrigJustPressed = lTrig && !_prevLTrigger;

            if (rGrip && rTrigJustPressed && rTracked && controller != null)
            {
                controller.CalibrateToVRController(rPos, rRot, isLeftController: false);
            }
            else if (lGrip && lTrigJustPressed && lTracked && controller != null)
            {
                controller.CalibrateToVRController(lPos, lRot, isLeftController: true);
            }
            else
            {
                // Grip alone (on release or press when not holding trigger) -> Cycle Arm Mode
                if ((rGrip && !_prevRGrip && !rTrig) || (lGrip && !_prevLGrip && !lTrig))
                {
                    controller?.CycleArmMode();
                }
            }

            // 4. Thumbstick Continuous Tuning
            Vector2 rStick = GetVec2Usage(rightDev, UnityEngine.XR.CommonUsages.primary2DAxis);
            Vector2 lStick = GetVec2Usage(leftDev, UnityEngine.XR.CommonUsages.primary2DAxis);
            Vector2 stick = rStick.sqrMagnitude > lStick.sqrMagnitude ? rStick : lStick;
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
                    if ((rGrip || lGrip) && receiver != null)
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

            // 5. Laser Pointer Raycast against 3D VR Menu Buttons
            UpdateLaserPointerAndClicks(rTracked, rPos, rRot, rTrigJustPressed && !rGrip, lTracked, lPos, lRot, lTrigJustPressed && !lGrip);

            _prevRPrimary = rPrim;
            _prevLPrimary = lPrim;
            _prevRSecondary = rSec;
            _prevLSecondary = lSec;
            _prevRGrip = rGrip;
            _prevLGrip = lGrip;
            _prevRTrigger = rTrig;
            _prevLTrigger = lTrig;
        }

        private void UpdateLaserPointerAndClicks(
            bool rTracked, Vector3 rPos, Quaternion rRot, bool rClick,
            bool lTracked, Vector3 lPos, Quaternion lRot, bool lClick)
        {
            if (!showVRDashboard || _laserLine == null)
            {
                if (_laserLine != null) _laserLine.enabled = false;
                if (_cursorDot != null) _cursorDot.gameObject.SetActive(false);
                return;
            }

            Vector3 rayOrigin;
            Vector3 rayDir;
            bool clickRequested = false;

            if (rTracked)
            {
                rayOrigin = rPos;
                rayDir = rRot * Vector3.forward;
                clickRequested = rClick;
            }
            else if (lTracked)
            {
                rayOrigin = lPos;
                rayDir = lRot * Vector3.forward;
                clickRequested = lClick;
            }
            else if (_vrCam != null)
            {
                // Fallback: Head-gaze pointer when no controller is powered on
                rayOrigin = _vrCam.transform.position + _vrCam.transform.up * -0.02f;
                rayDir = _vrCam.transform.forward;
                clickRequested = false;
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

            // Update hover visuals
            if (_hoveredButton != hitBtn)
            {
                if (_hoveredButton != null && _hoveredButton.BgRenderer != null)
                    SetMaterialColor(_hoveredButton.BgRenderer.gameObject, _hoveredButton.NormalColor);
                _hoveredButton = hitBtn;
                if (_hoveredButton != null && _hoveredButton.BgRenderer != null)
                    SetMaterialColor(_hoveredButton.BgRenderer.gameObject, _hoveredButton.HoverColor);
            }

            if (clickRequested && _hoveredButton != null)
            {
                _hoveredButton.OnClick?.Invoke();
            }

            // Draw laser only if pointing from a controller or hovering a button
            bool showRay = rTracked || lTracked || hitBtn != null;
            _laserLine.enabled = showRay;
            if (showRay)
            {
                _laserLine.SetPosition(0, rayOrigin);
                _laserLine.SetPosition(1, endPt);
            }

            if (_cursorDot != null)
            {
                _cursorDot.gameObject.SetActive(hitBtn != null || rTracked || lTracked);
                _cursorDot.position = endPt - rayDir * 0.004f;
            }
        }

        private bool ReadControllerPose(UnityEngine.XR.InputDevice dev, out Vector3 worldPos, out Quaternion worldRot)
        {
            worldPos = Vector3.zero;
            worldRot = Quaternion.identity;
            if (!dev.isValid) return false;

            if (dev.TryGetFeatureValue(UnityEngine.XR.CommonUsages.isTracked, out bool isTracked) && !isTracked)
                return false;

            if (!dev.TryGetFeatureValue(UnityEngine.XR.CommonUsages.devicePosition, out Vector3 localPos))
                return false;

            dev.TryGetFeatureValue(UnityEngine.XR.CommonUsages.deviceRotation, out Quaternion localRot);

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
            return true;
        }

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
                string autoStr = receiver.continuousAutoCalibration
                    ? (receiver.autoCalibrationLocked ? "AutoAlign: LOCKED" : "AutoAlign: SCANNING")
                    : "AutoAlign: MANUAL";
                _headerText.text = $"PIMAX + D455C v6.0  |  {fStr}  |  {autoStr}";
            }
            if (_statusText != null && controller != null)
            {
                _statusText.text = $"{controller.CalibrationStatusMessage}\n" +
                                   "Controller: [A/X] Menu | [B/Y] Mesh | [Grip] Arm | [Grip+Trig] Match Hand";
            }

            for (int i = 0; i < _buttons.Count; i++)
            {
                if (_buttons[i].DynamicLabel != null && _buttons[i].Label != null)
                {
                    _buttons[i].Label.text = _buttons[i].DynamicLabel();
                }
            }
        }

        private void BuildWorldSpaceVRDashboard()
        {
            _boardRoot = new GameObject("InHeadset_3D_VR_Dashboard");

            // Main dark slate panel (0.58m x 0.48m)
            GameObject backdrop = GameObject.CreatePrimitive(PrimitiveType.Cube);
            backdrop.name = "DashboardBackdrop";
            Destroy(backdrop.GetComponent<Collider>());
            backdrop.transform.SetParent(_boardRoot.transform, false);
            backdrop.transform.localScale = new Vector3(0.58f, 0.48f, 0.008f);
            SetMaterialColor(backdrop, new Color(0.08f, 0.11f, 0.16f));

            // Header & Status TextMeshes
            _headerText = CreateWorldText(_boardRoot.transform, "HeaderText", new Vector3(0f, 0.21f, -0.008f), 42, Color.cyan);
            _statusText = CreateWorldText(_boardRoot.transform, "StatusText", new Vector3(0f, 0.165f, -0.008f), 30, new Color(0.85f, 0.92f, 1.0f));

            // Row 1: Calibration actions
            CreateVRButton(new Vector3(-0.14f, 0.105f, -0.010f), new Vector2(0.26f, 0.048f),
                new Color(0.12f, 0.48f, 0.72f), () => "Calibrate Camera (C)",
                () => controller?.PerformOneClickCalibration());

            CreateVRButton(new Vector3(0.14f, 0.105f, -0.010f), new Vector2(0.26f, 0.048f),
                new Color(0.16f, 0.58f, 0.42f), () => "Match Controller (Grip+Trig)",
                () => TryCalibrateWithActiveController());

            // Row 2: Auto-Align & Experiment Task
            CreateVRButton(new Vector3(-0.14f, 0.045f, -0.010f), new Vector2(0.26f, 0.046f),
                new Color(0.22f, 0.44f, 0.56f),
                () => $"Auto-Align: {(receiver != null && receiver.continuousAutoCalibration ? "ON" : "OFF")}",
                () => controller?.ToggleContinuousAutoCalibration());

            CreateVRButton(new Vector3(0.14f, 0.045f, -0.010f), new Vector2(0.26f, 0.046f),
                new Color(0.42f, 0.32f, 0.62f),
                () => $"Task: {(controller != null ? controller.experimentPreset.ToString() : "General")}",
                () => controller?.CycleExperimentPreset());

            // Row 3: Arm Mode & 3D Mesh Style
            CreateVRButton(new Vector3(-0.14f, -0.015f, -0.010f), new Vector2(0.26f, 0.046f),
                new Color(0.62f, 0.48f, 0.14f),
                () => $"Arm: {(controller != null ? controller.activeArmMode.ToString() : "Right")}",
                () => controller?.CycleArmMode());

            CreateVRButton(new Vector3(0.14f, -0.015f, -0.010f), new Vector2(0.26f, 0.046f),
                new Color(0.20f, 0.54f, 0.36f),
                () => $"Mesh: {(controller != null ? controller.handVisualStyle.ToString() : "Microsoft")}",
                () => controller?.CycleVisualStyle());

            // Row 4: PD Controller Frequency (Smoothness)
            CreateVRButton(new Vector3(-0.20f, -0.075f, -0.010f), new Vector2(0.12f, 0.042f),
                new Color(0.25f, 0.30f, 0.40f), () => "PD -",
                () => { if (controller != null) controller.pdNaturalFrequencyHz = Mathf.Max(1.5f, controller.pdNaturalFrequencyHz - 0.5f); });

            CreateVRButton(new Vector3(0.0f, -0.075f, -0.010f), new Vector2(0.24f, 0.042f),
                new Color(0.16f, 0.20f, 0.28f),
                () => $"PD Smooth: {(controller != null ? controller.pdNaturalFrequencyHz : 4.8f):F1} Hz", null);

            CreateVRButton(new Vector3(0.20f, -0.075f, -0.010f), new Vector2(0.12f, 0.042f),
                new Color(0.25f, 0.30f, 0.40f), () => "PD +",
                () => { if (controller != null) controller.pdNaturalFrequencyHz = Mathf.Min(12.0f, controller.pdNaturalFrequencyHz + 0.5f); });

            // Row 5: Psychophysics Movement Gain
            CreateVRButton(new Vector3(-0.20f, -0.128f, -0.010f), new Vector2(0.12f, 0.042f),
                new Color(0.25f, 0.30f, 0.40f), () => "Gain -",
                () => { if (controller != null) controller.movementGain = Mathf.Max(0.2f, controller.movementGain - 0.1f); });

            CreateVRButton(new Vector3(0.0f, -0.128f, -0.010f), new Vector2(0.24f, 0.042f),
                new Color(0.16f, 0.20f, 0.28f),
                () => $"Force Gain: {(controller != null ? controller.movementGain : 1.0f):F2}x", null);

            CreateVRButton(new Vector3(0.20f, -0.128f, -0.010f), new Vector2(0.12f, 0.042f),
                new Color(0.25f, 0.30f, 0.40f), () => "Gain +",
                () => { if (controller != null) controller.movementGain = Mathf.Min(2.5f, controller.movementGain + 0.1f); });

            // Row 6: Shoulder Height & Save / Hide
            CreateVRButton(new Vector3(-0.20f, -0.185f, -0.010f), new Vector2(0.12f, 0.042f),
                new Color(0.25f, 0.30f, 0.40f), () => "Sh.Y -",
                () => { if (controller != null) controller.shoulderOffsetFromHead.y = Mathf.Max(-0.35f, controller.shoulderOffsetFromHead.y - 0.02f); });

            CreateVRButton(new Vector3(0.0f, -0.185f, -0.010f), new Vector2(0.24f, 0.042f),
                new Color(0.18f, 0.44f, 0.28f),
                () => "Save Session",
                () => controller?.SavePersistentSettings());

            CreateVRButton(new Vector3(0.20f, -0.185f, -0.010f), new Vector2(0.12f, 0.042f),
                new Color(0.25f, 0.30f, 0.40f), () => "Sh.Y +",
                () => { if (controller != null) controller.shoulderOffsetFromHead.y = Mathf.Min(0.05f, controller.shoulderOffsetFromHead.y + 0.02f); });

            // Row 7: Hide Menu button
            CreateVRButton(new Vector3(0.0f, -0.228f, -0.010f), new Vector2(0.36f, 0.036f),
                new Color(0.55f, 0.22f, 0.22f), () => "Hide VR Menu (Press A/X or Tab)",
                () => ToggleVRDashboard());

            // Laser Pointer LineRenderer + Cursor Dot
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

            _rightCtrlMarker = CreateControllerMarker("RightControllerPoseMarker", new Color(0.2f, 0.9f, 1.0f));
            _leftCtrlMarker = CreateControllerMarker("LeftControllerPoseMarker", new Color(1.0f, 0.7f, 0.2f));
        }

        private void TryCalibrateWithActiveController()
        {
            if (controller == null) return;
            UnityEngine.XR.InputDevice rightDev = InputDevices.GetDeviceAtXRNode(XRNode.RightHand);
            if (ReadControllerPose(rightDev, out Vector3 rPos, out Quaternion rRot))
            {
                controller.CalibrateToVRController(rPos, rRot, false);
                return;
            }
            UnityEngine.XR.InputDevice leftDev = InputDevices.GetDeviceAtXRNode(XRNode.LeftHand);
            if (ReadControllerPose(leftDev, out Vector3 lPos, out Quaternion lRot))
            {
                controller.CalibrateToVRController(lPos, lRot, true);
                return;
            }
            controller.PerformOneClickCalibration();
        }

        private Transform CreateControllerMarker(string name, Color col)
        {
            GameObject go = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            go.name = name;
            Destroy(go.GetComponent<Collider>());
            go.transform.SetParent(this.transform, false);
            go.transform.localScale = Vector3.one * 0.028f;
            SetMaterialColor(go, col);
            go.SetActive(false);
            return go.transform;
        }

        private void CreateVRButton(Vector3 localPos, Vector2 size, Color color, Func<string> labelFunc, Action onClick)
        {
            GameObject btnGo = GameObject.CreatePrimitive(PrimitiveType.Cube);
            btnGo.name = "VRBtn_" + labelFunc();
            btnGo.transform.SetParent(_boardRoot.transform, false);
            btnGo.transform.localPosition = localPos;
            btnGo.transform.localScale = new Vector3(size.x, size.y, 0.008f);
            SetMaterialColor(btnGo, color);

            TextMesh txt = CreateWorldText(btnGo.transform, "Label", new Vector3(0f, 0f, -0.6f), 34, Color.white);
            // Counteract parent button scale so text stays crisp & un-stretched
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
            tm.characterSize = 0.0042f;
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
