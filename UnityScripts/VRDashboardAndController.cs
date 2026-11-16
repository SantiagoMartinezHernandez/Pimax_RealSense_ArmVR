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
    /// v6.2 In-Headset 3D World-Space VR Dashboard + Guided 6-DOF 3D Path Calibration Arena:
    ///   1. Guided 8.5s 3D Path Calibration: spawns glowing 3D target orb(s) for Right, Left, or Both arms,
    ///      waits for [A/X]/[Space] (or holding inside orb), guides the user through a 5-stage 3D star +
    ///      figure-8 sweep, and runs lag-compensated Procrustes 3D alignment (Yaw/Pitch/Roll, Scale,
    ///      independent Left/Right shoulder sockets, and 6-DOF IMU wrist offset).
    ///   2. Hands-Free 3-Second Gaze-Dwell Auto-Click + Controller Trigger / Mouse / Enter instant click.
    ///   3. Dual-API Pimax OpenXR Controller Tracking (UnityEngine.XR.InputDevices + InputSystem XRController).
    /// </summary>
    public class VRDashboardAndController : MonoBehaviour
    {
        private enum GuidedCalibState
        {
            Idle = 0,
            WaitingForUserStart = 1,
            RunningPath = 2
        }

        [Header("References")]
        public ArmAvatarController controller;
        public RealSenseArmReceiver receiver;

        [Header("In-Headset 3D VR Dashboard Settings")]
        public bool showVRDashboard = true;
        public float dashboardDistance = 0.72f;
        public float dashboardSideAngleDeg = -22f;
        public float dashboardHeightOffset = -0.04f;

        [Header("Hands-Free Gaze-Dwell Click")]
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

        // Guided 3D Path Calibration Arena objects & state
        private GuidedCalibState _guidedState = GuidedCalibState.Idle;
        private GameObject _guidedArenaRoot;
        private Transform _rightTargetOrb;
        private Transform _leftTargetOrb;
        private TextMesh _guidedBannerText;
        private LineRenderer _rightPathPreviewLine;
        private LineRenderer _leftPathPreviewLine;
        private Vector3 _guidedArenaOriginPos;
        private Quaternion _guidedArenaHeadYaw = Quaternion.identity;
        private float _guidedTimer;
        private float _orbHoldStartTimer;
        private readonly List<TrajectoryCalibSample> _recordedSamples = new List<TrajectoryCalibSample>(1024);
        private const float GuidedPathDuration = 8.5f;

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

        private float _dwellTimer;
        private float _dwellCooldownTimer;
        private VRButtonItem _flashButton;
        private float _flashTimer;
        private bool _anyControllerTracked;

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
            BuildGuidedCalibrationArena();
            PositionDashboardInFrontOfUser(immediate: true);
            _boardRoot.SetActive(showVRDashboard);
            _guidedArenaRoot.SetActive(false);
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

            if (WasTabPressedThisFrame())
            {
                if (_guidedState != GuidedCalibState.Idle)
                {
                    CancelGuidedCalibration();
                }
                else
                {
                    ToggleVRDashboard();
                }
            }

            PollControllersAndUpdatePointer();
            RefreshDynamicButtonLabels();
        }

        public void ToggleVRDashboard()
        {
            if (_guidedState != GuidedCalibState.Idle) return;

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

        public void StartGuidedPathCalibrationPrompt()
        {
            if (_vrCam == null || controller == null) return;

            _guidedState = GuidedCalibState.WaitingForUserStart;
            _guidedTimer = 0f;
            _orbHoldStartTimer = 0f;
            _recordedSamples.Clear();

            // Lock arena frame to user's current seated head position & horizontal yaw
            _guidedArenaOriginPos = _vrCam.transform.position;
            _guidedArenaHeadYaw = Quaternion.Euler(0f, _vrCam.transform.eulerAngles.y, 0f);

            // Hide menu board so user has a clean workspace for the 3D orbs
            if (_boardRoot != null) _boardRoot.SetActive(false);
            if (_laserLine != null) _laserLine.enabled = false;
            if (_cursorDot != null) _cursorDot.gameObject.SetActive(false);

            _guidedArenaRoot.SetActive(true);
            UpdateGuidedPathPreviewLines();
        }

        public void CancelGuidedCalibration()
        {
            _guidedState = GuidedCalibState.Idle;
            _recordedSamples.Clear();
            if (_guidedArenaRoot != null) _guidedArenaRoot.SetActive(false);
            showVRDashboard = true;
            if (_boardRoot != null)
            {
                _boardRoot.SetActive(true);
                PositionDashboardInFrontOfUser(immediate: true);
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

            bool showMarkers = (_guidedState != GuidedCalibState.Idle) || right.Tracked || left.Tracked;
            UpdateControllerMarker(_rightCtrlMarker, right.Tracked && showMarkers, right.WorldPos, right.WorldRot);
            UpdateControllerMarker(_leftCtrlMarker, left.Tracked && showMarkers, left.WorldPos, left.WorldRot);

            bool primJustPressed = (right.Primary && !_prevRPrimary) || (left.Primary && !_prevLPrimary);
            bool secJustPressed = (right.Secondary && !_prevRSecondary) || (left.Secondary && !_prevLSecondary);
            bool rTrigJustPressed = right.Trigger && !_prevRTrigger;
            bool lTrigJustPressed = left.Trigger && !_prevLTrigger;

            // --- Handle Guided 6-DOF Path Calibration Routine if active ---
            if (_guidedState != GuidedCalibState.Idle)
            {
                UpdateGuidedCalibrationRoutine(
                    right, left,
                    primJustPressed || rTrigJustPressed || lTrigJustPressed || WasSpacePressedThisFrame(),
                    secJustPressed
                );

                _prevRPrimary = right.Primary;
                _prevLPrimary = left.Primary;
                _prevRSecondary = right.Secondary;
                _prevLSecondary = left.Secondary;
                _prevRGrip = right.Grip;
                _prevLGrip = left.Grip;
                _prevRTrigger = right.Trigger;
                _prevLTrigger = left.Trigger;
                return;
            }

            // --- Standard VR Dashboard & Controller Shortcuts ---
            if (primJustPressed)
            {
                ToggleVRDashboard();
            }

            if (secJustPressed)
            {
                controller?.CycleVisualStyle();
            }

            if (right.Grip && rTrigJustPressed && right.Tracked && controller != null)
            {
                controller.CalibrateToVRController(right.WorldPos, right.WorldRot, isLeftController: false);
            }
            else if (left.Grip && lTrigJustPressed && left.Tracked && controller != null)
            {
                controller.CalibrateToVRController(left.WorldPos, left.WorldRot, isLeftController: true);
            }
            else if ((right.Grip && !_prevRGrip && !right.Trigger) || (left.Grip && !_prevLGrip && !left.Trigger))
            {
                controller?.CycleArmMode();
            }

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

        private void UpdateGuidedCalibrationRoutine(
            ControllerState right, ControllerState left,
            bool startPressed, bool cancelPressed)
        {
            if (cancelPressed)
            {
                CancelGuidedCalibration();
                return;
            }

            ActiveArmMode mode = controller != null ? controller.activeArmMode : ActiveArmMode.RightArmOnly;
            bool needRight = (mode == ActiveArmMode.RightArmOnly || mode == ActiveArmMode.BothArms);
            bool needLeft = (mode == ActiveArmMode.LeftArmOnly || mode == ActiveArmMode.BothArms);

            _rightTargetOrb.gameObject.SetActive(needRight);
            _leftTargetOrb.gameObject.SetActive(needLeft);
            _rightPathPreviewLine.enabled = needRight;
            _leftPathPreviewLine.enabled = needLeft;

            // Position banner cleanly above the workspace
            if (_guidedBannerText != null)
            {
                _guidedBannerText.transform.position = _guidedArenaOriginPos + _guidedArenaHeadYaw * new Vector3(0f, 0.22f, 0.65f);
                _guidedBannerText.transform.rotation = _guidedArenaHeadYaw;
            }

            if (_guidedState == GuidedCalibState.WaitingForUserStart)
            {
                Vector3 rStartWorld = _guidedArenaOriginPos + _guidedArenaHeadYaw * EvaluateGuidedLocalWaypoint(0f, isRight: true);
                Vector3 lStartWorld = _guidedArenaOriginPos + _guidedArenaHeadYaw * EvaluateGuidedLocalWaypoint(0f, isRight: false);
                _rightTargetOrb.position = rStartWorld;
                _leftTargetOrb.position = lStartWorld;

                bool rInside = !needRight || (right.Tracked && Vector3.Distance(right.WorldPos, rStartWorld) < 0.12f);
                bool lInside = !needLeft || (left.Tracked && Vector3.Distance(left.WorldPos, lStartWorld) < 0.12f);

                SetMaterialColor(_rightTargetOrb.gameObject, rInside ? Color.green : Color.cyan);
                SetMaterialColor(_leftTargetOrb.gameObject, lInside ? Color.green : new Color(1f, 0.72f, 0.15f));

                if (rInside && lInside && ((needRight && right.Tracked) || (needLeft && left.Tracked)))
                {
                    _orbHoldStartTimer += Time.unscaledDeltaTime;
                }
                else
                {
                    _orbHoldStartTimer = 0f;
                }

                string armPrompt = mode == ActiveArmMode.BothArms
                    ? "BOTH Controllers"
                    : (mode == ActiveArmMode.LeftArmOnly ? "LEFT Controller" : "RIGHT Controller");

                string holdHint = _orbHoldStartTimer > 0.1f
                    ? $"Hold steady... starting in {Mathf.Max(0f, 1.8f - _orbHoldStartTimer):F1}s"
                    : "Press [A / X], [Trigger], or [Space] to begin! ([B/Y] or [Tab] to cancel)";

                string ctrlWarn = (!_anyControllerTracked)
                    ? "\n(No Pimax Controller detected — add 'Oculus Touch Profile' in OpenXR Settings)"
                    : "";

                _guidedBannerText.text = $"GUIDED 6-DOF 3D PATH CALIBRATION\nFirmly grab {armPrompt} and place inside Orb\n{holdHint}{ctrlWarn}";

                if (startPressed || _orbHoldStartTimer >= 1.8f)
                {
                    if (!_anyControllerTracked)
                    {
                        _guidedBannerText.text = "Cannot run 6-DOF Path: No VR Controller tracked!\nCheck OpenXR Interaction Profiles.";
                        return;
                    }
                    _guidedState = GuidedCalibState.RunningPath;
                    _guidedTimer = 0f;
                    _recordedSamples.Clear();
                }
                return;
            }

            if (_guidedState == GuidedCalibState.RunningPath)
            {
                _guidedTimer += Time.unscaledDeltaTime;
                float normT = Mathf.Clamp01(_guidedTimer / GuidedPathDuration);

                Vector3 rTargetWorld = _guidedArenaOriginPos + _guidedArenaHeadYaw * EvaluateGuidedLocalWaypoint(normT, isRight: true);
                Vector3 lTargetWorld = _guidedArenaOriginPos + _guidedArenaHeadYaw * EvaluateGuidedLocalWaypoint(normT, isRight: false);

                _rightTargetOrb.position = rTargetWorld;
                _leftTargetOrb.position = lTargetWorld;

                bool rOnTarget = !needRight || (right.Tracked && Vector3.Distance(right.WorldPos, rTargetWorld) < 0.14f);
                bool lOnTarget = !needLeft || (left.Tracked && Vector3.Distance(left.WorldPos, lTargetWorld) < 0.14f);

                SetMaterialColor(_rightTargetOrb.gameObject, rOnTarget ? Color.green : Color.cyan);
                SetMaterialColor(_leftTargetOrb.gameObject, lOnTarget ? Color.green : new Color(1f, 0.72f, 0.15f));

                string stageDesc = GetStageLabel(normT);
                float remain = Mathf.Max(0f, GuidedPathDuration - _guidedTimer);
                _guidedBannerText.text = $"FOLLOW THE ORB SMOOTHLY  ({remain:F1}s remaining)\n{stageDesc}\nSamples Recorded: {_recordedSamples.Count}";

                // Record synchronized 6-DOF Controller + Camera sample
                if (_vrCam != null && receiver != null)
                {
                    Quaternion curHeadYaw = Quaternion.Euler(0f, _vrCam.transform.eulerAngles.y, 0f);
                    _recordedSamples.Add(new TrajectoryCalibSample
                    {
                        Timestamp = Time.unscaledTime,
                        HeadPos = _vrCam.transform.position,
                        HeadYawRot = curHeadYaw,
                        HasRight = needRight && right.Tracked && receiver.trackingConfidence > 0.30f,
                        RightCamRelWrist = receiver.LocalShoulderToWrist,
                        RightCtrlWorldPos = right.WorldPos,
                        RightCtrlWorldRot = right.WorldRot,
                        RightConfidence = receiver.trackingConfidence,
                        HasLeft = needLeft && left.Tracked && receiver.leftTrackingConfidence > 0.30f,
                        LeftCamRelWrist = receiver.LocalLeftShoulderToWrist,
                        LeftCtrlWorldPos = left.WorldPos,
                        LeftCtrlWorldRot = left.WorldRot,
                        LeftConfidence = receiver.leftTrackingConfidence
                    });
                }

                if (_guidedTimer >= GuidedPathDuration)
                {
                    _guidedState = GuidedCalibState.Idle;
                    _guidedArenaRoot.SetActive(false);
                    showVRDashboard = true;
                    if (_boardRoot != null)
                    {
                        _boardRoot.SetActive(true);
                        PositionDashboardInFrontOfUser(immediate: true);
                    }
                    controller.SolveGuidedTrajectoryCalibration(_recordedSamples);
                }
            }
        }

        /// <summary>
        /// Evaluates the 5-Stage 3D Star + Figure-8 Calibration Trajectory in Head-Yaw local meters.
        /// Uses SmoothStep between corners so velocity naturally pauses at each 3D corner anchor!
        /// </summary>
        private static Vector3 EvaluateGuidedLocalWaypoint(float normT, bool isRight)
        {
            float s = isRight ? 1f : -1f;
            Vector3 p0 = new Vector3(0.08f * s, -0.18f, 0.34f); // Stage 1: Center Chest Midline
            Vector3 p1 = new Vector3(0.14f * s, -0.14f, 0.50f); // Stage 2: Forward Reach (+Z)
            Vector3 p2 = new Vector3(0.26f * s, 0.02f, 0.42f);  // Stage 3: High Lateral Reach (+Y, +-X)
            Vector3 p3 = new Vector3(0.12f * s, -0.28f, 0.38f); // Stage 4: Low Sweep (-Y)
            Vector3 p4 = new Vector3(0.08f * s, -0.18f, 0.36f); // Stage 5: Return to Center Handle

            if (normT <= 0.20f)
            {
                float u = Mathf.SmoothStep(0f, 1f, normT / 0.20f);
                return Vector3.Lerp(p0, p1, u);
            }
            if (normT <= 0.40f)
            {
                float u = Mathf.SmoothStep(0f, 1f, (normT - 0.20f) / 0.20f);
                return Vector3.Lerp(p1, p2, u);
            }
            if (normT <= 0.60f)
            {
                float u = Mathf.SmoothStep(0f, 1f, (normT - 0.40f) / 0.20f);
                return Vector3.Lerp(p2, p3, u);
            }
            if (normT <= 0.85f)
            {
                // Stage 5: Smooth Figure-8 / Arc Sweep across X/Y/Z before returning to center
                float seg = (normT - 0.60f) / 0.25f;
                float u = Mathf.SmoothStep(0f, 1f, seg);
                Vector3 basePos = Vector3.Lerp(p3, p4, u);
                float wave = Mathf.Sin(seg * Mathf.PI * 2f);
                basePos.x += 0.10f * s * wave;
                basePos.y += 0.06f * Mathf.Sin(seg * Mathf.PI);
                basePos.z += 0.05f * Mathf.Sin(seg * Mathf.PI);
                return basePos;
            }
            // Final settle pause at center handle (captures zero-velocity final IMU orientation)
            return p4;
        }

        private static string GetStageLabel(float normT)
        {
            if (normT <= 0.20f) return "Stage 1/5: Forward Depth Reach (+Z)";
            if (normT <= 0.40f) return "Stage 2/5: High Outward Reach (+Y, Pitch/Roll)";
            if (normT <= 0.60f) return "Stage 3/5: Low Diagonal Sweep (-Y)";
            if (normT <= 0.85f) return "Stage 4/5: Dynamic 3D Arc Sweep";
            return "Stage 5/5: Hold Steady at Center Handle...";
        }

        private void UpdateGuidedPathPreviewLines()
        {
            const int steps = 40;
            if (_rightPathPreviewLine != null)
            {
                _rightPathPreviewLine.positionCount = steps;
                for (int i = 0; i < steps; i++)
                {
                    float t = i / (float)(steps - 1);
                    _rightPathPreviewLine.SetPosition(i, _guidedArenaOriginPos + _guidedArenaHeadYaw * EvaluateGuidedLocalWaypoint(t, true));
                }
            }
            if (_leftPathPreviewLine != null)
            {
                _leftPathPreviewLine.positionCount = steps;
                for (int i = 0; i < steps; i++)
                {
                    float t = i / (float)(steps - 1);
                    _leftPathPreviewLine.SetPosition(i, _guidedArenaOriginPos + _guidedArenaHeadYaw * EvaluateGuidedLocalWaypoint(t, false));
                }
            }
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

            if (_hoveredButton != hitBtn)
            {
                if (_hoveredButton != null && _hoveredButton.BgRenderer != null && _hoveredButton != _flashButton)
                    SetMaterialColor(_hoveredButton.BgRenderer.gameObject, _hoveredButton.NormalColor);

                _hoveredButton = hitBtn;
                _dwellTimer = 0f;

                if (_hoveredButton != null && _hoveredButton.BgRenderer != null)
                    SetMaterialColor(_hoveredButton.BgRenderer.gameObject, _hoveredButton.HoverColor);
            }

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

        private static bool WasSpacePressedThisFrame()
        {
#if ENABLE_INPUT_SYSTEM
            if (Keyboard.current != null && Keyboard.current.spaceKey.wasPressedThisFrame)
                return true;
#endif
#if ENABLE_LEGACY_INPUT_MANAGER
            try { if (Input.GetKeyDown(KeyCode.Space)) return true; } catch { }
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
            if (!showVRDashboard || _boardRoot == null || _guidedState != GuidedCalibState.Idle) return;

            if (_headerText != null && controller != null && receiver != null)
            {
                string fStr = receiver.handFingersTracked ? "21-Fingers: ON" : "Arm Tracking";
                string ctrlStr = _anyControllerTracked ? "VR Ctrl: ON" : "Gaze Mode (3s Hold)";
                _headerText.text = $"PIMAX + D455C v6.2  |  {fStr}  |  {ctrlStr}";
            }
            if (_statusText != null && controller != null)
            {
                string hint = _anyControllerTracked
                    ? "Controller: [A/X] Menu | [B/Y] Mesh | [Grip] Arm | [Grip+Trig] Quick Match"
                    : "Tip: Add 'Oculus Touch Profile' in OpenXR Settings — OR Hold Gaze 3s to Click!";
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

        private void BuildGuidedCalibrationArena()
        {
            _guidedArenaRoot = new GameObject("Guided_3D_Path_Calibration_Arena");

            _guidedBannerText = CreateWorldText(_guidedArenaRoot.transform, "GuidedBannerText", Vector3.zero, 44, Color.white);

            _rightTargetOrb = GameObject.CreatePrimitive(PrimitiveType.Sphere).transform;
            _rightTargetOrb.name = "RightArmTargetOrb";
            Destroy(_rightTargetOrb.GetComponent<Collider>());
            _rightTargetOrb.SetParent(_guidedArenaRoot.transform, false);
            _rightTargetOrb.localScale = Vector3.one * 0.075f;
            SetMaterialColor(_rightTargetOrb.gameObject, Color.cyan);

            _leftTargetOrb = GameObject.CreatePrimitive(PrimitiveType.Sphere).transform;
            _leftTargetOrb.name = "LeftArmTargetOrb";
            Destroy(_leftTargetOrb.GetComponent<Collider>());
            _leftTargetOrb.SetParent(_guidedArenaRoot.transform, false);
            _leftTargetOrb.localScale = Vector3.one * 0.075f;
            SetMaterialColor(_leftTargetOrb.gameObject, new Color(1f, 0.72f, 0.15f));

            _rightPathPreviewLine = CreatePathPreviewLine("RightGuidedPathLine", new Color(0.2f, 0.85f, 1.0f, 0.5f));
            _leftPathPreviewLine = CreatePathPreviewLine("LeftGuidedPathLine", new Color(1.0f, 0.72f, 0.2f, 0.5f));
        }

        private LineRenderer CreatePathPreviewLine(string name, Color color)
        {
            GameObject go = new GameObject(name);
            go.transform.SetParent(_guidedArenaRoot.transform, false);
            LineRenderer lr = go.AddComponent<LineRenderer>();
            lr.startWidth = 0.005f;
            lr.endWidth = 0.005f;
            Shader unlit = Shader.Find("Unlit/Color") ?? Shader.Find("Sprites/Default");
            lr.sharedMaterial = new Material(unlit) { color = color };
            return lr;
        }

        private void BuildWorldSpaceVRDashboard()
        {
            _boardRoot = new GameObject("InHeadset_3D_VR_Dashboard");

            GameObject backdrop = GameObject.CreatePrimitive(PrimitiveType.Cube);
            backdrop.name = "DashboardBackdrop";
            Destroy(backdrop.GetComponent<Collider>());
            backdrop.transform.SetParent(_boardRoot.transform, false);
            backdrop.transform.localScale = new Vector3(0.58f, 0.54f, 0.008f);
            SetMaterialColor(backdrop, new Color(0.08f, 0.11f, 0.16f));

            _headerText = CreateWorldText(_boardRoot.transform, "HeaderText", new Vector3(0f, 0.240f, -0.008f), 40, Color.cyan);
            _statusText = CreateWorldText(_boardRoot.transform, "StatusText", new Vector3(0f, 0.192f, -0.008f), 28, new Color(0.85f, 0.92f, 1.0f));

            // Featured Top Row: Full 3D Guided 8s Path Calibration!
            CreateVRButton(new Vector3(0.0f, 0.130f, -0.010f), new Vector2(0.54f, 0.048f),
                new Color(0.14f, 0.62f, 0.42f), () => "GUIDED 8s 3D PATH CALIBRATION (All Axes + Both Arms)",
                () => StartGuidedPathCalibrationPrompt());

            // Row 2: Quick Snapshot Calibrations
            CreateVRButton(new Vector3(-0.14f, 0.074f, -0.010f), new Vector2(0.26f, 0.044f),
                new Color(0.12f, 0.48f, 0.72f), () => "Quick Camera Snap (C)",
                () => controller?.PerformOneClickCalibration());

            CreateVRButton(new Vector3(0.14f, 0.074f, -0.010f), new Vector2(0.26f, 0.044f),
                new Color(0.16f, 0.52f, 0.56f), () => "Quick Match Ctrl (Grip+Trig)",
                () => TryCalibrateWithActiveController());

            // Row 3: Auto-Align & Experiment Task
            CreateVRButton(new Vector3(-0.14f, 0.018f, -0.010f), new Vector2(0.26f, 0.044f),
                new Color(0.22f, 0.44f, 0.56f),
                () => $"Auto-Align: {(receiver != null && receiver.continuousAutoCalibration ? "ON" : "OFF")}",
                () => controller?.ToggleContinuousAutoCalibration());

            CreateVRButton(new Vector3(0.14f, 0.018f, -0.010f), new Vector2(0.26f, 0.044f),
                new Color(0.42f, 0.32f, 0.62f),
                () => $"Task: {(controller != null ? controller.experimentPreset.ToString() : "General")}",
                () => controller?.CycleExperimentPreset());

            // Row 4: Arm Mode & 3D Mesh Style
            CreateVRButton(new Vector3(-0.14f, -0.038f, -0.010f), new Vector2(0.26f, 0.044f),
                new Color(0.62f, 0.48f, 0.14f),
                () => $"Arm: {(controller != null ? controller.activeArmMode.ToString() : "Right")}",
                () => controller?.CycleArmMode());

            CreateVRButton(new Vector3(0.14f, -0.038f, -0.010f), new Vector2(0.26f, 0.044f),
                new Color(0.20f, 0.54f, 0.36f),
                () => $"Mesh: {(controller != null ? controller.handVisualStyle.ToString() : "Microsoft")}",
                () => controller?.CycleVisualStyle());

            // Row 5: PD Controller Frequency (Smoothness)
            CreateVRButton(new Vector3(-0.20f, -0.094f, -0.010f), new Vector2(0.12f, 0.040f),
                new Color(0.25f, 0.30f, 0.40f), () => "PD -",
                () => { if (controller != null) controller.pdNaturalFrequencyHz = Mathf.Max(1.5f, controller.pdNaturalFrequencyHz - 0.5f); });

            CreateVRButton(new Vector3(0.0f, -0.094f, -0.010f), new Vector2(0.24f, 0.040f),
                new Color(0.16f, 0.20f, 0.28f),
                () => $"PD Smooth: {(controller != null ? controller.pdNaturalFrequencyHz : 4.8f):F1} Hz", null);

            CreateVRButton(new Vector3(0.20f, -0.094f, -0.010f), new Vector2(0.12f, 0.040f),
                new Color(0.25f, 0.30f, 0.40f), () => "PD +",
                () => { if (controller != null) controller.pdNaturalFrequencyHz = Mathf.Min(12.0f, controller.pdNaturalFrequencyHz + 0.5f); });

            // Row 6: Psychophysics Movement Gain
            CreateVRButton(new Vector3(-0.20f, -0.144f, -0.010f), new Vector2(0.12f, 0.040f),
                new Color(0.25f, 0.30f, 0.40f), () => "Gain -",
                () => { if (controller != null) controller.movementGain = Mathf.Max(0.2f, controller.movementGain - 0.1f); });

            CreateVRButton(new Vector3(0.0f, -0.144f, -0.010f), new Vector2(0.24f, 0.040f),
                new Color(0.16f, 0.20f, 0.28f),
                () => $"Force Gain: {(controller != null ? controller.movementGain : 1.0f):F2}x", null);

            CreateVRButton(new Vector3(0.20f, -0.144f, -0.010f), new Vector2(0.12f, 0.040f),
                new Color(0.25f, 0.30f, 0.40f), () => "Gain +",
                () => { if (controller != null) controller.movementGain = Mathf.Min(2.5f, controller.movementGain + 0.1f); });

            // Row 7: Shoulder Height & Save
            CreateVRButton(new Vector3(-0.20f, -0.194f, -0.010f), new Vector2(0.12f, 0.040f),
                new Color(0.25f, 0.30f, 0.40f), () => "Sh.Y -",
                () => { if (controller != null) controller.shoulderOffsetFromHead.y = Mathf.Max(-0.35f, controller.shoulderOffsetFromHead.y - 0.02f); });

            CreateVRButton(new Vector3(0.0f, -0.194f, -0.010f), new Vector2(0.24f, 0.040f),
                new Color(0.18f, 0.44f, 0.28f),
                () => "Save Session",
                () => controller?.SavePersistentSettings());

            CreateVRButton(new Vector3(0.20f, -0.194f, -0.010f), new Vector2(0.12f, 0.040f),
                new Color(0.25f, 0.30f, 0.40f), () => "Sh.Y +",
                () => { if (controller != null) controller.shoulderOffsetFromHead.y = Mathf.Min(0.05f, controller.shoulderOffsetFromHead.y + 0.02f); });

            // Row 8: Hide Menu button
            CreateVRButton(new Vector3(0.0f, -0.240f, -0.010f), new Vector2(0.38f, 0.036f),
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
