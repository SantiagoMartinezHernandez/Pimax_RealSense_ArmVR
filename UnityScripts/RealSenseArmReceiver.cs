using System;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using UnityEngine;

namespace PsychophysicsVR.Tracking
{
    /// <summary>
    /// v6.0 Dual-Arm + Dual-21-Finger Receiver with Continuous 90/120Hz Kalman Prediction
    /// and Background Torso-Axis Auto-Calibration.
    /// Eliminates 30-60 Hz sample-and-hold judder by predicting joint states every VR frame
    /// and updates measurements with adaptive confidence covariance when UDP packets arrive.
    /// </summary>
    public class RealSenseArmReceiver : MonoBehaviour
    {
        [Header("Network Settings")]
        public int udpPort = 5055;

        [Header("Coordinate Calibration & Continuous Auto-Alignment")]
        public Transform avatarShoulderAnchor;
        public Transform leftAvatarShoulderAnchor;
        public Transform cameraOrientationReference;

        [Tooltip("When enabled, automatically aligns camera yaw & arm scale in the background from both shoulders—no manual per-user calibration required!")]
        public bool continuousAutoCalibration = true;

        [Tooltip("Total upper + forearm length of the Unity avatar in meters (used for auto-scaling).")]
        public float avatarTargetTotalArmLength = 0.53f;

        [Tooltip("Horizontal angle offset (degrees) of the physical D455C camera relative to the user.")]
        [Range(-180f, 180f)]
        public float cameraYawOffsetDeg = 0f;

        [Tooltip("Scale multiplier to match the participant's real arm length to the Unity avatar's arm length.")]
        public float armScale = 1.0f;

        [Tooltip("Mirror X axis (keep true when the D455C camera is facing the user).")]
        public bool mirrorX = true;

        [Header("Continuous-Discrete Kalman Filter Tuning")]
        [Tooltip("Process velocity noise (higher = faster response to sudden acceleration, lower = smoother).")]
        [Range(0.2f, 12.0f)]
        public float kalmanProcessNoiseVel = 2.8f;

        [Tooltip("Base measurement variance (higher = trusts Kalman model more over noisy sensor frames).")]
        [Range(0.00002f, 0.005f)]
        public float kalmanMeasurementNoiseBase = 0.00025f;

        [Tooltip("Biomechanical velocity damping rate (1/s) during inter-frame prediction.")]
        [Range(1.0f, 10.0f)]
        public float kalmanVelocityDamping = 3.5f;

        // Legacy fields kept serialized for compatibility
        [HideInInspector] public float filterMinCutoff = 2.2f;
        [HideInInspector] public float filterBeta = 0.22f;
        [HideInInspector] public float motionPredictionMs = 12f;

        [Header("Live Telemetry (Read-Only)")]
        [Range(0f, 1f)] public float trackingConfidence;
        [Range(0f, 1f)] public float leftTrackingConfidence;
        [Range(0f, 1f)] public float depthSensorValidRatio;
        public bool handFingersTracked;
        public bool leftHandFingersTracked;
        public float measuredRealArmLengthMeters;
        public int packetsReceivedCount;
        public int remotePythonArmMode = -1;
        public bool autoCalibrationLocked;

        // Right Arm filtered joint positions in Unity World Space (updated at full VR frame rate)
        public Vector3 ShoulderPos { get; private set; }
        public Vector3 ElbowPos { get; private set; }
        public Vector3 WristPos { get; private set; }
        public Vector3 IndexPos { get; private set; }
        public Vector3 PinkyPos { get; private set; }
        public readonly Vector3[] HandLandmarksWorld = new Vector3[21];

        // Left Arm filtered joint positions in Unity World Space
        public Vector3 LeftShoulderPos { get; private set; }
        public Vector3 LeftElbowPos { get; private set; }
        public Vector3 LeftWristPos { get; private set; }
        public Vector3 LeftIndexPos { get; private set; }
        public Vector3 LeftPinkyPos { get; private set; }
        public readonly Vector3[] LeftHandLandmarksWorld = new Vector3[21];

        // Un-rotated local vectors in Unity camera space
        public Vector3 LocalShoulderToWrist { get; private set; }
        public Vector3 LocalLeftShoulderToWrist { get; private set; }
        public Vector3 LocalOppositeToActiveShoulder { get; private set; }

        public bool HasValidTracking => trackingConfidence > 0.30f || leftTrackingConfidence > 0.30f;

        // Thread-safe raw receive buffer
        private readonly object _lock = new object();
        private Vector3 _rawShoulder, _rawElbow, _rawWrist, _rawIndex, _rawPinky, _rawOppShoulder;
        private readonly Vector3[] _rawHandRel = new Vector3[21];
        private bool _rawHandActive;
        private float _rawConfidence;
        private float _rawDepthRatio;

        private Vector3 _rawLeftShoulder, _rawLeftElbow, _rawLeftWrist, _rawLeftIndex, _rawLeftPinky;
        private readonly Vector3[] _rawLeftHandRel = new Vector3[21];
        private bool _rawLeftHandActive;
        private float _rawLeftConfidence;
        private int _rawPythonArmMode = -1;
        private bool _hasDualArmPacket;
        private bool _newPacketAvailable;

        // Continuous-Discrete 3D Kalman Filters (Right Arm)
        private KalmanFilter3D _kalmanRelElbow;
        private KalmanFilter3D _kalmanRelWrist;
        private KalmanFilter3D _kalmanRelIndex;
        private KalmanFilter3D _kalmanRelPinky;
        private KalmanFilter3D _kalmanChestAxis;
        private readonly OneEuroFilterVector3[] _fingerFilters = new OneEuroFilterVector3[21];
        private readonly Vector3[] _filteredHandRelOffsets = new Vector3[21];

        // Continuous-Discrete 3D Kalman Filters (Left Arm)
        private KalmanFilter3D _leftKalmanRelElbow;
        private KalmanFilter3D _leftKalmanRelWrist;
        private KalmanFilter3D _leftKalmanRelIndex;
        private KalmanFilter3D _leftKalmanRelPinky;
        private readonly OneEuroFilterVector3[] _leftFingerFilters = new OneEuroFilterVector3[21];
        private readonly Vector3[] _filteredLeftHandRelOffsets = new Vector3[21];

        private int _autoCalibSampleCount;
        private Camera _vrCam;

        private UdpClient _udpClient;
        private Thread _receiveThread;
        private volatile bool _isRunning;

        private void Awake()
        {
            _kalmanRelElbow = new KalmanFilter3D(kalmanProcessNoiseVel, kalmanMeasurementNoiseBase, kalmanVelocityDamping, 0.08f);
            _kalmanRelWrist = new KalmanFilter3D(kalmanProcessNoiseVel * 1.25f, kalmanMeasurementNoiseBase, kalmanVelocityDamping, 0.10f);
            _kalmanRelIndex = new KalmanFilter3D(kalmanProcessNoiseVel * 1.25f, kalmanMeasurementNoiseBase * 1.5f, kalmanVelocityDamping, 0.10f);
            _kalmanRelPinky = new KalmanFilter3D(kalmanProcessNoiseVel * 1.25f, kalmanMeasurementNoiseBase * 1.5f, kalmanVelocityDamping, 0.10f);
            _kalmanChestAxis = new KalmanFilter3D(0.4f, 0.002f, 4.0f, 0.05f);

            _leftKalmanRelElbow = new KalmanFilter3D(kalmanProcessNoiseVel, kalmanMeasurementNoiseBase, kalmanVelocityDamping, 0.08f);
            _leftKalmanRelWrist = new KalmanFilter3D(kalmanProcessNoiseVel * 1.25f, kalmanMeasurementNoiseBase, kalmanVelocityDamping, 0.10f);
            _leftKalmanRelIndex = new KalmanFilter3D(kalmanProcessNoiseVel * 1.25f, kalmanMeasurementNoiseBase * 1.5f, kalmanVelocityDamping, 0.10f);
            _leftKalmanRelPinky = new KalmanFilter3D(kalmanProcessNoiseVel * 1.25f, kalmanMeasurementNoiseBase * 1.5f, kalmanVelocityDamping, 0.10f);

            for (int i = 0; i < 21; i++)
            {
                _fingerFilters[i] = new OneEuroFilterVector3(3.2f, 0.25f, 1.0f, 4.0f);
                _leftFingerFilters[i] = new OneEuroFilterVector3(3.2f, 0.25f, 1.0f, 4.0f);
            }

            if (avatarShoulderAnchor == null)
                avatarShoulderAnchor = this.transform;
            if (cameraOrientationReference == null)
                cameraOrientationReference = this.transform;
        }

        private void Start()
        {
            _vrCam = Camera.main != null ? Camera.main : UnityEngine.Object.FindFirstObjectByType<Camera>();
            StartUdpReceiver();
        }

        private void OnValidate()
        {
            SyncKalmanParams();
        }

        private void SyncKalmanParams()
        {
            if (_kalmanRelElbow == null) return;

            ApplyKalmanTuning(_kalmanRelElbow, 1.0f);
            ApplyKalmanTuning(_kalmanRelWrist, 1.25f);
            ApplyKalmanTuning(_kalmanRelIndex, 1.25f);
            ApplyKalmanTuning(_kalmanRelPinky, 1.25f);

            ApplyKalmanTuning(_leftKalmanRelElbow, 1.0f);
            ApplyKalmanTuning(_leftKalmanRelWrist, 1.25f);
            ApplyKalmanTuning(_leftKalmanRelIndex, 1.25f);
            ApplyKalmanTuning(_leftKalmanRelPinky, 1.25f);
        }

        private void ApplyKalmanTuning(KalmanFilter3D kf, float velMultiplier)
        {
            kf.processNoiseVel = kalmanProcessNoiseVel * velMultiplier;
            kf.measurementNoiseBase = kalmanMeasurementNoiseBase;
            kf.velocityDampingRate = kalmanVelocityDamping;
        }

        private Vector3 ConvertRealSenseToUnityLocal(Vector3 rsPt)
        {
            float x = mirrorX ? -rsPt.x : rsPt.x;
            float y = -rsPt.y;
            float z = mirrorX ? -rsPt.z : rsPt.z;
            return new Vector3(x, y, z);
        }

        public void ResetAutoCalibrationLock()
        {
            _autoCalibSampleCount = 0;
            autoCalibrationLocked = false;
        }

        public void CalibrateArmScaleToAvatar(float avatarTotalArmLength)
        {
            avatarTargetTotalArmLength = avatarTotalArmLength;
            if (measuredRealArmLengthMeters > 0.25f && avatarTotalArmLength > 0.1f)
            {
                armScale = Mathf.Clamp(avatarTotalArmLength / measuredRealArmLengthMeters, 0.55f, 1.85f);
                Debug.Log($"[RealSenseArmReceiver] Calibrated armScale = {armScale:F3} " +
                          $"(Real D455C arm = {measuredRealArmLengthMeters:F3}m, Avatar arm = {avatarTotalArmLength:F3}m)");
            }
        }

        private void Update()
        {
            SyncKalmanParams();

            if (_vrCam == null)
                _vrCam = Camera.main;

            Vector3 s, e, w, idx, pky, oppS;
            Vector3 ls, le, lw, lidx, lpky;
            float conf, lConf, depthRatio;
            bool handActive, lHandActive, hasDual, hasNew;

            lock (_lock)
            {
                hasNew = _newPacketAvailable;
                s = _rawShoulder;
                e = _rawElbow;
                w = _rawWrist;
                idx = _rawIndex;
                pky = _rawPinky;
                oppS = _rawOppShoulder;
                conf = _rawConfidence;
                depthRatio = _rawDepthRatio;
                handActive = _rawHandActive;

                ls = _rawLeftShoulder;
                le = _rawLeftElbow;
                lw = _rawLeftWrist;
                lidx = _rawLeftIndex;
                lpky = _rawLeftPinky;
                lConf = _rawLeftConfidence;
                lHandActive = _rawLeftHandActive;
                hasDual = _hasDualArmPacket;
                remotePythonArmMode = _rawPythonArmMode;

                _newPacketAvailable = false;
            }

            if (!hasNew && trackingConfidence <= 0f && leftTrackingConfidence <= 0f) return;

            float dt = Mathf.Clamp(Time.unscaledDeltaTime, 0.001f, 0.05f);
            float now = Time.unscaledTime;

            // --- 1. Predict Continuously at 90/120 Hz VR Frame Rate (Zero Sample-and-Hold Staircase!) ---
            _kalmanRelElbow.Predict(dt);
            _kalmanRelWrist.Predict(dt);
            _kalmanRelIndex.Predict(dt);
            _kalmanRelPinky.Predict(dt);
            _kalmanChestAxis.Predict(dt);

            _leftKalmanRelElbow.Predict(dt);
            _leftKalmanRelWrist.Predict(dt);
            _leftKalmanRelIndex.Predict(dt);
            _leftKalmanRelPinky.Predict(dt);

            // --- 2. Apply Adaptive Kalman Measurement Update When UDP Packet Arrives ---
            if (hasNew)
            {
                packetsReceivedCount++;
                trackingConfidence = conf;
                leftTrackingConfidence = hasDual ? lConf : conf;
                depthSensorValidRatio = depthRatio;
                handFingersTracked = handActive;
                leftHandFingersTracked = hasDual ? lHandActive : false;

                float realUpper = Vector3.Distance(s, e);
                float realFore = Vector3.Distance(e, w);
                if (realUpper + realFore > 0.25f && realUpper + realFore < 1.1f)
                {
                    measuredRealArmLengthMeters = Mathf.Lerp(
                        measuredRealArmLengthMeters <= 0.01f ? (realUpper + realFore) : measuredRealArmLengthMeters,
                        realUpper + realFore,
                        0.10f
                    );
                }

                // Right Arm Kalman measurement update
                _kalmanRelElbow.UpdateMeasurement(e - s, conf);
                _kalmanRelWrist.UpdateMeasurement(w - s, conf);
                _kalmanRelIndex.UpdateMeasurement(idx - s, conf);
                _kalmanRelPinky.UpdateMeasurement(pky - s, conf);

                if (oppS.sqrMagnitude > 0.01f)
                {
                    _kalmanChestAxis.UpdateMeasurement(s - oppS, conf);
                }

                if (handActive)
                {
                    lock (_lock)
                    {
                        for (int i = 0; i < 21; i++)
                        {
                            _filteredHandRelOffsets[i] = _fingerFilters[i].Filter(_rawHandRel[i], now, 0f);
                        }
                    }
                }

                // Left Arm Kalman measurement update
                if (hasDual)
                {
                    _leftKalmanRelElbow.UpdateMeasurement(le - ls, lConf);
                    _leftKalmanRelWrist.UpdateMeasurement(lw - ls, lConf);
                    _leftKalmanRelIndex.UpdateMeasurement(lidx - ls, lConf);
                    _leftKalmanRelPinky.UpdateMeasurement(lpky - ls, lConf);

                    if (lHandActive)
                    {
                        lock (_lock)
                        {
                            for (int i = 0; i < 21; i++)
                            {
                                _filteredLeftHandRelOffsets[i] = _leftFingerFilters[i].Filter(_rawLeftHandRel[i], now, 0f);
                            }
                        }
                    }
                }

                // --- 3. Continuous Background Torso-Axis Auto-Calibration ---
                if (continuousAutoCalibration && oppS.sqrMagnitude > 0.04f && conf > 0.55f)
                {
                    RunBackgroundAutoCalibration(dt);
                }
            }

            // --- 4. Project Continuous Predicted Arm Positions Into Unity World Space Every Frame ---
            Quaternion baseRot = cameraOrientationReference != null ? cameraOrientationReference.rotation : Quaternion.identity;
            Quaternion yawRot = Quaternion.Euler(0f, cameraYawOffsetDeg, 0f) * baseRot;

            Vector3 worldRightShoulder = avatarShoulderAnchor != null ? avatarShoulderAnchor.position : transform.position;
            Vector3 worldLeftShoulder = leftAvatarShoulderAnchor != null ? leftAvatarShoulderAnchor.position : worldRightShoulder + new Vector3(-0.34f, 0f, 0f);

            Vector3 predRelElbow = _kalmanRelElbow.Position;
            Vector3 predRelWrist = _kalmanRelWrist.Position;
            Vector3 predRelIndex = _kalmanRelIndex.Position;
            Vector3 predRelPinky = _kalmanRelPinky.Position;

            LocalShoulderToWrist = predRelWrist;
            LocalOppositeToActiveShoulder = _kalmanChestAxis.Position;

            ShoulderPos = worldRightShoulder;
            ElbowPos = worldRightShoulder + yawRot * (predRelElbow * armScale);
            WristPos = worldRightShoulder + yawRot * (predRelWrist * armScale);
            IndexPos = worldRightShoulder + yawRot * (predRelIndex * armScale);
            PinkyPos = worldRightShoulder + yawRot * (predRelPinky * armScale);

            if (handFingersTracked)
            {
                for (int i = 0; i < 21; i++)
                {
                    HandLandmarksWorld[i] = WristPos + yawRot * _filteredHandRelOffsets[i];
                }
                IndexPos = HandLandmarksWorld[5];
                PinkyPos = HandLandmarksWorld[17];
            }

            if (_leftKalmanRelWrist.Initialized)
            {
                Vector3 lPredElbow = _leftKalmanRelElbow.Position;
                Vector3 lPredWrist = _leftKalmanRelWrist.Position;
                Vector3 lPredIndex = _leftKalmanRelIndex.Position;
                Vector3 lPredPinky = _leftKalmanRelPinky.Position;

                LocalLeftShoulderToWrist = lPredWrist;

                LeftShoulderPos = worldLeftShoulder;
                LeftElbowPos = worldLeftShoulder + yawRot * (lPredElbow * armScale);
                LeftWristPos = worldLeftShoulder + yawRot * (lPredWrist * armScale);
                LeftIndexPos = worldLeftShoulder + yawRot * (lPredIndex * armScale);
                LeftPinkyPos = worldLeftShoulder + yawRot * (lPredPinky * armScale);

                if (leftHandFingersTracked)
                {
                    for (int i = 0; i < 21; i++)
                    {
                        LeftHandLandmarksWorld[i] = LeftWristPos + yawRot * _filteredLeftHandRelOffsets[i];
                    }
                    LeftIndexPos = LeftHandLandmarksWorld[5];
                    LeftPinkyPos = LeftHandLandmarksWorld[17];
                }
            }
        }

        private void RunBackgroundAutoCalibration(float dt)
        {
            Vector3 chestRightXZ = new Vector3(_kalmanChestAxis.Position.x, 0f, _kalmanChestAxis.Position.z);
            if (_vrCam == null || chestRightXZ.magnitude < 0.14f) return;

            Vector3 vrRightXZ = new Vector3(_vrCam.transform.right.x, 0f, _vrCam.transform.right.z).normalized;
            float rawChestAngle = Mathf.Atan2(chestRightXZ.x, chestRightXZ.z) * Mathf.Rad2Deg;
            float targetRightAngle = Mathf.Atan2(vrRightXZ.x, vrRightXZ.z) * Mathf.Rad2Deg;
            float targetYaw = Mathf.DeltaAngle(rawChestAngle, targetRightAngle);

            _autoCalibSampleCount++;
            if (_autoCalibSampleCount <= 30)
            {
                // Initial fast lock within the first ~1 second of seating
                cameraYawOffsetDeg = Mathf.LerpAngle(cameraYawOffsetDeg, targetYaw, 0.15f);
                if (measuredRealArmLengthMeters > 0.28f && avatarTargetTotalArmLength > 0.1f)
                {
                    float targetScale = Mathf.Clamp(avatarTargetTotalArmLength / measuredRealArmLengthMeters, 0.65f, 1.50f);
                    armScale = Mathf.Lerp(armScale, targetScale, 0.12f);
                }
                if (_autoCalibSampleCount == 30)
                    autoCalibrationLocked = true;
            }
            else
            {
                // Gentle background drift compensation (max 0.5 deg/sec so trials are never disrupted)
                cameraYawOffsetDeg = Mathf.MoveTowardsAngle(cameraYawOffsetDeg, targetYaw, 0.5f * Mathf.Max(dt, 0.016f));
                if (measuredRealArmLengthMeters > 0.28f && avatarTargetTotalArmLength > 0.1f)
                {
                    float targetScale = Mathf.Clamp(avatarTargetTotalArmLength / measuredRealArmLengthMeters, 0.65f, 1.50f);
                    armScale = Mathf.MoveTowards(armScale, targetScale, 0.01f * Mathf.Max(dt, 0.016f));
                }
            }
        }

        private void StartUdpReceiver()
        {
            _isRunning = true;
            _receiveThread = new Thread(UdpListenLoop) { IsBackground = true };
            _receiveThread.Start();
        }

        private void UdpListenLoop()
        {
            try
            {
                _udpClient = new UdpClient(udpPort);
                _udpClient.Client.ReceiveBufferSize = 65536;
                IPEndPoint remoteEp = new IPEndPoint(IPAddress.Any, 0);

                while (_isRunning)
                {
                    byte[] data = _udpClient.Receive(ref remoteEp);
                    while (_udpClient.Available > 0)
                    {
                        data = _udpClient.Receive(ref remoteEp);
                    }

                    if (data != null && data.Length >= 64)
                    {
                        Vector3 s = ConvertRealSenseToUnityLocal(ReadVec3(data, 0));
                        Vector3 e = ConvertRealSenseToUnityLocal(ReadVec3(data, 12));
                        Vector3 w = ConvertRealSenseToUnityLocal(ReadVec3(data, 24));
                        Vector3 idx = ConvertRealSenseToUnityLocal(ReadVec3(data, 36));
                        Vector3 pky = ConvertRealSenseToUnityLocal(ReadVec3(data, 48));
                        float conf = BitConverter.ToSingle(data, 60);
                        float dRatio = data.Length >= 68 ? BitConverter.ToSingle(data, 64) : 1.0f;

                        Vector3 oppS = Vector3.zero;
                        bool handAct = false;

                        if (data.Length >= 336)
                        {
                            oppS = ConvertRealSenseToUnityLocal(ReadVec3(data, 68));
                            handAct = BitConverter.ToSingle(data, 80) > 0.5f;
                        }

                        bool hasDual = data.Length >= 656;
                        Vector3 ls = Vector3.zero, le = Vector3.zero, lw = Vector3.zero, lidx = Vector3.zero, lpky = Vector3.zero;
                        float lConf = 0f;
                        bool lHandAct = false;

                        if (hasDual)
                        {
                            ls = ConvertRealSenseToUnityLocal(ReadVec3(data, 336));
                            le = ConvertRealSenseToUnityLocal(ReadVec3(data, 348));
                            lw = ConvertRealSenseToUnityLocal(ReadVec3(data, 360));
                            lidx = ConvertRealSenseToUnityLocal(ReadVec3(data, 372));
                            lpky = ConvertRealSenseToUnityLocal(ReadVec3(data, 384));
                            lConf = BitConverter.ToSingle(data, 396);
                            lHandAct = BitConverter.ToSingle(data, 400) > 0.5f;
                        }

                        lock (_lock)
                        {
                            _rawShoulder = s;
                            _rawElbow = e;
                            _rawWrist = w;
                            _rawIndex = idx;
                            _rawPinky = pky;
                            _rawOppShoulder = oppS;
                            _rawConfidence = conf;
                            _rawDepthRatio = dRatio;
                            _rawHandActive = handAct;

                            if (handAct && data.Length >= 336)
                            {
                                int baseOff = 84;
                                for (int i = 0; i < 21; i++)
                                {
                                    _rawHandRel[i] = ConvertRealSenseToUnityLocal(ReadVec3(data, baseOff + i * 12));
                                }
                            }

                            _hasDualArmPacket = hasDual;
                            if (hasDual)
                            {
                                _rawLeftShoulder = ls;
                                _rawLeftElbow = le;
                                _rawLeftWrist = lw;
                                _rawLeftIndex = lidx;
                                _rawLeftPinky = lpky;
                                _rawLeftConfidence = lConf;
                                _rawLeftHandActive = lHandAct;

                                if (lHandAct)
                                {
                                    int lBaseOff = 404;
                                    for (int i = 0; i < 21; i++)
                                    {
                                        _rawLeftHandRel[i] = ConvertRealSenseToUnityLocal(ReadVec3(data, lBaseOff + i * 12));
                                    }
                                }

                                if (data.Length >= 660)
                                {
                                    _rawPythonArmMode = Mathf.RoundToInt(BitConverter.ToSingle(data, 656));
                                }
                            }

                            _newPacketAvailable = true;
                        }
                    }
                }
            }
            catch (SocketException)
            {
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"[RealSenseArmReceiver] UDP Error: {ex.Message}");
            }
        }

        private static Vector3 ReadVec3(byte[] buf, int offset)
        {
            return new Vector3(
                BitConverter.ToSingle(buf, offset),
                BitConverter.ToSingle(buf, offset + 4),
                BitConverter.ToSingle(buf, offset + 8)
            );
        }

        private void OnDestroy()
        {
            _isRunning = false;
            _udpClient?.Close();
            if (_receiveThread != null && _receiveThread.IsAlive)
            {
                _receiveThread.Join(200);
            }
        }

        private void OnDrawGizmos()
        {
            if (!Application.isPlaying || !HasValidTracking) return;

            Gizmos.color = Color.Lerp(new Color(1f, 0.5f, 0f), Color.green, depthSensorValidRatio);
            Gizmos.DrawSphere(ShoulderPos, 0.025f);
            Gizmos.DrawSphere(ElbowPos, 0.025f);
            Gizmos.DrawSphere(WristPos, 0.02f);
            Gizmos.DrawLine(ShoulderPos, ElbowPos);
            Gizmos.DrawLine(ElbowPos, WristPos);

            if (leftTrackingConfidence > 0.30f)
            {
                Gizmos.color = Color.cyan;
                Gizmos.DrawSphere(LeftShoulderPos, 0.025f);
                Gizmos.DrawSphere(LeftElbowPos, 0.025f);
                Gizmos.DrawSphere(LeftWristPos, 0.02f);
                Gizmos.DrawLine(LeftShoulderPos, LeftElbowPos);
                Gizmos.DrawLine(LeftElbowPos, LeftWristPos);
            }
        }
    }
}
