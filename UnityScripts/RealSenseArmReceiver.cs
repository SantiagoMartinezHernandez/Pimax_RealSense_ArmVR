using System;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using UnityEngine;

namespace PsychophysicsVR.Tracking
{
    /// <summary>
    /// Dual-Arm + Dual-21-Finger Receiver for Intel RealSense D455C + MediaPipe (v4.0).
    /// Receives both Right and Left arms + both 21-joint hands simultaneously in a single
    /// 656-byte UDP packet, applies kinematic-gated 1-Euro filtering, and anchors each arm
    /// to its respective VR shoulder socket.
    /// </summary>
    public class RealSenseArmReceiver : MonoBehaviour
    {
        [Header("Network Settings")]
        public int udpPort = 5055;

        [Header("Coordinate Calibration & Alignment")]
        public Transform avatarShoulderAnchor;
        public Transform leftAvatarShoulderAnchor;
        public Transform cameraOrientationReference;

        [Tooltip("Horizontal angle offset (degrees) of the physical D455C camera relative to the user.")]
        [Range(-180f, 180f)]
        public float cameraYawOffsetDeg = 0f;

        [Tooltip("Scale multiplier to match the participant's real arm length to the Unity avatar's arm length.")]
        public float armScale = 1.0f;

        [Tooltip("Mirror X axis (keep true when the D455C camera is facing the user).")]
        public bool mirrorX = true;

        [Header("Speed & State-Informed 1-Euro Filter Tuning")]
        [Range(0.5f, 8.0f)]
        public float filterMinCutoff = 2.2f;

        [Range(0.01f, 1.0f)]
        public float filterBeta = 0.22f;

        [Tooltip("Velocity Lead Prediction in milliseconds (10-15 ms gives snappy response without overshoot during long reaches).")]
        [Range(0f, 35f)]
        public float motionPredictionMs = 12f;

        [Header("Live Telemetry (Read-Only)")]
        [Range(0f, 1f)] public float trackingConfidence;
        [Range(0f, 1f)] public float leftTrackingConfidence;
        [Range(0f, 1f)] public float depthSensorValidRatio;
        public bool handFingersTracked;
        public bool leftHandFingersTracked;
        public float measuredRealArmLengthMeters;
        public int packetsReceivedCount;
        public int remotePythonArmMode = -1;

        // Right Arm filtered joint positions in Unity World Space
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

        // Un-rotated local vectors in Unity camera space (for calibration)
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

        // Right arm 1-Euro filters
        private OneEuroFilterVector3 _relElbowFilter;
        private OneEuroFilterVector3 _relWristFilter;
        private OneEuroFilterVector3 _relIndexFilter;
        private OneEuroFilterVector3 _relPinkyFilter;
        private OneEuroFilterVector3 _chestAxisFilter;
        private readonly OneEuroFilterVector3[] _fingerFilters = new OneEuroFilterVector3[21];

        // Left arm 1-Euro filters
        private OneEuroFilterVector3 _leftRelElbowFilter;
        private OneEuroFilterVector3 _leftRelWristFilter;
        private OneEuroFilterVector3 _leftRelIndexFilter;
        private OneEuroFilterVector3 _leftRelPinkyFilter;
        private readonly OneEuroFilterVector3[] _leftFingerFilters = new OneEuroFilterVector3[21];

        private UdpClient _udpClient;
        private Thread _receiveThread;
        private volatile bool _isRunning;

        private void Awake()
        {
            _relElbowFilter = new OneEuroFilterVector3(filterMinCutoff, filterBeta, 1.0f, 4.2f);
            _relWristFilter = new OneEuroFilterVector3(filterMinCutoff, filterBeta, 1.0f, 5.5f);
            _relIndexFilter = new OneEuroFilterVector3(filterMinCutoff, filterBeta, 1.0f, 6.0f);
            _relPinkyFilter = new OneEuroFilterVector3(filterMinCutoff, filterBeta, 1.0f, 6.0f);
            _chestAxisFilter = new OneEuroFilterVector3(1.2f, 0.04f, 1.0f, 2.0f);

            _leftRelElbowFilter = new OneEuroFilterVector3(filterMinCutoff, filterBeta, 1.0f, 4.2f);
            _leftRelWristFilter = new OneEuroFilterVector3(filterMinCutoff, filterBeta, 1.0f, 5.5f);
            _leftRelIndexFilter = new OneEuroFilterVector3(filterMinCutoff, filterBeta, 1.0f, 6.0f);
            _leftRelPinkyFilter = new OneEuroFilterVector3(filterMinCutoff, filterBeta, 1.0f, 6.0f);

            for (int i = 0; i < 21; i++)
            {
                _fingerFilters[i] = new OneEuroFilterVector3(3.0f, 0.30f, 1.0f, 4.0f);
                _leftFingerFilters[i] = new OneEuroFilterVector3(3.0f, 0.30f, 1.0f, 4.0f);
            }

            if (avatarShoulderAnchor == null)
                avatarShoulderAnchor = this.transform;
            if (cameraOrientationReference == null)
                cameraOrientationReference = this.transform;
        }

        private void Start()
        {
            StartUdpReceiver();
        }

        private void OnValidate()
        {
            UpdateFilterParams();
        }

        private void UpdateFilterParams()
        {
            if (_relElbowFilter != null)
            {
                _relElbowFilter.minCutoff = _relWristFilter.minCutoff = _relIndexFilter.minCutoff = _relPinkyFilter.minCutoff = filterMinCutoff;
                _relElbowFilter.beta = _relWristFilter.beta = _relIndexFilter.beta = _relPinkyFilter.beta = filterBeta;

                _leftRelElbowFilter.minCutoff = _leftRelWristFilter.minCutoff = _leftRelIndexFilter.minCutoff = _leftRelPinkyFilter.minCutoff = filterMinCutoff;
                _leftRelElbowFilter.beta = _leftRelWristFilter.beta = _leftRelIndexFilter.beta = _leftRelPinkyFilter.beta = filterBeta;
            }
        }

        private Vector3 ConvertRealSenseToUnityLocal(Vector3 rsPt)
        {
            float x = mirrorX ? -rsPt.x : rsPt.x;
            float y = -rsPt.y;
            float z = mirrorX ? -rsPt.z : rsPt.z;
            return new Vector3(x, y, z);
        }

        public void CalibrateArmScaleToAvatar(float avatarTotalArmLength)
        {
            if (measuredRealArmLengthMeters > 0.25f && avatarTotalArmLength > 0.1f)
            {
                armScale = Mathf.Clamp(avatarTotalArmLength / measuredRealArmLengthMeters, 0.5f, 2.0f);
                Debug.Log($"[RealSenseArmReceiver] Calibrated armScale = {armScale:F3} " +
                          $"(Real D455C arm = {measuredRealArmLengthMeters:F3}m, Avatar arm = {avatarTotalArmLength:F3}m)");
            }
        }

        private void Update()
        {
            UpdateFilterParams();

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

            Quaternion baseRot = cameraOrientationReference != null ? cameraOrientationReference.rotation : Quaternion.identity;
            Quaternion yawRot = Quaternion.Euler(0f, cameraYawOffsetDeg, 0f) * baseRot;
            Vector3 worldRightShoulder = avatarShoulderAnchor != null ? avatarShoulderAnchor.position : transform.position;
            Vector3 worldLeftShoulder = leftAvatarShoulderAnchor != null ? leftAvatarShoulderAnchor.position : worldRightShoulder + new Vector3(-0.34f, 0f, 0f);

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
                        0.12f
                    );
                }

                float now = Time.unscaledTime;
                float predSec = motionPredictionMs * 0.001f;

                // --- 1. Process Right Arm ---
                Vector3 filtRelElbow = _relElbowFilter.Filter(e - s, now, predSec);
                Vector3 filtRelWrist = _relWristFilter.Filter(w - s, now, predSec);
                Vector3 filtRelIndex = _relIndexFilter.Filter(idx - s, now, predSec * 0.5f);
                Vector3 filtRelPinky = _relPinkyFilter.Filter(pky - s, now, predSec * 0.5f);

                LocalShoulderToWrist = filtRelWrist;
                if (oppS.sqrMagnitude > 0.01f)
                {
                    LocalOppositeToActiveShoulder = _chestAxisFilter.Filter(s - oppS, now, 0f);
                }

                ShoulderPos = worldRightShoulder;
                ElbowPos = worldRightShoulder + yawRot * (filtRelElbow * armScale);
                WristPos = worldRightShoulder + yawRot * (filtRelWrist * armScale);
                IndexPos = worldRightShoulder + yawRot * (filtRelIndex * armScale);
                PinkyPos = worldRightShoulder + yawRot * (filtRelPinky * armScale);

                if (handActive)
                {
                    lock (_lock)
                    {
                        for (int i = 0; i < 21; i++)
                        {
                            Vector3 filtRelFinger = _fingerFilters[i].Filter(_rawHandRel[i], now, predSec * 0.3f);
                            HandLandmarksWorld[i] = WristPos + yawRot * filtRelFinger;
                        }
                    }
                    IndexPos = HandLandmarksWorld[5];
                    PinkyPos = HandLandmarksWorld[17];
                }

                // --- 2. Process Left Arm ---
                if (hasDual)
                {
                    Vector3 lFiltRelElbow = _leftRelElbowFilter.Filter(le - ls, now, predSec);
                    Vector3 lFiltRelWrist = _leftRelWristFilter.Filter(lw - ls, now, predSec);
                    Vector3 lFiltRelIndex = _leftRelIndexFilter.Filter(lidx - ls, now, predSec * 0.5f);
                    Vector3 lFiltRelPinky = _leftRelPinkyFilter.Filter(lpky - ls, now, predSec * 0.5f);

                    LocalLeftShoulderToWrist = lFiltRelWrist;

                    LeftShoulderPos = worldLeftShoulder;
                    LeftElbowPos = worldLeftShoulder + yawRot * (lFiltRelElbow * armScale);
                    LeftWristPos = worldLeftShoulder + yawRot * (lFiltRelWrist * armScale);
                    LeftIndexPos = worldLeftShoulder + yawRot * (lFiltRelIndex * armScale);
                    LeftPinkyPos = worldLeftShoulder + yawRot * (lFiltRelPinky * armScale);

                    if (lHandActive)
                    {
                        lock (_lock)
                        {
                            for (int i = 0; i < 21; i++)
                            {
                                Vector3 lFiltRelFinger = _leftFingerFilters[i].Filter(_rawLeftHandRel[i], now, predSec * 0.3f);
                                LeftHandLandmarksWorld[i] = LeftWristPos + yawRot * lFiltRelFinger;
                            }
                        }
                        LeftIndexPos = LeftHandLandmarksWorld[5];
                        LeftPinkyPos = LeftHandLandmarksWorld[17];
                    }
                }
            }
            else
            {
                if (avatarShoulderAnchor != null)
                {
                    Vector3 delta = worldRightShoulder - ShoulderPos;
                    ShoulderPos += delta;
                    ElbowPos += delta;
                    WristPos += delta;
                    IndexPos += delta;
                    PinkyPos += delta;
                    for (int i = 0; i < 21; i++)
                        HandLandmarksWorld[i] += delta;
                }
                if (leftAvatarShoulderAnchor != null)
                {
                    Vector3 lDelta = worldLeftShoulder - LeftShoulderPos;
                    LeftShoulderPos += lDelta;
                    LeftElbowPos += lDelta;
                    LeftWristPos += lDelta;
                    LeftIndexPos += lDelta;
                    LeftPinkyPos += lDelta;
                    for (int i = 0; i < 21; i++)
                        LeftHandLandmarksWorld[i] += lDelta;
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
                            // Floats 84..100 (byte offset 336..400)
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
                                    int lBaseOff = 404; // Float 101 * 4 bytes
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
