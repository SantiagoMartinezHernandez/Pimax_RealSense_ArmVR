using System;
using UnityEngine;

namespace PsychophysicsVR.Tracking
{
    /// <summary>
    /// Smooth C-infinity biomechanical barrier functions to prevent hard clamp jumps
    /// when joints approach anatomical limits.
    /// </summary>
    public static class BiomechanicalMath
    {
        public static float SoftPlus(float x, float sharpness = 25f)
        {
            float sx = sharpness * x;
            if (sx > 20f) return x;
            if (sx < -20f) return 0f;
            return Mathf.Log(1f + Mathf.Exp(sx)) / sharpness;
        }

        public static float SoftMax(float x, float minVal, float sharpness = 25f)
        {
            return minVal + SoftPlus(x - minVal, sharpness);
        }

        public static float SoftMin(float x, float maxVal, float sharpness = 25f)
        {
            return maxVal - SoftPlus(maxVal - x, sharpness);
        }
    }

    /// <summary>
    /// Continuous-Discrete Constant-Velocity 3D Kalman Filter.
    /// - Predict(dt) runs on every 90/120 Hz VR render frame to eliminate 30-60 Hz sample-and-hold staircase judder.
    /// - UpdateMeasurement(z, confidence) runs when UDP packets arrive, using adaptive confidence noise R
    ///   and Huber innovation clamping to reject single-frame depth/landmark outliers.
    /// </summary>
    [Serializable]
    public class KalmanFilter3D
    {
        public float processNoisePos = 0.0005f;
        public float processNoiseVel = 2.8f;
        public float measurementNoiseBase = 0.00025f;
        public float velocityDampingRate = 3.5f;
        public float huberInnovationRadius = 0.08f;

        private KalmanAxis _kx, _ky, _kz;
        public Vector3 Position => new Vector3(_kx.p, _ky.p, _kz.p);
        public Vector3 Velocity => new Vector3(_kx.v, _ky.v, _kz.v);
        public bool Initialized { get; private set; }

        public KalmanFilter3D(
            float processNoiseVel = 2.8f,
            float measurementNoiseBase = 0.00025f,
            float velocityDampingRate = 3.5f,
            float huberInnovationRadius = 0.08f)
        {
            this.processNoiseVel = processNoiseVel;
            this.measurementNoiseBase = measurementNoiseBase;
            this.velocityDampingRate = velocityDampingRate;
            this.huberInnovationRadius = huberInnovationRadius;
            Reset();
        }

        public void Reset()
        {
            _kx = new KalmanAxis();
            _ky = new KalmanAxis();
            _kz = new KalmanAxis();
            Initialized = false;
        }

        public Vector3 Predict(float dt)
        {
            if (!Initialized) return Vector3.zero;
            dt = Mathf.Clamp(dt, 0.0005f, 0.05f);
            float drag = Mathf.Exp(-velocityDampingRate * dt);

            _kx.Predict(dt, drag, processNoisePos, processNoiseVel);
            _ky.Predict(dt, drag, processNoisePos, processNoiseVel);
            _kz.Predict(dt, drag, processNoisePos, processNoiseVel);
            return Position;
        }

        public void UpdateMeasurement(Vector3 z, float confidence)
        {
            if (!Initialized)
            {
                _kx.Init(z.x);
                _ky.Init(z.y);
                _kz.Init(z.z);
                Initialized = true;
                return;
            }

            float conf = Mathf.Clamp(confidence, 0.1f, 1.0f);
            float r = measurementNoiseBase / (conf * conf);

            // Huber-clamp the 3D innovation vector so a single glitch packet cannot kick the velocity state
            Vector3 innov = z - Position;
            float mag = innov.magnitude;
            if (mag > huberInnovationRadius && huberInnovationRadius > 0.0001f)
            {
                z = Position + innov * (huberInnovationRadius / mag);
            }

            _kx.Update(z.x, r);
            _ky.Update(z.y, r);
            _kz.Update(z.z, r);
        }

        private struct KalmanAxis
        {
            public float p, v;
            public float p00, p01, p11;

            public void Init(float val)
            {
                p = val;
                v = 0f;
                p00 = 0.01f;
                p01 = 0f;
                p11 = 0.25f;
            }

            public void Predict(float dt, float drag, float qPos, float qVel)
            {
                p += v * dt;
                v *= drag;

                float newP00 = p00 + 2f * dt * p01 + dt * dt * p11 + qPos * dt;
                float newP01 = drag * (p01 + dt * p11);
                float newP11 = drag * drag * p11 + qVel * dt;

                p00 = newP00;
                p01 = newP01;
                p11 = newP11;
            }

            public void Update(float z, float r)
            {
                float s = p00 + r;
                if (s < 1e-7f) return;

                float k0 = p00 / s;
                float k1 = p01 / s;
                float y = z - p;

                p += k0 * y;
                v += k1 * y;

                float oldP00 = p00;
                float oldP01 = p01;
                p00 = Mathf.Max(1e-6f, (1f - k0) * oldP00);
                p01 = (1f - k0) * oldP01;
                p11 = Mathf.Max(1e-6f, p11 - k1 * oldP01);
            }
        }
    }

    /// <summary>
    /// Critically Damped Second-Order (CDSO) PD Spring-Damper for 3D Vectors (zeta = 1.0).
    /// Closed-form exact analytical step: zero overshoot, finite acceleration, absorbs high-frequency noise.
    /// </summary>
    [Serializable]
    public class CriticallyDampedSpring3D
    {
        public Vector3 Value { get; private set; }
        public Vector3 Velocity { get; private set; }
        public bool Initialized { get; private set; }

        public void Reset()
        {
            Value = Vector3.zero;
            Velocity = Vector3.zero;
            Initialized = false;
        }

        public void Snap(Vector3 value)
        {
            Value = value;
            Velocity = Vector3.zero;
            Initialized = true;
        }

        public Vector3 Step(Vector3 target, float dt, float naturalFreqHz = 4.8f)
        {
            if (!Initialized)
            {
                Snap(target);
                return Value;
            }

            dt = Mathf.Clamp(dt, 0.0005f, 0.05f);
            float omega = 2f * Mathf.PI * Mathf.Max(0.5f, naturalFreqHz);
            float expTerm = Mathf.Exp(-omega * dt);

            Vector3 delta = Value - target;
            Vector3 temp = (Velocity + omega * delta) * dt;

            Value = target + expTerm * (delta + temp);
            Velocity = expTerm * (Velocity - omega * temp);
            return Value;
        }
    }

    /// <summary>
    /// Critically Damped Second-Order (CDSO) PD Controller for Quaternions (zeta = 1.0).
    /// </summary>
    [Serializable]
    public class CriticallyDampedQuaternion
    {
        public Quaternion Value { get; private set; } = Quaternion.identity;
        public Vector3 AngularVelocity { get; private set; }
        public bool Initialized { get; private set; }

        public void Reset()
        {
            Value = Quaternion.identity;
            AngularVelocity = Vector3.zero;
            Initialized = false;
        }

        public void Snap(Quaternion q)
        {
            Value = q;
            AngularVelocity = Vector3.zero;
            Initialized = true;
        }

        public Quaternion Step(Quaternion target, float dt, float naturalFreqHz = 5.5f)
        {
            if (!Initialized)
            {
                Snap(target);
                return Value;
            }

            dt = Mathf.Clamp(dt, 0.0005f, 0.05f);
            if (Quaternion.Dot(Value, target) < 0f)
            {
                target = new Quaternion(-target.x, -target.y, -target.z, -target.w);
            }

            Quaternion errQ = Value * Quaternion.Inverse(target);
            errQ.ToAngleAxis(out float angleDeg, out Vector3 axis);
            if (angleDeg > 180f) angleDeg -= 360f;

            Vector3 delta = (axis.sqrMagnitude > 1e-6f && Mathf.Abs(angleDeg) > 1e-4f)
                ? axis.normalized * (angleDeg * Mathf.Deg2Rad)
                : Vector3.zero;

            float omega = 2f * Mathf.PI * Mathf.Max(0.5f, naturalFreqHz);
            float expTerm = Mathf.Exp(-omega * dt);
            Vector3 temp = (AngularVelocity + omega * delta) * dt;

            Vector3 nextDelta = expTerm * (delta + temp);
            AngularVelocity = expTerm * (AngularVelocity - omega * temp);

            float nextDeg = nextDelta.magnitude * Mathf.Rad2Deg;
            if (nextDeg > 1e-4f)
            {
                Value = (Quaternion.AngleAxis(nextDeg, nextDelta.normalized) * target).normalized;
            }
            else
            {
                Value = target;
            }
            return Value;
        }
    }

    /// <summary>
    /// State-Informed 1-Euro Filter for Vector3 signals (retained for finger & auxiliary signals).
    /// </summary>
    [Serializable]
    public class OneEuroFilterVector3
    {
        public float minCutoff = 2.4f;
        public float beta = 0.25f;
        public float dCutoff = 1.0f;
        public float maxSpeedMetersPerSec = 5.5f;

        private LowPassFilter _x, _y, _z;
        private LowPassFilter _dx, _dy, _dz;
        private float _lastTime;
        private bool _initialized;

        public Vector3 LastVelocity { get; private set; }
        public Vector3 LastFilteredValue => new Vector3(_x.LastValue, _y.LastValue, _z.LastValue);

        public OneEuroFilterVector3(float minCutoff = 2.4f, float beta = 0.25f, float dCutoff = 1.0f, float maxSpeed = 5.5f)
        {
            this.minCutoff = minCutoff;
            this.beta = beta;
            this.dCutoff = dCutoff;
            this.maxSpeedMetersPerSec = maxSpeed;
            Reset();
        }

        public void Reset()
        {
            _x = new LowPassFilter();
            _y = new LowPassFilter();
            _z = new LowPassFilter();
            _dx = new LowPassFilter();
            _dy = new LowPassFilter();
            _dz = new LowPassFilter();
            _lastTime = -1f;
            _initialized = false;
            LastVelocity = Vector3.zero;
        }

        public Vector3 Filter(Vector3 value, float timestamp, float predictionSeconds = 0f)
        {
            if (!_initialized || timestamp <= _lastTime)
            {
                _initialized = true;
                _lastTime = timestamp;
                return new Vector3(
                    _x.Filter(value.x, 1f),
                    _y.Filter(value.y, 1f),
                    _z.Filter(value.z, 1f)
                );
            }

            float dt = Mathf.Clamp(timestamp - _lastTime, 0.001f, 0.2f);
            _lastTime = timestamp;
            float rate = 1.0f / dt;

            Vector3 prev = LastFilteredValue;
            Vector3 expected = prev + LastVelocity * (dt * 0.5f);
            Vector3 innovation = value - expected;
            float maxStep = maxSpeedMetersPerSec * dt;
            if (innovation.magnitude > maxStep && maxStep > 0.0001f)
            {
                value = expected + innovation.normalized * maxStep;
            }

            float dx = (value.x - _x.LastValue) * rate;
            float dy = (value.y - _y.LastValue) * rate;
            float dz = (value.z - _z.LastValue) * rate;

            float alphaD = Alpha(rate, dCutoff);
            float edx = _dx.Filter(dx, alphaD);
            float edy = _dy.Filter(dy, alphaD);
            float edz = _dz.Filter(dz, alphaD);

            LastVelocity = new Vector3(edx, edy, edz);

            float cutoffX = minCutoff + beta * Mathf.Abs(edx);
            float cutoffY = minCutoff + beta * Mathf.Abs(edy);
            float cutoffZ = minCutoff + beta * Mathf.Abs(edz);

            Vector3 filtered = new Vector3(
                _x.Filter(value.x, Alpha(rate, cutoffX)),
                _y.Filter(value.y, Alpha(rate, cutoffY)),
                _z.Filter(value.z, Alpha(rate, cutoffZ))
            );

            if (predictionSeconds > 0.0001f)
            {
                Vector3 lead = Vector3.ClampMagnitude(LastVelocity * predictionSeconds, 0.035f);
                filtered += lead;
            }

            return filtered;
        }

        private static float Alpha(float rate, float cutoff)
        {
            float tau = 1.0f / (2.0f * Mathf.PI * cutoff);
            float te = 1.0f / rate;
            return 1.0f / (1.0f + tau / te);
        }

        private class LowPassFilter
        {
            public float LastValue { get; private set; }
            private bool _hasValue;

            public float Filter(float value, float alpha)
            {
                if (!_hasValue)
                {
                    _hasValue = true;
                    LastValue = value;
                    return value;
                }
                LastValue = alpha * value + (1.0f - alpha) * LastValue;
                return LastValue;
            }
        }
    }
}
