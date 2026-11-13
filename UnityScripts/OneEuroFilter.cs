using System;
using UnityEngine;

namespace PsychophysicsVR.Tracking
{
    /// <summary>
    /// State-Informed 1-Euro Filter for Vector3 signals with Kinematic Velocity Gating & Prediction.
    /// Prevents sudden depth/tracking spikes during long arm reaches by bounding maximum
    /// frame-to-frame innovation against human biomechanics before computing derivatives.
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

            // State-informed kinematic innovation gate: prevent single-frame tracking glitches
            // during extended arm reaches from spiking the derivative!
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

            // Smoothly damp velocity lead prediction when arm is nearly fully extended or decelerating
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
