using UnityEngine;

namespace Nav3D.Common
{
    /// <summary>
    /// Position-prescribed tween: distance = progressCurve(normalizedTime) * totalLength, giving an exact
    /// arrival time regardless of curve shape (a zero-slope curve region cannot stall the run, since
    /// normalized time keeps advancing independently of it). CurrentSpeed is derived (delta distance /
    /// deltaTime) since the profile itself is driven by time, not speed.
    /// </summary>
    public sealed class EasedSpeedProfile : ISpeedProfile
    {
        #region Constants

        //resolution of the progress-curve inversion in Seek: the curve is sampled this many times over
        //[0,1] and the bracketing pair is interpolated. 64 puts the worst-case seek error near half a
        //percent of a lap for a well-behaved ease curve, at a cost paid once per seek, never per frame
        const int SEEK_SAMPLES = 64;

        #endregion

        #region Attributes

        float m_NormalizedTime;
        int   m_Direction = 1;
        float m_Distance;

        #endregion

        #region Properties

        public float CurrentSpeed { get; private set; }

        #endregion

        #region Public methods

        public void Reset(float _StartDistance)
        {
            //a start offset needs the curve to be inverted, which Reset has no access to: the owner
            //applies it by calling Seek right after Reset (Spline3DMover.ResetMotionState does)
            m_NormalizedTime = 0f;
            m_Direction      = 1;
            m_Distance       = 0f;
            CurrentSpeed     = 0f;
        }

        public void SetDistance(float _Distance)
        {
            //inert by design: this profile is driven by a normalized-time clock, so the next Advance
            //recomputes distance from that clock and overwrites anything written here. Use Seek to move
            //the clock itself — that is what an external jump has to do in a time-prescribed profile
            m_Distance = _Distance;
        }

        /// <summary>
        /// Moves the profile's clock to the point whose prescribed distance is nearest
        /// <paramref name="_Distance"/> — the seek an external position jump needs in a time-prescribed
        /// profile (<see cref="SetDistance"/> alone cannot survive the next Advance). Inverts the
        /// progress curve by sampling, so it works for any curve shape; a non-monotonic curve resolves
        /// to its closest sample. The motion then CONTINUES from there at that point's curve speed
        /// instead of restarting the tween.
        /// </summary>
        public void Seek(float _Distance, AnimationCurve _ProgressCurve, float _TotalLength)
        {
            if (_ProgressCurve == null || _TotalLength <= 0f)
            {
                SetDistance(_Distance);

                return;
            }

            m_NormalizedTime = InvertProgressCurve(Mathf.Clamp01(_Distance / _TotalLength), _ProgressCurve);
            m_Distance       = _ProgressCurve.Evaluate(m_NormalizedTime) * _TotalLength;
            CurrentSpeed     = 0f;
        }

        public float Advance(float _DeltaTime, float _Duration, AnimationCurve _ProgressCurve, float _TotalLength,
            MotionEndBehavior _EndBehavior, out bool _Finished)
        {
            float oldDistance = m_Distance;

            m_NormalizedTime += _DeltaTime / Mathf.Max(_Duration, 0.001f) * m_Direction;

            _Finished = false;

            switch (_EndBehavior)
            {
                case MotionEndBehavior.STOP:
                    if (m_NormalizedTime >= 1f)
                    {
                        m_NormalizedTime = 1f;
                        m_Distance       = _ProgressCurve.Evaluate(1f) * _TotalLength;
                        _Finished        = true;

                        UpdateCurrentSpeed(oldDistance, _DeltaTime);

                        return m_Distance;
                    }

                    break;

                case MotionEndBehavior.LOOP:
                    m_NormalizedTime = Mathf.Repeat(m_NormalizedTime, 1f);
                    break;

                case MotionEndBehavior.PING_PONG:
                    while (m_NormalizedTime > 1f || m_NormalizedTime < 0f)
                    {
                        m_NormalizedTime = m_NormalizedTime > 1f ? 2f - m_NormalizedTime : -m_NormalizedTime;
                        m_Direction      = -m_Direction;
                    }

                    break;
            }

            m_Distance = _ProgressCurve.Evaluate(m_NormalizedTime) * _TotalLength;

            UpdateCurrentSpeed(oldDistance, _DeltaTime);

            return m_Distance;
        }

        #endregion

        #region Service methods

        //smallest normalized time whose curve value reaches _TargetProgress; falls back to the closest
        //sample when the curve never crosses it (non-monotonic or out-of-range curves)
        static float InvertProgressCurve(float _TargetProgress, AnimationCurve _ProgressCurve)
        {
            float previousValue = _ProgressCurve.Evaluate(0f);
            float closestTime   = 0f;
            float closestError  = Mathf.Abs(previousValue - _TargetProgress);

            for (int i = 1; i <= SEEK_SAMPLES; i++)
            {
                float time  = (float) i / SEEK_SAMPLES;
                float value = _ProgressCurve.Evaluate(time);
                float error = Mathf.Abs(value - _TargetProgress);

                if (error < closestError)
                {
                    closestError = error;
                    closestTime  = time;
                }

                //the target lies inside this sample pair: interpolate within it and stop at the first
                //crossing, so a ping-pong-shaped curve seeks to its outbound leg
                if ((previousValue - _TargetProgress) * (value - _TargetProgress) <= 0f && !Mathf.Approximately(previousValue, value))
                    return Mathf.Lerp((float) (i - 1) / SEEK_SAMPLES, time, (_TargetProgress - previousValue) / (value - previousValue));

                previousValue = value;
            }

            return closestTime;
        }

        void UpdateCurrentSpeed(float _OldDistance, float _DeltaTime)
        {
            CurrentSpeed = _DeltaTime > 1e-6f ? (m_Distance - _OldDistance) / _DeltaTime : 0f;
        }

        #endregion
    }
}
