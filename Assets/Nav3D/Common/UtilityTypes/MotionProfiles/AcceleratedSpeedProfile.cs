using UnityEngine;

namespace Nav3D.Common
{
    /// <summary>
    /// Trapezoidal kinematics: accelerate, cruise, brake into the ends. The brake envelope
    /// (v <= sqrt(2 * deceleration * remaining)) guarantees the end is reached exactly; a speed floor prevents
    /// the classic asymptotic-creep never-arrives problem. Braking is skipped when looping.
    /// </summary>
    public sealed class AcceleratedSpeedProfile : ISpeedProfile
    {
        #region Constants

        //minimal speed floor while braking, guarantees the ends are actually reached
        const float BRAKE_SPEED_FLOOR = 0.05f;

        #endregion

        #region Attributes

        float m_Distance;
        int   m_Direction = 1;
        float m_ScalarVelocity;

        #endregion

        #region Properties

        public float CurrentSpeed { get; private set; }

        #endregion

        #region Public methods

        public void Reset(float _StartDistance)
        {
            m_Distance       = _StartDistance;
            m_Direction      = 1;
            m_ScalarVelocity = 0f;
            CurrentSpeed     = 0f;
        }

        public void SetDistance(float _Distance)
        {
            m_Distance = _Distance;
        }

        /// <summary>
        /// Overrides the current scalar velocity (e.g. to preserve an entry speed across run restarts).
        /// </summary>
        public void SetScalarVelocity(float _Velocity)
        {
            m_ScalarVelocity = Mathf.Max(0f, _Velocity);
            CurrentSpeed     = m_ScalarVelocity;
        }

        public float Advance(
                float             _DeltaTime,
                float             _Acceleration,
                float             _Deceleration,
                float             _MaxSpeed,
                float             _TotalLength,
                MotionEndBehavior _EndBehavior,
                out bool          _Finished
            )
        {
            float remaining = m_Direction > 0 ? _TotalLength - m_Distance : m_Distance;

            m_ScalarVelocity += _Acceleration * _DeltaTime;
            m_ScalarVelocity =  Mathf.Min(m_ScalarVelocity, _MaxSpeed);

            if (_EndBehavior != MotionEndBehavior.LOOP)
            {
                float brakeLimit = Mathf.Sqrt(2f * _Deceleration * Mathf.Max(remaining, 0f));
                m_ScalarVelocity = Mathf.Min(m_ScalarVelocity, Mathf.Max(brakeLimit, BRAKE_SPEED_FLOOR));
            }

            CurrentSpeed = m_ScalarVelocity;

            m_Distance += m_ScalarVelocity * _DeltaTime * m_Direction;

            _Finished = RailBounds.Resolve(ref m_Distance, ref m_Direction, _TotalLength, _EndBehavior);

            return m_Distance;
        }

        #endregion
    }
}