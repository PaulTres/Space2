namespace Nav3D.Common
{
    /// <summary>
    /// Flat speed: distance advances by speed * deltaTime every frame. Constancy is exact because this
    /// operates in arc-length space.
    /// </summary>
    public sealed class ConstantSpeedProfile : ISpeedProfile
    {
        float m_Distance;
        int   m_Direction = 1;

        public float CurrentSpeed { get; private set; }

        public void Reset(float _StartDistance)
        {
            m_Distance  = _StartDistance;
            m_Direction = 1;
            CurrentSpeed = 0f;
        }

        public void SetDistance(float _Distance)
        {
            m_Distance = _Distance;
        }

        public float Advance(float _DeltaTime, float _Speed, float _TotalLength, MotionEndBehavior _EndBehavior, out bool _Finished)
        {
            CurrentSpeed = _Speed;

            m_Distance += _Speed * _DeltaTime * m_Direction;

            _Finished = RailBounds.Resolve(ref m_Distance, ref m_Direction, _TotalLength, _EndBehavior);

            return m_Distance;
        }
    }
}
