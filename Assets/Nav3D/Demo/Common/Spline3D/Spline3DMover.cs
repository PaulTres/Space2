using System;
using UnityEngine;
using Nav3D.Common;

namespace Nav3D.Demo
{
    /// <summary>Moves the object along a <see cref="Spline3D"/>. The on-rail modes (CONSTANT, EASED, ACCELERATED) keep it on the curve and differ
    /// only in how the distance advances (<see cref="ISpeedProfile"/>); FLIGHT leaves the rail and chases the spline under inertia and a lateral-G limit.</summary>
    public partial class Spline3DMover : MonoBehaviour
    {

        #region Nested types

        public enum MotionMode
        {
            //constant speed
            CONSTANT,

            //duration + progress curve tween (exact arrival time)
            EASED,

            //trapezoidal kinematics: accelerate, cruise, brake into the ends
            ACCELERATED,

            //free flight guided by the spline: inertia, drift, turn slowdown, banking
            FLIGHT
        }

        #endregion

        #region Constants

        //speed below which the flight arrival is considered complete
        const float FLIGHT_ARRIVE_SPEED = 0.1f;

        //carrot advance step, fraction of the lookahead distance
        const float CARROT_STEP_FACTOR = 0.2f;
        const int   CARROT_STEP_LIMIT  = 100;

        #endregion

        #region Serialized fields

        [SerializeField] Spline3D          m_Spline;
        [SerializeField] MotionMode        m_MotionMode  = MotionMode.CONSTANT;
        [SerializeField] MotionEndBehavior m_EndBehavior = MotionEndBehavior.LOOP;

        [SerializeField] bool m_PlayOnStart = true;

        //initial arc-length offset from the spline start (not applied to Eased mode)
        [SerializeField] float m_StartDistance;

        //rotation
        [SerializeField] bool  m_AlignToPath   = true;
        [SerializeField] float m_RotationSpeed = 360f;

        //Constant
        [SerializeField] float m_Speed = 3f;

        //Eased
        [SerializeField] float          m_Duration      = 5f;
        [SerializeField] AnimationCurve m_ProgressCurve = AnimationCurve.EaseInOut(0f, 0f, 1f, 1f);

        //Accelerated
        [SerializeField] float m_Acceleration = 2f;
        [SerializeField] float m_Deceleration = 2f;
        [SerializeField] float m_MaxSpeed     = 5f;

        //Flight
        [SerializeField] float m_FlightMaxSpeed = 6f;

        [SerializeField] float m_FlightAcceleration = 8f;

        //how far ahead along the curve the pursuit target stays
        [SerializeField] float m_Lookahead = 2f;

        //turn-G limit: bounds the turn rate and enforces slowdown in sharp turns
        [SerializeField] float m_MaxLateralAcceleration = 6f;

        //allowed swing-out from the path in corners: relaxes the anticipatory turn braking, world units
        [SerializeField] float m_MaxDeviation = 1f;

        //steer back toward the path when the actual distance from it exceeds MaxDeviation
        [SerializeField] bool  m_HardDeviationLimit;
        [SerializeField] bool  m_Banking      = true;
        [SerializeField] float m_MaxBankAngle = 45f;

        [SerializeField] float m_BankSmoothing = 4f;

        //Perlin-noise wandering of the pursuit target, world units (0 = off); makes insect-like motion
        [SerializeField] float m_Wander          = 0f;
        [SerializeField] float m_WanderFrequency = 1f;

        #endregion

        #region Attributes

        bool m_IsPlaying;

        //on-rail state - arc-length distance placement (ApplyRailTransform) plus facing, generic across modes
        float m_Distance;

        float m_LastDistance;

        //facing sign for path alignment, follows the actual movement direction
        float m_FacingSign = 1f;

        //on-rail speed/progress math, one per mode: Common/UtilityTypes/MotionProfiles/
        readonly ConstantSpeedProfile    m_ConstantProfile    = new ConstantSpeedProfile();
        readonly EasedSpeedProfile       m_EasedProfile       = new EasedSpeedProfile();
        readonly AcceleratedSpeedProfile m_AcceleratedProfile = new AcceleratedSpeedProfile();
        readonly FlightSpeedProfile      m_FlightSpeedProfile = new FlightSpeedProfile();

        //flight state - steering (position/heading), separate from the speed profile above
        Vector3 m_Velocity;
        float   m_CarrotDistance;
        int     m_CarrotDirection = 1;
        float   m_CurrentBank;
        float   m_WanderSeed;

        #endregion

        #region Events

        //fired when the Stop end behavior finishes the run; invoked from Update
        public event Action OnFinished;

        #endregion

        #region Properties

        public bool IsPlaying => m_IsPlaying;

        public Spline3D Spline
        {
            get => m_Spline;
            set => m_Spline = value;
        }

        /// <summary>Current arc-length distance along the spline (the carrot distance in Flight mode).</summary>
        public float Distance
        {
            get => m_MotionMode == MotionMode.FLIGHT ? m_CarrotDistance : m_Distance;
            set
            {
                m_Distance       = value;
                m_CarrotDistance = value;

                //the on-rail profiles share one distance, so a jump applies to whichever is or becomes active
                m_ConstantProfile.SetDistance(value);
                m_AcceleratedProfile.SetDistance(value);

                //the eased profile runs on a normalized-time clock and would overwrite a written distance on its next Advance;
                //seeking the clock (curve inversion) is what lets a mid-course jump continue from there
                SeekEasedProfile(value);
            }
        }

        /// <summary>Normalized progress along the spline.</summary>
        public float Progress
        {
            get
            {
                if (m_Spline == null || m_Spline.TotalLength <= 0f)
                    return 0f;

                return Mathf.Clamp01(Distance / m_Spline.TotalLength);
            }
        }

        public float CurrentSpeed
        {
            get
            {
                return m_MotionMode switch
                {
                    MotionMode.CONSTANT => m_Speed,
                    MotionMode.ACCELERATED => m_AcceleratedProfile.CurrentSpeed,
                    MotionMode.FLIGHT => m_Velocity.magnitude,
                    _ => 0f
                };
            }
        }

        /// <summary>Configuration, not live switching: set it while stopped, before Play(). Switching mid-motion is unsupported by design.</summary>
        public MotionMode Mode
        {
            get => m_MotionMode;
            set => m_MotionMode = value;
        }

        /// <summary>End behavior at the spline bounds. Configuration like <see cref="Mode"/>: set it before Play().</summary>
        public MotionEndBehavior EndBehavior
        {
            get => m_EndBehavior;
            set => m_EndBehavior = value;
        }

        /// <summary>Eased-mode lap duration, s. Exposed for runtime-created movers; read live each frame.</summary>
        public float Duration
        {
            get => m_Duration;
            set => m_Duration = Mathf.Max(value, 0.01f);
        }

        /// <summary>Arc-length offset applied on every reset (Start/Stop). Configuration like <see cref="Mode"/>. Honored by every on-rail mode, Eased included.</summary>
        public float StartDistance
        {
            get => m_StartDistance;
            set => m_StartDistance = Mathf.Max(value, 0f);
        }

        /// <summary>Best-effort upper bound on the mode's speed, for sanity checks. Not a guarantee for Eased: it has no speed cap,
        /// so its bound is a multiple of the average.</summary>
        public float NominalTopSpeed
        {
            get
            {
                return m_MotionMode switch
                {
                    MotionMode.CONSTANT => m_Speed,
                    MotionMode.ACCELERATED => m_MaxSpeed,
                    MotionMode.FLIGHT => m_FlightMaxSpeed,
                    MotionMode.EASED when m_Spline != null => 3f * m_Spline.TotalLength / Mathf.Max(m_Duration, 0.001f),
                    _ => float.PositiveInfinity
                };
            }
        }

        #endregion

        #region Public methods

        public void Play()
        {
            m_IsPlaying = true;
        }

        public void Pause()
        {
            m_IsPlaying = false;
        }

        /// <summary>Stops playback and resets the motion state to the start.</summary>
        public void Stop()
        {
            m_IsPlaying = false;
            ResetMotionState();
        }

        public void Restart()
        {
            Stop();
            Play();
        }

        #endregion

        #region Service methods

        void ResetMotionState()
        {
            m_Distance     = m_StartDistance;
            m_LastDistance = m_StartDistance;
            m_FacingSign   = 1f;

            m_ConstantProfile.Reset(m_StartDistance);
            m_EasedProfile.Reset(m_StartDistance);
            m_AcceleratedProfile.Reset(m_StartDistance);
            m_FlightSpeedProfile.Reset();

            //the eased profile cannot apply the offset itself (no access to the progress curve); seeking its clock here is what
            //makes StartDistance mean the same in every on-rail mode
            if (m_StartDistance > 0f)
                SeekEasedProfile(m_StartDistance);

            m_Velocity        = Vector3.zero;
            m_CarrotDistance  = m_StartDistance;
            m_CarrotDirection = 1;
            m_CurrentBank     = 0f;
        }

        //inverts the progress curve, which needs the curve and the spline length: data only the mover holds
        void SeekEasedProfile(float _Distance)
        {
            m_EasedProfile.Seek(_Distance, m_ProgressCurve, m_Spline != null ? m_Spline.TotalLength : 0f);
        }

        void Finish()
        {
            m_IsPlaying = false;
            OnFinished?.Invoke();
        }

        #region On-rail motion

        void UpdateOnRail(float _DeltaTime, float _Length)
        {
            bool  finished = false;
            float distance = m_Distance;

            switch (m_MotionMode)
            {
                case MotionMode.CONSTANT:
                    distance = m_ConstantProfile.Advance(_DeltaTime, m_Speed, _Length, m_EndBehavior, out finished);
                    break;

                case MotionMode.EASED:
                    distance = m_EasedProfile.Advance(_DeltaTime, m_Duration, m_ProgressCurve, _Length, m_EndBehavior, out finished);
                    break;

                case MotionMode.ACCELERATED:
                    distance = m_AcceleratedProfile.Advance(_DeltaTime, m_Acceleration, m_Deceleration, m_MaxSpeed, _Length, m_EndBehavior, out finished);
                    break;
            }

            m_Distance = distance;

            ApplyRailTransform(distance, _DeltaTime);

            //raised last: a handler may Stop(), Restart() or seek the mover, and this frame's pose is already placed
            if (finished)
                Finish();
        }

        void ApplyRailTransform(float _Distance, float _DeltaTime)
        {
            transform.position = m_Spline.GetPointAtDistance(_Distance);

            if (!m_AlignToPath)
            {
                m_LastDistance = _Distance;
                return;
            }

            //face where the object actually moves (roll-back and ping-pong flip the sign)
            float delta = _Distance - m_LastDistance;

            if (Mathf.Abs(delta) > 1e-6f)
                m_FacingSign = Mathf.Sign(delta);

            m_LastDistance = _Distance;

            Vector3 tangent = m_Spline.GetDirectionAtDistance(_Distance) * m_FacingSign;

            if (tangent.sqrMagnitude < 1e-6f)
                return;

            Quaternion targetRotation = Quaternion.LookRotation(tangent, Vector3.up);
            transform.rotation = Quaternion.RotateTowards(transform.rotation, targetRotation, m_RotationSpeed * _DeltaTime);
        }

        #endregion

        #region Flight motion

        void UpdateFlight(float _DeltaTime, float _Length)
        {
            Vector3 position = transform.position;

            //overshoot recovery: when the carrot is beyond the leash, rejoin the path at the nearest point
            //instead of flying back to the missed section
            float leash = Mathf.Max(2f * m_Lookahead, m_Lookahead + m_MaxDeviation);

            float carrotDistance  = m_CarrotDistance;
            int   carrotDirection = m_CarrotDirection;

            if ((m_Spline.GetPointAtDistance(carrotDistance) - position).sqrMagnitude > leash * leash)
                carrotDistance = m_Spline.GetClosestPointDistance(position);

            bool carrotAtEnd = AdvanceCarrot(m_Spline, position, _Length, m_Lookahead, m_EndBehavior, ref carrotDistance, ref carrotDirection);

            m_CarrotDistance  = carrotDistance;
            m_CarrotDirection = carrotDirection;

            Vector3 carrotPoint = m_Spline.GetPointAtDistance(carrotDistance);

            if (m_Wander > 0f)
                carrotPoint += GetWanderOffset();

            Vector3 toCarrot    = carrotPoint - position;
            float   carrotRange = toCarrot.magnitude;
            Vector3 pursuitDir  = carrotRange > 1e-4f ? toCarrot / carrotRange : transform.forward;

            //hard deviation limit: past MaxDeviation the steering blends toward the path, fully at 2x MaxDeviation
            if (m_HardDeviationLimit)
            {
                Vector3 nearestPoint = m_Spline.GetPointAtDistance(m_Spline.GetClosestPointDistance(position));
                Vector3 toPath       = nearestPoint - position;
                float   deviation    = toPath.magnitude;

                if (deviation > m_MaxDeviation)
                {
                    float correction = Mathf.Clamp01((deviation - m_MaxDeviation) / Mathf.Max(m_MaxDeviation, 0.01f));
                    pursuitDir = Vector3.Slerp(pursuitDir, toPath / deviation, correction).normalized;
                }
            }

            float   speed   = m_Velocity.magnitude;
            Vector3 heading = speed > 0.01f ? m_Velocity / speed : pursuitDir;

            //pure-pursuit arc curvature; the speed budget it caps into is FlightSpeedProfile's
            float turnAngle = Mathf.Acos(Mathf.Clamp(Vector3.Dot(heading, pursuitDir), -1f, 1f));
            float curvature = 2f * Mathf.Sin(turnAngle) / Mathf.Max(carrotRange, 0.01f);

            float? arrivalRemaining = m_EndBehavior == MotionEndBehavior.STOP && carrotAtEnd ? carrotRange : null;

            float newSpeed = m_FlightSpeedProfile.Advance(
                    _DeltaTime,
                    m_FlightMaxSpeed,
                    m_FlightAcceleration,
                    m_MaxLateralAcceleration,
                    m_MaxDeviation,
                    curvature,
                    arrivalRemaining
                );

            //turn rate is bounded by the same lateral acceleration: fast flight means wide turns
            float maxTurnRadians = m_MaxLateralAcceleration / Mathf.Max(speed, 0.5f) * _DeltaTime;

            Vector3 newHeading = Vector3.RotateTowards(heading, pursuitDir, maxTurnRadians, 0f).normalized;

            m_Velocity         =  newHeading * newSpeed;
            transform.position += m_Velocity * _DeltaTime;

            if (m_AlignToPath)
                ApplyFlightRotation(heading, newHeading, newSpeed, _DeltaTime);

            if (m_EndBehavior == MotionEndBehavior.STOP && carrotAtEnd && carrotRange < m_Lookahead * 0.25f && newSpeed <= FLIGHT_ARRIVE_SPEED)
            {
                transform.position = m_Spline.GetPointAtDistance(_Length);
                Finish();
            }
        }

        //keeps the pursuit target ~lookahead ahead of the object; returns true when it rests at the spline end
        static bool AdvanceCarrot(
                Spline3D          _Spline,
                Vector3           _Position,
                float             _Length,
                float             _Lookahead,
                MotionEndBehavior _EndBehavior,
                ref float         _CarrotDistance,
                ref int           _CarrotDirection
            )
        {
            float step         = _Lookahead * CARROT_STEP_FACTOR;
            float sqrLookahead = _Lookahead * _Lookahead;

            for (int i = 0; i < CARROT_STEP_LIMIT; i++)
            {
                Vector3 carrotPoint = _Spline.GetPointAtDistance(_CarrotDistance);

                if ((carrotPoint - _Position).sqrMagnitude >= sqrLookahead)
                    break;

                _CarrotDistance += step * _CarrotDirection;

                switch (_EndBehavior)
                {
                    case MotionEndBehavior.STOP:
                        if (_CarrotDistance >= _Length)
                        {
                            _CarrotDistance = _Length;
                            return true;
                        }

                        break;

                    case MotionEndBehavior.LOOP:
                        _CarrotDistance = Mathf.Repeat(_CarrotDistance, _Length);
                        break;

                    case MotionEndBehavior.PING_PONG:
                        if (_CarrotDistance > _Length || _CarrotDistance < 0f)
                        {
                            _CarrotDistance  = _CarrotDistance > _Length ? 2f * _Length - _CarrotDistance : -_CarrotDistance;
                            _CarrotDirection = -_CarrotDirection;
                        }

                        break;
                }
            }

            return _EndBehavior == MotionEndBehavior.STOP && _CarrotDistance >= _Length;
        }

        void ApplyFlightRotation(Vector3 _OldHeading, Vector3 _NewHeading, float _Speed, float _DeltaTime)
        {
            if (_NewHeading.sqrMagnitude < 1e-6f)
                return;

            float bankTarget = 0f;

            if (m_Banking && _DeltaTime > 0f)
            {
                //signed yaw rate approximation: positive = right turn
                float yawRate      = Vector3.Dot(Vector3.Cross(_OldHeading, _NewHeading), Vector3.up) / _DeltaTime;
                float lateralAccel = yawRate                                                          * _Speed;

                bankTarget = Mathf.Clamp(Mathf.Atan2(lateralAccel, 9.81f) * Mathf.Rad2Deg, -m_MaxBankAngle, m_MaxBankAngle);
            }

            m_CurrentBank = Mathf.Lerp(m_CurrentBank, bankTarget, 1f - Mathf.Exp(-m_BankSmoothing * _DeltaTime));

            Vector3 up = Quaternion.AngleAxis(-m_CurrentBank, _NewHeading) * Vector3.up;

            Quaternion targetRotation = Quaternion.LookRotation(_NewHeading, up);
            transform.rotation = Quaternion.RotateTowards(transform.rotation, targetRotation, m_RotationSpeed * _DeltaTime);
        }

        Vector3 GetWanderOffset()
        {
            float t = Time.time * m_WanderFrequency + m_WanderSeed;

            return new Vector3(
                    Mathf.PerlinNoise(t, 0.37f) - 0.5f,
                    Mathf.PerlinNoise(t, 7.13f) - 0.5f,
                    Mathf.PerlinNoise(t, 13.7f) - 0.5f
                ) * (2f * m_Wander);
        }

        #endregion

        #endregion

        #region Unity events

        void Start()
        {
            m_WanderSeed = (GetHashCode() & 0xFFFF) * 0.6180339f;

            ResetMotionState();

            if (m_PlayOnStart)
                Play();
        }

        void Update()
        {
            if (!m_IsPlaying || m_Spline == null || m_Spline.PointCount < 2)
                return;

            float length = m_Spline.TotalLength;

            if (length <= 0f)
                return;

            if (m_MotionMode == MotionMode.FLIGHT)
                UpdateFlight(Time.deltaTime, length);
            else
                UpdateOnRail(Time.deltaTime, length);
        }

        void OnValidate()
        {
            m_Speed                  = Mathf.Max(m_Speed, 0f);
            m_Duration               = Mathf.Max(m_Duration, 0.01f);
            m_Acceleration           = Mathf.Max(m_Acceleration, 0.01f);
            m_Deceleration           = Mathf.Max(m_Deceleration, 0.01f);
            m_MaxSpeed               = Mathf.Max(m_MaxSpeed, 0.01f);
            m_FlightMaxSpeed         = Mathf.Max(m_FlightMaxSpeed, 0.01f);
            m_FlightAcceleration     = Mathf.Max(m_FlightAcceleration, 0.01f);
            m_Lookahead              = Mathf.Max(m_Lookahead, 0.05f);
            m_MaxLateralAcceleration = Mathf.Max(m_MaxLateralAcceleration, 0.01f);
            m_MaxDeviation           = Mathf.Max(m_MaxDeviation, 0f);
            m_MaxBankAngle           = Mathf.Clamp(m_MaxBankAngle, 0f, 89f);
            m_BankSmoothing          = Mathf.Max(m_BankSmoothing, 0.01f);
            m_Wander                 = Mathf.Max(m_Wander, 0f);
            m_WanderFrequency        = Mathf.Max(m_WanderFrequency, 0f);
            m_RotationSpeed          = Mathf.Max(m_RotationSpeed, 0f);
            m_StartDistance          = Mathf.Max(m_StartDistance, 0f);
        }

        #endregion

    }
}