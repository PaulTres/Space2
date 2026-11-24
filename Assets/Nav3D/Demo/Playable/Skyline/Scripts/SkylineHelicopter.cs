using System;
using UnityEngine;

namespace Nav3D.Demo
{
    //A helicopter's body and its two scripted flight legs.
    //The Nav3DAgent on the same object flies the route; bank, nose dip and rotors derive from the transform's velocity. The eased
    //descent onto the deck and the climb back are flown here: a deck point lies on the boundary of occupied volume, and an agent would arrive at full speed.
    public class SkylineHelicopter : MonoBehaviour
    {
        #region Nested types

        public enum FlightState
        {
            FLYING,
            LANDING,
            LANDED,
            TAKING_OFF,
        }

        #endregion

        #region Serialized fields

        //the child that banks and pitches; the root is never tilted
        [SerializeField] Transform m_Body;
        [SerializeField] Transform m_MainRotor;
        [SerializeField] Transform m_TailRotor;

        [SerializeField] float m_MaxBankAngle       = 28f;
        [SerializeField] float m_MaxPitchAngle      = 12f;
        [SerializeField] float m_BankPerYawRate     = 0.25f;   //degrees of bank per degree/second of heading change
        [SerializeField] float m_PitchPerAccel      = 2.5f;    //degrees of pitch per u/s² of forward acceleration
        [SerializeField] float m_AttitudeSmoothing  = 5f;
        [SerializeField] float m_RotorSpinSpeed     = 1800f;   //degrees per second at full spin
        [SerializeField] float m_VerticalSpeed      = 4f;      //u/s for the scripted descent and climb
        [SerializeField] float m_RotorSpinUpSeconds = 1.2f;

        //below this speed the heading is left alone: a hovering body has no direction to face
        [SerializeField] float m_MinHeadingSpeed = 0.25f;
        [SerializeField] float m_IdleRotorFactor = 0.15f;

        //how close the agent must come to a pad's hover point before the descent takes over
        [SerializeField] float m_HoverReachDistance = 1.5f;

        #endregion

        #region Attributes

        FlightState m_State = FlightState.FLYING;

        SkylineLandingPad m_Pad;
        Action            m_OnLegFinished;

        //velocity estimated from the transform, because the agent owns the motion
        Vector3 m_LastPosition;
        Vector3 m_Velocity;
        float   m_LastHeading;

        float m_Bank;
        float m_Pitch;
        float m_RotorFactor = 1f;

        Vector3 m_LegFrom;
        Vector3 m_LegTo;
        float   m_LegDuration;
        float   m_LegElapsed;

        #endregion

        #region Properties

        public FlightState State => m_State;

        //the owner must not issue orders while a scripted leg owns the transform
        public bool Busy => m_State == FlightState.LANDING || m_State == FlightState.TAKING_OFF;

        public SkylineLandingPad Pad => m_Pad;

        public float HoverReachDistance => m_HoverReachDistance;

        #endregion

        #region Unity events

        void Start()
        {
            m_LastPosition = transform.position;
            m_LastHeading  = transform.eulerAngles.y;
        }

        void Update()
        {
            float dt = Time.deltaTime;

            if (dt <= 0f)
                return;

            if (Busy)
                AdvanceLeg(dt);

            //the agent (or the scripted leg) owns the motion: the look is derived from where the transform went this frame
            Vector3 position     = transform.position;
            Vector3 velocity     = (position - m_LastPosition) / dt;
            Vector3 acceleration = (velocity - m_Velocity) / dt;

            m_LastPosition = position;
            m_Velocity     = velocity;

            float heading = FaceHeading(velocity);
            float yawRate = Mathf.DeltaAngle(m_LastHeading, heading) / dt;

            m_LastHeading = heading;

            Lean(yawRate, Vector3.Dot(acceleration, transform.forward), Busy || m_State == FlightState.LANDED, dt);
            SpinRotors(dt);
        }

        #endregion

        #region Public methods

        //from wherever the transform is now; the caller has already stopped the agent at the hover point
        public void Land(SkylineLandingPad _Pad, Action _OnLanded)
        {
            m_Pad           = _Pad;
            m_OnLegFinished = _OnLanded;

            BeginLeg(transform.position, _Pad.TouchdownPoint, FlightState.LANDING);
        }

        //_OnAirborne fires at the hover point, where the next Nav3D order starts
        public void TakeOff(Action _OnAirborne)
        {
            if (m_Pad == null)
                return;

            m_OnLegFinished = _OnAirborne;

            BeginLeg(transform.position, m_Pad.HoverPoint, FlightState.TAKING_OFF);
        }

        public void RestOn(SkylineLandingPad _Pad)
        {
            m_Pad         = _Pad;
            m_State       = FlightState.LANDED;
            m_RotorFactor = m_IdleRotorFactor;

            transform.position = _Pad.TouchdownPoint;

            _Pad.Claim(this);
        }

        //Back to the air without a pad: the owner's approach failed, or it re-targeted mid-approach
        public void ReleasePad()
        {
            if (m_Pad != null)
                m_Pad.Release(this);

            m_Pad = null;
        }

        #endregion

        #region Service methods

        void BeginLeg(Vector3 _From, Vector3 _To, FlightState _State)
        {
            m_LegFrom     = _From;
            m_LegTo       = _To;
            m_LegDuration = Mathf.Max(Vector3.Distance(_From, _To) / Mathf.Max(m_VerticalSpeed, 0.01f), m_RotorSpinUpSeconds);
            m_LegElapsed  = 0f;
            m_State       = _State;
        }

        void AdvanceLeg(float _Dt)
        {
            m_LegElapsed += _Dt;

            float t = Mathf.Clamp01(m_LegElapsed / m_LegDuration);

            //ease both ends: a helicopter neither drops onto a deck nor leaps off it
            float eased = t * t * (3f - 2f * t);

            transform.position = Vector3.Lerp(m_LegFrom, m_LegTo, eased);

            if (t < 1f)
                return;

            FlightState finished = m_State;

            m_State = finished == FlightState.LANDING ? FlightState.LANDED : FlightState.FLYING;

            Action callback = m_OnLegFinished;

            m_OnLegFinished = null;

            callback?.Invoke();
        }

        //the agent turns the root too while an order runs and the two agree (both follow the velocity); this matters on the
        //scripted legs and in the last metres of an arrival. Returns the heading the root has now
        float FaceHeading(Vector3 _Velocity)
        {
            Vector3 flat = new Vector3(_Velocity.x, 0f, _Velocity.z);

            //a hovering body has no direction to face
            if (flat.magnitude >= m_MinHeadingSpeed)
                transform.rotation = Quaternion.LookRotation(flat, Vector3.up);

            return transform.eulerAngles.y;
        }

        //_Level: on a pad and on the scripted legs the body hangs straight
        void Lean(float _YawRate, float _ForwardAcceleration, bool _Level, float _Dt)
        {
            //a right turn (positive yaw rate) rolls the body right (negative Z)
            float targetBank = _Level ? 0f : Mathf.Clamp(-_YawRate * m_BankPerYawRate, -m_MaxBankAngle, m_MaxBankAngle);

            //nose down when gaining speed, up when shedding it
            float targetPitch = _Level ? 0f : Mathf.Clamp(_ForwardAcceleration * m_PitchPerAccel, -m_MaxPitchAngle, m_MaxPitchAngle);

            float k = 1f - Mathf.Exp(-m_AttitudeSmoothing * _Dt);

            m_Bank  = Mathf.Lerp(m_Bank, targetBank, k);
            m_Pitch = Mathf.Lerp(m_Pitch, targetPitch, k);

            m_Body.localRotation = Quaternion.Euler(m_Pitch, 0f, m_Bank);
        }

        void SpinRotors(float _Dt)
        {
            float targetFactor = m_State == FlightState.LANDED ? m_IdleRotorFactor : 1f;
            float rate         = 1f / Mathf.Max(m_RotorSpinUpSeconds, 0.01f);

            m_RotorFactor = Mathf.MoveTowards(m_RotorFactor, targetFactor, rate * _Dt);

            float degrees = m_RotorSpinSpeed * m_RotorFactor * _Dt;

            m_MainRotor.Rotate(0f, degrees, 0f, Space.Self);
            m_TailRotor.Rotate(degrees, 0f, 0f, Space.Self);
        }

        #endregion
    }
}
