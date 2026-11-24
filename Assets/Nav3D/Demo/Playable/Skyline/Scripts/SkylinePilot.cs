using Nav3D.API;
using System;
using UnityEngine;

namespace Nav3D.Demo
{
    //The player's helicopter as an order taker: a designation becomes a take-off followed by the order, a flight to the clicked
    //point or to a pad's hover point, then the landing. The scenario decides what a click means and speaks for what happens;
    //the standing order and the pad claim live here, as the courier's do on its own helicopter
    public class SkylinePilot : MonoBehaviour
    {
        #region Nested types

        //what a designation became, for the scenario to say
        public enum Verdict
        {
            REFUSED_LANDING,
            REFUSED_TAKING_OFF,
            LIFTING_OFF,
            FLYING_TO_PAD,
            FLYING_TO_POINT,
        }

        #endregion

        #region Serialized fields

        //the Nav3DAgent on the same object flies the route
        [SerializeField] SkylineHelicopter m_Helicopter;

        [SerializeField] SkylineLandingPad m_HomePad;

        #endregion

        #region Attributes

        Nav3DAgent m_Agent;

        Vector3?          m_Target;
        SkylineLandingPad m_TargetPad;

        //a designation that arrived while landed waits for the take-off to finish
        bool              m_HasPendingOrder;
        Vector3           m_PendingPoint;
        SkylineLandingPad m_PendingPad;

        #endregion

        #region Events

        public event Action                    Airborne;
        public event Action<SkylineLandingPad> Landing;
        public event Action                    Landed;
        public event Action<PathfindingResult> OrderFailed;

        #endregion

        #region Properties

        public SkylineHelicopter Helicopter => m_Helicopter;

        public SkylineLandingPad TargetPad => m_TargetPad;

        public Vector3? Target => m_Target;

        #endregion

        #region Unity events

        void Awake()
        {
            m_Agent = GetComponent<Nav3DAgent>();
        }

        void Start()
        {
            m_Helicopter.RestOn(m_HomePad);
        }

        #endregion

        #region Public methods

        //refused while landing or lifting off; from the deck it becomes a take-off followed by the order
        public Verdict Designate(Vector3 _Point, SkylineLandingPad _Pad)
        {
            switch (m_Helicopter.State)
            {
                case SkylineHelicopter.FlightState.LANDING:
                    return Verdict.REFUSED_LANDING;

                case SkylineHelicopter.FlightState.TAKING_OFF:
                    return Verdict.REFUSED_TAKING_OFF;

                case SkylineHelicopter.FlightState.LANDED:
                    m_HasPendingOrder = true;
                    m_PendingPoint    = _Point;
                    m_PendingPad      = _Pad;

                    m_Helicopter.TakeOff(OnAirborne);

                    return Verdict.LIFTING_OFF;

                default:
                    Order(_Point, _Pad);

                    return _Pad != null ? Verdict.FLYING_TO_PAD : Verdict.FLYING_TO_POINT;
            }
        }

        #endregion

        #region Service methods

        void OnAirborne()
        {
            m_Helicopter.ReleasePad();

            Airborne?.Invoke();

            if (!m_HasPendingOrder)
                return;

            m_HasPendingOrder = false;

            Order(m_PendingPoint, m_PendingPad);
        }

        //a new designation replaces the standing one outright; nothing is queued
        void Order(Vector3 _Point, SkylineLandingPad _Pad)
        {
            //leaving a pad's approach for somewhere else: the claim goes with it
            if (m_TargetPad != null && m_TargetPad != _Pad)
                m_TargetPad.Release(m_Helicopter);

            m_TargetPad = _Pad;

            if (_Pad != null)
            {
                //the player always wins a pad: a courier holding this claim sees it change and re-targets
                _Pad.Claim(m_Helicopter);

                m_Target = _Pad.HoverPoint;

                m_Agent.MoveTo(
                        _Pad.HoverPoint,
                        _OnReach: () => OnHoverReached(_Pad),
                        _OnPathfindingFail: OnOrderFailed,
                        _ReachDistance: m_Helicopter.HoverReachDistance,
                        _InterruptCurrentOrderImmediate: true);

                return;
            }

            m_Target = _Point;

            m_Agent.MoveTo(
                    _Point,
                    _OnReach: OnPointReached,
                    _OnPathfindingFail: OnOrderFailed,
                    _InterruptCurrentOrderImmediate: true);
        }

        void OnPointReached()
        {
            m_Target = null;
        }

        //the Nav3D order is done at the hover point; the descent is the helicopter's own leg
        void OnHoverReached(SkylineLandingPad _Pad)
        {
            //a later designation may have replaced this approach before its callback landed
            if (m_TargetPad != _Pad)
                return;

            m_Agent.Stop();

            Landing?.Invoke(_Pad);

            m_Helicopter.Land(_Pad, OnLanded);
        }

        void OnLanded()
        {
            m_Target    = null;
            m_TargetPad = null;

            Landed?.Invoke();
        }

        //nothing is retried on the player's behalf
        void OnOrderFailed(PathfindingResult _Result)
        {
            if (m_TargetPad != null)
            {
                m_TargetPad.Release(m_Helicopter);

                m_TargetPad = null;
            }

            m_Target = null;

            OrderFailed?.Invoke(_Result);
        }

        #endregion
    }
}
