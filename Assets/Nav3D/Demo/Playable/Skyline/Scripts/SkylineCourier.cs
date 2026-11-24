using Nav3D.API;
using System.Collections.Generic;
using UnityEngine;

namespace Nav3D.Demo
{
    //The courier, a second agent flying its own errands: a free pad, the hover point, land, wait, lift off,
    //the next.
    //Shows an order chain and re-tasking mid-order: the player always wins a pad, so a courier whose claim is taken chooses another.
    public class SkylineCourier : MonoBehaviour
    {
        #region Serialized fields

        [SerializeField] SkylineScenario   m_Scenario;
        [SerializeField] SkylineHelicopter m_Helicopter;

        [SerializeField] SkylineLandingPad m_HomePad;

        //absolute; each cruise point is raised to the hover height of a taller pad beneath it — a cruise point inside a tower
        //is an intermediate target in occupied volume, and the whole order is refused
        [SerializeField] float m_CruiseAltitude = 100f;

        //how long the courier rests on a deck before its next errand
        [SerializeField] float m_DeckDwellSeconds = 4f;

        //a refused route: wait this long, then try another pad rather than hammering the same one
        [SerializeField] float m_RetrySeconds = 2f;

        //a leg shorter than this is no leg: the point is dropped from the order (see BeginErrand)
        [SerializeField] float m_DegenerateLegDistance = 0.5f;

        #endregion

        #region Attributes

        readonly List<SkylineLandingPad> m_FreePads = new List<SkylineLandingPad>();

        Nav3DAgent        m_Agent;
        SkylineLandingPad m_TargetPad;

        float m_WaitUntil;
        bool  m_Waiting;

        int    m_Errands;
        int    m_Yields;
        int    m_Refusals;
        string m_LastResult = "—";

        #endregion

        #region Properties

        public int Errands => m_Errands;

        //how many times the player took the pad this courier was heading for and it chose another
        public int Yields => m_Yields;

        public SkylineLandingPad TargetPad => m_TargetPad;

        //errands the search refused
        public int Refusals => m_Refusals;

        public string StatusLine
        {
            get
            {
                if (m_Helicopter == null)
                    return "—";

                string target = m_TargetPad != null ? $" → {m_TargetPad.name}" : string.Empty;

                return $"{m_Helicopter.State}{target}  ({m_Errands} errands, {m_Yields} yielded, last {m_LastResult})";
            }
        }

        #endregion

        #region Unity events

        void Start()
        {
            m_Agent = m_Helicopter.GetComponent<Nav3DAgent>();

            m_Helicopter.RestOn(m_HomePad);

            //the first errand leaves the deck once the world exists
            m_Waiting   = true;
            m_WaitUntil = 0f;
        }

        void Update()
        {
            if (!m_Scenario.WorldReady)
                return;

            //the player took the pad this errand was heading for: choose another, mid-order
            if (m_TargetPad != null && m_TargetPad.Claimant != m_Helicopter && m_Helicopter.State == SkylineHelicopter.FlightState.FLYING)
            {
                m_Yields++;

                m_TargetPad = null;

                BeginErrand();

                return;
            }

            if (m_Waiting && Time.time >= m_WaitUntil)
            {
                m_Waiting = false;

                Depart();
            }
        }

        #endregion

        #region Service methods

        //from a deck: lift off first, the errand starts at the top; from the air: straight away
        void Depart()
        {
            if (m_Helicopter.State == SkylineHelicopter.FlightState.LANDED)
            {
                m_Helicopter.TakeOff(() =>
                {
                    m_Helicopter.ReleasePad();

                    BeginErrand();
                });

                return;
            }

            if (m_Helicopter.State == SkylineHelicopter.FlightState.FLYING)
                BeginErrand();
        }

        void BeginErrand()
        {
            SkylineLandingPad pad = PickFreePad();

            if (pad == null)
            {
                //every pad claimed (the player holds one, the rest are none) — try again shortly
                Wait(m_RetrySeconds);

                return;
            }

            pad.Claim(m_Helicopter);

            m_TargetPad = pad;

            Vector3 here = m_Helicopter.transform.position;

            //three legs of one order: up to the cruise band, across, down to the hover point. A leg that would not move the
            //courier is dropped: a pad above the cruise band puts the climb point where the courier already is, and two coinciding
            //points in one order are a duplicate the path optimizer reports as an error
            float climbAltitude   = Mathf.Max(m_CruiseAltitude, here.y);
            float descentAltitude = Mathf.Max(m_CruiseAltitude, pad.HoverPoint.y);

            List<Vector3> legs = new List<Vector3>(3);

            AppendLeg(legs, here, new Vector3(here.x, climbAltitude, here.z));
            AppendLeg(legs, here, new Vector3(pad.HoverPoint.x, descentAltitude, pad.HoverPoint.z));
            AppendLeg(legs, here, pad.HoverPoint);

            //no leg left: the courier already hovers over the pad it picked, the errand is the descent alone
            if (legs.Count == 0)
            {
                OnHoverReached(pad);

                return;
            }

            m_Agent.MoveToPoints(
                    legs.ToArray(),
                    _ReachDistance: m_Helicopter.HoverReachDistance,
                    _OnFinished: () => OnHoverReached(pad),
                    _OnPathfindingFail: OnErrandFailed,
                    _InterruptCurrentOrderImmediate: true);
        }

        //a point is a leg only if it is somewhere else than the previous one (the courier's own place for the first)
        void AppendLeg(List<Vector3> _Legs, Vector3 _Here, Vector3 _Point)
        {
            Vector3 previous = _Legs.Count > 0 ? _Legs[_Legs.Count - 1] : _Here;

            if (Vector3.Distance(previous, _Point) < m_DegenerateLegDistance)
                return;

            _Legs.Add(_Point);
        }

        void OnHoverReached(SkylineLandingPad _Pad)
        {
            //superseded by a re-target before this callback landed
            if (m_TargetPad != _Pad)
                return;

            m_LastResult = nameof(PathfindingResultCode.SUCCEEDED);

            m_Agent.Stop();

            m_Helicopter.Land(_Pad, () =>
            {
                m_Errands++;
                m_TargetPad = null;

                Wait(m_DeckDwellSeconds);
            });
        }

        void OnErrandFailed(PathfindingResult _Result)
        {
            m_LastResult = _Result.Result.ToString();
            m_Refusals++;

            Debug.Log($"[{nameof(SkylineCourier)}] errand to {(m_TargetPad != null ? m_TargetPad.name : "?")} refused: {_Result.Result}");

            if (m_TargetPad != null)
            {
                m_TargetPad.Release(m_Helicopter);

                m_TargetPad = null;
            }

            Wait(m_RetrySeconds);
        }

        void Wait(float _Seconds)
        {
            m_Waiting   = true;
            m_WaitUntil = Time.time + _Seconds;
        }

        //random among the free pads: a deterministic pick would make the errands a fixed loop
        SkylineLandingPad PickFreePad()
        {
            m_FreePads.Clear();

            foreach (SkylineLandingPad pad in m_Scenario.Pads)
            {
                if (pad != null && !pad.IsClaimed && pad != m_Helicopter.Pad)
                    m_FreePads.Add(pad);
            }

            return m_FreePads.Count > 0 ? m_FreePads[Random.Range(0, m_FreePads.Count)] : null;
        }

        #endregion
    }
}
