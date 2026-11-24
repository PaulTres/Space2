using Nav3D.API;
using System;
using System.Collections.Generic;
using UnityEngine;

namespace Nav3D.Demo
{
    //The one agent of the scene and the orders it flies. A click gives it a destination, Shift + click adds waypoints
    //to it and closes them into a loop; left alone it shuttles between the two rings, so the scene is never a still picture.
    public class PathFollowingCourier : MonoBehaviour
    {
        #region Serialized fields

        [SerializeField] Nav3DAgent m_Agent;

        //the shuttle's two ends, and what a click-free scene flies between
        [SerializeField] Transform m_RingA;
        [SerializeField] Transform m_RingB;

        [Tooltip("The shuttle takes over this long after the last order you gave")]
        [SerializeField] float m_IdleTakeoverSeconds = 8f;

        [Tooltip("A beat at the destination before the next leg, so an arrival reads as an arrival")]
        [SerializeField] float m_ArrivalHoldSeconds = 1.4f;

        //a query that fails (a destination inside a column, a starved timeout) pauses the loop instead of ending it
        [SerializeField] float m_FailureHoldSeconds = 2f;

        [Tooltip("The most waypoints one route can hold")]
        [SerializeField] int m_MaxWaypoints = 8;

        [Tooltip("A passed point the agent reports within this distance of a waypoint counts as that waypoint")]
        [SerializeField] float m_WaypointMatchDistance = 2f;

        #endregion

        #region Attributes

        //the route: every waypoint of the standing order in the order it was given, kept whole while the order stands so the first
        //one can still be clicked to close the loop. Empty when the courier is waiting
        readonly List<Vector3> m_Route = new List<Vector3>();

        //the waypoint the courier is flying to now
        int  m_NextWaypoint;
        bool m_Looped;

        float m_ReleaseTime;
        float m_LastClickTime = float.NegativeInfinity;
        bool  m_ShuttleToB    = true;

        #endregion

        #region Properties

        //set by the scenario once Nav3D has initialized: an order before that has no navigation space to answer it
        public bool Armed { get; set; }

        public IReadOnlyList<Vector3> Route => m_Route;

        public bool HasOrder => m_Route.Count > 0;
        public bool Looped   => m_Looped;

        public int MaxWaypoints => m_MaxWaypoints;

        //an open route of two points or more can be closed into a loop
        public bool CanCloseLoop => !m_Looped && m_Route.Count >= 2;

        //Nav3DAgent.Config is the agent's own config once it has initialized — a copy taken from the wired asset — so
        //the Tab switch changes this courier and leaves the asset in Configs/ exactly as it was authored
        public bool SmoothPath
        {
            get => m_Agent.Config.SmoothPath;
            set => m_Agent.Config.SmoothPath = value;
        }

        #endregion

        #region Events

        public event Action<PathfindingResult> OrderFailed;

        #endregion

        #region Public methods

        //driven by the scenario, not by Unity: the frame order the scene needs is courier, then aim, then the line
        public void Tick()
        {
            if (!Armed || HasOrder || Time.time < m_ReleaseTime)
                return;

            //the hand keeps the courier as long as it keeps giving it somewhere to go
            if (Time.time < m_LastClickTime + m_IdleTakeoverSeconds)
                return;

            Order(m_ShuttleToB ? m_RingB.position : m_RingA.position);

            m_ShuttleToB = !m_ShuttleToB;
        }

        //a plain click: a new route of one point
        public void OrderByHand(Vector3 _Point)
        {
            m_LastClickTime = Time.time;

            Order(_Point);
        }

        //Shift + click: one more waypoint on the end of the standing route. A closed loop is a finished shape, so the point starts
        //a new route instead. False when the route is full
        public bool AppendByHand(Vector3 _Point)
        {
            if (!Armed)
                return false;

            m_LastClickTime = Time.time;

            if (m_Looped)
                ClearRoute();

            if (m_Route.Count >= m_MaxWaypoints)
                return false;

            m_Route.Add(_Point);

            Issue();

            return true;
        }

        //Shift + click on the first waypoint: the route is flown round and round from here on
        public void CloseLoopByHand()
        {
            if (!Armed || !CanCloseLoop)
                return;

            m_LastClickTime = Time.time;
            m_Looped        = true;

            Issue();
        }

        //the smoothing switch changes what the next query returns, so the standing order is asked again
        public void Requery()
        {
            if (HasOrder)
                Issue();
        }

        #endregion

        #region Service methods

        void Order(Vector3 _Point)
        {
            if (!Armed)
                return;

            ClearRoute();

            m_Route.Add(_Point);

            Issue();
        }

        void ClearRoute()
        {
            m_Route.Clear();

            m_NextWaypoint = 0;
            m_Looped       = false;
        }

        //The standing route goes to the agent as one order: MoveTo for a single point, MoveToPoints for more. An open route is
        //issued from the waypoint the courier is flying to, so a point added on the way does not send it back to the start; a
        //loop is issued as the same ring turned to begin at that waypoint
        void Issue()
        {
            if (m_Route.Count == 1)
            {
                m_Agent.MoveTo(
                        m_Route[0],
                        _OnReach: OnReach,
                        _OnPathfindingFail: OnFail,
                        _InterruptCurrentOrderImmediate: true);

                return;
            }

            int       count  = m_Looped ? m_Route.Count : m_Route.Count - m_NextWaypoint;
            Vector3[] points = new Vector3[count];

            for (int i = 0; i < count; i++)
                points[i] = m_Route[(m_NextWaypoint + i) % m_Route.Count];

            m_Agent.MoveToPoints(
                    points,
                    _Loop: m_Looped,
                    _OnTargetPassed: OnWaypointPassed,
                    _OnFinished: OnReach,
                    _OnPathfindingFail: OnFail,
                    _SkipUnpassableTargets: true,
                    _InterruptCurrentOrderImmediate: true);
        }

        //The agent reports the point it passed, not its number, and a re-planned loop can report one twice — so the waypoint is
        //found by where it is, never by counting
        void OnWaypointPassed(Vector3 _Point)
        {
            int   passed   = -1;
            float distance = m_WaypointMatchDistance;

            for (int i = 0; i < m_Route.Count; i++)
            {
                float toWaypoint = Vector3.Distance(m_Route[i], _Point);

                if (toWaypoint >= distance)
                    continue;

                passed   = i;
                distance = toWaypoint;
            }

            if (passed < 0)
                return;

            m_NextWaypoint = m_Looped ? (passed + 1) % m_Route.Count : Mathf.Min(passed + 1, m_Route.Count - 1);
        }

        void OnReach()
        {
            ClearRoute();

            m_ReleaseTime = Time.time + m_ArrivalHoldSeconds;
        }

        void OnFail(PathfindingResult _Result)
        {
            ClearRoute();

            m_ReleaseTime = Time.time + m_FailureHoldSeconds;

            OrderFailed?.Invoke(_Result);
        }

        #endregion
    }
}
