using Nav3D.LocalAvoidance.SupportingMath;
using Nav3D.Common;
using UnityEngine;
using System;
using System.Collections.Generic;

namespace Nav3D.Pathfinding
{
    public class PathFollowData
    {
        #region Constants

        readonly string LOG_CTOR       = $"{nameof(PathFollowData)}.ctor: {{0}}";
        readonly string LOG_INVALIDATE = $"{nameof(PathFollowData)}.{nameof(Invalidate)}: {{0}}";

        #endregion

        #region Attributes

        readonly Vector3[]       m_Path;
        readonly int[]           m_TargetIndices;
        readonly int[]           m_TargetSources;
        readonly Action<Vector3> m_OnTargetPassed;
        readonly Action          m_OnLastTargetReached;

        readonly float m_ReachDistance;
        readonly float m_ReachDistanceSqr;

        readonly Log m_Log;

        int     m_NextIndex;
        Vector3 m_NextPoint;
        int     m_NextTargetInTargetsIndex;
        Vector3 m_NextTarget;

        //cumulative arc length from the path start to each point (V5 speed-profile support)
        readonly float[] m_CumulativeLengths;

        #endregion

        #region Properties

        public bool IsValid { get; private set; } = true;

        // the clearance radius the polyline this data follows was certified with at hand-off
        //(0 = no promise) — the avoidance stage's certified-corridor gate reads it together with
        //GetDistToClosestOnPath
        public float CertifiedRadius { get; }

        //The input point the target being reported to the target-passed callback stands for (-1: none or unknown).
        //Valid while that callback runs.
        public int PassedTargetSource { get; private set; } = -1;

        internal bool HasSingleTargetSource
        {
            get
            {
                int first = -1;

                foreach (int source in m_TargetSources ?? Array.Empty<int>())
                {
                    if (source < 0)
                        continue;

                    if (first >= 0 && first != source)
                        return false;

                    first = source;
                }

                return first >= 0;
            }
        }

        /// <summary>
        /// Total arc length of the followed trajectory, world units.
        /// </summary>
        public float TotalLength => m_CumulativeLengths[^1];

        #endregion

        #region Constructors

        /// <summary>
        /// PathFollowData ctor.
        /// </summary>
        /// <param name="_Trajectory">The given path to follow.</param>
        /// <param name="_TargetIndices">The indices of the targets among all trajectory points.</param>
        /// <param name="_FollowerPoint">The initial position of the agent.</param>
        /// <param name="_ReachDist">Required distance from agent to target point to mark last as reached.</param>
        /// <param name="_OnTargetPassed">The action to execute when the target point passed.</param>
        /// <param name="_OnLastTargetReached">The action to execute when the last target point reached.</param>
        /// <param name="_Log">Log.</param>
        /// <param name="_CertifiedRadius"> clearance radius the trajectory is certified with (0 = none).</param>
        /// <param name="_TargetSources">For each target, the input point it stands for (-1 for an inserted one); null when unknown.</param>
        public PathFollowData(
            Vector3[]       _Trajectory,
            int[]           _TargetIndices,
            Vector3         _FollowerPoint,
            float           _ReachDist,
            Action<Vector3> _OnTargetPassed,
            Action          _OnLastTargetReached,
            Log             _Log,
            float           _CertifiedRadius = 0f,
            int[]           _TargetSources   = null)
        {
            CertifiedRadius = _CertifiedRadius;

            m_Log = _Log;
            m_Log?.WriteFormat(LOG_CTOR, GetHashCode());

            m_Path                = _Trajectory;
            m_TargetIndices       = _TargetIndices;
            m_TargetSources       = _TargetSources;
            m_OnTargetPassed      = _OnTargetPassed;
            m_OnLastTargetReached = _OnLastTargetReached;

            m_ReachDistance    = _ReachDist;
            m_ReachDistanceSqr = _ReachDist * _ReachDist;

            m_CumulativeLengths = new float[_Trajectory.Length];

            for (int i = 1; i < _Trajectory.Length; i++)
            {
                m_CumulativeLengths[i] = m_CumulativeLengths[i - 1] + (_Trajectory[i] - _Trajectory[i - 1]).magnitude;
            }

            GetClosestPoint(_FollowerPoint, _ReachDist, out m_NextPoint, out m_NextIndex, out m_NextTargetInTargetsIndex);

            m_NextTarget = m_Path[m_TargetIndices[m_NextTargetInTargetsIndex]];
        }

        #endregion

        #region Public methods

        public string GetNextTargetInfo()
        {
            return
                $"Next point index: {m_NextIndex}/{m_Path.Length - 1}, "                                    +
                $"next target index: {m_NextTargetInTargetsIndex}, "                                        +
                $"next target index on path: {m_TargetIndices[m_NextTargetInTargetsIndex]}, "               +
                $"next target position: {m_Path[m_TargetIndices[m_NextTargetInTargetsIndex]]}";
        }

        public Vector3 GetMovePoint(float _Speed, Vector3 _CurrentFollowerPosition)
        {
            m_NextPoint = m_Path[m_NextIndex];
            Vector3 curToNext      = m_NextPoint - _CurrentFollowerPosition;
            float   toNextDistance = curToNext.magnitude;

            void CheckTargetPoints()
            {
                if (m_NextTargetInTargetsIndex >= m_TargetIndices.Length || m_TargetIndices[m_NextTargetInTargetsIndex] >= m_NextIndex)
                    return;

                InvokeTargetPassed();

                if (m_NextTargetInTargetsIndex >= m_TargetIndices.Length - 1)
                    return;

                m_NextTargetInTargetsIndex++;
                m_NextTarget = m_Path[m_TargetIndices[m_NextTargetInTargetsIndex]];
            }

            while (toNextDistance <= _Speed + m_ReachDistance && m_NextIndex < m_Path.Length - 1)
            {
                _Speed                   -= toNextDistance;
                _CurrentFollowerPosition =  m_NextPoint;

                CheckTargetPoints();

                m_NextIndex++;
                m_NextPoint = m_Path[m_NextIndex];

                curToNext      = m_NextPoint - _CurrentFollowerPosition;
                toNextDistance = curToNext.magnitude;

                if (m_NextIndex == m_Path.Length - 1)
                    CheckTargetPoints();
            }

            //A target the cursor has already moved past must be certified even when the advance loop above does
            //not run again this tick — otherwise it stays "unpassed" for a whole leg, and a deviation re-plan
            //puts it back at the head of the request, sending the agent the way it came. Bounded by
            //construction: every firing call advances the cursor. The final target is deliberately excluded,
            //since it would not advance and its certification belongs to the arrival block below.
            while (m_NextTargetInTargetsIndex < m_TargetIndices.Length - 1 &&
                   m_TargetIndices[m_NextTargetInTargetsIndex] < m_NextIndex)
            {
                CheckTargetPoints();
            }

            //Has the last target been passed? The comparison is squared-against-SQUARED on both sides
            //. Several trailing targets can resolve to the final
            //trajectory index — a Loop request's closing point collapses onto it in the postprocessing dedupe —
            //and the cursor can never move past that index, so all of them are certified here rather than
            //requiring the cursor to reach each one.
            if (m_NextIndex                                            == m_Path.Length - 1 &&
                m_TargetIndices[m_NextTargetInTargetsIndex]            == m_NextIndex       &&
                (m_NextTarget - _CurrentFollowerPosition).sqrMagnitude <= m_ReachDistanceSqr)
            {
                while (m_NextTargetInTargetsIndex < m_TargetIndices.Length - 1)
                {
                    InvokeTargetPassed();

                    m_NextTargetInTargetsIndex++;
                    m_NextTarget = m_Path[m_TargetIndices[m_NextTargetInTargetsIndex]];
                }

                InvokeTargetPassed();
                m_OnLastTargetReached?.Invoke();
            }

            return m_NextPoint;
        }

        //Skips path points that became BLOCKED since the path was found and lands on the first free one ahead.
        //An obstacle appearing around an agent can leave the carrot inside a body, and following then commands
        //motion into it for as long as the point stays there. Deliberately bounded and conservative:
        //  - at most _MaxLookahead points, so a sealed corridor cannot wind the cursor to the path's end — a
        //    blocked route is the re-path layer's business;
        //  - the cursor moves only when a free point is found, so a failed scan changes nothing;
        //  - it never lands on or past the final index, so the ordinary advance still performs the final
        //    approach and fires the arrival certification. Targets the
        //    skip jumped over are caught up by the passed-target loop in GetMovePoint.
        //Returns true when the cursor moved.
        public bool TryAdvancePastBlockedPoints(Func<Vector3, bool> _IsBlocked, int _MaxLookahead)
        {
            if (m_NextIndex >= m_Path.Length - 1 || !_IsBlocked(m_Path[m_NextIndex]))
                return false;

            int limit = Mathf.Min(m_NextIndex + _MaxLookahead, m_Path.Length - 2);

            for (int index = m_NextIndex + 1; index <= limit; index++)
            {
                if (_IsBlocked(m_Path[index]))
                    continue;

                m_NextIndex = index;
                m_NextPoint = m_Path[index];

                return true;
            }

            return false;
        }

        public Vector3[] GetUnpassedTargets(bool _Loop)
        {
            List<Vector3> result = new List<Vector3>(m_TargetIndices.Length);

            for (int i = m_NextTargetInTargetsIndex; i < m_TargetIndices.Length; i++)
            {
                result.Add(m_Path[m_TargetIndices[i]]);
            }

            if (!_Loop)
                return result.ToArray();

            for (int i = 0; i < m_NextTargetInTargetsIndex; i++)
            {
                result.Add(m_Path[m_TargetIndices[i]]);
            }

            return result.ToArray();
        }

        /// <summary>
        /// Arc-length distance left to travel: from the follower position to the next path point, plus the
        /// path suffix beyond it. Uses the current follow index (i.e. the pre-tick state when called before
        /// GetMovePoint — the one-tick lag is negligible for speed profiling).
        /// </summary>
        public float GetRemainingDistance(Vector3 _CurrentFollowerPosition)
        {
            int nextIndex = Mathf.Min(m_NextIndex, m_Path.Length - 1);

            return (m_Path[nextIndex] - _CurrentFollowerPosition).magnitude + TotalLength - m_CumulativeLengths[nextIndex];
        }

        public void GetDistToClosestOnPath(Vector3 _Point, out Vector3 _Closest, out float _Magnitude)
        {
            int nextIndex = m_NextIndex;
            int preIndex;

            if (m_NextIndex == m_Path.Length)
                nextIndex--;

            preIndex = nextIndex - 1;

            if (nextIndex == 0)
            {
                _Closest   = m_Path[nextIndex];
                _Magnitude = (_Closest - _Point).magnitude;

                return;
            }

            Vector3 next = m_Path[nextIndex];
            Vector3 pre  = m_Path[preIndex];

            //The case when the path contains only two equal points
            if (next == pre)
            {
                _Closest   = next;
                _Magnitude = 0;

                return;
            }

            Vector3 closestOnLine = new Straight(next - pre, pre).ClosestPoint(_Point);

            float preMagn     = (pre           - _Point).magnitude;
            float nextMagn    = (next          - _Point).magnitude;
            float closestMagn = (closestOnLine - _Point).magnitude;

            if (preMagn < nextMagn && preMagn < closestMagn)
            {
                _Closest   = pre;
                _Magnitude = preMagn;

                return;
            }

            if (nextMagn < preMagn && nextMagn < closestMagn)
            {
                _Closest   = next;
                _Magnitude = nextMagn;

                return;
            }

            _Closest   = closestOnLine;
            _Magnitude = closestMagn;
        }

        public void RestartFollowing(Vector3 _FollowerPoint, float _ReachDist)
        {
            //the parameter was misnamed _ReachDistSqr pre-2.2 while every caller passed a plain
            //distance
            GetClosestPoint(_FollowerPoint, _ReachDist, out m_NextPoint, out m_NextIndex, out m_NextTargetInTargetsIndex);
            m_NextTarget = m_Path[m_TargetIndices[m_NextTargetInTargetsIndex]];
        }

        public void Invalidate()
        {
            m_Log?.WriteFormat(LOG_INVALIDATE, GetHashCode());

            IsValid = false;
        }

        #endregion

        #region Internal methods

        internal int[] GetUnpassedTargetSources(bool _Loop)
        {
            if (m_TargetSources == null)
                return null;

            int   count  = _Loop ? m_TargetSources.Length : m_TargetSources.Length - m_NextTargetInTargetsIndex;
            int[] result = new int[count];

            for (int i = 0; i < count; i++)
                result[i] = m_TargetSources[(m_NextTargetInTargetsIndex + i) % m_TargetSources.Length];

            return result;
        }

        #endregion

        #region Service methods

        /// <summary>
        /// Picks the initial follow cursor point on the path for the given follower position.
        /// </summary>
        /// <param name="_Point">Given point</param>
        /// <param name="_ReachDist">The distance to a point required to consider the point reached.</param>
        /// <param name="_ClosestPoint">The path point to chase first</param>
        /// <param name="_Index">Index of that point via path point indices</param>
        /// <param name="_NextTargetInTargetsIndex">Index of the next target via targets indices</param>
        void GetClosestPoint(
            Vector3     _Point,
            float       _ReachDist,
            out Vector3 _ClosestPoint,
            out int     _Index,
            out int     _NextTargetInTargetsIndex)
        {
            //callers pass a plain distance — square it for the sqrMagnitude comparison (the parameter
            //was misnamed _ReachDistSqr pre-2.2 while receiving unsquared values)
            float reachDistSqr = _ReachDist * _ReachDist;

            //if the start and the end of a path are in the neighborhood of the initial point, then we chase start point
            if ((m_Path[0] - _Point).sqrMagnitude < reachDistSqr && (m_Path[m_Path.Length - 1] - _Point).sqrMagnitude < reachDistSqr)
            {
                _ClosestPoint             = m_Path[0];
                _Index                    = 0;
                _NextTargetInTargetsIndex = 0;

                return;
            }

            //Project the follower onto the path SEGMENTS and chase the containing segment's forward endpoint.
            //Picking the closest vertex instead has no forward preference, and a re-path whose trajectory starts
            //at the request-time position — up to a metre or so behind a cruising agent — then makes the agent
            //physically turn around and chase that stale start.
            int   minSegmentIndex = 0;
            float minSqrMagn      = float.MaxValue;

            for (int i = 0; i < m_Path.Length - 1; i++)
            {
                Vector3 closestOnSegment = new Segment3(m_Path[i], m_Path[i + 1]).GetClosestPoint(_Point);
                float   curSqrMagn       = (closestOnSegment - _Point).sqrMagnitude;

                if (curSqrMagn < minSqrMagn)
                {
                    minSegmentIndex = i;
                    minSqrMagn      = curSqrMagn;
                }
            }

            //the rear endpoint remains the cursor start only while still within the reach radius: the very next
            //GetMovePoint advance consumes it and fires its target-pass callback (starting past it would drop
            //the pass event of a target the agent has effectively reached); anything farther behind is never chased
            int index = m_Path.Length == 1 || (m_Path[minSegmentIndex] - _Point).sqrMagnitude <= reachDistSqr
                ? minSegmentIndex
                : minSegmentIndex + 1;

            //A trajectory may carry vertices past the highest target index — a loop's deduped closing point can
            //leave trailing target slots unmatched — so the forward-endpoint cursor could land where no target
            //index >= cursor exists. Cap it at the highest target index: targets at or behind the cursor still
            //fire through the normal advance machinery.
            int maxTargetIndex = 0;

            foreach (int targetIndex in m_TargetIndices)
                maxTargetIndex = Mathf.Max(maxTargetIndex, targetIndex);

            index = Mathf.Min(index, maxTargetIndex);

            _NextTargetInTargetsIndex = m_TargetIndices.FindIndex(_TargetIndex => _TargetIndex >= index);

            _ClosestPoint = m_Path[index];
            _Index        = index;
        }

        void InvokeTargetPassed()
        {
            PassedTargetSource = m_TargetSources?[m_NextTargetInTargetsIndex] ?? -1;
            m_OnTargetPassed?.Invoke(m_Path[m_TargetIndices[m_NextTargetInTargetsIndex]]);
        }

        #endregion
    }
}
