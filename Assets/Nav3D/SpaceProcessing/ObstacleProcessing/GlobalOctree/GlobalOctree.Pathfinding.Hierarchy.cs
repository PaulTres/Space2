using Nav3D.API;
using Nav3D.Common;
using Nav3D.Pathfinding;
using Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.Hierarchy;
using Leaf = Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.Elements.Leaf;
using System;
using System.Collections.Generic;
using System.Threading;
using UnityEngine;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree
{
    //Hierarchical pathfinding (roadmap A4): a request planned on the region map first and refined leg by leg — the
    //point lane (Phase 1) and the radius-aware lane (Phase 2, the same plan with the radius carried into every leg,
    //every doorway crossing capsule-tested and the route cache keyed by radius). A coarse A* (the same core, TNode = a doorway and the side of it) runs from the start's
    //region to the goal's over the doorways; each coarse edge is a route INSIDE one region — the start or goal to a
    //doorway, searched now and restricted to that region, or doorway to doorway, searched once and cached on the map.
    //The refined route is the legs concatenated. The restriction is the mechanism, not an optimisation: the flat search
    //spends the slack of an underestimating heuristic in the start room's skin wherever it starts, and a
    //leg that may not leave its region has nowhere to spend it. Anything that fails here — no map, one region, no
    //coarse route, a leg refused — falls back to the flat search, so the hierarchy can only ever cost time, never a
    //route.
    partial class GlobalOctree
    {
        #region Nested types

        //a route from a request's endpoint to one doorway of its region, searched for this request only
        sealed class EndpointLeg
        {
            public List<Vector3> Waypoints { get; }
            public float         Length    { get; }

            public EndpointLeg(List<Vector3> _Waypoints, float _Length)
            {
                Waypoints = _Waypoints;
                Length    = _Length;
            }
        }

        #endregion

        #region Nested types — plan accounting

        //how a plan attempt ended: PLANNED keeps the map armed (NavTree.NoteRegionPlan); for the dev counters
        //(GlobalOctree.Pathfinding.Hierarchy.DevCounters.cs) the price of asking on a world where the hierarchy cannot
        //pay is the time spent before REFUSED_BY_GUARD and NO_COARSE_ROUTE
        internal enum HierarchyPlanOutcome
        {
            NO_PLAN_CHEAP,        //no region, one region, no doorways, adjacent or a refused pair: answered from the map alone
            NO_COARSE_ROUTE,      //the coarse search found no route (its legs were searched)
            REFUSED_BY_GUARD,     //a plan was found and the detour guard refused it (its legs were searched)
            LEG_FAILED,           //a cached leg no longer held while the chain was assembled
            PLANNED,              //a plan was delivered
            COUNT
        }

        #endregion

        #region Constants

        //the coarse graph's start node; every other node is doorway * 2 + side (0 = on region A's side, 1 = region B's)
        const int COARSE_START_NODE = -1;

        //A plan is kept only where it can pay: HPA*'s suboptimality is the detour through the doorway points crossed, at
        //most twice a doorway's extent each (a route may cross the contact sheet anywhere; the plan crosses it at one
        //point), so a plan whose doorways sum to more than this share of its length (plus this allowance) is handed to
        //the flat search instead. TIGHT — hundreds of 6 u slots read as passages, crossings as rooms, and room-to-room
        //contacts as sheets 54 u tall — is where the plan came out 1.2–2.3× the flat route and saved nothing; TUNNEL's
        //two 7 u mouths on a 200 u route pass.
        const float PLAN_DETOUR_SHARE     = 0.1f;
        const float PLAN_DETOUR_ALLOWANCE = 8f;

        #endregion

        #region Attributes (static)

        //T of the region map, as a multiple of MaxAgentRadius: a passage the shell tiles wall to wall (narrower than
        //4r) has under 2r of clearance and falls out of every core. A per-world setting's default, not a constant
        //(the pavilion's rooms exist only at ~24r for its 0.5 u body).
        internal static float HierarchyCoreClearanceFactor { get; set; } = 2f;

        #endregion

        #region Service methods — the hierarchical attempt

        //Under the read lock the flat attempt already holds (the legs re-enter it recursively). _Radius 0 is the point
        //lane; above it every leg is the radius-aware attempt and the request's reposition flags apply to its own
        //endpoints only — a doorway point is an intermediate that must be met as it is, never dipped to. Null = no
        //plan here, the caller runs the flat search.
        List<Vector3> FindPathHierarchical(
                RegionMap         _Map,
                Vector3           _Start,
                Vector3           _GoalPoint,
                float             _Radius,
                bool              _RepositionStart,
                bool              _RepositionGoal,
                CancellationToken _CancellationTokenExternal,
                CancellationToken _CancellationTokenTimeout,
                Action<Leaf>      _OnLeafOpened
            )
        {
            DevHierarchyPlanBegin();

            List<Vector3> planned = FindPathHierarchicalCore(_Map, _Start, _GoalPoint, _Radius, _RepositionStart, _RepositionGoal, _CancellationTokenExternal, _CancellationTokenTimeout, _OnLeafOpened, out HierarchyPlanOutcome outcome);

            DevHierarchyPlanEnd(outcome);

            m_NavTree.NoteRegionPlan(outcome == HierarchyPlanOutcome.PLANNED);

            return planned;
        }

        //dev accounting of the plan's outcome and wall time, implemented in the [Nav3DDevOnly] DevCounters partial
        partial void DevHierarchyPlanBegin();
        partial void DevHierarchyPlanEnd(HierarchyPlanOutcome _Outcome);

        //a confined leg's endpoint resolution (both ends: the leaf lookups, the sealed-volume test, a nearest-free-point search
        //when an end lies in a non-traversable cell) — what re-resolving the ends on every leg costs
        partial void DevLegResolveBegin();
        partial void DevLegResolveEnd(bool _Searched);

        List<Vector3> FindPathHierarchicalCore(
                RegionMap                _Map,
                Vector3                  _Start,
                Vector3                  _GoalPoint,
                float                    _Radius,
                bool                     _RepositionStart,
                bool                     _RepositionGoal,
                CancellationToken        _CancellationTokenExternal,
                CancellationToken        _CancellationTokenTimeout,
                Action<Leaf>             _OnLeafOpened,
                out HierarchyPlanOutcome _Outcome
            )
        {
            _Outcome = HierarchyPlanOutcome.NO_PLAN_CHEAP;

            int startRegion = _Map.RegionOfLeafCentre(_Start);
            int goalRegion  = _Map.RegionOfLeafCentre(_GoalPoint);

            if (startRegion == 0 || goalRegion == 0 || startRegion == goalRegion)
                return null;

            RegionMap.Region start = _Map.Regions[startRegion];
            RegionMap.Region goal  = _Map.Regions[goalRegion];

            if (start.Doorways.Count == 0 || goal.Doorways.Count == 0)
                return null;

            //adjacent regions run the flat search, and a pair the guard below refused once is not planned
            //again on this map — where the hierarchy cannot pay, it must not keep charging
            if (_Map.AreAdjacent(startRegion, goalRegion) || _Map.IsPairRefused(startRegion, goalRegion))
                return null;

            Dictionary<int, EndpointLeg> startLegs = new Dictionary<int, EndpointLeg>();
            Dictionary<int, EndpointLeg> goalLegs  = new Dictionary<int, EndpointLeg>();
            Dictionary<int, bool>        passable  = new Dictionary<int, bool>();

            //the crossing itself, one cell to the next: the point lane tests the segment, the radius-aware lane the
            //capsule — and skips a doorway the field already knows is too narrow for the body
            bool DoorwayPassable(RegionMap.Doorway _Doorway)
            {
                if (passable.TryGetValue(_Doorway.Id, out bool known))
                    return known;

                bool blocked;

                if (_Radius <= 0f)
                    blocked = m_NavTree.SegmentIntersectOccupiedLeafInternal(new Segment3(_Doorway.PointInA, _Doorway.PointInB));
                else
                {
                    blocked = _Doorway.CrossingClearance < _Radius;

                    if (!blocked)
                        HierarchyVolumetricBlocked(_Doorway.PointInA, _Doorway.PointInB, _Radius, ref blocked);
                }

                passable[_Doorway.Id] = !blocked;

                return !blocked;
            }

            //fills the run's own edge buffer (GlobalAStar owns and reuses it)
            void Successors(int _Node, List<NavEdge<int>> edges)
            {
                if (_Node == COARSE_START_NODE)
                {
                    foreach (RegionMap.Doorway doorway in start.Doorways)
                    {
                        if (!DoorwayPassable(doorway))
                            continue;

                        EndpointLeg leg = EndpointLegTo(_Map, startLegs, doorway, _Start, doorway.PointIn(startRegion), startRegion, _Radius, _RepositionStart, false, _CancellationTokenExternal, _CancellationTokenTimeout, _OnLeafOpened);

                        if (leg != null)
                            edges.Add(NavEdge<int>.ToNode(CoarseNode(doorway, startRegion), leg.Length, false));
                    }

                    return;
                }

                RegionMap.Doorway at   = _Map.Doorways[_Node >> 1];
                int               here = (_Node & 1) == 0 ? at.RegionA : at.RegionB;
                int               next = at.OtherRegion(here);
                float             hop  = Vector3.Distance(at.PointIn(here), at.PointIn(next));

                if (next == goalRegion)
                {
                    EndpointLeg leg = EndpointLegTo(_Map, goalLegs, at, at.PointIn(next), _GoalPoint, next, _Radius, false, _RepositionGoal, _CancellationTokenExternal, _CancellationTokenTimeout, _OnLeafOpened);

                    if (leg != null)
                        edges.Add(NavEdge<int>.ToGoal(hop + leg.Length, false));
                }

                foreach (RegionMap.Doorway other in _Map.Regions[next].Doorways)
                {
                    if (ReferenceEquals(other, at) || !DoorwayPassable(other))
                        continue;

                    RegionMap.CachedRoute route = RouteBetween(_Map, next, at, other, _Radius, _CancellationTokenExternal, _CancellationTokenTimeout, _OnLeafOpened);

                    if (route?.Waypoints != null)
                        edges.Add(NavEdge<int>.ToNode(CoarseNode(other, next), hop + route.Length, false));
                }

                return;
            }

            using GlobalAStar<int> coarse = new GlobalAStar<int>
            {
                Successors               = Successors,
                Heuristic                = _Node => Vector3.Distance(_Node == COARSE_START_NODE ? _Start : CoarsePoint(_Map, _Node), _GoalPoint),
                CancellationTokenControl = _CancellationTokenExternal,
                CancellationTokenTimeout = _CancellationTokenTimeout
            };

            GlobalAStarResult<int> plan = coarse.Run(COARSE_START_NODE, 0f, false);

            if (plan.Code != PathfindingResultCode.SUCCEEDED || plan.Chain == null || plan.Chain.Count < 2)
            {
                _Outcome = HierarchyPlanOutcome.NO_COARSE_ROUTE;

                return null;
            }

            //the detour guard: the doorways crossed, each up to twice its clearance wide, against the plan's length
            float planLength = 0f;
            float detours    = 0f;

            for (int i = 1; i < plan.Chain.Count; i++)
                detours += 2f * _Map.Doorways[plan.Chain[i] >> 1].Extent;

            {
                RegionMap.Doorway first = _Map.Doorways[plan.Chain[1] >> 1];
                RegionMap.Doorway last  = _Map.Doorways[plan.Chain[plan.Chain.Count - 1] >> 1];

                planLength += startLegs[first.Id].Length + goalLegs[last.Id].Length;

                for (int i = 2; i < plan.Chain.Count; i++)
                {
                    int arrived = (plan.Chain[i] & 1) == 0 ? _Map.Doorways[plan.Chain[i] >> 1].RegionA : _Map.Doorways[plan.Chain[i] >> 1].RegionB;

                    if (_Map.TryGetRoute(arrived, _Map.Doorways[plan.Chain[i - 1] >> 1], _Map.Doorways[plan.Chain[i] >> 1], _Radius, out RegionMap.CachedRoute between) && between.Waypoints != null)
                        planLength += between.Length;
                }
            }

            if (detours > planLength * PLAN_DETOUR_SHARE + PLAN_DETOUR_ALLOWANCE)
            {
                _Map.RefusePair(startRegion, goalRegion);

                _Outcome = HierarchyPlanOutcome.REFUSED_BY_GUARD;

                return null;
            }

            //the chain: start node, then one node per doorway crossed, arriving on the side of the region just left
            List<Vector3> points = new List<Vector3> { _Start };

            for (int i = 1; i < plan.Chain.Count; i++)
            {
                RegionMap.Doorway doorway = _Map.Doorways[plan.Chain[i] >> 1];
                int               arrived = (plan.Chain[i] & 1) == 0 ? doorway.RegionA : doorway.RegionB;
                int               beyond  = doorway.OtherRegion(arrived);

                if (i == 1)
                    Append(points, startLegs[doorway.Id].Waypoints);
                else
                {
                    RegionMap.Doorway previous = _Map.Doorways[plan.Chain[i - 1] >> 1];

                    _Map.TryGetRoute(arrived, previous, doorway, _Radius, out RegionMap.CachedRoute route);

                    if (route?.Waypoints == null)
                    {
                        _Outcome = HierarchyPlanOutcome.LEG_FAILED;

                        return null;
                    }

                    Append(points, RouteFrom(route, previous.PointIn(arrived)));
                }

                //through the doorway
                Append(points, new List<Vector3> { doorway.PointIn(arrived), doorway.PointIn(beyond) });

                if (i == plan.Chain.Count - 1)
                    Append(points, goalLegs[doorway.Id].Waypoints);
            }

            _Outcome = HierarchyPlanOutcome.PLANNED;

            return points;
        }

        static int CoarseNode(RegionMap.Doorway _Doorway, int _Side) => _Doorway.Id * 2 + (_Side == _Doorway.RegionA ? 0 : 1);

        static Vector3 CoarsePoint(RegionMap _Map, int _Node)
        {
            RegionMap.Doorway doorway = _Map.Doorways[_Node >> 1];

            return (_Node & 1) == 0 ? doorway.PointInA : doorway.PointInB;
        }

        //the endpoint's route to (or from) one doorway, restricted to the endpoint's region; memoised per request
        EndpointLeg EndpointLegTo(
                RegionMap                    _Map,
                Dictionary<int, EndpointLeg> _Memo,
                RegionMap.Doorway            _Doorway,
                Vector3                      _From,
                Vector3                      _To,
                int                          _Region,
                float                        _Radius,
                bool                         _RepositionFrom,
                bool                         _RepositionTo,
                CancellationToken            _CancellationTokenExternal,
                CancellationToken            _CancellationTokenTimeout,
                Action<Leaf>                 _OnLeafOpened
            )
        {
            if (_Memo.TryGetValue(_Doorway.Id, out EndpointLeg known))
                return known;

            List<Vector3> waypoints = RestrictedRoute(_Map, _Region, _From, _To, _Radius, _RepositionFrom, _RepositionTo, _CancellationTokenExternal, _CancellationTokenTimeout, _OnLeafOpened);
            EndpointLeg   leg       = waypoints == null ? null : new EndpointLeg(waypoints, Length(waypoints));

            _Memo[_Doorway.Id] = leg;

            return leg;
        }

        //the route between two doorways inside one region at one radius, searched once per map and cached on it; a
        //route cached before a publish is re-checked (by segment, or by capsule at its radius) before it is trusted,
        //and searched again when it no longer holds
        RegionMap.CachedRoute RouteBetween(
                RegionMap         _Map,
                int               _Region,
                RegionMap.Doorway _A,
                RegionMap.Doorway _B,
                float             _Radius,
                CancellationToken _CancellationTokenExternal,
                CancellationToken _CancellationTokenTimeout,
                Action<Leaf>      _OnLeafOpened
            )
        {
            int epoch = m_NavTree.PublishEpoch;

            if (_Map.TryGetRoute(_Region, _A, _B, _Radius, out RegionMap.CachedRoute cached) && (cached.Epoch == epoch || RouteStillClear(cached, _Radius)))
                return cached;

            //searched from the lower id, so both directions fill one entry and read it the same way
            RegionMap.Doorway from = _A.Id < _B.Id ? _A : _B;
            RegionMap.Doorway to   = _A.Id < _B.Id ? _B : _A;

            List<Vector3> waypoints = RestrictedRoute(_Map, _Region, from.PointIn(_Region), to.PointIn(_Region), _Radius, _Radius <= 0f, _Radius <= 0f, _CancellationTokenExternal, _CancellationTokenTimeout, _OnLeafOpened);

            cached = new RegionMap.CachedRoute(waypoints, waypoints == null ? float.PositiveInfinity : Length(waypoints), epoch);

            _Map.StoreRoute(_Region, _A, _B, _Radius, cached);

            return cached;
        }

        bool RouteStillClear(RegionMap.CachedRoute _Route, float _Radius)
        {
            if (_Route.Waypoints == null)
                return false;

            for (int i = 0; i + 1 < _Route.Waypoints.Count; i++)
            {
                bool blocked = false;

                if (_Radius <= 0f)
                    blocked = m_NavTree.SegmentIntersectOccupiedLeafInternal(new Segment3(_Route.Waypoints[i], _Route.Waypoints[i + 1]));
                else
                    HierarchyVolumetricBlocked(_Route.Waypoints[i], _Route.Waypoints[i + 1], _Radius, ref blocked);

                if (blocked)
                    return false;
            }

            return true;
        }

        //today's search — the point attempt, or the radius-aware one above radius 0 — admitting only the leaves of one
        //region (a leaf whose cell is solid takes the nearest navigable cell's region; one with none that close is
        //admitted, so the restriction errs open)
        List<Vector3> RestrictedRoute(
                RegionMap         _Map,
                int               _Region,
                Vector3           _From,
                Vector3           _To,
                float             _Radius,
                bool              _RepositionFrom,
                bool              _RepositionTo,
                CancellationToken _CancellationTokenExternal,
                CancellationToken _CancellationTokenTimeout,
                Action<Leaf>      _OnLeafOpened
            )
        {
            Dictionary<Leaf, bool> memo = new Dictionary<Leaf, bool>();

            bool Admit(Leaf _Leaf)
            {
                if (memo.TryGetValue(_Leaf, out bool admitted))
                    return admitted;

                int region = _Map.RegionOfLeafCentre(_Leaf.Bounds.center);

                admitted = region == 0 || region == _Region;

                memo[_Leaf] = admitted;

                return admitted;
            }

            if (_Radius > 0f)
            {
                List<Vector3> volumetric = null;

                HierarchyVolumetricLeg(_From, _To, _Radius, _RepositionFrom, _RepositionTo, _CancellationTokenExternal, _CancellationTokenTimeout, _OnLeafOpened, Admit, ref volumetric);

                return volumetric;
            }

            (List<Vector3> Path, PathfindingResultCode Code) result = FindPathAttempt(
                    _From,
                    _To,
                    _TryRepositionStartToFreeLeaf: true,
                    _TryRepositionTargetToFreeLeaf: true,
                    _CancellationTokenExternal,
                    _CancellationTokenTimeout,
                    _AllowYield: false,
                    out bool _,
                    _OnLeafOpened,
                    null,
                    null,
                    Admit
                );

            return result.Code == PathfindingResultCode.SUCCEEDED ? result.Path : null;
        }

        //The radius-aware lane's half of the plan — its leg search and its capsule tests — implemented in
        //GlobalOctree.Pathfinding.Hierarchy.Volumetric.cs beside the lane it belongs to; where that file is absent no
        //request carries a radius, and these calls compile out.
        partial void HierarchyVolumetricLeg(
                Vector3           _From,
                Vector3           _To,
                float             _Radius,
                bool              _RepositionFrom,
                bool              _RepositionTo,
                CancellationToken _CancellationTokenExternal,
                CancellationToken _CancellationTokenTimeout,
                Action<Leaf>      _OnLeafOpened,
                Func<Leaf, bool>  _Admit,
                ref List<Vector3> _Waypoints
            );

        partial void HierarchyVolumetricBlocked(Vector3 _A, Vector3 _B, float _Radius, ref bool _Blocked);

        //a cached route read in the direction the plan crosses it
        static List<Vector3> RouteFrom(RegionMap.CachedRoute _Route, Vector3 _From)
        {
            List<Vector3> waypoints = _Route.Waypoints;

            if ((waypoints[0] - _From).sqrMagnitude <= (waypoints[waypoints.Count - 1] - _From).sqrMagnitude)
                return waypoints;

            List<Vector3> reversed = new List<Vector3>(waypoints);

            reversed.Reverse();

            return reversed;
        }

        //appends a leg, dropping a joint that repeats the last point
        static void Append(List<Vector3> _Points, List<Vector3> _Leg)
        {
            foreach (Vector3 point in _Leg)
            {
                if (_Points.Count == 0 || (_Points[_Points.Count - 1] - point).sqrMagnitude > PATHFINDING_EPSILON)
                    _Points.Add(point);
            }
        }

        static float Length(List<Vector3> _Points)
        {
            float length = 0f;

            for (int i = 1; i < _Points.Count; i++)
                length += Vector3.Distance(_Points[i - 1], _Points[i]);

            return length;
        }

        #endregion
    }
}
