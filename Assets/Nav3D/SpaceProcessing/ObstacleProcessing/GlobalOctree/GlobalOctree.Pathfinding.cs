using Nav3D.API;
using Nav3D.Common;
using Nav3D.Obstacles;
using Nav3D.Pathfinding;
using System;
using System.Collections.Generic;
using System.Threading;
using UnityEngine;
using Leaf = Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.Elements.Leaf;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree
{
    //Pathfinding read surface over the global tree.
    //
    //The whole single-fragment search is one root-agnostic A* (GlobalAStar). All octree knowledge is confined to the
    //injected successor function (Expand) and the build-time open-face classification (NavTree.ClassifyOpenFaces).
    //Free-space straight travel is modelled as edges, so the search space is contiguous and the
    //algorithm never branches on "free space vs. octree region". Every read here runs under m_NavTree.EnterReadLock
    //so it never observes a partially-published tree during a concurrent obstacle operation.
    partial class GlobalOctree
    {
        #region Constants

        const float PATHFINDING_EPSILON = 0.00001f;

        //How many times one fragment search may stand aside for a waiting publish before it stops yielding and runs
        //to completion the blocking way. The budget is what makes starvation impossible: in a world that publishes
        //faster than it searches (a streamed sky under fire), an unbudgeted protocol would restart the same search
        //forever and no agent would ever get a route. Spending it costs the publish one ordinary wait — exactly the
        //behaviour of every build before 2026-09-04 — so the worst case is the old one, reached only after the
        //cheap attempts failed.
        const int SEARCH_YIELD_BUDGET = 3;

        #endregion

        #region Attributes (static)

        //Stands in for a leaf that was never wired, so the successor functions can walk Leaf.FreeNeighborsLocked
        //without a null branch and without falling back to the copying property. Never mutated.
        static readonly HashSet<Leaf> s_EmptyNeighbors = new HashSet<Leaf>();

        #endregion

        #region Attributes (static)

        //Hierarchical routing is enabled by default; this switch supports comparison with the flat search.
        internal static bool HierarchyEnabled { get; set; } = true;

        #endregion

        #region Attributes

        //The yield predicate handed to every search this octree runs: a search stands aside for a waiting publish and
        //RESUMES afterwards when the publish replaced none of the roots it holds leaves in (restart-only, the 2026-09-04
        //protocol spent Skyrift's three-attempt budget inside one publish; with the resume the publish's worst single
        //wait fell from 0.24–2.0 s to 8–73 ms). Cached because a lambda per search would be an allocation on the hot
        //path. Reads "a publish is waiting for the write lock" (NavTree.WriteIntentRaised).
        Func<bool> m_SearchYieldPredicate;

        #endregion

        #region Properties

        //Both search lanes ask for it through here, so the delegate is created at most once per octree
        Func<bool> SearchYieldPredicate => m_SearchYieldPredicate ??= () => m_NavTree.WriteIntentRaised;

        #endregion

        #region Public methods — pathfinding read surface

        public bool SegmentIntersectOccupiedLeaf(Segment3 _Segment) => m_NavTree.SegmentIntersectOccupiedLeaf(_Segment);

        //Single-fragment pathfinding A→B. Returns the world-space waypoint list (including A and B) and a result code.
        //Free space is traversed straight (no graph work); occupied regions are navigated via the free-leaf graph, and
        //the search leaves a region through any boundary node whose toward-B face opens to air — chosen by the one
        //global frontier, so the exit is optimal, not greedy.
        //_OnLeafOpened / _ChainOut are dev-debug-only sinks (default null on the production path): _OnLeafOpened
        //receives every leaf in expansion order and _ChainOut is filled with the final nav-node chain. When null they
        //cost a single not-taken branch each, so the production search is unaffected. The A* shots of Dev/MarketingShots draw them.
        public (List<Vector3> Path, PathfindingResultCode Code) FindPath(
                Vector3           _Start,
                Vector3           _Target,
                bool              _TryRepositionStartToFreeLeaf,
                bool              _TryRepositionTargetToFreeLeaf,
                CancellationToken _CancellationTokenExternal,
                CancellationToken _CancellationTokenTimeout,
                Action<Leaf>      _OnLeafOpened = null,
                List<Leaf>        _ChainOut     = null
            )
        {
            //the search is second in line behind A tree change.
            //An attempt that meets a waiting publish stands aside and, when the publish replaced nothing it holds,
            //resumes inside the attempt (FindPathAttempt). When it did, the attempt abandons itself, releases the read
            //lock on the way out and is run again here — the publish's teardown severs the subtrees it replaced, so a
            //search may carry node references across a lock release only after proving none of them is in one. A
            //run that starts after the publish is by construction a run on the published tree. Nothing partial
            //survives an attempt (the chain is filled only on success), so a restart is invisible to the caller apart
            //from the time it costs — the one exception being the dev expansion hook, whose only consumer is the A*
            //visualization scenario.
            //the hierarchy's map; null while the first build runs on its worker, and the search is flat meanwhile
            Hierarchy.RegionMap map = HierarchyEnabled ? m_NavTree.RegionMapForSearch() : null;

            for (int attempt = 0; ; attempt++)
            {
                (List<Vector3> Path, PathfindingResultCode Code) result = FindPathAttempt(
                        _Start,
                        _Target,
                        _TryRepositionStartToFreeLeaf,
                        _TryRepositionTargetToFreeLeaf,
                        _CancellationTokenExternal,
                        _CancellationTokenTimeout,
                        attempt < SEARCH_YIELD_BUDGET,
                        out bool yielded,
                        _OnLeafOpened,
                        _ChainOut,
                        map
                    );

                if (!yielded)
                {
                    return result;
                }

                m_NavTree.DevNoteSearchYieldHook(true);
            }
        }

        //one attempt of the search above. _Yielded true = it stood aside for a waiting publish and returned nothing;
        //every other outcome is the search's own and final. _Map plans the request on the hierarchy first
        //(GlobalOctree.Pathfinding.Hierarchy.cs); _Admit restricts the successor function to the leaves it accepts —
        //the hierarchy's legs run this same attempt, confined to one region.
        (List<Vector3> Path, PathfindingResultCode Code) FindPathAttempt(
                Vector3             _Start,
                Vector3             _Target,
                bool                _TryRepositionStartToFreeLeaf,
                bool                _TryRepositionTargetToFreeLeaf,
                CancellationToken   _CancellationTokenExternal,
                CancellationToken   _CancellationTokenTimeout,
                bool                _AllowYield,
                out bool            _Yielded,
                Action<Leaf>        _OnLeafOpened,
                List<Leaf>          _ChainOut,
                Hierarchy.RegionMap _Map   = null,
                Func<Leaf, bool>    _Admit = null
            )
        {
            _Yielded = false;

            if ((_Start - _Target).sqrMagnitude < PATHFINDING_EPSILON)
                return (new List<Vector3> { _Start, _Target }, PathfindingResultCode.SUCCEEDED);

            m_NavTree.EnterReadLock("FindPath (point lane)");
            try
            {
                int publishSerial = m_NavTree.PublishSerial;

                m_NavTree.DevSearchPhaseStartHook();

                if (_Admit != null)
                    DevLegResolveBegin();

                SealedVolumeSnapshot sealedVolumes = SealedVolumeSnapshot.Current;

                //resolve start: one inside an obstacle or a sealed volume leaves it by the nearest point of free space,
                //and the path keeps its real origin as the first waypoint
                Vector3 origin      = _Start;
                Leaf    startLeaf   = m_NavTree.GetClosestOrEmbracingLeaf(_Start);
                bool    startSealed = sealedVolumes.Contains(_Start);

                if (startSealed || (startLeaf != null && startLeaf.Occupied))
                {
                    if (!_TryRepositionStartToFreeLeaf || !m_NavTree.TryFindNearestNavigablePointInternal(_Start, null, out _Start, out startLeaf))
                        return (null, startSealed ? PathfindingResultCode.START_POINT_INSIDE_SEALED_VOLUME : PathfindingResultCode.START_POINT_INSIDE_OBSTACLE);
                }

                //resolve target: one inside an obstacle or a sealed volume is replaced by the nearest point of free space,
                //and the path ends there — not where it was asked into
                Vector3 goalPoint    = _Target;
                Leaf    targetLeaf   = m_NavTree.GetClosestOrEmbracingLeaf(_Target);
                bool    targetSealed = sealedVolumes.Contains(_Target);

                if (targetSealed || (targetLeaf != null && targetLeaf.Occupied))
                {
                    if (!_TryRepositionTargetToFreeLeaf || !m_NavTree.TryFindNearestNavigablePointInternal(_Target, null, out goalPoint, out _))
                        return (null, targetSealed ? PathfindingResultCode.TARGET_POINT_INSIDE_SEALED_VOLUME : PathfindingResultCode.TARGET_POINT_INSIDE_OBSTACLE);

                    _Target = goalPoint;
                }

                if (_Admit != null)
                    DevLegResolveEnd(origin != _Start || startSealed || targetSealed || targetLeaf != null && targetLeaf.Occupied);

                //fast path — straight shot is clear: no graph work at all
                if (!m_NavTree.SegmentIntersectOccupiedLeafInternal(new Segment3(_Start, goalPoint)) && !sealedVolumes.SegmentEnters(_Start, goalPoint))
                    return (FromOrigin(origin, BuildDirectPath(_Start, goalPoint, _Target)), PathfindingResultCode.SUCCEEDED);

                //the box graph: the answer whenever the start can be anchored in a box — a route or a refusal alike, since the
                //box graph covers everything the leaf search can reach (GlobalOctree.Pathfinding.Boxes.cs). A confined hierarchy
                //leg and the dev expansion trace search the leaves.
                if (BoxSearchEnabled && _Admit == null && _OnLeafOpened == null && _ChainOut == null && TryAnchorInBoxes(_Start, startLeaf, goalPoint, out Vector3 anchor))
                {
                    if (TryFindPathOverBoxes(_Start, anchor, goalPoint, _Target, 0f, _CancellationTokenExternal, _CancellationTokenTimeout,
                                             out (List<Vector3> Path, PathfindingResultCode Code) boxed))
                    {
                        m_NavTree.DevSearchPhaseEndHook("point: box search");

                        return boxed.Path != null ? (FromOrigin(origin, boxed.Path), boxed.Code) : boxed;
                    }
                }

                //the hierarchy: a plan over the regions, refined leg by leg; nothing found = the flat search below
                if (_Map != null)
                {
                    List<Vector3> planned = FindPathHierarchical(_Map, _Start, goalPoint, 0f, _TryRepositionStartToFreeLeaf, _TryRepositionTargetToFreeLeaf, _CancellationTokenExternal, _CancellationTokenTimeout, _OnLeafOpened);

                    if (planned != null)
                    {
                        if (goalPoint != _Target)
                            planned.Add(_Target);

                        return (FromOrigin(origin, planned), PathfindingResultCode.SUCCEEDED);
                    }
                }

                //seed the search
                Leaf  seedLeaf;
                float seedG;

                if (startLeaf != null)
                {
                    seedLeaf = startLeaf;
                    seedG    = 0f;
                }
                else
                {
                    //A is in open space and the straight shot is blocked: enter the first region the ray hits
                    Leaf entry = m_NavTree.RayMarchEntryLeaf(_Start, goalPoint);

                    if (entry == null || entry.Occupied)
                    {
                        //A ray can enter an obstacle straight out of unrepresented space, and that is ordinary.
                        //This branch used to assert the opposite — "every obstacle is wrapped in a free inflation
                        //shell, so a ray reaching an occupied leaf must first cross a free one" — and log an error
                        //whenever it did not hold. Measured at terrain scale it does not hold on **1.2 % of
                        //queries**, ~5 000 error lines per run on a perfectly valid tree, because roots exist only
                        //where obstacles are and the shell is built inside them: nothing puts a free layer on the
                        //side of a root that faces space holding no root at all. The ray then crosses unrepresented
                        //space and meets an occupied leaf as its first represented node.
                        //Reseeding from a free neighbour of that leaf is therefore the normal entry path here, not a
                        //recovery from a defect — it routes successfully 64 % of the time, and every one of the
                        //remaining refusals was confirmed correct: a breadth-first walk of the free graph found the
                        //start and goal in genuinely different components in 42 of 42 sampled cases (either side of
                        //the terrain surface, or a sealed cavity inside it).
                        //What is worth an error is the case below, where no anchor exists at all: ~22 per run, and
                        //the only one where the engine gives up for want of a seed.
                        Leaf firstOccupied = m_NavTree.RayMarchFirstOccupiedLeaf(_Start, goalPoint);
                        Leaf recovered     = firstOccupied != null ? RecoverSeedAt(firstOccupied, _Start) : null;

                        if (recovered == null || recovered.Occupied)
                        {
                            //the dev hook classifies the ray and compiles out of release with its [Nav3DDevOnly]
                            //partial, leaving the messages exactly as they read here
                            string marchDiagnosis = null;

                            m_NavTree.DevDescribeEntryMarchFailureHook(_Start, goalPoint, firstOccupied, recovered, ref marchDiagnosis);

                            //Two causes end up here and only one of them is a fault. When the recovery march found
                            //no occupied leaf at all, the sampling walk and the exact segment test disagree — the
                            //coarse step grazed past thin geometry. That is the known sampling trade-off of the
                            //entry march, not a construction fault, and it must not be reported as one (nor at
                            //error level). Only "the obstacle is there and has no free leaf against it" is the
                            //genuine construction fault worth an error.
                            if (firstOccupied == null)
                            {
                                Debug.LogWarning(
                                        $"[{nameof(GlobalOctree)}] Pathfinding: open-space start {_Start} → {goalPoint} is blocked per the " +
                                        "exact segment test, but the sampling entry march found no occupied leaf along the ray — a sampling " +
                                        "miss on thin geometry, so the search has no seed and reports no path." +
                                        marchDiagnosis
                                    );
                            }
                            else
                            {
                                Debug.LogError(
                                        $"[{nameof(GlobalOctree)}] Pathfinding: open-space start {_Start} → {goalPoint} reached an obstacle " +
                                        "with no free leaf anywhere against it, so the search has nowhere to start. This is a construction " +
                                        "fault: an obstacle must be covered by free leaves on at least one side." +
                                        marchDiagnosis
                                    );
                            }

                            return (null, PathfindingResultCode.PATH_DOES_NOT_EXIST);
                        }

                        seedLeaf = recovered;
                        seedG    = Vector3.Distance(_Start, recovered.Bounds.center);

                        //tag the query so its own result is reported below
                        m_NavTree.DevNoteEntryMarchRecoveredHook();
                    }
                    else
                    {
                        seedLeaf = entry;
                        seedG    = Vector3.Distance(_Start, entry.Bounds.center);
                    }
                }

                m_NavTree.DevSearchPhaseEndHook("point: resolve+fastpath+seed");

                using GlobalAStar<Leaf> astar = new GlobalAStar<Leaf>
                {
                    Successors               = (_Leaf, _Edges) => Expand(_Leaf, goalPoint, _Edges, _Admit),
                    Heuristic                = _Leaf => Vector3.Distance(_Leaf.Bounds.center, goalPoint),
                    CancellationTokenControl = _CancellationTokenExternal,
                    CancellationTokenTimeout = _CancellationTokenTimeout,
                    YieldRequested           = _AllowYield ? SearchYieldPredicate : null,
                    OnLeafOpened             = _OnLeafOpened
                };

                m_NavTree.DevSearchPhaseStartHook();

                GlobalAStarResult<Leaf> result = astar.Run(seedLeaf, seedG, true);

                //Standing aside does not have to throw the search away. The lock is released for the publish and taken
                //again; if that publish (or any since) replaced no root this search holds a leaf in, every reference it
                //holds is still a live leaf, and the run continues where it stopped. Otherwise it restarts, as before.
                //Leaves in a replaced root are the only ones the teardown can sever, and they are exactly the ones
                //this check refuses. What a resumed run cannot know is that an edge it relaxed before the publish
                //changed — the path layer re-examines a route crossing a change published during its search.
                while (result.Yielded)
                {
                    m_NavTree.DevNoteSearchYieldHook(false);

                    m_NavTree.ExitReadLock();
                    m_NavTree.EnterReadLock("FindPath (point lane)");

                    if (m_NavTree.AnyLeafReplacedSince(publishSerial, astar.OpenedNodes))
                    {
                        m_NavTree.DevSearchPhaseEndHook("point: A* run");
                        m_NavTree.DevNoteSearchOutcomeHook("point", result.Code, true, astar.Expansions);

                        _Yielded = true;

                        return (null, PathfindingResultCode.CANCELLED);
                    }

                    publishSerial = m_NavTree.PublishSerial;

                    m_NavTree.DevNoteSearchResumedHook();

                    result = astar.Resume();
                }

                m_NavTree.DevSearchPhaseEndHook("point: A* run");
                m_NavTree.DevNoteSearchOutcomeHook("point", result.Code, result.Yielded, astar.Expansions);

                if (_Admit == null)
                    m_NavTree.NoteFlatSearchCost(astar.Expansions);

                if (result.Yielded)
                    return YieldAttempt(out _Yielded);

                //reports only for a tagged query or a PATH_DOES_NOT_EXIST, so an ordinary one pays a branch on a
                //bool. The code goes in as the ENUM — stringifying it here would allocate on every request, and the
                //hook wrapper survives release (only the [Nav3DDevOnly] partial behind it compiles away).
                m_NavTree.DevReportPathOutcomeHook(result.Code, _Start, goalPoint, startLeaf, targetLeaf);

                if (result.Code != PathfindingResultCode.SUCCEEDED)
                    return (null, result.Code);

                if (_ChainOut != null && result.Chain != null)
                    _ChainOut.AddRange(result.Chain);

                return (FromOrigin(origin, BuildWaypoints(_Start, goalPoint, _Target, result)), PathfindingResultCode.SUCCEEDED);
            }
            finally
            {
                m_NavTree.ExitReadLock();
            }
        }

        #endregion

        #region Service methods — endpoints

        //a path whose start was moved out of an obstacle still begins where the requester stands
        static List<Vector3> FromOrigin(Vector3 _Origin, List<Vector3> _Path)
        {
            if (_Path.Count > 0 && (_Path[0] - _Origin).sqrMagnitude >= PATHFINDING_EPSILON)
                _Path.Insert(0, _Origin);

            return _Path;
        }

        //The seed a ray from open space takes when it meets an obstacle before any free cell: the free space nearest the
        //point where the ray met it, within the reach the old six-face probe had (three cells past the leaf).
        Leaf RecoverSeedAt(Leaf _FirstOccupied, Vector3 _From)
        {
            float reach = _FirstOccupied.Bounds.size.x + 3f * m_MinBucketSizeReal;

            return m_NavTree.TryFindNearestNavigablePointInternal(_FirstOccupied.Bounds.ClosestPoint(_From), reach, null, out _, out Leaf recovered)
                ? recovered
                : null;
        }

        #endregion

        #region Service methods — the yield protocol

        //The bail-out an attempt takes when its A* meets a waiting publish — shared by both lanes (which is why it
        //lives in this file rather than the volumetric one: that file is a Pro extension and can be absent). An out
        //parameter rather than a field, because many searches run concurrently on one octree.
        (List<Vector3> Path, PathfindingResultCode Code) YieldAttempt(out bool _Yielded)
        {
            _Yielded = true;

            m_NavTree.DevNoteSearchYieldHook(false);

            return (null, PathfindingResultCode.CANCELLED);
        }

        #endregion

        #region Service methods — successor function (the only octree-aware step)

        //Successor edges of a nav node, into the run's own buffer: in-region face neighbours, plus — only from an edge node whose toward-B face is
        //open — a free-space straight shot that either reaches B or enters the next region the ray hits.
        void Expand(Leaf _Leaf, Vector3 _GoalPoint, List<NavEdge<Leaf>> _Edges, Func<Leaf, bool> _Admit = null)
        {
            List<NavEdge<Leaf>> edges = _Edges;

            Vector3 center = _Leaf.Bounds.center;

            //B falls inside this leaf → arrival
            if (_Leaf.Bounds.Contains(_GoalPoint))
                edges.Add(NavEdge<Leaf>.ToGoal(Vector3.Distance(center, _GoalPoint), true));

            //in-region navigation
            //no-copy walk: the whole search runs under one read lock, so the set cannot change under it
            //(Leaf.FreeNeighborsLocked carries the full argument)
            HashSet<Leaf> freeNeighbors = _Leaf.FreeNeighborsLocked;

            foreach (Leaf neighbor in freeNeighbors ?? s_EmptyNeighbors)
            {
                if (_Admit == null || _Admit(neighbor))
                    edges.Add(NavEdge<Leaf>.ToNode(neighbor, Vector3.Distance(center, neighbor.Bounds.center), false));
            }

            //free-space straight shot toward B (only when this node can actually shoot into open air toward B, and never
            //through an authored sealed volume)
            if (IsEdgeOpenTowardGoal(_Leaf, _GoalPoint) && !SealedVolumeSnapshot.Current.SegmentEnters(center, _GoalPoint))
            {
                Segment3 toGoal = new Segment3(center, _GoalPoint);

                if (!m_NavTree.SegmentIntersectOccupiedLeafInternal(toGoal))
                {
                    edges.Add(NavEdge<Leaf>.ToGoal(Vector3.Distance(center, _GoalPoint), true));
                }
                else
                {
                    Leaf entry = m_NavTree.RayMarchEntryLeaf(center, _GoalPoint);

                    if (entry != null && !ReferenceEquals(entry, _Leaf) && !entry.Occupied && (_Admit == null || _Admit(entry)))
                        edges.Add(NavEdge<Leaf>.ToNode(entry, Vector3.Distance(center, entry.Bounds.center), true));
                }
            }
        }

        //An edge node may launch a free-space edge only if the face the toward-B direction penetrates is open to air.
        bool IsEdgeOpenTowardGoal(Leaf _Leaf, Vector3 _GoalPoint)
        {
            if (!_Leaf.IsEdgeNode)
                return false;

            return _Leaf.IsFaceOpen(DominantFaceBit(_GoalPoint - _Leaf.Bounds.center));
        }

        #endregion

        #region Service methods — path assembly

        static List<Vector3> BuildDirectPath(Vector3 _Start, Vector3 _GoalPoint, Vector3 _Target)
        {
            List<Vector3> points = new List<Vector3> { _Start, _GoalPoint };

            if (_GoalPoint != _Target)
                points.Add(_Target);

            return points;
        }

        static List<Vector3> BuildWaypoints(Vector3 _Start, Vector3 _GoalPoint, Vector3 _Target, GlobalAStarResult<Leaf> _Result)
        {
            List<Leaf> chain = _Result.Chain;

            List<Vector3> points = new List<Vector3>(chain.Count * 2 + 3) { _Start };

            for (int i = 0; i < chain.Count; i++)
            {
                points.Add(chain[i].Bounds.center);

                //insert the shared-face contact point for in-region (non-straight) steps
                if (i < chain.Count - 1 && !_Result.StepStraight[i])
                    points.Add(chain[i].GetAdjacentContactPoint(chain[i + 1]));
            }

            //when the target was repositioned, _GoalPoint is the repositioned leaf's center — if the chain ends in
            //that very leaf its center is already appended, and a verbatim Add would duplicate the point
            if ((points[^1] - _GoalPoint).sqrMagnitude > PATHFINDING_EPSILON)
                points.Add(_GoalPoint);

            if (_GoalPoint != _Target)
                points.Add(_Target);

            return points;
        }

        #endregion

        #region Service methods — goal-direction face lookup

        //The single face a direction vector penetrates (its dominant axis + sign), as a FaceBit. Consumes the
        //OpenFaces data ClassifyOpenFaces produces (moved to NavTree.cs as build-time write surface —
        //this lookup stays here as the algorithm's own consumer of that data, used only by IsEdgeOpenTowardGoal above).
        static int DominantFaceBit(Vector3 _Direction)
        {
            float ax = Mathf.Abs(_Direction.x);
            float ay = Mathf.Abs(_Direction.y);
            float az = Mathf.Abs(_Direction.z);

            if (ax >= ay && ax >= az) return _Direction.x >= 0 ? 0 : 1;
            if (ay >= az) return _Direction.y >= 0 ? 2 : 3;
            return _Direction.z >= 0 ? 4 : 5;
        }

        #endregion
    }
}
