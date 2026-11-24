using Nav3D.API;
using Nav3D.Common;
using Nav3D.SpaceProcessing.ObstacleProcessing.UtilityTypes;
using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using System.Linq;
using System.Threading;
using UnityEngine;
using Node = Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.Elements.Node;
using Fork = Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.Elements.Fork;
using ForkChildOctIndex = Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.Elements.ForkChildOctIndex;
using Leaf = Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.Elements.Leaf;
using NodeSpatialGrid = Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.SpatialStorages.NodeSpatialGrid;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree
{
    //The octree's live read surface, extracted out of the GlobalOctree god-partial.
    // Holds m_RootsGrid, m_TreeLock (+ m_TriangleClusterStorage,
    //guarded by the same lock, see the field's own comment below), every read-query method, and open-face
    //classification. Pure code motion: no query logic, lock semantics, or ordering changed, only where the code lives.
    //Holds a GlobalOctree back-reference for RootSize/MinBucketSizeReal, mirroring RootBuilder's own precedent.
    //m_ActiveRootsGrid (the construction/staging alias) deliberately stays on GlobalOctree — readers never use it.
    //(partial: the volumetric (capsule) read queries live in NavTree.Volumetric.cs — a Nav3D Pro extension file.)
    internal sealed partial class NavTree
    {
        readonly GlobalOctree m_Octree;

        //The live tree that readers traverse (occupancy/body queries, and pathfinding once it traverses the global
        //tree). Mutated only by PublishStagedRegion, under the write lock; the field itself is never reassigned.
        readonly NodeSpatialGrid m_RootsGrid;

        //Spatial storage for triangle clusters. Intended for accessing obstacle data during pathfinding.
        readonly BoundablesSpatialHashMap<TriangleCluster> m_TriangleClusterStorage;

        //Guards the live read surface (m_RootsGrid and its leaves, and m_TriangleClusterStorage) against in-flight
        //obstacle operations. Readers (occupancy/body queries, pathfinding traversal) take the read lock; an
        //operation takes the write lock only for its brief publish, so a reader can never observe a partially-
        //published tree. SupportsRecursion lets an operation that holds the write lock also enter a read-locked
        //method without deadlocking.
        readonly ReaderWriterLockSlim m_TreeLock = new ReaderWriterLockSlim(LockRecursionPolicy.SupportsRecursion);

        //"A publish wants the write lock" — raised before EnterWriteLock blocks and dropped before it releases, so a
        //long reader can step aside instead of making the publish wait for it. A counter rather than a flag: nothing
        //in the engine takes the write lock concurrently (operations are serialized on one executor), but a counter
        //cannot be left raised by an interleaving the way a shared bool can. See EnterWriteLock below.
        int m_WriteIntent;

        //How many publishes the live tree has taken, and the serial of the last one that replaced each root (every root
        //of its clone set: footprint roots are torn down after the lock, ring roots' boundary originals are left
        //dead). Written only under the write lock, read under the read lock — which is what lets a search that
        //stood aside for a publish prove it holds nothing the publish replaced, and resume instead of restarting.
        //An entry outlives its root on purpose: a dropped root's leaves are as dead as a replaced one's.
        int m_PublishSerial;
        int m_SeveredAtSerial;

        readonly Dictionary<Vector3Int, int> m_RootReplacedAtSerial = new Dictionary<Vector3Int, int>();

        const float PATHFINDING_EPSILON = 0.00001f;

        public NavTree(GlobalOctree _Octree, int _LayersCount)
        {
            m_Octree                 = _Octree;
            m_TriangleClusterStorage = new BoundablesSpatialHashMap<TriangleCluster>();
            m_RootsGrid              = new NodeSpatialGrid(_LayersCount);
        }

        #region Lock surface

        //dev instrumentation hooks. Declared here (shipped code) and implemented in NavTree.DevTimings.cs, which is
        //[Nav3DDevOnly] and therefore deleted by the release preparer — an unimplemented partial void compiles to
        //nothing, so these calls vanish with it. Same mechanism as GlobalOctree's DevOpTimer* hooks.
        partial void DevReadWaitStart();
        partial void DevReadWaitEnd();

        //The write side of the same instrument: how long a publish waits for the readers in flight before it can
        //install its patch. The read-wait counters above measure what a queued publish costs its readers; these
        //measure what the readers cost the publish — and until 2026-09-04 that half was simply not measured.
        partial void DevWriteWaitStart();
        partial void DevWriteWaitEnd();

        //How long a reader HOLDS the lock, which is what the publish's wait is made of. Depth-guarded: the lock
        //supports recursion, and a nested acquisition restarting the timer would erase the outer hold.
        partial void DevReadHoldStart();
        partial void DevReadHoldEnd();

        //who is holding it. A hold of a second says nothing until it has a name — and the first measurement of the
        //holds found long ones the search-phase brackets did not account for. The argument is a literal,
        //so in a release build the unimplemented partial removes the call and the literal with it.
        partial void DevReadHoldLabel(string _Label);

        //where a search spends its hold. The lock is taken for a whole search fragment, and only the A* loop can
        //yield — so if the long holds live in the phases around that loop (endpoint resolution, the straight-shot
        //segment/capsule test, waypoint assembly), yielding cannot shorten them and the instrument says so.
        partial void DevSearchPhaseStart();
        partial void DevSearchPhaseEnd(string _Phase);

        internal void DevSearchPhaseStartHook() => DevSearchPhaseStart();

        internal void DevSearchPhaseEndHook(string _Phase) => DevSearchPhaseEnd(_Phase);

        //A search stood aside for a waiting publish (_Restarted false) or was restarted afterwards (true). The pair
        //is what says whether the yield protocol is doing its job or thrashing: a restart that never leads to a
        //completed search is starvation, which is what the restart budget bounds.
        partial void DevNoteSearchYield(bool _Restarted);

        internal void DevNoteSearchYieldHook(bool _Restarted) => DevNoteSearchYield(_Restarted);

        //A search that stood aside continued where it stopped, the publish having replaced nothing it held.
        partial void DevNoteSearchResumed();

        internal void DevNoteSearchResumedHook() => DevNoteSearchResumed();

        //the search population: every A* run reported once with its lane,
        //its result code, whether it stood aside for a publish, and how many nodes it closed — bucketed by the dev
        //timings into "which outcome owns the long holds" and "how big is a long search". Called right after the
        //"A* run" phase closes, so the implementation reads that phase's elapsed time off the thread-static watch.
        //The enum travels as an enum for the same reason DevReportPathOutcome's does (no ToString on the hot path).
        partial void DevNoteSearchOutcome(string _Lane, PathfindingResultCode _Code, bool _Yielded, int _Expansions);

        internal void DevNoteSearchOutcomeHook(string _Lane, PathfindingResultCode _Code, bool _Yielded, int _Expansions)
            => DevNoteSearchOutcome(_Lane, _Code, _Yielded, _Expansions);

        //Sub-step instrumentation for the publish's ring back-ref work — the unlocked search and the two things
        //PublishStagedRegion does under the lock. Its own watch, deliberately not GlobalOctree's DevPublishPhase*
        //pair: that one is a single thread-static stopwatch already bracketing these calls, so reusing it here would
        //restart the outer measurement mid-flight.
        partial void DevPublishStepStart();
        partial void DevPublishStepEnd(string  _Step);
        partial void DevPublishStepCounts(int  _CloneSetRoots, int _ClonePairs, int _RingPairs, int _NeighborsWalked, int _Repoints);
        partial void DevCloneWorkCounts(int    _ClonedLeaves,  int _RefsWired);
        partial void DevCloneVerdictCounts(int _DoomedSkipped, int _NarrowedCloned);

        //clone anatomy: the measurement that has to precede any narrowing of the ring clone — how much
        //of the clone set is ring, how much of the ring is the boundary layer, and which ring clone leaves an
        //operation actually mutates. Implemented in the [Nav3DDevOnly] NavTree.DevCloneAnatomy.cs partial, behind its
        //own flag (off by default), so an ordinary timing run does not pay for the walk.
        partial void DevCloneAnatomySnapshot(HashSet<Vector3Int> _CloneSetIndices, Dictionary<Leaf, Leaf> _OriginalToClone, NodeSpatialGrid _StagingGrid);

        partial void DevCloneAnatomyCheck(
                HashSet<Vector3Int> _CloneSetIndices,
                HashSet<Vector3Int> _RingRootIndices,
                HashSet<Vector3Int> _FootprintRootIndices,
                bool                _FullRingClone,
                NodeSpatialGrid     _StagingGrid
            );

        //entry-MARCH failure diagnosis: why did the ray reach an obstacle
        //with no free shell leaf before it? The failing branch already logs; this appends the reason, so a run that
        //reproduces the condition explains it instead of merely counting it. Only ever called from that branch,
        //which is exceptional, so its cost is irrelevant — and it compiles out with its [Nav3DDevOnly] partial.
        //Implemented in NavTree.DevEntryMarch.cs.
        partial void DevDescribeEntryMarchFailure(Vector3 _From, Vector3 _To, Leaf _FirstOccupied, Leaf _Recovered, ref string _Description);

        //The other half of P9, and it must be measured rather than inferred: a query that reseeds successfully still
        //mostly ends as PATH_DOES_NOT_EXIST, and the first write-up of that row got its cause wrong precisely by
        //correlating two run totals instead of tagging the individual query. These two mark the query and then report
        //its actual result code.
        //It reports for a reseeded query and for any PATH_DOES_NOT_EXIST, because the same run answered a second
        //question with the same line: this branch accounts for only about a third of a terrain run's `noPath`, and
        //where the rest comes from is asked by classifying where the start and the goal sit — a goal in space that
        //holds no root at all is unreachable by construction, since arrival is `Leaf.Bounds.Contains(goal)`.
        //The result code travels as the ENUM, not a string: this wrapper is ordinary shipped code that survives
        //release (only the partial behind it compiles away), so a string here would put a per-request ToString()
        //allocation on the hot path of every release build. The dev implementation stringifies where it logs.
        partial void DevNoteEntryMarchRecovered();
        partial void DevReportPathOutcome(PathfindingResultCode _ResultCode, Vector3 _Start, Vector3 _Goal, Leaf _StartLeaf, Leaf _TargetLeaf);

        internal void DevNoteEntryMarchRecoveredHook() => DevNoteEntryMarchRecovered();

        //who asked: the requester id of the search about to run on this thread, so the
        //outcome line above can name it — a no-path storm is one requester asking the same thing again. Static and
        //thread-static behind it: the search runs synchronously on the thread that set it.
        static partial void DevSetRequester(string _RequesterID);

        internal static void DevSetRequesterHook(string _RequesterID) => DevSetRequester(_RequesterID);

        internal void DevReportPathOutcomeHook(PathfindingResultCode _Code, Vector3 _Start, Vector3 _Goal, Leaf _StartLeaf, Leaf _TargetLeaf)
            => DevReportPathOutcome(_Code, _Start, _Goal, _StartLeaf, _TargetLeaf);

        //GlobalOctree is a separate class, so it cannot call a partial method — same wrapper mechanism as below.
        internal void DevDescribeEntryMarchFailureHook(Vector3 _From, Vector3 _To, Leaf _FirstOccupied, Leaf _Recovered, ref string _Description)
            => DevDescribeEntryMarchFailure(_From, _To, _FirstOccupied, _Recovered, ref _Description);

        //StagingTransaction is a separate class, so it cannot call a partial method — same wrapper mechanism as
        //GlobalOctree's DevOpPhase*Hook pair.
        internal void DevCloneAnatomySnapshotHook(HashSet<Vector3Int> _CloneSetIndices, Dictionary<Leaf, Leaf> _OriginalToClone, NodeSpatialGrid _StagingGrid)
            => DevCloneAnatomySnapshot(_CloneSetIndices, _OriginalToClone, _StagingGrid);

        internal void DevCloneAnatomyCheckHook(
                HashSet<Vector3Int> _CloneSetIndices,
                HashSet<Vector3Int> _RingRootIndices,
                HashSet<Vector3Int> _FootprintRootIndices,
                bool                _FullRingClone,
                NodeSpatialGrid     _StagingGrid
            )
            => DevCloneAnatomyCheck(_CloneSetIndices, _RingRootIndices, _FootprintRootIndices, _FullRingClone, _StagingGrid);

        //detached-subtree TEARDOWN. Unity's Boehm collector scans thread stacks and registers conservatively, and
        //a stale slot left by an operation's own deep construction frames is enough to keep a dead node alive past
        //every collection. On its own that pins one node, but a replaced subtree is a densely linked graph, so one
        //pinned member retains the whole island — measured on a streamed world as ~30 MB of heap retained per
        //obstacle add/remove cycle. Severing the dead graph's internal links caps every such pin at one node.
        //
        //CollectReplacedOriginals runs before the write lock (read-only, and operations are serialized); the
        //teardown runs after it, when no reader can reach the replaced originals — the write lock drained every
        //reader that predated it, and readers do not carry node references across lock releases — the one reader
        //that does, a point-lane search resuming after a yield, first proves through AnyLeafReplacedSince that none
        //of its leaves is in a root this publish replaced, and restarts otherwise. Ring originals are
        //deliberately not collected, since the partial ring clone shares their interiors with the live tree, and
        //neither are bake-cache nodes, since installed pre-baked roots are clones of the cache image. Only
        //wholly-DEAD subtrees are collected: a whole-replaced or dropped footprint root in full, a NARROWED root
        //only down to its planned cells' originals.

        //The verdict-aware form (Phase 2): a footprint root replaced or dropped wholesale contributes its whole
        //live subtree (entirely dead after the publish — the original P28 argument), but a NARROWED root's subtree
        //is mostly SHARED with the staged root that replaces it, so tearing it down would sever the live tree
        //itself. For those, exactly the planned CELLS' original subtrees are collected — each was cloned in full
        //and its clone then pruned and rebuilt, so the original is wholly dead. The narrowed root's dead spine
        //forks and boundary-layer leaf originals are left to the collector, the same bounded residue the ring's
        //replaced boundary originals already are.
        //_Collected: the caller's (scratch) list, cleared here; returned for the callers' convenience.
        internal List<Node> CollectReplacedOriginals(StagingTransaction _Transaction, List<Node> _Collected)
        {
            List<Node> collected = _Collected;

            collected.Clear();

            if (!_Transaction.CloneNarrowed)
            {
                foreach (Vector3Int rootIndex in _Transaction.FootprintRootIndices)
                {
                    if (m_RootsGrid.TryGetNode(0, rootIndex, out Node liveRoot))
                        collected.Add(liveRoot);
                }

                return collected;
            }

            foreach (Vector3Int rootIndex in _Transaction.FootprintRootIndices)
            {
                if (!m_RootsGrid.TryGetNode(0, rootIndex, out Node liveRoot))
                    continue;

                if (!_Transaction.RebuiltCellsByRoot.TryGetValue(rootIndex, out List<StagingTransaction.RebuiltCell> cells))
                {
                    collected.Add(liveRoot);

                    continue;
                }

                foreach (StagingTransaction.RebuiltCell cell in cells)
                {
                    Node cellOriginal = DescendToCell(liveRoot, cell.Index, cell.Layer);

                    if (cellOriginal != null)
                        collected.Add(cellOriginal);
                }
            }

            return collected;
        }

        //The live original node at an exact cell position, by index arithmetic (a cell centre lies on a boundary
        //plane at every coarser layer, so point containment would be a coin flip). Returns null when the tree is
        //not subdivided down to the cell — which the narrowing plan precludes for a planned cell, but a defensive
        //null keeps the teardown collection from ever grabbing a COARSER (still live) node.
        static Node DescendToCell(Node _Root, Vector3Int _CellIndex, int _CellLayer)
        {
            Node node = _Root;

            while (node is Fork fork && fork.GridLayer < _CellLayer)
            {
                int childLayer = fork.GridLayer + 1;
                int divisor    = 1 << (_CellLayer - childLayer);

                Vector3Int wantedIndex = new Vector3Int(
                        Mathf.FloorToInt(_CellIndex.x / (float) divisor),
                        Mathf.FloorToInt(_CellIndex.y / (float) divisor),
                        Mathf.FloorToInt(_CellIndex.z / (float) divisor)
                    );

                Node next = null;

                foreach ((ForkChildOctIndex OctIndex, Vector3Int GridIndex) child in UtilsSpatialGrid.GetChildrenIndices(fork.Index))
                {
                    if (child.GridIndex != wantedIndex)
                        continue;

                    fork.ChildrenMap.TryGetValue(child.OctIndex, out next);

                    break;
                }

                if (next == null)
                    return null;

                node = next;
            }

            return node != null && node.GridLayer == _CellLayer ? node : null;
        }

        //_Stack: the caller's (scratch) walk stack, empty on entry and on exit.
        internal static void TearDownDetachedSubtrees(List<Node> _ReplacedOriginals, Stack<Node> _Stack)
        {
            if (_ReplacedOriginals == null)
                return;

            Stack<Node> stack = _Stack;

            stack.Clear();

            foreach (Node root in _ReplacedOriginals)
                stack.Push(root);

            while (stack.Count > 0)
            {
                Node node = stack.Pop();

                if (node is Leaf leaf)
                {
                    //the set object goes too, not only its contents — see Leaf.DetachAllFreeNeighbors
                    leaf.DetachAllFreeNeighbors();
                }
                else if (node is Fork fork)

                {
                    foreach (KeyValuePair<ForkChildOctIndex, Node> pair in fork.ChildrenMap)
                        stack.Push(pair.Value);

                    fork.ChildrenMap.Clear();
                }
            }
        }

        //the whole live tree, unlinked — the epoch's death (GlobalOctree.ReleaseWorld), under the write lock so no search
        //is inside. The same walk the per-operation teardown runs over a replaced subtree, over every root, and then
        //the two maps that name every root and every cluster. A reader arriving afterwards finds an empty world, which
        //is the true answer about a world that is gone
        internal void SeverWorld()
        {
            SeverRegionMap();

            EnterWriteLock();

            try
            {
                ResetPointSolids();
                TearDownDetachedSubtrees(new List<Node>(m_RootsGrid.GetAllAtLevel(0)), new Stack<Node>());

                m_PublishSerial++;
                m_SeveredAtSerial = m_PublishSerial;

                m_RootsGrid.Clear();
                m_TriangleClusterStorage.Clear();

                ClearBoxes();
            }
            finally
            {
                ExitWriteLock();
            }
        }

        //every read query goes through this, so the wait is measured once, in one place. The wait — not the query —
        //is the interesting number: a reader blocked here is blocked by a publish holding the write lock, and on the
        //main thread that is frame time lost.
        void AcquireReadLock()
        {
            DevReadWaitStart();

            //the wait is a profiler sample of its own: on the main thread it is the frame time a publish costs, and
            //there it also counts toward Nav3DMainThreadCost (the adaptive search concurrency's signal)
            UnityEngine.Profiling.Profiler.BeginSample("Nav3D.TreeReadWait");

            bool onMainThread = Nav3DMainThreadCost.IsMainThread;
            long waitBegin    = onMainThread ? Nav3DMainThreadCost.Begin() : 0L;

            m_TreeLock.EnterReadLock();

            if (onMainThread)
                Nav3DMainThreadCost.End(waitBegin);

            UnityEngine.Profiling.Profiler.EndSample();

            DevReadWaitEnd();
            DevReadHoldStart();
        }

        public void EnterReadLock() => AcquireReadLock();

        //the labelled form for the callers outside this class that hold the lock for more than a query — the two
        //search lanes and the editor's gizmo fill
        public void EnterReadLock(string _Label)
        {
            AcquireReadLock();
            DevReadHoldLabel(_Label);
        }

        public void ExitReadLock() => ReleaseReadLock();

        //the one release point, so the hold is measured where the wait already is. The hold is the other half of the
        //convoy: a publish waits for the readers already inside, so what the longest reader holds for is the
        //publish's wait. Every read query in this class releases through here.
        void ReleaseReadLock()
        {
            DevReadHoldEnd();

            m_TreeLock.ExitReadLock();
        }

        //the publish'S side of the lock, and the signal that MAKES pathfinding second in line (2026-09-04,
        //). ReaderWriterLockSlim prefers writers, so a publish
        //that queues here also freezes every new reader — but it still waits for the readers already inside, and the
        //A* holds the read lock for its whole search (up to the request timeout, 2 s by default, over as many
        //concurrent searches as there are pathfinding tasks). Raising the intent before blocking lets those searches
        //see that a tree change is waiting and step aside (GlobalAStar.YieldRequested), which is what turns "the
        //publish waits for the longest search in flight" into "the publish waits for one A* expansion".
        public void EnterWriteLock()
        {
            Interlocked.Increment(ref m_WriteIntent);

            DevWriteWaitStart();

            m_TreeLock.EnterWriteLock();

            DevWriteWaitEnd();
        }

        public void ExitWriteLock()
        {
            //dropped while the write lock is still HELD, so no reader can observe the intent after the publish is
            //done: lowering it after the release would leave a window in which a search that just re-entered the
            //read lock yields to a publish that has already finished, burning one of its restarts for nothing
            Interlocked.Decrement(ref m_WriteIntent);

            m_TreeLock.ExitWriteLock();
        }

        //Read by the pathfinding search between node expansions. True = a publish is waiting (or writing), and a
        //long reader is what it is waiting for.
        public bool WriteIntentRaised => Volatile.Read(ref m_WriteIntent) > 0;

        //Read under the read lock: the publishes the reader's view already contains.
        public int PublishSerial => m_PublishSerial;

        //Whether a publish after _Serial replaced the root of any of _Leaves — i.e. whether a reader that held these
        //leaves across a lock release may hold one that is dead, or about to be torn down. Called under the read lock.
        public bool AnyLeafReplacedSince(int _Serial, IEnumerable<Leaf> _Leaves)
        {
            if (m_PublishSerial == _Serial)
                return false;

            if (m_SeveredAtSerial > _Serial)
                return true;

            //neighbouring leaves mostly share a root, so the dictionary is asked once per run of one root
            Vector3Int lastRoot    = default;
            bool       hasLastRoot = false;

            foreach (Leaf leaf in _Leaves)
            {
                Vector3Int rootIndex = leaf.RootIndex;

                if (hasLastRoot && rootIndex == lastRoot)
                    continue;

                lastRoot    = rootIndex;
                hasLastRoot = true;

                if (m_RootReplacedAtSerial.TryGetValue(rootIndex, out int replacedAt) && replacedAt > _Serial)
                    return true;
            }

            return false;
        }

        //The live grid reference itself — needed only by StagingTransaction.Dispose() to restore the construction
        //alias back onto the live tree once an operation finishes publishing.
        public NodeSpatialGrid LiveRootsGrid => m_RootsGrid;

        //The live cluster storage reference itself — needed by construction/operations code (RuntimeOperations.cs /
        //PreBakedOperations.cs) that reads existing clusters crossing a root during a build, and by the deferred
        //Add/Remove mutations those ops queue to run inside StagingTransaction.Publish()'s write lock (mirroring
        //PublishStagedRegion's own live-tree mutation, just for cluster storage instead of the node tree).
        public BoundablesSpatialHashMap<TriangleCluster> TriangleClusterStorage => m_TriangleClusterStorage;

        #endregion

        #region Read queries

        public bool IsBoundsCrossAnyObstacleBody(Bounds _Bounds)
        {
            AcquireReadLock();
            DevReadHoldLabel("IsBoundsCrossAnyObstacleBody");

            try
            {
                if (!m_TriangleClusterStorage.TryGetCrossingBoundables(_Bounds, out HashSet<TriangleCluster> crossingClusters))
                    return false;

                return crossingClusters.Any(_Cluster => _Cluster.Bounds.Intersects(_Bounds));
            }
            finally
            {
                ReleaseReadLock();
            }
        }

        public bool IsPointInsideOccupiedLeaf(Vector3 _Point)
        {
            AcquireReadLock();
            DevReadHoldLabel("IsPointInsideOccupiedLeaf");

            try
            {
                return PointInsideOccupiedLeafInternal(_Point);
            }
            finally
            {
                ReleaseReadLock();
            }
        }

        //the per-TICK main-THREAD form (the standing rule "a per-frame main-thread reader never blocks").
        //Refused = a publish holds the lock or is queued for it; the caller keeps the answer it had and asks again
        //next tick. Its callers — the agent's carrot-blocked probe and the avoidance stage's escape state — already
        //cache their verdict between checks, so one tick of staleness is inside what they tolerate by design.
        public bool TryIsPointInsideOccupiedLeaf(Vector3 _Point, out bool _Result)
        {
            if (!m_TreeLock.TryEnterReadLock(0))
            {
                DevReadDeclined();

                _Result = false;

                return false;
            }

            DevReadHoldStart();
            DevReadHoldLabel("TryIsPointInsideOccupiedLeaf");

            try
            {
                _Result = PointInsideOccupiedLeafInternal(_Point);

                return true;
            }
            finally
            {
                ReleaseReadLock();
            }
        }

        //under the read lock
        bool PointInsideOccupiedLeafInternal(Vector3 _Point)
        {
            if (!m_RootsGrid.TryGetNode(0, UtilsSpatialGrid.GetBucketIndex(_Point, m_Octree.RootSize), out Node root))
                return false;

            return PointInsideOccupiedLeaf(root, _Point);
        }

        static bool PointInsideOccupiedLeaf(Node _CurrentNode, Vector3 _Point)
        {
            if (_CurrentNode.Bounds.Contains(_Point))
            {
                if (_CurrentNode is Leaf leaf)
                    return leaf.Occupied;

                foreach (KeyValuePair<Elements.ForkChildOctIndex, Node> childData in (_CurrentNode as Fork)!.ChildrenMap)
                {
                    if (PointInsideOccupiedLeaf(childData.Value, _Point))
                        return true;
                }
            }

            return false;
        }

        //The local-avoidance read: every triangle of every registered cluster that crosses the bounds. Answered
        //through each cluster's own BVH (TriangleCluster.CollectCrossedTriangles), not by walking its triangle set:
        //until 2026-09-03 this was a LINQ scan of every triangle of every crossing cluster, so an agent beside an
        //18 000-triangle island tested all 18 000 boxes per refresh — and a region change marks every agent inside
        //it dirty at once, which put 80 agents × 18 000 tests into one FixedUpdate (measured 735 ms on the Skyrift
        //endurance probe's frame-cost arm).
        public Triangle[] GetIntersectedObstaclesTriangles(Bounds _Bounds)
        {
            AcquireReadLock();
            DevReadHoldLabel("GetIntersectedObstaclesTriangles (blocking)");

            try
            {
                return CollectIntersectedObstaclesTriangles(_Bounds);
            }
            finally
            {
                ReleaseReadLock();
            }
        }

        //the non-BLOCKING read for the main thread. ReaderWriterLockSlim prefers writers: once a publish
        //is QUEUED behind a long worker read (an A* holds the read lock for its whole search), every new reader waits
        //for that read and the publish — the local-avoidance refresh, on the main thread, measured 388–417 ms of
        //waiting in one frame on the Skyrift endurance probe's frame-cost arm, the worst frames of the run. The
        //avoidance refresh does not need the answer this tick: it keeps the triangles it has and asks again next tick
        //(the movers-dirty flag stays set), which is the same staleness its 12-tick backstop already tolerates. So it
        //asks with a zero timeout and takes "not now" for an answer.
        public bool TryGetIntersectedObstaclesTriangles(Bounds _Bounds, out Triangle[] _Result)
        {
            if (!m_TreeLock.TryEnterReadLock(0))
            {
                DevReadDeclined();

                _Result = null;

                return false;
            }

            DevReadHoldStart();
            DevReadHoldLabel("TryGetIntersectedObstaclesTriangles");

            try
            {
                _Result = CollectIntersectedObstaclesTriangles(_Bounds);

                return true;
            }
            finally
            {
                ReleaseReadLock();
            }
        }

        partial void DevReadDeclined();

        //under the read lock: every triangle of every registered cluster crossing the bounds, through the clusters' BVHs
        Triangle[] CollectIntersectedObstaclesTriangles(Bounds _Bounds)
        {
            {
                if (!m_TriangleClusterStorage.TryGetCrossingBoundables(_Bounds, out HashSet<TriangleCluster> crossingClusters))
                    return Array.Empty<Triangle>();

                HashSet<Triangle> crossing = t_CrossingTrianglesScratch ??= new HashSet<Triangle>();

                crossing.Clear();

                t_CrossingBoundsScratch    ??= new Bounds[1];
                t_CrossingBoundsScratch[0] =   _Bounds;

                foreach (TriangleCluster cluster in crossingClusters)
                    cluster.CollectCrossedTriangles(t_CrossingBoundsScratch, crossing);

                if (crossing.Count == 0)
                    return Array.Empty<Triangle>();

                Triangle[] result = new Triangle[crossing.Count];

                crossing.CopyTo(result);
                crossing.Clear();

                return result;
            }
        }

        //per-thread scratch for the query above (the main thread's agents and the pathfinding workers both read)
        [ThreadStatic] static HashSet<Triangle> t_CrossingTrianglesScratch;
        [ThreadStatic] static Bounds[]          t_CrossingBoundsScratch;

        public bool SegmentIntersectOccupiedLeaf(Segment3 _Segment)
        {
            AcquireReadLock();
            DevReadHoldLabel("SegmentIntersectOccupiedLeaf");

            try
            {
                return SegmentIntersectOccupiedLeafInternal(_Segment);
            }
            finally
            {
                ReleaseReadLock();
            }
        }

        //The per-tick main-thread form of the query above — same contract as TryIsPointInsideOccupiedLeaf. Its caller
        //is the pursuit line-of-sight check, which already re-asks only every few ticks and keeps its last verdict in
        //between.
        public bool TrySegmentIntersectOccupiedLeaf(Segment3 _Segment, out bool _Result)
        {
            if (!m_TreeLock.TryEnterReadLock(0))
            {
                DevReadDeclined();

                _Result = false;

                return false;
            }

            DevReadHoldStart();
            DevReadHoldLabel("TrySegmentIntersectOccupiedLeaf");

            try
            {
                _Result = SegmentIntersectOccupiedLeafInternal(_Segment);

                return true;
            }
            finally
            {
                ReleaseReadLock();
            }
        }

        public bool SegmentIntersectOccupiedLeafInternal(Segment3 _Segment)
        {
            Bounds segmentBounds = new Bounds();
            segmentBounds.SetMinMax(Vector3.Min(_Segment.Start, _Segment.End), Vector3.Max(_Segment.Start, _Segment.End));

            UtilsSpatialGrid.GetCrossedBucketsRange(segmentBounds, m_Octree.RootSize, out Vector3Int minIndex, out Vector3Int maxIndex);

            for (int x = minIndex.x; x <= maxIndex.x; x++)
                for (int y = minIndex.y; y <= maxIndex.y; y++)
                    for (int z = minIndex.z; z <= maxIndex.z; z++)
                    {
                        if (m_RootsGrid.TryGetNode(0, new Vector3Int(x, y, z), out Node root) && SegmentHitsOccupied(root, _Segment))
                            return true;
                    }

            return false;
        }

        static bool SegmentHitsOccupied(Node _Node, Segment3 _Segment)
        {
            if (!_Node.Bounds.IntersectSegment(_Segment))
                return false;

            if (_Node is Leaf leaf)
                return leaf.Occupied;

            foreach (Node child in ((Fork) _Node).ChildrenMap.Values)
            {
                if (SegmentHitsOccupied(child, _Segment))
                    return true;
            }

            return false;
        }

        //Marches the ray _From→_To through the tree, returning the last free leaf before the first occupied leaf the
        //ray would enter (the entry node into the blocking region). The collision decision itself is made exactly by
        //SegmentIntersectOccupiedLeafInternal; this sampling only locates a reasonable entry leaf, so a coarse step is
        //fine (postprocessing cleans residual error).
        public Leaf RayMarchEntryLeaf(Vector3 _From, Vector3 _To)
        {
            Vector3 delta    = _To - _From;
            float   distance = delta.magnitude;

            if (distance < PATHFINDING_EPSILON)
                return GetClosestOrEmbracingLeaf(_From);

            float   step      = m_Octree.MinBucketSizeReal * 0.5f;
            int     steps     = Mathf.Max(1, Mathf.CeilToInt(distance / step));
            Vector3 direction = delta / distance;
            Leaf    lastFree  = null;

            for (int i = 0; i <= steps; i++)
            {
                Vector3 point = _From + direction * Mathf.Min(i * step, distance);
                Leaf    leaf  = GetClosestOrEmbracingLeaf(point);

                if (leaf == null)
                    continue; //open-space sample (no root)

                if (leaf.Occupied)
                    return lastFree;

                lastFree = leaf;
            }

            return lastFree;
        }

        //Marches _From→_To and returns the first occupied leaf the ray enters (null if none). Recovery anchor for the
        //open-space-start path: when RayMarchEntryLeaf finds no free shell entry, a free neighbour of this leaf reseeds
        //the search so it can route around the obstacle.
        public Leaf RayMarchFirstOccupiedLeaf(Vector3 _From, Vector3 _To)
        {
            Vector3 delta    = _To - _From;
            float   distance = delta.magnitude;

            if (distance < PATHFINDING_EPSILON)
                return null;

            float   step      = m_Octree.MinBucketSizeReal * 0.5f;
            int     steps     = Mathf.Max(1, Mathf.CeilToInt(distance / step));
            Vector3 direction = delta / distance;

            for (int i = 0; i <= steps; i++)
            {
                Vector3 point = _From + direction * Mathf.Min(i * step, distance);
                Leaf    leaf  = GetClosestOrEmbracingLeaf(point);

                if (leaf != null && leaf.Occupied)
                    return leaf;
            }

            return null;
        }

        public Leaf GetClosestOrEmbracingLeaf(Vector3 _Point)
        {
            return m_RootsGrid.TryGetNode(0, UtilsSpatialGrid.GetBucketIndex(_Point, m_Octree.RootSize), out Node root)
                ? root.GetClosestOrEmbracingLeaf(_Point)
                : null;
        }

        //leaf snapshot reader (debug visualization). Copies leaves out as plain structs under one read-lock hold per
        //root (below) — the consumer keeps no tree references afterwards. Per leaf it also classifies which faces look at
        //free-or-absent space (the boundary crust): the same centre-point sample ClassifyOpenFacesStaged uses, stepped
        //out by less than one min bucket so it lands in the immediately-adjacent cell whatever its size. A face whose
        //neighbour is subdivided finer than this leaf is judged by the cell under the face centre alone — exact in the
        //common case (occupied terminals sit at max layer, so neighbours are same-size or coarser) and an accepted
        //approximation under coarsening resolution regions. Rebuilt by the consumer only on OnRegionChanged, never per
        //frame.
        //
        //_IncludeFree adds the navigable leaves — the side of the graph a route actually travels. It is a parameter
        //rather than the default because free leaves cost the same per-leaf face classification while being the
        //larger population in an open world, and only a consumer that draws them should pay for them.
        //
        //_Graph, when given, collects the stored navigation graph in the same holds: every free neighbour pair once,
        //with its contact point, plus the edge nodes. It needs no face classification, so a graph-only snapshot
        //skips that cost for free leaves. Roots outside _Graph.Scope contribute no edges.
        public void SnapshotLeaves(List<NavLeafInfo> _Result, bool _IncludeFree = false, NavGraphSnapshot _Graph = null)
        {
            float outStep = m_Octree.MinBucketSizeReal * 0.25f;

            Node[] roots = m_RootsGrid.GetAllAtLevel(0);

            //the roots whose free leaves emit edges; a pair whose other leaf lies in a root outside this set is emitted
            //from this side whatever the order, or it would be lost
            HashSet<Vector3Int> graphRoots = null;

            if (_Graph != null)
            {
                graphRoots = new HashSet<Vector3Int>();

                foreach (Node root in roots)
                {
                    if (_Graph.Scope == null || _Graph.Scope.Value.Intersects(root.Bounds))
                        graphRoots.Add(root.Index);
                }
            }

            //one hold per root, not one per snapshot.
            //A whole-world walk of 120–300 K leaves held the lock for up to 1.1 s at streamed-world scale — measured
            //as the second-longest holder in the tree, behind only the A* — and a publish waits for exactly that.
            //Releasing between roots gives the publish a window at every root and costs this reader nothing it
            //promises: the snapshot is a copy of plain structs for DEBUG drawing and censuses, so a picture whose
            //roots come from either side of one publish is as true as the drawing was ever going to be, and no tree
            //reference leaves this method. What must not happen is a release mid-DESCENT — a root's own subtree is
            //walked inside one hold, because the teardown severs replaced subtrees the moment the lock releases.
            //(A root the enumerator hands over after such a publish is torn down rather than corrupt: its forks
            //answer with no children, so the walk yields fewer leaves, never a bad one.) The one thing a mixed-epoch
            //picture can get wrong is an edge between two roots read either side of a publish: drawn twice or not at
            //all. A consumer that needs an exact edge set snapshots a quiescent world.
            foreach (Node root in roots)
            {
                AcquireReadLock();
                DevReadHoldLabel("SnapshotLeaves (one root)");

                try
                {
                    NavGraphSnapshot graph = graphRoots != null && graphRoots.Contains(root.Index) ? _Graph : null;

                    CollectLeaves(root, outStep, _IncludeFree, _Result, graph, graphRoots);
                }
                finally
                {
                    ReleaseReadLock();
                }
            }
        }

        //Recursive half of SnapshotLeaves; the caller holds the read lock for the whole walk.
        void CollectLeaves(Node _Node, float _OutStep, bool _IncludeFree, List<NavLeafInfo> _Result, NavGraphSnapshot _Graph, HashSet<Vector3Int> _GraphRoots)
        {
            if (_Node is Leaf leaf)
            {
                if (leaf.Occupied)
                {
                    _Result.Add(new NavLeafInfo(leaf.Bounds, true, leaf.Interior, ClassifySnapshotFaces(leaf, _OutStep)));

                    return;
                }

                if (_IncludeFree)
                    _Result.Add(new NavLeafInfo(leaf.Bounds, false, false, ClassifySnapshotFaces(leaf, _OutStep)));

                if (_Graph != null)
                    CollectGraph(leaf, _Graph, _GraphRoots);

                return;
            }

            foreach (KeyValuePair<Elements.ForkChildOctIndex, Node> childData in ((Fork) _Node).ChildrenMap)
            {
                CollectLeaves(childData.Value, _OutStep, _IncludeFree, _Result, _Graph, _GraphRoots);
            }
        }

        byte ClassifySnapshotFaces(Leaf _Leaf, float _OutStep)
        {
            Bounds  bounds = _Leaf.Bounds;
            Vector3 center = bounds.center;
            float   half   = bounds.extents.x;

            byte openFaces = 0;

            for (int bit = 0; bit < 6; bit++)
            {
                Vector3 sample = center + FaceNormal(bit) * (half + _OutStep);

                Leaf adjacent = GetClosestOrEmbracingLeaf(sample);

                if (adjacent == null || !adjacent.Occupied)
                    openFaces |= (byte) (1 << bit);
            }

            return openFaces;
        }

        //Emits each undirected pair from the leaf whose centre orders first. The contact point is the same computed
        //from either side (it is the face centre of the smaller leaf), so emitting once loses nothing. The walk runs
        //under the read lock, which is the contract of FreeNeighborsLocked — no copy of the set is taken.
        void CollectGraph(Leaf _Leaf, NavGraphSnapshot _Graph, HashSet<Vector3Int> _GraphRoots)
        {
            Vector3 center = _Leaf.Bounds.center;

            if (_Leaf.IsEdgeNode)
                _Graph.EdgeNodes.Add(_Leaf.Bounds);

            HashSet<Leaf> neighbors = _Leaf.FreeNeighborsLocked;

            if (neighbors == null)
                return;

            foreach (Leaf neighbor in neighbors)
            {
                Vector3 neighborCenter = neighbor.Bounds.center;

                if (!OrdersBefore(center, neighborCenter) &&
                    _GraphRoots.Contains(neighbor.RootIndex))
                    continue;

                _Graph.Edges.Add(new NavGraphEdge(center, _Leaf.GetAdjacentContactPoint(neighbor), neighborCenter));
            }
        }

        //Leaf IDs cannot order a pair: two separately baked obstacles deserialize with overlapping IDs. Two distinct
        //cells never share a centre, and each centre is computed from its own leaf identically from both sides.
        static bool OrdersBefore(Vector3 _A, Vector3 _B)
        {
            if (_A.x != _B.x)
                return _A.x < _B.x;

            if (_A.y != _B.y)
                return _A.y < _B.y;

            return _A.z < _B.z;
        }

        #endregion

        #region Staging → live publish (called only by StagingTransaction)

        //Clones the live roots at the given indices into a working set the operation can prune/complement/rebuild
        //without mutating the live tree. The clones are a faithful replica: an edge to a leaf inside the cloned set
        //is remapped to that leaf's clone, while an edge to a leaf that stayed live is preserved as a forward ref
        //whose matching back-ref the publish repoints. Indices with no live root are skipped. _OriginalToClone is
        //populated with every original->clone leaf pair and is the authority on what a clone is — every later pass
        //asks it rather than deriving cloneness from a root index.
        //Unlocked, which the single-writer invariant makes safe: this read always happens before any concurrent
        //write could be in flight.
        //
        //ring ROOTS are CLONED partially, and this is the one place that decides it. A ring root is in the set only
        //so construction's writes stay off the live tree, and an operation writes to nothing in it but its boundary
        //layer — the leaves against a face looking into a root this operation rebuilds. The interior is roughly two
        //thirds of everything an operation used to copy, so it is SHARED with the live tree instead.
        //_FullRingClone opts back out: a caller that installs subtrees it did not clone, or widens
        //RebuiltRootIndices after this call, cannot be described by the boundary predicate — the pre-baked add is
        //exactly that caller.
        //_Transaction carries the binding per-root verdicts: a DOOMED root is not cloned at all, a NARROWED root is
        //cloned down to its cells' subtrees plus the boundary layers the classification writes to, and a ring root
        //scopes to the plan as before.
        public Dictionary<Vector3Int, Node> CloneRootsForStaging(
                IEnumerable<Vector3Int> _RootIndices,
                HashSet<Vector3Int>     _RingRootIndices,
                //the transaction's frozen BoundaryFaceRootIndices: the roots a boundary layer is owed toward —
                //whole/dropped/complement footprint roots under a bound plan, the whole footprint without one
                HashSet<Vector3Int> _BoundaryFaceRootIndices,
                bool                _FullRingClone,
                StagingTransaction  _Transaction,
                //the transaction's (scratch) original→clone map, empty on entry, filled here
                Dictionary<Leaf, Leaf> _OriginalToClone,
                //a scratch snapshot buffer for the wiring pass below
                List<Leaf> _NeighborBuffer
            )
        {
            Dictionary<Vector3Int, Node> clonedRoots = new Dictionary<Vector3Int, Node>();

            //The two passes are timed apart because they scale differently and a narrowing would hit them
            //differently: the first allocates one node per node of every cloned subtree, the second walks every
            //cloned leaf's neighbour list.
            DevPublishStepStart();

            float outStep = m_Octree.MinBucketSizeReal * 0.25f;

            int devDoomedSkipped  = 0;
            int devNarrowedCloned = 0;

            foreach (Vector3Int rootIndex in _RootIndices)
            {
                if (clonedRoots.ContainsKey(rootIndex))
                    continue;

                if (!m_RootsGrid.TryGetNode(0, rootIndex, out Node liveRoot))
                    continue;

                if (_FullRingClone || !_RingRootIndices.Contains(rootIndex))
                {
                    //A footprint root'S clone FOLLOWS its verdict, and what makes that sound is that nothing here
                    //rests on a geometric guess:
                    //  - the plan is binding: verdicts are decided in the transaction's constructor on the live
                    //    tree, the clone is cut from them here, and RebuildRoots follows them — a divergence throws
                    //    instead of rebuilding whole what was cloned narrow;
                    //  - the graph-shaped writes no geometric predicate could bound are ROUTED by ownership rather
                    //    than bounded: both go through StagingTransaction.EditFreeNeighbor, which defers a
                    //    shared-owner edit to the publish's write lock;
                    //  - refs from surviving shared leaves into replaced originals are repointed under that lock;
                    //  - the scoped predicate below is the same definition the open-face classification scopes its
                    //    writes by, so the classification cannot write to a leaf this clone left shared.

                    //A DOOMED root (no surviving geometry — the publish drops it) is not cloned at all: the clone
                    //existed to keep the prune's ref-cleaning off the live tree, and with no clone there is nothing
                    //to prune — the wiring pass below simply never creates a clone ref into a doomed root, which is
                    //the same end state. The staged-or-live view answers "nothing there" exactly as it does after
                    //the publish drops the root, so the classification needs no special case.
                    if (GlobalOctree.DoomedRootCloneSkipEnabled && _Transaction != null && _Transaction.IsDoomedRoot(rootIndex))
                    {
                        _Transaction.MarkCloneNarrowed();

                        devDoomedSkipped++;

                        continue;
                    }

                    List<StagingTransaction.RebuiltCell> ownCellsAround;

                    if (GlobalOctree.NarrowCloneEnabled                        &&
                        !_FullRingClone                                        &&
                        _Transaction != null                                   &&
                        _Transaction.RebuiltCellsByRoot.ContainsKey(rootIndex) &&
                        (ownCellsAround = _Transaction.CollectCellsAround(rootIndex)) != null)
                    {
                        //the NARROWED footprint clone: the spine down to the planned cells, each cell's subtree in
                        //full (retiring a cell clears its leaves' refs — a write, so a cell may never be shared),
                        //the leaves in/against every cell this or a neighbouring root replaces, and the leaves
                        //against a face toward another footprint root (whose rebuild changes what is adjacent
                        //across that face). Everything else stays SHARED with the live tree, exactly as a ring
                        //root's interior already does — and by the ring's own predicate pair, so the clone and the
                        //classification cannot drift.
                        byte facesTowardFootprint = FacesTowardRoots(rootIndex, _BoundaryFaceRootIndices);

                        Bounds narrowedRootBounds = UtilsSpatialGrid.GetBucketBounds(rootIndex, m_Octree.RootSize);

                        clonedRoots[rootIndex] = liveRoot.CloneSubtreeScoped(
                                _Node => TouchesAnyFace(_Node.Bounds, narrowedRootBounds, facesTowardFootprint, outStep) ||
                                         TouchesAnyCell(_Node.Bounds, ownCellsAround, outStep),
                                _OriginalToClone
                            );

                        _Transaction.MarkCloneNarrowed();

                        devNarrowedCloned++;

                        continue;
                    }

                    clonedRoots[rootIndex] = liveRoot.CloneSubtree(_OriginalToClone);

                    continue;
                }

                //Cells this operation replaces in this root or across one of its faces: a ring leaf against one of
                //them is written to exactly as a leaf facing a rebuilt root is, and a narrowed root is no longer in
                //the rebuilt set — so without this the ring would stop cloning leaves the construction writes to.
                List<StagingTransaction.RebuiltCell> cellsAround = _Transaction?.CollectCellsAround(rootIndex);

                byte   facesTowardRebuilt = FacesTowardRoots(rootIndex, _BoundaryFaceRootIndices);
                Bounds rootBounds         = UtilsSpatialGrid.GetBucketBounds(rootIndex, m_Octree.RootSize);

                //A ring root bordering nothing rebuilt has no boundary layer at all, so this returns the live root
                //itself and the publish re-installs the same instance — a no-op, exactly as it should be.
                clonedRoots[rootIndex] = liveRoot.CloneSubtreeScoped(
                        _Node => TouchesAnyFace(_Node.Bounds, rootBounds, facesTowardRebuilt, outStep) ||
                                 TouchesAnyCell(_Node.Bounds, cellsAround, outStep),
                        _OriginalToClone
                    );
            }

            DevPublishStepEnd("stage: clone subtrees (unlocked)");
            DevPublishStepStart();

            int devRefsWired = 0;

            //second pass: now that the whole region is cloned, wire the clone leaves' FreeNeighbors.
            foreach (KeyValuePair<Leaf, Leaf> pair in _OriginalToClone)
            {
                Leaf original = pair.Key;
                Leaf clone    = pair.Value;

                original.CopyFreeNeighborsTo(_NeighborBuffer);

                foreach (Leaf originalNeighbor in _NeighborBuffer)
                {
                    devRefsWired++;

                    if (_OriginalToClone.TryGetValue(originalNeighbor, out Leaf cloneNeighbor))
                        clone.AddFreeNeighbor(cloneNeighbor);
                    else
                    {
                        //A neighbour in a DOOMED root is dying with it and was deliberately never cloned: wiring a
                        //clone ref at it would hand the published tree a dangling edge, and severing such refs was
                        //the only work the doomed root's clone-then-prune ever did — skipping the wire is the sever.
                        if (devDoomedSkipped > 0 && _Transaction != null && _Transaction.IsDoomedRoot(RootIndexOfLeaf(originalNeighbor)))
                            continue;

                        clone.AddFreeNeighbor(originalNeighbor);
                    }
                }
            }

            DevPublishStepEnd("stage: wire clone neighbours (unlocked)");
            DevCloneWorkCounts(_OriginalToClone.Count, devRefsWired);
            DevCloneVerdictCounts(devDoomedSkipped, devNarrowedCloned);

            return clonedRoots;
        }

        //Works out which outer live leaves must have a back-ref repointed when this transaction publishes, without
        //changing anything. Runs unlocked, before the write lock, for the same reason ClassifyOpenFacesStaged does:
        //measured at 27.58 ms of a 27.59 ms write-lock hold, it was the publish stall, and it needs the live tree
        //for nothing but leaf identity. Its cost is proportional to the clone set while its result is proportional
        //to the ring's outward boundary, so no narrowing changes that ratio much — taking it off the lock removes
        //all of it from frame time.
        //
        //Why unlocked is safe, and this argument must keep holding:
        //  - it READS only the transaction's own clones, whose FreeNeighbors are final by this point, plus outer
        //    live leaves' identity and immutable Bounds — never their mutable ref sets;
        //  - it WRITES nothing, so no reader can observe an intermediate state;
        //  - operations are serialized, and the write lock taken immediately afterwards is the memory barrier.
        //**Rule: this must stay a pure search.** The moment it mutates anything reader-visible it goes back under
        //the lock.
        //
        //Which pairs qualify: a ring leaf's clone REPLACES the original at publish, so every leaf that survives the
        //publish still pointing at that original holds a stale ref. "Survives" is answered by identity — a
        //neighbour is replaced iff it is itself a clone — not by whether its root is in the clone set. Under the
        //partial ring clone the two answers differ: a SHARED ring interior leaf sits in a clone-set root and is not
        //replaced, and its ref into a cloned boundary leaf is exactly the one that would silently rot.
        public List<(Leaf Outer, Leaf Original, Leaf Clone)> ComputeRingBackRefRepoints(
                HashSet<Vector3Int>                           _CloneSetIndices,
                HashSet<Vector3Int>                           _RingRootIndices,
                StagingTransaction                            _Transaction,
                bool                                          _FootprintPartiallyCloned,
                List<(Leaf Outer, Leaf Original, Leaf Clone)> _Repoints,
                List<Leaf>                                    _NeighborBuffer
            )
        {
            DevPublishStepStart();

            Dictionary<Leaf, Leaf> originalToClone = _Transaction.OriginalToClone;

            List<(Leaf Outer, Leaf Original, Leaf Clone)> repoints = _Repoints;

            repoints.Clear();

            int devRingPairs       = 0;
            int devNeighborsWalked = 0;

            foreach (KeyValuePair<Leaf, Leaf> pair in originalToClone)
            {
                Leaf originalLeaf = pair.Key;
                Leaf cloneLeaf    = pair.Value;

                //Only ring leaves can be pointed at from a leaf this operation does not replace — a leaf of a rebuilt
                //root borders only the clone set, and everything it borders there is a clone.
                //that premise DIES with A partially-CLONED footprint. Once a footprint root keeps
                //shared leaves of its own, one of them can point straight at an original this operation clones, and
                //skipping it leaves the shared leaf referencing a leaf the publish removes: measured, when this
                //filter was left in place, as 1 628 DANGLING_REF + 2 114 MISSING_FREE_NEIGHBOR + 510 UNILATERAL_REF
                //in one operation of the hive-tower world these were measured in. So when the footprint is partially
                //cloned every clone is considered,
                //and the identity test below still keeps the result exact.
                if (!_FootprintPartiallyCloned && !_RingRootIndices.Contains(RootIndexOfLeaf(originalLeaf)))
                    continue;

                devRingPairs++;

                cloneLeaf.CopyFreeNeighborsTo(_NeighborBuffer);

                foreach (Leaf neighbor in _NeighborBuffer)
                {
                    devNeighborsWalked++;

                    //itself a clone → installed by this publish, so its own ref set is already the new one.
                    if (_Transaction.IsClone(neighbor))
                        continue;

                    repoints.Add((neighbor, originalLeaf, cloneLeaf));
                }
            }

            DevPublishStepEnd("search ring back-refs (unlocked)");
            DevPublishStepCounts(_CloneSetIndices.Count, originalToClone.Count, devRingPairs, devNeighborsWalked, repoints.Count);

            return repoints;
        }

        //Publishes a region built off-tree (in _StagingGrid, on clones of _CloneSetIndices) into the live
        //m_RootsGrid. must run while holding the write lock so no reader observes a partial transition.
        //
        //The whole clone set (rebuilt affected roots + structurally-unchanged ring clones) is installed wholesale,
        //so all refs inside the set (affected↔affected, affected↔ring, ring↔ring) are already consistent. The refs
        //crossing the clone-set boundary are fixed by applying _RingBackRefRepoints, which ComputeRingBackRefRepoints
        //worked out before the lock was taken — this method deliberately performs only the mutations themselves, so
        //the reader stall is proportional to what CHANGES rather than to what had to be searched to find it.
        //
        //A clone-set index that has no staged root means the cell reverted to empty space (removal path: a footprint
        //root no longer reached by any obstacle) — its live root is dropped. (For the add path this only ever hits a
        //ring index that never had a live root, so the drop is a no-op.)
        public void PublishStagedRegion(
                HashSet<Vector3Int>                           _CloneSetIndices,
                NodeSpatialGrid                               _StagingGrid,
                List<(Leaf Outer, Leaf Original, Leaf Clone)> _RingBackRefRepoints,
                //Free-neighbour edits the free-graph repair asked for on leaves this operation does not replace.
                //Same shape and the same reason as the ring back-refs above: computed unlocked, applied here.
                List<(Leaf Owner, Leaf Neighbour, bool Add)> _DeferredSharedGraphEdits,
                //The roots whose occupancy this operation may have changed — the transaction's RebuiltRootIndices
                //plus the roots it narrowed (RebuiltCellsByRoot.Keys); ring roots are re-wired only and are not in
                //it. Consumed by every stored annotation's invalidation (NavTree.Epoch.cs): what
                //the rebuild did, from the same authority that scopes the classification and the repair.
                HashSet<Vector3Int> _RebuiltRootIndices
            )
        {
            //epoch + per-root rebuild versions, under the write lock, before any reader can see the new tree
            NoteRebuild(_RebuiltRootIndices);

            m_PublishSerial++;

            foreach (Vector3Int rootIndex in _CloneSetIndices)
                m_RootReplacedAtSerial[rootIndex] = m_PublishSerial;

            DevPublishStepStart();

            //1) install every staged root in place of its live counterpart; drop live roots with no staged replacement.
            foreach (Vector3Int rootIndex in _CloneSetIndices)
            {
                if (_StagingGrid.TryGetNode(0, rootIndex, out Node stagedRoot))
                    m_RootsGrid.AddNode(stagedRoot); //AddNode replaces atomically — no transient "root missing" window
                else
                    m_RootsGrid.RemoveNode(0, rootIndex);
            }

            DevPublishStepEnd("install staged roots");
            DevPublishStepStart();

            //2) repoint outer live leaves' back-refs from the original ring leaves onto their installed clones.
            foreach ((Leaf outer, Leaf original, Leaf clone) in _RingBackRefRepoints)
            {
                outer.RemoveFreeNeighbor(original);
                outer.AddFreeNeighbor(clone);
            }

            DevPublishStepEnd("apply ring back-refs");
            DevPublishStepStart();

            //3) apply the free-graph repair's edits to leaves this operation does not replace. Order against (2) is
            //immaterial and the reason is worth stating rather than assuming: (2) retargets a ref from an original
            //this publish replaces onto its clone, while these sever refs to DISCARDED clones and add refs to the
            //leaves that replaced them — the same leaf can own both kinds of edit, but never for the same neighbour.
            if (_DeferredSharedGraphEdits != null)
            {
                foreach ((Leaf owner, Leaf neighbour, bool add) in _DeferredSharedGraphEdits)
                {
                    if (add)
                        owner.AddFreeNeighbor(neighbour);
                    else
                        owner.RemoveFreeNeighbor(neighbour);
                }
            }

            DevPublishStepEnd("apply deferred shared-leaf graph edits");
        }

        //The root (layer-0) index the leaf belongs to, regardless of the leaf's own layer. Derived from the leaf's
        //own grid index (integer floor-division, see Node.RootIndex) rather than from its world position — this is
        //asked ~150 k times per publish. DevAssertRootIndex checks the two agree whenever the equivalence oracle
        //is on; it is a no-op otherwise.
        Vector3Int RootIndexOfLeaf(Leaf _Leaf)
        {
            Vector3Int rootIndex = _Leaf.RootIndex;

            DevAssertRootIndex(_Leaf, rootIndex);

            return rootIndex;
        }

        //DEV: implemented in the [Nav3DDevOnly] NavTree.DevValidation.cs partial; compiles out with it.
        partial void DevAssertRootIndex(Leaf _Leaf, Vector3Int _DerivedRootIndex);

        #endregion

        #region Open-face classification (build-time)

        //Recomputes the OpenFaces classification for the free leaves of the staged roots (affected u boundary ring).
        //
        //Runs on the staged tree, before the publish takes the write lock. Inside it, this was 85 ms of the ~113 ms
        //the lock was held for a runtime obstacle operation — the single largest reason a moving obstacle costs
        //frames, since every main-thread agent query blocks for that window. Nothing about what is computed changed,
        //only when.
        //
        //Safe unlocked because the leaves written are the transaction's own CLONES: unreachable from the live grid
        //until PublishStagedRegion installs them, so no reader can observe a half-classified leaf, and the write
        //lock taken immediately afterwards is the memory barrier. Operations are serialized, so nothing else
        //mutates the tree meanwhile.
        //
        //The one thing the move changed is the adjacency sample: post-publish, sampling the live tree saw "live
        //minus replaced roots, plus the staged ones"; pre-publish that view must be assembled explicitly, which is
        //what GetStagedOrLiveLeaf does, bit for bit.
        //
        //scope: a leaf is re-classified only where this operation can have CHANGED its adjacency — every leaf of a
        //rebuilt root, but in a ring root only the leaves touching a face that looks into one. A deeper ring leaf
        //sees the same six neighbours before and after, and its bits are carried over by Leaf.CloneSubtree. A root
        //rebuilt cell by cell narrows this further, to the leaves its cells hold plus the ones touching them across
        //a face. Equivalence is asserted rather than argued: DevAssertOpenFaceEquivalence re-runs the full
        //classification into a shadow and compares every bit.
        public void ClassifyOpenFacesStaged(
                HashSet<Vector3Int> _CloneSetIndices,
                HashSet<Vector3Int> _RebuiltRootIndices,
                StagingTransaction  _Transaction,
                NodeSpatialGrid     _StagingGrid
            )
        {
            //DEV: sweeps the whole live tree before this operation touches it, so a leaf whose bits went stale under
            //an EARLIER operation can be attributed to that operation instead of being re-reported forever by the
            //equivalence oracle below (which only ever sees the leaves this operation staged).
            DevAuditLiveOpenFaces(_CloneSetIndices, _RebuiltRootIndices, _Transaction);

            //the operation's scratch set (one per root, cleared between roots); a transaction-less caller gets a local one
            HashSet<Leaf> scratch = _Transaction != null ? m_Octree.OperationScratch.ClassifyLeaves : new HashSet<Leaf>();

            //Step just past a face to land in the immediately-adjacent cell, whatever its size. Smaller than the
            //finest leaf (m_MinBucketSizeReal), so it never overshoots a thin neighbour.
            float outStep = m_Octree.MinBucketSizeReal * 0.25f;

            //experiment (`GlobalOctree.ParallelClassifyEnabled`, ships OFF): this pass is 1 478 ms of a 6 969 ms
            //Skyline add — the whole publish phase, since the write-locked install itself is 0.2 ms — and it is
            //root-PARTITIONED by construction: a leaf belongs to exactly one root, so no leaf is visited twice and
            //the only writes are to a leaf's own open-face bits. Everything it reads is final by this point (the
            //staged tree, the live tree, the transaction's cell plan), so the parallel form differs from the
            //sequential one only in the scratch set, which becomes one per root instead of one reused.
            void ClassifyRoot(Vector3Int _RootIndex, HashSet<Leaf> _Scratch)
            {
                Vector3Int rootIndex = _RootIndex;

                if (!_StagingGrid.TryGetNode(0, rootIndex, out Node root))
                    return;

                bool rootRebuilt = _RebuiltRootIndices.Contains(rootIndex);

                //for A ring root: which of its own six faces look into a root this operation TOUCHES at all — the
                //whole footprint, not the post-narrowing rebuilt set. This is the same set the clone cuts the ring's
                //boundary layer against, and exactly one definition must be shared by the two.
                //they had DRIFTED, and it cost a silent defect: Phase 3 removes a
                //cell-rebuilt root from RebuiltRootIndices, so a ring leaf facing such a root was CLONED (the clone
                //asks the footprint) but never re-CLASSIFIED (this pass asked the narrowed set) — and kept the
                //open-face bits `Leaf.CloneSubtree` carried over, which the operation had just made stale. Measured:
                //~30 leaves per run where the scoped result disagreed with a full sweep, every one of them
                //`skipped=True` with `cloneWouldKeep=True`.
                //Narrowing inside a root is unaffected: a narrowed root is still not `rootRebuilt`, so its own leaves
                //are still reached only through their cells.
                HashSet<Vector3Int> boundaryAgainst = _Transaction?.BoundaryFaceRootIndices ?? _RebuiltRootIndices;

                byte facesTowardRebuilt = rootRebuilt ? (byte) 0 : FacesTowardRoots(rootIndex, boundaryAgainst);

                Bounds rootBounds = UtilsSpatialGrid.GetBucketBounds(rootIndex, m_Octree.RootSize);

                //Every cell replaced in this root or across one of its faces — collected once per root, because a
                //per-leaf scan of every cell in the operation is what made this pass cost more than it saved.
                List<StagingTransaction.RebuiltCell> cellsAround = _Transaction?.CollectCellsAround(rootIndex);

                bool touchesRebuiltCell = cellsAround != null;

                //a root that borders nothing rebuilt and holds no rebuilt cell cannot have changed at all (it is in
                //the clone set only because it is a face neighbour of the footprint, and construction merely
                //re-wires its refs)
                if (!rootRebuilt && facesTowardRebuilt == 0 && !touchesRebuiltCell)
                    return;

                _Scratch.Clear();
                root.GetAllContainedFreeLeaves(_Scratch);

                foreach (Leaf leaf in _Scratch)
                {
                    //Ring leaf that touches no face looking into a rebuilt root: adjacency unchanged, keep the bits
                    //the clone carried over. Under the partial ring clone this leaf is not even a clone — it is the
                    //live leaf, shared with the tree readers are traversing — so the same predicate is now what
                    //keeps this unlocked pass from writing into the live tree, not only what saves it work.
                    //Skip a leaf only when nothing this operation replaced can reach it: it is not in a wholly
                    //rebuilt root, it does not sit against a face looking into one, and no narrowed cell touches
                    //it. Under a partial ring clone such a leaf is not even a clone — it is the live leaf, shared
                    //with the tree readers traverse — so this predicate is what keeps an unlocked pass out of the
                    //live tree, not merely what saves it work.
                    if (!rootRebuilt                                                       &&
                        !IsRingBoundaryLeaf(leaf, rootBounds, facesTowardRebuilt, outStep) &&
                        !TouchesAnyCell(leaf.Bounds, cellsAround, outStep))
                    {
                        DevNoteWatchedLeafDecision(leaf, rootRebuilt, true);

                        continue;
                    }

                    leaf.ClearOpenFaces();

                    Vector3 center = leaf.Bounds.center;
                    float   half   = leaf.Bounds.extents.x;

                    for (int bit = 0; bit < 6; bit++)
                    {
                        Vector3 sample = center + FaceNormal(bit) * (half + outStep);

                        //A face is open iff nothing is adjacent across it: no free leaf (graph edge), no occupied leaf
                        //(wall) — just unconstructed navigable space (beyond the obstacle's inflation shell, another
                        //region, or the void). The construction only tiles the obstacle+inflation region, so the open
                        //boundary is the shell edge (usually inside a root), not the root boundary.
                        if (GetStagedOrLiveLeaf(sample, _CloneSetIndices, _StagingGrid) == null)
                            leaf.SetFaceOpen(bit);
                    }

                    DevNoteWatchedLeafDecision(leaf, rootRebuilt, false);
                }
            }

            if (GlobalOctree.ParallelClassifyEnabled)
            {
                //one scratch set per task, not per root: the partitioner hands each worker a slice of the clone set
                Parallel.ForEach(
                        _CloneSetIndices,
                        () => new HashSet<Leaf>(),
                        (_RootIndex, _State, _Local) =>
                        {
                            ClassifyRoot(_RootIndex, _Local);

                            return _Local;
                        },
                        _Local =>
                        {
                        }
                    );
            }
            else
            {
                foreach (Vector3Int rootIndex in _CloneSetIndices)
                    ClassifyRoot(rootIndex, scratch);
            }

            DevAssertOpenFaceEquivalence(_CloneSetIndices, _RebuiltRootIndices, _Transaction, _StagingGrid);

            //DEV: what this operation left at one watched point, seen from inside the operation — the staged tree the
            //classification actually sampled, not the live tree an audit sees afterwards.
            DevReportWatchedLeaf(_CloneSetIndices, _RebuiltRootIndices, _Transaction, _StagingGrid);
        }

        //DEV: re-runs the full (unscoped) classification into a shadow and compares every bit of every clone-set
        //leaf against what the scoped pass above left. Implemented in the [Nav3DDevOnly] NavTree.DevValidation.cs
        //partial and off by default, so it costs one not-taken branch; the scoping gates turn it on.
        partial void DevAssertOpenFaceEquivalence(
                HashSet<Vector3Int> _CloneSetIndices,
                HashSet<Vector3Int> _RebuiltRootIndices,
                StagingTransaction  _Transaction,
                NodeSpatialGrid     _StagingGrid
            );

        //DEV: the same comparison asked of the live tree rather than of this operation's clone set, before the
        //operation changes anything. Implemented in the [Nav3DDevOnly] NavTree.DevValidation.cs partial and off by
        //default, so it costs one not-taken branch. Takes this operation's scoping sets so it can keep them and
        //re-judge them against the next audit's findings — the operation that broke a leaf is the one before the
        //audit that first sees it.
        partial void DevAuditLiveOpenFaces(
                HashSet<Vector3Int> _CloneSetIndices,
                HashSet<Vector3Int> _RebuiltRootIndices,
                StagingTransaction  _Transaction
            );

        //DEV: what the pass decided about the leaf covering the watched point, recorded at the decision itself rather
        //than reconstructed from the sets afterwards. Called from both branches, so it costs no extra predicate.
        partial void DevNoteWatchedLeafDecision(Leaf _Leaf, bool _RootRebuilt, bool _Skipped);

        //DEV: a single watched world point, reported once per operation from inside the staged view. The audit above
        //can only see what an operation left behind; this sees what the operation itself was looking at.
        partial void DevReportWatchedLeaf(
                HashSet<Vector3Int> _CloneSetIndices,
                HashSet<Vector3Int> _RebuiltRootIndices,
                StagingTransaction  _Transaction,
                NodeSpatialGrid     _StagingGrid
            );

        //The full-sweep classification of one leaf, i.e. what the unscoped pass would write. Kept next to the pass
        //it is derived from so the two cannot drift; used by the scoped pass's equivalence assertion.
        internal byte ClassifyLeafOpenFaces(Leaf _Leaf, HashSet<Vector3Int> _CloneSetIndices, NodeSpatialGrid _StagingGrid)
        {
            float   outStep = m_Octree.MinBucketSizeReal * 0.25f;
            Vector3 center  = _Leaf.Bounds.center;
            float   half    = _Leaf.Bounds.extents.x;
            byte    faces   = 0;

            for (int bit = 0; bit < 6; bit++)
            {
                Vector3 sample = center + FaceNormal(bit) * (half + outStep);

                if (GetStagedOrLiveLeaf(sample, _CloneSetIndices, _StagingGrid) == null)
                    faces |= (byte) (1 << bit);
            }

            return faces;
        }

        //Face bits (0..5 = +x,-x,+y,-y,+z,-z) of _RootIndex whose face NEIGHBOUR root is in _Roots.
        static byte FacesTowardRoots(Vector3Int _RootIndex, HashSet<Vector3Int> _Roots)
        {
            byte faces = 0;

            for (int bit = 0; bit < 6; bit++)
            {
                Vector3 normal = FaceNormal(bit);

                Vector3Int neighbor = _RootIndex + new Vector3Int((int) normal.x, (int) normal.y, (int) normal.z);

                if (_Roots.Contains(neighbor))
                    faces |= (byte) (1 << bit);
            }

            return faces;
        }

        //the ring's boundary layer, defined once. Two passes depend on this being the same set: the staging clone
        //copies exactly these leaves and shares the rest with the live tree, and the classification re-classifies
        //exactly these leaves and keeps the carried-over bits for the rest. If the two ever disagreed, the
        //classification would write into a leaf the clone left shared — i.e. into the live tree, unlocked.
        //Does the volume lie in, or against, any cell a narrowed rebuild replaced? The tolerance is the
        //classification's own outward sample step, so "against" means exactly as far as a face sample can see —
        //the same equivalence IsRingBoundaryLeaf keeps with the clone's boundary layer.
        static bool TouchesAnyCell(Bounds _Bounds, List<StagingTransaction.RebuiltCell> _Cells, float _Tolerance)
        {
            if (_Cells == null || _Cells.Count == 0)
                return false;

            Bounds probe = new Bounds(_Bounds.center, _Bounds.size + new Vector3(_Tolerance, _Tolerance, _Tolerance) * 2f);

            foreach (StagingTransaction.RebuiltCell cell in _Cells)
            {
                if (cell.Bounds.Intersects(probe))
                    return true;
            }

            return false;
        }

        static bool IsRingBoundaryLeaf(Leaf _Leaf, Bounds _RootBounds, byte _FacesTowardRebuilt, float _Tolerance)
        {
            return TouchesAnyFace(_Leaf.Bounds, _RootBounds, _FacesTowardRebuilt, _Tolerance);
        }

        //Does the leaf touch any of the given faces of its root? A leaf classifies across a root boundary only when
        //it sits against that boundary — its face sample steps out by less than one min bucket, so a leaf set back
        //from the face samples into its own root and can see nothing outside it.
        static bool TouchesAnyFace(Bounds _LeafBounds, Bounds _RootBounds, byte _Faces, float _Tolerance)
        {
            for (int bit = 0; bit < 6; bit++)
            {
                if ((_Faces & (1 << bit)) == 0)
                    continue;

                Vector3 normal = FaceNormal(bit);
                bool    upward = normal.x + normal.y + normal.z > 0f;

                //the leaf's outward face plane against the root's, along this one axis
                float leafFace = Vector3.Dot(normal, upward ? _LeafBounds.max : _LeafBounds.min);
                float rootFace = Vector3.Dot(normal, upward ? _RootBounds.max : _RootBounds.min);

                if (Mathf.Abs(leafFace - rootFace) <= _Tolerance)
                    return true;
            }

            return false;
        }

        //The tree as it will be once this transaction publishes: staged roots where the transaction staged them, live
        //roots everywhere else. A clone-set root missing from the staging grid resolves to null, which is also what it
        //resolves to after publish (PublishStagedRegion drops clone-set roots whose footprint reverted to empty), so
        //the two agree in that case as well.
        Leaf GetStagedOrLiveLeaf(Vector3 _Point, HashSet<Vector3Int> _CloneSetIndices, NodeSpatialGrid _StagingGrid)
        {
            Vector3Int rootIndex = UtilsSpatialGrid.GetBucketIndex(_Point, m_Octree.RootSize);

            if (_CloneSetIndices.Contains(rootIndex))
            {
                return _StagingGrid.TryGetNode(0, rootIndex, out Node stagedRoot)
                    ? stagedRoot.GetClosestOrEmbracingLeaf(_Point)
                    : null;
            }

            //outside the staged region the live tree is untouched by this operation, so it is already the final answer
            return m_RootsGrid.TryGetNode(0, rootIndex, out Node liveRoot)
                ? liveRoot.GetClosestOrEmbracingLeaf(_Point)
                : null;
        }

        //FaceBit (0..5 = +x,-x,+y,-y,+z,-z, matching UtilsSpatialGrid.FacesDirections) → outward unit normal.
        static Vector3 FaceNormal(int _FaceBit)
        {
            switch (_FaceBit)
            {
                case 0:
                    return Vector3.right;
                case 1:
                    return Vector3.left;
                case 2:
                    return Vector3.up;
                case 3:
                    return Vector3.down;
                case 4:
                    return Vector3.forward;
                default:
                    return Vector3.back;
            }
        }

        #endregion

    }
}
