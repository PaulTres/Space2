using Nav3D.API;
using Nav3D.Common;
using Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.Elements;
using Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.SpatialStorages;
using Nav3D.SpaceProcessing.ObstacleProcessing.UtilityTypes;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using UnityEngine;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree
{
    partial class GlobalOctree
    {

        #region Service methods

        //Body moved to AddObstacleWorker.cs (a standalone class); this stays a thin forwarder
        //with the exact original signature — it's passed as a bare method-group delegate to ObstacleAdditionOperation
        //(GlobalOctree.cs:150,169), which requires this signature unchanged.
        void AddObstacle(ObstacleInfo _ObstacleInfo, ObstacleAdditionProgress _AdditionProgress, CancellationToken _CancellationToken, Action _OnFinish, bool _EditMode)
            => m_AddObstacleWorker.Execute(_ObstacleInfo, _AdditionProgress, _CancellationToken, _OnFinish, _EditMode);

        //The add operation, reachable from UpdateObstacleWorker (a standalone class): a move of an obstacle that turns
        //out not to be registered is an addition, and the worker delegates rather than duplicating the add's logic.
        internal void ExecuteAddOperation(
                ObstacleInfo             _ObstacleInfo,
                ObstacleAdditionProgress _AdditionProgress,
                CancellationToken        _CancellationToken,
                Action                   _OnFinish,
                bool                     _EditMode
            )
            => m_AddObstacleWorker.Execute(_ObstacleInfo, _AdditionProgress, _CancellationToken, _OnFinish, _EditMode);

        //dev regression guard (implemented in the [Nav3DDevOnly] GlobalOctree.DevValidation.cs partial; the call is
        //compiled out automatically once the release preparer strips that implementation). Runs on the staged tree just
        //before publish and reports any surviving triangle in the removed obstacle's footprint left without occupied
        //coverage — an UNCOVERED_TRIANGLE attributed to the remove op (earlier than the post-op validator's report).
        partial void DevValidateRemovalCoverage(int _ID, HashSet<Vector3Int> _FootprintRoots, HashSet<Vector3Int> _StagedRootIndices, string _OpLabel);

        //dev regression guard (implemented in the [Nav3DDevOnly] GlobalOctree.DevValidation.cs partial; auto-stripped
        //with it). Runs on the staged tree just before AddObstacle publishes and reports any triangle of this op's
        //clusters left without an occupied leaf — an UNCOVERED_TRIANGLE attributed to the add op.
        partial void DevValidateAdditionCoverage(
                int               _ID,
                TriangleCluster[] _Clusters,
                //roots actually present in the staging tree (footprint + ring); triangles outside them are covered by
                //untouched live roots and must be skipped, not probed against the (partial) staging tree.
                HashSet<Vector3Int> _StagedRootIndices
            );

        //dev uniform staged-coverage assertion (implemented in the [Nav3DDevOnly] GlobalOctree.DevValidation.cs
        //partial; auto-stripped with it) — the post-build pre-publish coverage guard, made uniform, for the
        //skeleton ops without a specialized guard (region update, pre-baked add). Probes clusters registered on the
        //staged roots plus the op's in-flight (deferred-registration) clusters clipped to the staged roots.
        partial void DevValidateStagedCoverage(
                string                                             _OpLabel,
                HashSet<Vector3Int>                                _RebuiltRootIndices,
                HashSet<Vector3Int>                                _StagedRootIndices,
                HashSet<Vector3Int>                                _InFlightRootIndices,
                IEnumerable<TriangleCluster>                       _InFlightClusters,
                IReadOnlyDictionary<TriangleCluster, Vector3Int[]> _InFlightClusterRoots,
                IReadOnlyDictionary<TriangleCluster, HashSet<int>> _InFlightClusterOwners
            );

        //dev root-path tracing (implemented in the [Nav3DDevOnly] GlobalOctree.DevValidation.cs partial; the calls —
        //arguments included — compile out with it). Ops on the staging skeleton record which path handled each staged
        //root (verbatim / contested / pic-reconstruct / prune branch…) plus free-form op notes;
        //DevValidateStagedCoverage prints the trace per violating root, so a staged-coverage hole names the exact
        //construction path that produced it instead of requiring another instrumented re-run.
        partial void DevClearRootPathTrace();
        partial void DevTraceRootPath(Vector3Int               _RootIndex,   string _Path);
        partial void DevTraceRootsPath(IEnumerable<Vector3Int> _RootIndices, string _Path);
        partial void DevTraceOpNote(string                     _Note);

        //DEV: the staged-coverage guard's positive control — withholds the triangles the control asks for from the set
        //an op is about to rebuild from, so the staged region genuinely loses coverage and the guard must report it
        //(implemented in the [Nav3DDevOnly] GlobalOctree.DevValidation.cs partial; the call compiles out with it).
        //A declared seam, not a plain static: the shipped PreBakedAddWorker calls it, so a direct call to the dev
        //partial's own method left the release build with a dangling reference (found 2026-09-08 by the Pro
        //stripped-compile gate, which is the only thing that ever compiles the stripped tree).
        static partial void DevWithholdTriangles(HashSet<Triangle> _Triangles);

        //DEV: the three successive triangle sets of the pre-baked add's reconstruction, so a staged-coverage hole can
        //name the filter that dropped the triangle instead of only reporting that it is gone (implemented in the
        //[Nav3DDevOnly] GlobalOctree.DevValidation.cs partial; compiles out with it).
        partial void DevRecordPicTriangleSets(HashSet<Triangle> _Considered, HashSet<Triangle> _Reused, HashSet<Triangle> _Storage);

        //DEV: the cut subtrees' bounds — the exact volume PruneOctreeForRegionsAddition scopes its reusable-triangle
        //filter to. A triangle that the rebuild needed but that touches none of these names the scoping as the defect.
        partial void DevRecordPruneCutBounds(Bounds[] _CutBounds);

        //dev op-timing hooks (implemented in the [Nav3DDevOnly] GlobalOctree.DevTimings.cs partial; the calls compile
        //out with it — a partial method without an implementation is removed at compile time, arguments included).
        //Start/Finish bracket a successful operation; Abandon (called in each op's finally) discards an unfinished one
        //so cancelled/failed/no-op runs don't skew the recorded timing baseline; PublishStart/End bracket the write-locked
        //publish — the reader-stall window. Opt-in via OpTimingBaselineRecorder (Tests/); zero-cost when disabled.
        partial void DevOpTimerStart(string _OpName);
        partial void DevOpTimerPublishStart();
        partial void DevOpTimerPublishEnd();
        partial void DevOpTimerFinish();
        partial void DevOpTimerAbandon();

        //dev pre-baked cache watch (implemented in the [Nav3DDevOnly] GlobalOctree.DevBakeCacheWatch.cs partial, Pro
        //only; compiles out with it): a fingerprint of every cached root taken as deserialized and after each
        //operation, so an operation that writes into the cache's node objects — installed by reference by the
        //pre-baked add — is named the moment it does.
        partial void DevBakeCacheWatchLoaded(int     _DeserializedLeaves, int _DeserializedLayer0Leaves);
        partial void DevBakeCacheWatchOpStart(string _OpName);
        partial void DevBakeCacheWatchOpFinish();

        //Sub-phases within the write-locked publish. The publish hold turned out to be ~109 ms per operation — the
        //dominant cost of a runtime obstacle change, because every main-thread octree query blocks behind it — and
        //"the publish is slow" is not actionable without knowing which of its three steps is slow.
        partial void DevPublishPhaseStart();
        partial void DevPublishPhaseEnd(string _Phase);

        //Sub-phases of the whole operation, outside the publish. The publish is fully attributed by the
        //hooks above and accounts for ~150 ms of a ~495 ms add; the remaining ~345 ms — clustering, the staging
        //clone, the rebuild, the free-graph repair — had no instrumentation at all, which is why the moving-obstacle
        //tracking ceiling (~1 completed re-add/s) could not be attacked without guessing first.
        //Separate watch from the publish phases on purpose: these brackets enclose the publish ones, and one
        //shared stopwatch cannot nest.
        partial void DevOpPhaseStart();
        partial void DevOpPhaseEnd(string _Phase);

        //The third level: steps inside an op phase. Once the partial ring clone landed, RebuildRoots was
        //the largest single item left in an operation (195 ms of 366) and had never been broken down — "the rebuild is
        //slow" is not actionable without knowing which of its six steps is slow. Unlike the two levels above, these
        //brackets nest (a geometry-view step inside the rebuild's) and some of them run on the construction's worker
        //threads, so the implementation keeps a per-thread stack of watches rather than one stopwatch.
        //Behind its own flag (DevStepTimingsEnabled), not DevTimingsEnabled: the protocol's own op-timing baseline
        //must not silently gain this instrumentation's per-call overhead while the baseline's drift question is
        //open.
        partial void DevStepStart();
        partial void DevStepEnd(string _Step);

        //Counters next to those timings: a step that is slow because it does a lot of necessary work and one that is
        //slow because it walks thousands of candidates to make a handful of edges need opposite fixes, and the timing
        //alone cannot tell them apart (the same distinction the ring back-ref search's walked-vs-moved counts drew).
        partial void DevCount(string _Counter, long _Amount);

        //Cheap per-operation work count (a handful of calls per op: roots rebuilt, edge leaves gathered, deferred edits) —
        //recorded whenever timings are enabled, unlike DevCount, so the protocol arm carries the work counters without the
        //third level's per-call overhead.
        partial void DevCountWork(string _Counter, long _Amount);

        //The shipped seam for a work counter raised by a type that holds no octree (a cluster's solids, a part's BVH, the
        //chunk builder): null in a release, wired to the dev timings while they are enabled. A null check per call is
        //its whole production cost.
        internal static Action<string, long> DevWorkSink { get; set; }

        //Takes the grid rather than its size: the size costs a walk, which must not be paid when the instrumentation
        //is merely compiled in but switched off (a plain DevCount argument would be evaluated regardless).
        partial void DevCountGridLeaves(LeavesSpatialGrid _Grid);

        //The oracle for the face-cell index that replaced FormEdgeConnectionsForAddedRoots' pairwise sweep: it
        //re-runs the sweep and asserts that every pair it calls adjacent is linked in both directions in the tree the
        //index actually produced. Off by default (it restores the very cost the index removed).
        partial void DevCheckEdgeConnectionEquivalence(HashSet<Leaf> _AddedEdgeLeaves, HashSet<Leaf> _NeighborEdgeLeaves, StagingTransaction _Transaction);

        //Internal, non-partial forwarders for StagingTransaction.cs (a standalone class, so it can
        //no longer call the above private partial hooks directly). must stay separate methods, not just widening the
        //partial methods above to internal: an "extended" (non-private) partial method requires an implementation to
        //exist, which would break release builds once GlobalOctree.DevTimings.cs is stripped. These forwarders are
        //always compiled; their bodies elide to nothing when the partial method they call has no implementation, so
        //the zero-cost-when-stripped property is preserved exactly, just with one extra (trivially inlined) hop.
        internal void DevOpTimerPublishStartHook()          => DevOpTimerPublishStart();
        internal void DevOpTimerPublishEndHook()            => DevOpTimerPublishEnd();
        internal void DevPublishPhaseStartHook()            => DevPublishPhaseStart();
        internal void DevPublishPhaseEndHook(string _Phase) => DevPublishPhaseEnd(_Phase);
        internal void DevOpTimerAbandonHook()               => DevOpTimerAbandon();

        //Same fix, extended: AddObstacleWorker.cs/RemoveObstacleWorker.cs/RegionUpdateWorker.cs
        //(standalone classes) call these dev hooks directly in their own bodies.
        internal void DevOpTimerStartHook(string _OpName)
        {
            DevOpTimerStart(_OpName);
            DevBakeCacheWatchOpStart(_OpName);
        }

        //the cache watch runs after the timer stops, so its walk never lands in a recorded op timing
        internal void DevOpTimerFinishHook()
        {
            DevOpTimerFinish();
            DevBakeCacheWatchOpFinish();
        }

        internal void DevOpPhaseStartHook()            => DevOpPhaseStart();
        internal void DevOpPhaseEndHook(string _Phase) => DevOpPhaseEnd(_Phase);

        //Same fix again, for RootBuilder.cs (a standalone class): ConstructRoots's own steps — dispatch, the wait on
        //the root tasks, the AddNode loop, and Node.Create/ComplementAbsentChildren inside each root's task.
        internal void DevStepStartHook()           => DevStepStart();
        internal void DevStepEndHook(string _Step) => DevStepEnd(_Step);

        internal void DevCountWorkHook(string _Counter, long _Amount) => DevCountWork(_Counter, _Amount);

        //CPU bracket for a pool task the operation spawned — called from Fork / RootBuilder / TriangleStorage, which
        //hold no octree instance, hence static partial + static forwarder (same zero-cost-when-stripped contract).
        static partial void DevTaskCpuStartImpl();
        static partial void DevTaskCpuEndImpl();

        internal static void DevTaskCpuStart()                                                         => DevTaskCpuStartImpl();
        internal static void DevTaskCpuEnd()                                                           => DevTaskCpuEndImpl();
        internal        void DevClearRootPathTraceHook()                                               => DevClearRootPathTrace();
        internal        void DevTraceRootsPathHook(IEnumerable<Vector3Int> _RootIndices, string _Path) => DevTraceRootsPath(_RootIndices, _Path);

        internal void DevValidateAdditionCoverageHook(int _ID, TriangleCluster[] _Clusters, HashSet<Vector3Int> _StagedRootIndices)
            => DevValidateAdditionCoverage(_ID, _Clusters, _StagedRootIndices);

        internal void DevValidateStagedCoverageHook(
                string                                             _OpLabel,
                HashSet<Vector3Int>                                _RebuiltRootIndices,
                HashSet<Vector3Int>                                _StagedRootIndices,
                HashSet<Vector3Int>                                _InFlightRootIndices,
                IEnumerable<TriangleCluster>                       _InFlightClusters,
                IReadOnlyDictionary<TriangleCluster, Vector3Int[]> _InFlightClusterRoots,
                IReadOnlyDictionary<TriangleCluster, HashSet<int>> _InFlightClusterOwners = null
            )
            => DevValidateStagedCoverage(_OpLabel, _RebuiltRootIndices, _StagedRootIndices, _InFlightRootIndices, _InFlightClusters, _InFlightClusterRoots, _InFlightClusterOwners);

        internal void DevValidateRemovalCoverageHook(int _ID, HashSet<Vector3Int> _FootprintRoots, HashSet<Vector3Int> _StagedRootIndices, string _OpLabel)
            => DevValidateRemovalCoverage(_ID, _FootprintRoots, _StagedRootIndices, _OpLabel);

        //Same fix, extended: PreBakedAddWorker.cs (a standalone class) calls these two
        //directly in ProcessIntersectingCachedClusters/AddObstacleFromCache.
        internal void DevTraceRootPathHook(Vector3Int _RootIndex, string _Path) => DevTraceRootPath(_RootIndex, _Path);

        internal void DevRecordPicTriangleSetsHook(HashSet<Triangle> _Considered, HashSet<Triangle> _Reused, HashSet<Triangle> _Storage)
            => DevRecordPicTriangleSets(_Considered, _Reused, _Storage);

        internal        void DevTraceOpNoteHook(string                  _Note)      => DevTraceOpNote(_Note);
        internal static void DevWithholdTrianglesHook(HashSet<Triangle> _Triangles) => DevWithholdTriangles(_Triangles);

        //Concrete geometry delta a RebuildRoots call applies against the registry's current state: which of this
        //op's clusters are being removed (their triangles/bounds must be excluded even though
        //m_NavTree.TriangleClusterStorage hasn't caught up yet — that removal is deferred to publish) and which are being
        //added (in-flight, not yet registered — used by future add/pre-baked-add steps). Both empty for a region
        //update, which changes only subdivision depth, never geometry.
        //internal (not private): AddObstacleWorker/RemoveObstacleWorker/RegionUpdateWorker (standalone
        //classes) construct these to pass into RebuildRoots (which stays on GlobalOctree).
        internal sealed class GeometryDelta
        {
            public static GeometryDelta None { get; } = new GeometryDelta(null, null);

            public IReadOnlyCollection<TriangleCluster> RemovedClusters { get; }
            public IReadOnlyCollection<TriangleCluster> AddedClusters   { get; }

            //volumes whose cells change although no delta cluster reaches them: the inside of a body this operation opens
            //or closes (GlobalOctree.WithClosureChanges)
            public IReadOnlyCollection<Bounds> ClosureChanges { get; }

            //where the delta's geometry actually changed, when the operation knows it narrower than its clusters' bounds
            //(the partial update: the changed chunks' boxes, inflated like a safe bound); null = the clusters' bounds.
            public IReadOnlyCollection<Bounds> CutVolumes { get; }

            public GeometryDelta(
                    IReadOnlyCollection<TriangleCluster> _Removed,
                    IReadOnlyCollection<TriangleCluster> _Added,
                    IReadOnlyCollection<Bounds>          _ClosureChanges = null,
                    IReadOnlyCollection<Bounds>          _CutVolumes     = null
                )
            {
                RemovedClusters = _Removed        ?? Array.Empty<TriangleCluster>();
                AddedClusters   = _Added          ?? Array.Empty<TriangleCluster>();
                ClosureChanges  = _ClosureChanges ?? Array.Empty<Bounds>();
                CutVolumes      = _CutVolumes;
            }
        }

        //The op's single combined geometry view: every surviving cluster crossing the footprint, one
        //shared TriangleStorage instead of a per-root rebuild (the BVH already scopes IsIntersect(rootBounds), so
        //per-root storage was pure duplication), plus their safe bounds (the existing octant-skip filter
        //Node.Create/ComplementAbsentChildren take as _TriangleClusters) and the clusters themselves (used to test
        //whether a root still has any reason to exist).
        sealed class RootGeometryView
        {
            public IOccupancyStorage Storage           { get; }
            public Bounds[]          ClusterBounds     { get; }
            public TriangleCluster[] SurvivingClusters { get; }

            public RootGeometryView(IOccupancyStorage _Storage, Bounds[] _ClusterBounds, TriangleCluster[] _SurvivingClusters)
            {
                Storage           = _Storage;
                ClusterBounds     = _ClusterBounds;
                SurvivingClusters = _SurvivingClusters;
            }
        }

        RootGeometryView BuildRootGeometryView(Bounds[] _ViewBoxes, GeometryDelta _Delta)
        {
            DevStepStart();

            HashSet<TriangleCluster> survivingClusters = new HashSet<TriangleCluster>();

            //collected straight into the set — the per-root query used to allocate a set, a list and an array each
            foreach (Bounds box in _ViewBoxes)
                m_NavTree.TriangleClusterStorage.CollectCrossingBoundables(box, survivingClusters);

            //m_NavTree.TriangleClusterStorage removal is deferred to the caller's locked publish (ID-map atomicity), so
            //a cluster this op is removing is still visible here — exclude it explicitly.
            survivingClusters.ExceptWith(_Delta.RemovedClusters);

            //in-flight clusters not yet registered (future add/pre-baked-add steps) — empty for remove/region-update.
            foreach (TriangleCluster addedCluster in _Delta.AddedClusters)
            foreach (Bounds box in _ViewBoxes)
                if (addedCluster.Bounds.Intersects(box))
                {
                    survivingClusters.Add(addedCluster);
                    break;
                }

            TriangleCluster[] survivingClustersArray = survivingClusters.ToArray();

            DevStepEnd("geometry view: crossing clusters");

            //the clusters' own parts under one index, no triangle set and no BVH of the operation's own; the solids are
            //the same clusters, uncut, since a ray from a cell leaves the footprint before it has met the whole surface
            DevStepStart();

            ClusterForestStorage forest = new ClusterForestStorage(survivingClustersArray)
            {
                Solids = CreateSolidRegions(survivingClustersArray, _Delta)
            };

            DevCountWork("geometry view: parts indexed", forest.PartCount);

            DevStepEnd("geometry view: part forest + solids");

            return new RootGeometryView(forest, survivingClustersArray.Select(_Cluster => _Cluster.Bounds).ToArray(), survivingClustersArray);
        }

        //The single rebuild primitive: a root's subtree is a pure function of the geometry and resolution regions
        //crossing it — Subtree(root) == Build(GeometryAt(root), RegionsAt(root)). Every root in the footprint is
        //pruned then rebuilt against one per-op geometry view. Full tear-down and from-scratch rebuild is the
        //default, except for a root that is both gaining new geometry and already hosts an oversized cluster (see
        //IsRootComplementEligible), where a full rebuild would re-derive an already-correct large subtree for no
        //benefit; that one case complements the existing node in place. Runs against the already-staged tree.
        //internal, not private, and it stays on GlobalOctree by design: it touches m_ActiveRootsGrid,
        //ConstructionProgress, DevTraceRootPath and several other members directly, and the standalone workers call
        //it through their back-reference.
        //_AllowNarrowing = false forces every footprint root to be rebuilt whole. The pre-baked add passes it and
        //must: it installs bake-time subtrees it never cloned and widens the transaction's rebuilt-root set to the
        //whole clone set afterwards, so its notion of "what this operation replaces" is the root, not a cell inside
        //one. Narrowing it leaves stale occupancy behind.
        internal void RebuildRoots(
                HashSet<Vector3Int> _FootprintRootIndices,
                GeometryDelta       _Delta,
                LeavesSpatialGrid   _LeavesSpatialGrid,
                CancellationToken   _CancellationToken,
                //The operation's own transaction, so a narrowed root can record which cells it replaced: the two
                //passes scoped by the rebuilt set need that, and after narrowing they cannot get it from the root
                //index alone. A caller passing null gets whole-root rebuilds, which is what the pre-baked add wants.
                StagingTransaction _Transaction    = null,
                bool               _AllowNarrowing = true
            )
        {
            if (_FootprintRootIndices.Count == 0)
                return;

            //Where this operation's occupancy can change at all — everything outside it is provably untouched, which
            //is what lets a root be rebuilt in part.
            List<RootBuilder.SubtreeRebuildTarget> subtreeTargets = new List<RootBuilder.SubtreeRebuildTarget>();

            List<List<(Fork Fork, Fork Parent, ForkChildOctIndex Slot)>> narrowedSpines = new List<List<(Fork, Fork, ForkChildOctIndex)>>();

            //The boxes the geometry view must cover: a whole root for a root rebuilt or complemented whole, the planned
            //cells for a narrowed one. Decided by the loop below, so the view is built after it — a view over every
            //footprint root collected the whole island's triangles for a crater.
            List<Bounds> viewBoxes = new List<Bounds>();

            //Step brackets (this whole method, and the prune loop's four inner ones) exist because RebuildRoots is the
            //largest item left in an operation and had never been broken down.
            DevStepStart();

            List<Vector3Int>             rootsToRebuild    = new List<Vector3Int>(_FootprintRootIndices.Count);
            Dictionary<Vector3Int, Node> rootsToComplement = new Dictionary<Vector3Int, Node>();

            foreach (Vector3Int rootIndex in _FootprintRootIndices)
            {
                Bounds rootBounds      = UtilsSpatialGrid.GetBucketBounds(rootIndex, RootSize);
                bool   hasExistingNode = m_ActiveRootsGrid.TryGetNode(0, rootIndex, out Node existingNode);

                //the plan is binding (Phase 2): when the transaction carries per-root VERDICTS (decided in its
                //constructor, on the live tree, before anything was cloned), every decision below follows the
                //verdict instead of re-deriving it — because the clone was cut from that same plan, and a rebuild
                //that decides differently writes into leaves the clone left shared with the live tree. A root the
                //plan does not know (built into fresh space, or a transaction that never planned) keeps the legacy
                //per-root decisions, which are then trivially consistent with a whole clone.
                StagingTransaction.RootRebuildVerdict verdict = StagingTransaction.RootRebuildVerdict.WHOLE;

                bool hasVerdict = _AllowNarrowing        &&
                                  _Transaction != null   &&
                                  _Transaction.PlanBound &&
                                  _Transaction.TryGetVerdict(rootIndex, out verdict);

                if (hasExistingNode && (hasVerdict
                        ? verdict == StagingTransaction.RootRebuildVerdict.COMPLEMENT
                        : IsRootComplementEligible(rootBounds, _Delta)))
                {
                    //Keep the existing node in place — ConstructRoots's own complement branch (a Fork present in
                    //_PartiallyConstructedRoots) fills only the new gaps, never re-deriving the oversized existing
                    //cluster's already-correct occupancy this root carries.
                    //this branch OUTRANKS the NARROWED one below, so a root the transaction PLANNED to narrow can
                    //land here — and then only the plan says "28 cells", while the complement adds nodes anywhere in
                    //the root. The transaction has to be told, or the classification and the repair keep scoping to
                    //cells that describe nothing.
                    _Transaction?.RestoreWholeRootRebuild(rootIndex);

                    rootsToComplement.Add(rootIndex, existingNode);
                    viewBoxes.Add(rootBounds);
                    DevTraceRootPath(rootIndex, "rebuild-complement(oversized-neighbor)");
                    continue;
                }

                //the NARROWED path — rebuild only the cells the cut volume touches and leave the rest of the root
                //(its nodes, its leaves, their refs) exactly as it stands. Two preconditions beyond the cell set
                //itself: the operation must have a cut volume (a region update does not, and must fall through), and
                //the root must still be justified by surviving geometry — a root that reverted to empty space has to
                //be DROPPED, which is a whole-root decision.
                //The narrowed path runs against the plan the transaction made on the live tree before it cloned —
                //never a set re-derived here, because the clone was scoped to that plan and a different set would
                //reach leaves the clone deliberately left shared with the live tree.
                if (hasExistingNode                                                                                               &&
                    _AllowNarrowing                                                                                               &&
                    _Transaction != null                                                                                          &&
                    _Transaction.RebuiltCellsByRoot.TryGetValue(rootIndex, out List<StagingTransaction.RebuiltCell> plannedCells) &&
                    //a bound plan issues a NARROWED verdict only where surviving geometry justifies the root, so the
                    //view test is only re-derived for the legacy (unbound) path
                    (hasVerdict || RootHasSurvivingGeometry(rootBounds, _Delta)) &&
                    TryResolvePlannedCells(
                            rootIndex,
                            plannedCells,
                            out List<RootBuilder.SubtreeRebuildTarget> rootTargets,
                            out List<(Fork Fork, Fork Parent, ForkChildOctIndex Slot)> rootSpine
                        ))
                {
                    DevStepStart();

                    foreach (RootBuilder.SubtreeRebuildTarget target in rootTargets)
                        PruneRebuildCell(target, _LeavesSpatialGrid);

                    DevStepEnd("prune: narrowed cells + their neighbourhood");

                    subtreeTargets.AddRange(rootTargets);
                    narrowedSpines.Add(rootSpine);

                    //the view covers the cells and whatever touches their faces (inclusive, as the occupancy test is)
                    foreach (StagingTransaction.RebuiltCell cell in plannedCells)
                        viewBoxes.Add(cell.Bounds.Enlarge(VIEW_CELL_MARGIN));

                    DevCountWork("rebuild: cells rebuilt in place of a root", rootTargets.Count);
                    DevTraceRootPath(rootIndex, $"rebuild-narrowed({rootTargets.Count} cells)");

                    continue;
                }

                //Everything from here down rebuilds the root whole (or drops it), so a plan that narrowed it is no
                //longer true of anything: the root is put back into the rebuilt set and its cells are dropped. The
                //three ways to arrive here are a root whose surviving geometry no longer justifies it, a plan the
                //staged tree cannot present, and narrowing being off or unavailable — all of them decided per root,
                //here, after the plan was made in the transaction's constructor.
                _Transaction?.RestoreWholeRootRebuild(rootIndex);

                if (hasExistingNode)
                {
                    DevCountWork("rebuild: roots rebuilt WHOLE", 1);

                    DevStepStart();

                    //the two per-root sets are the operation's scratch sets (cleared per root) — each used to be a
                    //fresh HashSet holding a whole root's leaves, per root, per operation
                    OperationScratch pruneScratch = ActiveScratch;

                    HashSet<Leaf> containedLeaves = pruneScratch?.PruneContained ?? new HashSet<Leaf>();
                    HashSet<Leaf> neighborLeaves  = pruneScratch?.PruneNeighbors ?? new HashSet<Leaf>();
                    List<Leaf>    neighborBuffer  = pruneScratch?.NeighborBuffer ?? new List<Leaf>();

                    containedLeaves.Clear();
                    existingNode.GetAllContainedFreeLeaves(containedLeaves);

                    DevStepEnd("prune: gather contained leaves");

                    DevStepStart();

                    GetAllNeighborsAndCleanRefs(containedLeaves, _LeavesSpatialGrid, neighborLeaves, neighborBuffer);

                    DevStepEnd("prune: neighbours + clean refs");

                    DevStepStart();

                    _LeavesSpatialGrid.SetLeaves(neighborLeaves);

                    DevStepEnd("prune: seed grid with neighbours");

                    DevStepStart();

                    m_ActiveRootsGrid.RemoveNode(0, rootIndex);

                    DevStepEnd("prune: drop root node");

                    //hub severing: each of these scratch sets holds a whole root's
                    //leaves — containedLeaves the staged clones this prune just detached, neighborLeaves their
                    //neighbourhood (now copied into the construction grid). A conservatively pinned stale set
                    //would retain all of them individually; the sets' job is done — sever them.
                    containedLeaves.Clear();
                    neighborLeaves.Clear();
                }

                //a root survives only where some surviving cluster's safe (inflated) bounds still reach it (the
                //core model: "roots exist only where obstacles are") — otherwise it stays dropped, no rebuild. Under
                //a bound plan the verdict already made that call (DOOMED = drop; anything else that reaches this
                //line = survivors exist), and a DOOMED root whose clone was skipped never even had a staged node to
                //prune above — the publish drops its live root by the ordinary "no staged replacement" rule.
                if (hasVerdict
                        ? verdict != StagingTransaction.RootRebuildVerdict.DOOMED
                        : RootHasSurvivingGeometry(rootBounds, _Delta))
                {
                    rootsToRebuild.Add(rootIndex);
                    viewBoxes.Add(rootBounds);
                }
                else
                    DevTraceRootPath(rootIndex, "rebuild-dropped(no surviving geometry)");
            }

            DevStepEnd("rebuild: prune footprint roots");

            DevStepStart();

            RootGeometryView view = BuildRootGeometryView(viewBoxes.ToArray(), _Delta);

            DevStepEnd("rebuild: geometry view");

            DevStepStart();

            if (rootsToRebuild.Count > 0 || rootsToComplement.Count > 0)
                m_RootBuilder.ConstructRoots(
                        rootsToRebuild.Concat(rootsToComplement.Keys).ToArray(),
                        //complement-eligible roots get their own narrowly-scoped per-root storage (existing crossing
                        //clusters + this op's added clusters, same helper AddObstacle already used before this
                        //convergence); every other root shares the one per-op view.Storage.
                        _RootIndex => rootsToComplement.ContainsKey(_RootIndex)
                            ? (IOccupancyStorage) BuildRootCombinedStorage(_RootIndex, _Delta.AddedClusters)
                            : view.Storage,
                        view.ClusterBounds,
                        _LeavesSpatialGrid,
                        _CancellationToken,
                        //non-empty only for complement-eligible roots — forces ConstructRoots's complement branch
                        //for exactly those; every other root is absent from this dict, forcing Node.Create's
                        //from-scratch path (the delete-by-default default lane).
                        rootsToComplement,
                        m_ActiveRootsGrid,
                        ConstructionProgress,
                        m_BaseLayersCount
                    );

            DevStepEnd("rebuild: ConstructRoots");

            DevStepStart();

            m_RootBuilder.ConstructSubtrees(
                    subtreeTargets,
                    _RootIndex => view.Storage,
                    view.ClusterBounds,
                    _LeavesSpatialGrid,
                    _CancellationToken,
                    ConstructionProgress,
                    m_BaseLayersCount
                );

            DevStepEnd("rebuild: ConstructSubtrees");

            //A cell that lost its reason to exist takes its slot with it, which can leave the fork above it holding
            //nothing — a node covering a volume the tree no longer represents. Swept bottom-up — the spine list is
            //built deepest-first (TryResolvePlannedCells walks from the cell's parent layer toward the root), so a
            //forward iteration visits the deepest fork first and a chain of childless forks collapses in one pass.
            foreach (List<(Fork Fork, Fork Parent, ForkChildOctIndex Slot)> spine in narrowedSpines)
            {
                foreach ((Fork Fork, Fork Parent, ForkChildOctIndex Slot) spineEntry in spine)
                {
                    if (spineEntry.Fork.ChildrenMap.Count == 0)
                        spineEntry.Parent.ChildrenMap.Remove(spineEntry.Slot);
                }
            }

            if (_CancellationToken.IsCancellationRequested)
                return;

            DevStepStart();

            PurgeGridGhosts(_LeavesSpatialGrid);

            DevStepEnd("rebuild: PurgeGridGhosts");

            DevStepStart();

            _LeavesSpatialGrid.FormLeavesConnections(ConstructionProgress, _Transaction);

            DevStepEnd("rebuild: FormLeavesConnections");

            //the deferred-edit queue after each pass that feeds it, so a move in the cumulative repair counter can
            //be attributed to the pass that produced it (grid pass here, edge pass below, the repair last)
            DevCountWork("grid pass: edits deferred so far", _Transaction?.DeferredSharedGraphEdits.Count ?? 0);

            //the construction grid's own size — what the two grid-wide passes above and below are proportional to
            DevCountGridLeaves(_LeavesSpatialGrid);

            DevStepStart();

            FormEdgeConnectionsForAddedRoots(_FootprintRootIndices, false, _Transaction);

            DevStepEnd("rebuild: FormEdgeConnections");

            DevCountWork("edge pass: edits deferred so far", _Transaction?.DeferredSharedGraphEdits.Count ?? 0);
        }


        //the depth CAP of a narrowed rebuild, in layers below the root, and the ceiling on how many cells one root
        //may narrow to. Both are parameters flagged as "measure, do not argue": deeper cells rebuild less each but
        //multiply the boundary between rebuilt and untouched space, and a cut needing more cells than the cap is
        //cheaper rebuilt as one root. At 7 layers / 0.5 min bucket the root is 32 u and depth 3 makes a cell 4 u —
        //the hive-tower bulkhead the narrowing was measured on landed on a few dozen of them (that demo has since
        //left the suite; its numbers stay in the record).
        const int NARROW_REBUILD_DEPTH     = 3;
        const int NARROW_REBUILD_CELLS_MAX = 128;

        //the margin a narrowed cell's view box gets, so a triangle touching the cell's face from outside — which the
        //inclusive occupancy test counts — is in the view
        const float VIEW_CELL_MARGIN = 0.01f;

        //A/B switch for the narrowed rebuild, ON by default. Two reasons it is a switch and not a constant: the
        //protocol trusts only an A/B taken on one build in one window, and every
        //failure mode of this change is silent — so the ability to answer "is this violation mine?" in twelve
        //minutes, without editing code, is part of the instrument rather than a convenience.
        internal static bool NarrowRebuildEnabled { get; set; } = true;

        //A/B switch for the narrowed footprint clone (Phase 2), ON by default: a planned-narrowed footprint root is
        //cloned down to its rebuilt cells' subtrees plus the same boundary layers the classification writes to,
        //instead of whole. Sound only because the plan is binding (verdicts are decided in the transaction's
        //constructor and RebuildRoots follows them; a divergence throws instead of writing into shared leaves) and
        //because every graph-shaped write is routed by ownership (StagingTransaction.EditFreeNeighbor defers
        //shared-owner edits to the publish's write lock).
        internal static bool NarrowCloneEnabled { get; set; } = true;

        //A/B switch for skipping the clone of a DOOMED root — one whose surviving geometry no longer justifies it,
        //so the publish DROPS it (the remove-to-empty path a streamed world despawns islands through). Cloning a
        //subtree only to discard it wholesale was pure allocation: the clone existed to keep the prune's ref-cleaning
        //writes off the live tree, and with the clone skipped those writes are not needed at all — the ring wiring
        //simply never creates a clone ref into a doomed root (the same end state the prune used to produce), and the
        //dropped space classifies as open by the same staged-or-live view the publish leaves behind.
        internal static bool DoomedRootCloneSkipEnabled { get; set; } = true;

        //A/B switch for the registry rollback an abandoned operation performs, ON by
        //default: OFF leaves the inline half of the cluster retirement and of the raw root index applied, which is
        //the pre-fix shape and the state the KeyNotFoundException came from. It exists so the negative control can
        //prove it still SEES the defect — a control that has never failed certifies nothing.
        internal static bool RegistryRollbackEnabled { get; set; } = true;

        //the operation in flight's buffers — taken by its StagingTransaction, released by its Dispose
        internal OperationScratch OperationScratch => m_OperationScratch;

        //the same, or null outside an operation's transaction (a caller then falls back to local buffers)
        OperationScratch ActiveScratch => m_OperationScratch.InUse ? m_OperationScratch : null;


        //Does any cluster that SURVIVES this operation still reach the root? The exact per-root form of the
        //surviving-clusters test BuildRootGeometryView + RebuildRoots make: registered clusters crossing the root,
        //minus the delta's removed clusters (their storage removal is deferred to the publish and must be excluded
        //here the same way), plus the delta's in-flight added clusters. Computed in the transaction's constructor to
        //decide a root's rebuild verdict before anything is cloned, and it answers identically there and at rebuild
        //time: operations are serialized and every reader-visible registry change is deferred to the publish.
        internal bool RootHasSurvivingGeometry(Bounds _RootBounds, GeometryDelta _Delta)
        {
            //the query set is the operation's scratch (this is asked once per footprint root at plan time)
            HashSet<TriangleCluster> crossingClusters = ActiveScratch?.ClusterQuery ?? new HashSet<TriangleCluster>();

            crossingClusters.Clear();

            if (m_NavTree.TriangleClusterStorage.CollectCrossingBoundables(_RootBounds, crossingClusters))
            {
                foreach (TriangleCluster cluster in crossingClusters)
                {
                    if (cluster.Bounds.Intersects(_RootBounds) && !_Delta.RemovedClusters.Contains(cluster))
                        return true;
                }
            }

            foreach (TriangleCluster addedCluster in _Delta.AddedClusters)
            {
                if (addedCluster.Bounds.Intersects(_RootBounds))
                    return true;
            }

            return false;
        }

        //The operation's cut volume: where its occupancy can change at all. It is exactly the delta's own clusters'
        //safe (inflated) bounds — a cell outside every cluster bound gets no node at all (Fork.ProduceChildren skips
        //it) and a cell's occupancy is decided by the geometry inside it, so nothing beyond these boxes can differ
        //before and after. empty for an operation with no delta (the region update), which is exactly the case where
        //the narrowing must not engage.
        internal static Bounds[] BuildCutVolume(GeometryDelta _Delta)
        {
            List<Bounds> volumes = new List<Bounds>(_Delta.AddedClusters.Count + _Delta.RemovedClusters.Count);

            //an operation that knows where its geometry changed (the partial update) cuts there, not at its clusters
            if (_Delta.CutVolumes != null)
            {
                volumes.AddRange(_Delta.CutVolumes);
            }
            else
            {
                foreach (TriangleCluster cluster in _Delta.AddedClusters)
                    volumes.Add(cluster.Bounds);

                foreach (TriangleCluster cluster in _Delta.RemovedClusters)
                    volumes.Add(cluster.Bounds);
            }

            volumes.AddRange(_Delta.ClosureChanges);

            return volumes.ToArray();
        }

        static bool IntersectsCut(Bounds[] _CutVolume, Bounds _Bounds)
        {
            foreach (Bounds cut in _CutVolume)
            {
                if (cut.Intersects(_Bounds))
                    return true;
            }

            return false;
        }

        //The rebuild set for one root: the shallowest existing nodes that together cover the cut volume, capped by
        //depth and by count. Returns false when the narrowing must not engage, and every refusal is a case this
        //phase deliberately does not handle rather than one it handles badly:
        //  - the root node is a Leaf — nothing to descend into;
        //  - a cell the cut touches has no node: filling it needs a spine of forks this phase does not build, so the
        //    root is rebuilt whole instead. Absent children are normal (Fork.ProduceChildren skips a child outside
        //    every cluster bound), and an add is exactly the operation that gives them a reason to exist;
        //  - the set grows past the cell cap — the cut covers so much of the root that rebuilding it whole is the
        //    cheaper answer.
        //the plan, computed on the live tree in the transaction's constructor — before anything is cloned, because
        //the clone's scope and the rebuild's scope must be the same set. If the rebuild re-derived its own cells
        //from the staged tree afterwards, a scoped clone could hand it a cell whose subtree is SHARED with the live
        //tree, and retiring that cell clears its leaves' refs — an unlocked write into the tree readers traverse.
        //Cells are therefore recorded as POSITIONS (index + layer), never as references to nodes that are about to
        //be replaced by clones.
        //A cut too wide for the cap at the finest depth is planned one level coarser before the root is given up:
        //a radius-8 crater in a 2 u / 5-layer world reaches 216 four-unit cells of one root, which the cap refused and
        //rebuilt whole, where 27–64 eight-unit cells do the same work in a fraction of the root. A cell with no node
        //stays a refusal at every depth.
        internal bool TryPlanNarrowedRebuild(Vector3Int _RootIndex, Node _RootNode, Bounds[] _CutVolume, out List<StagingTransaction.RebuiltCell> _Cells)
        {
            _Cells = new List<StagingTransaction.RebuiltCell>();

            if (!NarrowRebuildEnabled || _CutVolume == null || _CutVolume.Length == 0)
                return false;

            if (!(_RootNode is Fork rootFork))
                return false;

            for (int depth = NARROW_REBUILD_DEPTH; depth >= 1; depth--)
            {
                _Cells.Clear();

                RebuildPlanRefusal refusal = DescendForRebuildSet(rootFork, _CutVolume, _Cells, depth);

                if (refusal == RebuildPlanRefusal.NONE)
                    return _Cells.Count > 0;

                if (refusal == RebuildPlanRefusal.NO_NODE)
                    return false;

                DevCountWork("plan: depth fallback (cell cap)", 1);
            }

            return false;
        }

        enum RebuildPlanRefusal : byte
        {
            NONE,
            NO_NODE,
            CELL_CAP
        }

        RebuildPlanRefusal DescendForRebuildSet(Fork _Fork, Bounds[] _CutVolume, List<StagingTransaction.RebuiltCell> _Cells, int _MaxDepth)
        {
            float childSize = GetBucketSizeOnLayer(_Fork.GridLayer + 1);

            foreach ((ForkChildOctIndex OctIndex, Vector3Int GridIndex) child in UtilsSpatialGrid.GetChildrenIndices(_Fork.Index))
            {
                Bounds childBounds = UtilsSpatialGrid.GetBucketBounds(child.GridIndex, childSize);

                if (!IntersectsCut(_CutVolume, childBounds))
                    continue;

                if (!_Fork.ChildrenMap.TryGetValue(child.OctIndex, out Node childNode) || childNode == null)
                    return RebuildPlanRefusal.NO_NODE;

                if (childNode is Fork childFork && childNode.GridLayer < _MaxDepth)
                {
                    RebuildPlanRefusal refusal = DescendForRebuildSet(childFork, _CutVolume, _Cells, _MaxDepth);

                    if (refusal != RebuildPlanRefusal.NONE)
                        return refusal;

                    continue;
                }

                if (_Cells.Count >= NARROW_REBUILD_CELLS_MAX)
                    return RebuildPlanRefusal.CELL_CAP;

                _Cells.Add(new StagingTransaction.RebuiltCell(child.GridIndex, _Fork.GridLayer + 1, childBounds));
            }

            return RebuildPlanRefusal.NONE;
        }

        //Resolves the planned cells against the staged tree: the parent fork that holds each cell (a clone, because
        //the clone scope was cut from this same plan) and the slot it sits in. Returns false if the staged tree does
        //not present the planned shape, which must never happen — the plan was taken from the tree this one clones.
        bool TryResolvePlannedCells(
                Vector3Int                                                 _RootIndex,
                List<StagingTransaction.RebuiltCell>                       _Cells,
                out List<RootBuilder.SubtreeRebuildTarget>                 _Targets,
                out List<(Fork Fork, Fork Parent, ForkChildOctIndex Slot)> _Spine
            )
        {
            _Targets = new List<RootBuilder.SubtreeRebuildTarget>(_Cells.Count);
            _Spine   = new List<(Fork, Fork, ForkChildOctIndex)>();

            foreach (StagingTransaction.RebuiltCell cell in _Cells)
            {
                Vector3Int parentIndex = CellIndexAtLayer(cell.Index, cell.Layer, cell.Layer - 1);

                if (!(FindNodeCoveringCell(parentIndex, cell.Layer - 1) is Fork parentFork) || parentFork.GridLayer != cell.Layer - 1)
                    return false;

                ForkChildOctIndex slot  = default;
                bool              found = false;

                foreach ((ForkChildOctIndex OctIndex, Vector3Int GridIndex) child in UtilsSpatialGrid.GetChildrenIndices(parentFork.Index))
                {
                    if (child.GridIndex != cell.Index)
                        continue;

                    slot  = child.OctIndex;
                    found = true;

                    break;
                }

                if (!found)
                    return false;

                _Targets.Add(new RootBuilder.SubtreeRebuildTarget(_RootIndex, cell.Index, cell.Layer, GetBucketSizeOnLayer(cell.Layer), parentFork, slot));

                //the spine above the cell, so a fork the rebuild leaves childless can be swept away with it
                for (int layer = cell.Layer - 1; layer >= 1; layer--)
                {
                    Vector3Int spineIndex = CellIndexAtLayer(cell.Index, cell.Layer, layer);
                    Vector3Int aboveIndex = CellIndexAtLayer(cell.Index, cell.Layer, layer - 1);

                    if (!(FindNodeCoveringCell(spineIndex, layer) is Fork spineFork) || spineFork.GridLayer != layer)
                        break;

                    if (!(FindNodeCoveringCell(aboveIndex, layer - 1) is Fork aboveFork) || aboveFork.GridLayer != layer - 1)
                        break;

                    ForkChildOctIndex spineSlot  = default;
                    bool              spineFound = false;

                    foreach ((ForkChildOctIndex OctIndex, Vector3Int GridIndex) child in UtilsSpatialGrid.GetChildrenIndices(aboveFork.Index))
                    {
                        if (child.GridIndex != spineIndex)
                            continue;

                        spineSlot  = child.OctIndex;
                        spineFound = true;

                        break;
                    }

                    if (spineFound)
                        _Spine.Add((spineFork, aboveFork, spineSlot));
                }
            }

            return true;
        }

        //Retires one cell before it is rebuilt, and seeds the construction grid with everything the rebuilt cell will
        //have to reconnect to. Two seeds, and the second one is the whole reason this method exists:
        //  1. the neighbours of the cell's own free leaves, refs cleaned — the same move the whole-root prune makes;
        //  2. the free leaves across each of the cell's six faces. Seed (1) walks free-neighbour refs, so it cannot
        //     see a leaf that sits across an OCCUPIED face leaf — there is no ref to walk. If the rebuild frees that
        //     face, the connection would simply never form, and a missing free-graph edge is invisible until a route
        //     cannot be found. The whole-root path has the same hole and closes it with
        //     FormEdgeConnectionsForAddedRoots over the root's faces; a narrowed cell has faces inside a root, which
        //     that pass does not visit, so its other side is seeded here and FormLeavesConnections wires it.
        void PruneRebuildCell(RootBuilder.SubtreeRebuildTarget _Target, LeavesSpatialGrid _LeavesSpatialGrid)
        {
            //the operation's scratch sets, cleared per cell (see the whole-root prune above)
            OperationScratch scratch = ActiveScratch;

            if (_Target.Parent.ChildrenMap.TryGetValue(_Target.Slot, out Node cellNode) && cellNode != null)
            {
                HashSet<Leaf> cellLeaves     = scratch?.PruneContained ?? new HashSet<Leaf>();
                HashSet<Leaf> neighborLeaves = scratch?.PruneNeighbors ?? new HashSet<Leaf>();
                List<Leaf>    neighborBuffer = scratch?.NeighborBuffer ?? new List<Leaf>();

                cellLeaves.Clear();
                cellNode.GetAllContainedFreeLeaves(cellLeaves);

                GetAllNeighborsAndCleanRefs(cellLeaves, _LeavesSpatialGrid, neighborLeaves, neighborBuffer);

                _LeavesSpatialGrid.SetLeaves(neighborLeaves);

                cellLeaves.Clear();
                neighborLeaves.Clear();
            }

            HashSet<Leaf> acrossFaces = scratch?.PruneAcrossFaces ?? new HashSet<Leaf>();

            acrossFaces.Clear();

            foreach (Vector3Int direction in UtilsSpatialGrid.FacesDirections)
            {
                Node acrossNode = FindNodeCoveringCell(_Target.Index + direction, _Target.Layer);

                acrossNode?.GetEdgeLeaves(-direction, acrossFaces);
            }

            if (acrossFaces.Count > 0)
                _LeavesSpatialGrid.SetLeaves(acrossFaces);

            acrossFaces.Clear();
        }


        //The node covering a given cell in the tree under construction: the node at that position if the tree is
        //subdivided that far, the coarser node containing it otherwise, or null where the tree holds nothing there.
        //Descends by index arithmetic rather than by point containment — a cell's centre lies exactly on a boundary
        //plane at every coarser layer, so a Contains test would be a coin flip.
        Node FindNodeCoveringCell(Vector3Int _CellIndex, int _Layer)
        {
            if (!m_ActiveRootsGrid.TryGetNode(0, CellIndexAtLayer(_CellIndex, _Layer, 0), out Node node) || node == null)
                return null;

            while (node is Fork fork && fork.GridLayer < _Layer)
            {
                Vector3Int wantedIndex = CellIndexAtLayer(_CellIndex, _Layer, fork.GridLayer + 1);

                Node next = null;

                foreach ((ForkChildOctIndex OctIndex, Vector3Int GridIndex) child in UtilsSpatialGrid.GetChildrenIndices(fork.Index))
                {
                    if (child.GridIndex != wantedIndex)
                        continue;

                    fork.ChildrenMap.TryGetValue(child.OctIndex, out next);

                    break;
                }

                //nothing at that slot: this fork is as deep as the tree goes here, and it is what covers the cell
                if (next == null)
                    return node;

                node = next;
            }

            return node;
        }

        //Floor division, component-wise: the grid extends into negative world space, so -1 / 2 must be -1, not 0
        //(Vector3Int's own operator/ truncates toward zero — the same trap Node.RootIndex documents).
        static Vector3Int CellIndexAtLayer(Vector3Int _Index, int _FromLayer, int _ToLayer)
        {
            int divisor = 1 << (_FromLayer - _ToLayer);

            return new Vector3Int(
                    Mathf.FloorToInt(_Index.x / (float) divisor),
                    Mathf.FloorToInt(_Index.y / (float) divisor),
                    Mathf.FloorToInt(_Index.z / (float) divisor)
                );
        }

        //A root is complement-eligible when: (1) nothing in _Delta.RemovedClusters crosses it — complement can
        //only add occupancy, never coarsen/free a stale cell, so removing anything here requires a full rebuild;
        //(2) this op is actually adding new geometry there (some _Delta.AddedClusters crosses it) — otherwise
        //there is nothing to complement, and this also correctly excludes region-update's null-delta case, which
        //never has an added cluster crossing anything; (3) an EXISTING (already-registered) cluster crossing this
        //root is "oversized" — fails ClusterBoundedBySingleRoot, the same reconstruction-cost proxy
        //ProcessIntersectingCachedClusters already uses for the identical reason, reused here rather than inventing
        //a new one. Provably false for every root RemoveObstacleInternal or UpdateObstaclesInBounds ever calls
        //RebuildRoots with (condition 1 or 2 always fails there respectively) — this only ever activates for
        //AddObstacle and pre-baked add's contested-merge/reconciliation.
        //internal (not private): the StagingTransaction constructor asks it at plan time, so a complement-eligible
        //root is never planned as narrowed and the rebuild-time branch below cannot diverge from the plan.
        internal bool IsRootComplementEligible(Bounds _RootBounds, GeometryDelta _Delta)
        {
            foreach (TriangleCluster removedCluster in _Delta.RemovedClusters)
                if (removedCluster.Bounds.Intersects(_RootBounds))
                    return false;

            bool addsGeometryHere = false;

            foreach (TriangleCluster addedCluster in _Delta.AddedClusters)
                if (addedCluster.Bounds.Intersects(_RootBounds))
                {
                    addsGeometryHere = true;
                    break;
                }

            if (!addsGeometryHere)
                return false;

            HashSet<TriangleCluster> existingClusters = ActiveScratch?.ClusterQuery ?? new HashSet<TriangleCluster>();

            existingClusters.Clear();

            if (!m_NavTree.TriangleClusterStorage.CollectCrossingBoundables(_RootBounds, existingClusters))
                return false;

            foreach (TriangleCluster cluster in existingClusters)
            {
                if (!ClusterBoundedBySingleRoot(cluster))
                    return true;
            }

            return false;
        }

        #endregion

    }
}
