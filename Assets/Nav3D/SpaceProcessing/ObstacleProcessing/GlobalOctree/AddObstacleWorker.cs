using Nav3D.API;
using Nav3D.Common;
using Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.SpatialStorages;
using Nav3D.SpaceProcessing.ObstacleProcessing.UtilityTypes;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using UnityEngine;
using GraphConstructionProgress = Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.OperationsProgress.GraphConstructionProgress;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree
{
    //The runtime "add obstacle" operation, extracted out of the GlobalOctree god-partial (formerly
    //GlobalOctree.RuntimeOperations.cs's AddObstacle). Pure code motion — same
    //logic, same order, same GlobalOctree members touched (now via the back-reference instead of implicit same-class
    //access). RebuildRoots (the shared rebuild primitive) deliberately stays on GlobalOctree — see its own comment.
    internal sealed class AddObstacleWorker
    {

        #region Attributes

        readonly GlobalOctree m_Octree;

        #endregion

        #region Constructors

        public AddObstacleWorker(GlobalOctree _Octree)
        {
            m_Octree = _Octree;
        }

        #endregion

        #region Public methods

        //The heavy build runs off the live tree (on staging clones) without the write lock, so readers keep
        //traversing the old tree throughout construction. Only the publish below — installing the staged tree and
        //applying the NavTree.TriangleClusterStorage changes readers query — takes the write lock. During the build the
        //only shared state mutated is the staging grid and the scalar construction params (bucket sizes / layer
        //count), neither of which the read surface reads; the live tree and TriangleClusterStorage are only read.
        //That is safe because obstacle operations are serialized, so during a build there is no concurrent writer.
        public void Execute(ObstacleInfo _ObstacleInfo, ObstacleAdditionProgress _AdditionProgress, CancellationToken _CancellationToken, Action _OnFinish, bool _EditMode)
        {
            try
            {
                //Already there — nothing to build. The addition's post-condition ("the obstacle is present in
                //navigation") already holds, so this still reports completion, exactly as RemoveObstacleWorker reports
                //a no-op removal. Returning silently is a defect that predates the move operation but was unreachable
                //in practice until it: a caller waiting on OnObstacleAdded waits forever, and every such caller is a
                //gate-on-completion pattern the docs prescribe. It became reachable because a move requested before the
                //obstacle's own deferred first add (a scene transform reports hasChanged on frame one) registers the
                //obstacle through this worker, so the queued first add arrives second and swallowed its own event —
                //which is exactly how ObstacleGridMoverTest came to hang at step zero.
                if (m_Octree.GeometryRegistry.IsObstacleRegistered(_ObstacleInfo.ID))
                {
                    _AdditionProgress?.SetFinished();

                    ReportFinished(_OnFinish, _EditMode);

                    return;
                }

                m_Octree.DevOpTimerStartHook("add");

                //the operation's reusable triangle list and clustering buffers (OperationScratch) — the clustering
                //was the largest transient of a streamed island's add after the tree itself
                List<Triangle> trianglesArray = m_Octree.OperationScratch.TriangleList;

                trianglesArray.Clear();
                trianglesArray.AddRange(_ObstacleInfo.Triangles);

                //Progress: triangle clustering phase. The runtime build path reported no intermediate status, so the
                //progress dialog stayed pinned on the INITIAL "Waiting for processing to begin.." for the whole build.
                _AdditionProgress?.SetStatus(ObstacleAdditionProgress.ObstacleAdditionStatus.OBSTACLES_CLUSTERIZATION);

                m_Octree.DevOpPhaseStartHook();

                //Cluster triangles and produce raw (no enlargement) and enlarged (safe area) cluster sets — through
                //chunks, so a later update can replace only what changed; the Lite add clusters flat, it never updates

                #if !LITE_VERSION

                ObstacleChunking.ChunkingResult chunking = null;
                TriangleCluster[]               rawClusters;
                TriangleCluster[]               safeClusters;

                if (GlobalOctree.ChunkedGeometryEnabled)
                {
                    chunking = ObstacleChunking.Build(
                            trianglesArray,
                            m_Octree.RootSize,
                            m_Octree.MaxAgentRadius,
                            m_Octree.MaxAgentRadius * 2,
                            null,
                            null,
                            null,
                            m_Octree.OperationScratch.Clustering
                        );

                    rawClusters  = chunking.RawClusters;
                    safeClusters = chunking.SafeClusters;
                }
                else
                {
                    (rawClusters, safeClusters) = UtilsTriangle.ClusterTrianglesDual(
                            trianglesArray,
                            m_Octree.MaxAgentRadius,
                            m_Octree.MaxAgentRadius * 2,
                            m_Octree.OperationScratch.Clustering
                        );
                }

                #else

                (TriangleCluster[] rawClusters, TriangleCluster[] safeClusters) = UtilsTriangle.ClusterTrianglesDual(
                        trianglesArray,
                        m_Octree.MaxAgentRadius,
                        m_Octree.MaxAgentRadius * 2,
                        m_Octree.OperationScratch.Clustering
                    );

                #endif

                //the clusters own their sets now; the list must not keep this obstacle's triangles alive
                trianglesArray.Clear();

                //these clusters are this operation's own (a fresh capture), so their geometry may be severed when a
                //later removal retires them — unlike a bake-cache cluster, which every pre-baked re-enable reinstalls
                foreach (TriangleCluster safeCluster in safeClusters)
                    safeCluster.MarkGeometryReleasable();

                //an obstacle marked hollow keeps its closed shells navigable — set before the clusters are seen
                foreach (TriangleCluster cluster in rawClusters.Concat(safeClusters))
                    cluster.HollowTriangles = _ObstacleInfo.HollowTriangles;

                m_Octree.UpdateMinBucketSize();
                m_Octree.FillBucketSizesCache();

                m_Octree.DevOpPhaseEndHook("add: clusterize");

                //The construction footprint — the only roots this add may structurally change: this obstacle's own
                //safe-cluster bounds. Clusters are never united with other obstacles' geometry (one owner per
                //cluster), so there is nothing else to bound the construction against — a shared root still
                //rebuilds correctly regardless.
                HashSet<Vector3Int> operationFootprint = new HashSet<Vector3Int>();

                foreach (TriangleCluster newSafeCluster in safeClusters)
                    operationFootprint.UnionWith(UtilsSpatialGrid.GetCrossedBucketsIndices(newSafeCluster.Bounds, m_Octree.RootSize));

                //Clone the affected roots plus a one-root face-neighbour ring into a private staging grid and point
                //construction at it (ActiveRootsGrid). The build below never touches the live tree; the finished
                //result is installed in one locked step by PublishStagedRegion.
                m_Octree.DevOpPhaseStartHook();

                GlobalOctree.GeometryDelta delta = m_Octree.WithClosureChanges(new GlobalOctree.GeometryDelta(null, safeClusters), operationFootprint);

                using StagingTransaction transaction = new StagingTransaction(
                        m_Octree,
                        operationFootprint,
                        _CancellationToken,
                        _FullRingClone: false,
                        //what this add changes — the delta the per-root verdicts, the clone and the rebuild are all cut from
                        _Delta: delta
                    );

                m_Octree.DevOpPhaseEndHook("add: stage (ring + clone)");

                //Progress: navigation-graph construction phase. Wire a fresh GraphConstructionProgress into the shared
                //addition progress so the dialog reflects octree build / connection forming (mirrors the pre-baked
                //addition path; the runtime path previously left ConstructionProgress null, so every progress call here
                //was a silent no-op).
                m_Octree.ConstructionProgress = GraphConstructionProgress.INITIAL;
                _AdditionProgress?.SetStatus(ObstacleAdditionProgress.ObstacleAdditionStatus.OBSTACLES_GRAPH_CONSTRUCTION);
                _AdditionProgress?.SetCurrentConstructionProgress(m_Octree.ConstructionProgress);

                m_Octree.ConstructionProgress.SetTotalRootsCount(operationFootprint.Count);
                m_Octree.ConstructionProgress.SetStatus(GraphConstructionProgress.ConstructionStatus.FILLING_TRIANGLE_STORAGE);

                //sized from the previous operations' grids and registered so the transaction's Dispose severs it with
                //the op's other hub collections (P28) and records its size for the next one (OperationScratch)
                LeavesSpatialGrid leavesSpatialGrid = transaction.RentConstructionGrid();

                m_Octree.ConstructionProgress.SetStatus(GraphConstructionProgress.ConstructionStatus.TREE_CONSTRUCTION_PREPARATION);
                m_Octree.ConstructionProgress.SetStatus(GraphConstructionProgress.ConstructionStatus.TREE_CONSTRUCTION);

                //One declarative rebuild over the whole footprint: every root is pruned/complemented and
                //rebuilt from the current geometry view (registry plus this obstacle's own new clusters as the added
                //delta — not yet registered, so TriangleClusterStorage can't see them yet).
                m_Octree.DevOpPhaseStartHook();

                m_Octree.RebuildRoots(operationFootprint, delta, leavesSpatialGrid, _CancellationToken, transaction);

                m_Octree.DevOpPhaseEndHook("add: RebuildRoots");

                m_Octree.ConstructionProgress.SetStatus(GraphConstructionProgress.ConstructionStatus.GRAPH_CONNECTIONS_BUILDING);
                m_Octree.ConstructionProgress.SetStatus(GraphConstructionProgress.ConstructionStatus.FINISHED);

                //ConstructRoots (inside RebuildRoots) returns silently when the token fires mid-build. Without this
                //checkpoint a cancelled add keeps going and PUBLISHES a partially built staged region — pruned
                //clones with holes, or clone-set cells whose staged root was removed and never rebuilt, which the
                //publish then DROPS from the live tree. Proven by DevValidateAdditionCoverage firing on the teardown
                //cancel (80 uncovered triangles of an in-flight add at play exit). Cancellation must abandon staging
                //and publish nothing.
                if (_CancellationToken.IsCancellationRequested)
                    return;

                //DEV: verify on the staged tree (ActiveRootsGrid is still the staging grid here) that every triangle
                //this add registers is covered by an occupied leaf before publish. Scoped to the staged roots
                //(transaction.CloneSetIndices): construction only rebuilds the footprint, so a triangle whose root
                //was not staged stays covered by its untouched live root and would be a false positive if probed
                //against the staging tree.
                //timed separately so its editor-ONLY cost is never mistaken for real op cost (it is stripped at release)
                m_Octree.DevOpPhaseStartHook();

                m_Octree.DevValidateAdditionCoverageHook(_ObstacleInfo.ID, safeClusters, transaction.CloneSetIndices);

                m_Octree.DevOpPhaseEndHook("add: DEV coverage guard");

                //Sever any stale FreeNeighbor ref a surviving clone-set leaf still holds to a discarded (rebuilt) clone,
                //and reconnect it to the replacement — a partial rebuild (e.g. a root shared with an overlapping obstacle)
                //otherwise publishes DANGLING_REF. Idempotent on a clean build (no stale ref → no change).
                m_Octree.DevOpPhaseStartHook();

                m_Octree.RepairStagedRegionFreeGraph(transaction.CloneSetIndices, transaction.RebuiltRootIndices, transaction, transaction.StagingGrid);

                m_Octree.DevOpPhaseEndHook("add: RepairStagedRegionFreeGraph");

                //Reader-visible bookkeeping (TriangleClusterStorage + the cluster/ID maps), queued to run inside
                //transaction.Publish()'s write lock, immediately after it installs the staged tree and reclassifies
                //open faces — atomically, so the triangle storage and its maps never diverge as seen by a later
                //operation.
                transaction.DeferredRegistryChanges.Add(() =>
                        {
                            m_Octree.GeometryRegistry.RegisterObstacleRaw(_ObstacleInfo.ID, rawClusters, _ObstacleInfo);

                            #if !LITE_VERSION

                            if (chunking != null)
                                m_Octree.GeometryRegistry.RegisterChunkTable(_ObstacleInfo.ID, chunking.Table);

                            #endif

                            foreach (TriangleCluster cluster in safeClusters)
                            {
                                m_Octree.NavTree.TriangleClusterStorage.Add(cluster);

                                Vector3Int[] roots = UtilsSpatialGrid.GetCrossedBucketsIndices(cluster.Bounds, m_Octree.RootSize);
                                m_Octree.GeometryRegistry.RegisterCluster(cluster, roots, new HashSet<int> { _ObstacleInfo.ID });
                            }
                        }
                    );

                //Publish — the only write-locked section. Install the staged region into the live tree and apply the
                //TriangleClusterStorage changes that readers query, atomically; the build above ran unlocked.
                //Readers are briefly excluded here while the new tree and cluster storage swap in.
                m_Octree.DevOpPhaseStartHook();

                transaction.Publish();

                m_Octree.DevOpPhaseEndHook("add: Publish (incl. unlocked pre-steps)");

                m_Octree.DevOpPhaseStartHook();

                //Invalidate the cached per-obstacle gizmo for every obstacle sharing the touched roots, since any of
                //them may have different occupied leaves after this addition.
                HashSet<int> gizmoAffectedIDs = m_Octree.GeometryRegistry.CollectObstacleIDsInRoots(transaction.CloneSetIndices);
                m_Octree.ResetGizmosDataForObstaclesHook(gizmoAffectedIDs.ToArray());

                //newly-occupied/reconnected space: re-path stored paths crossing it and mark agents obstacle-dirty.
                //Notify with this operation's own footprint (the clusters just added), not the clone-set AABB — the
                //clone set carries a one-root publish ring that changed nothing agents can observe. One volume per
                //cluster, never their union: an obstacle whose parts are far apart (a parent with distant child
                //meshes) would otherwise declare the empty space between them changed too.
                //Skip in edit mode (baking) — there are no live runtime paths/agents to invalidate.
                if (!_EditMode)
                {
                    m_Octree.NotifyRegionChanged(
                            transaction.CloneSetIndices,
                            GlobalOctree.TryGetClusterVolumes(safeClusters, out List<Bounds> addedVolumes) ? addedVolumes : null
                        );
                }

                m_Octree.DevOpPhaseEndHook("add: gizmo invalidation + notify");

                _AdditionProgress.SetFinished();

                m_Octree.DevOpTimerFinishHook();

                ReportFinished(_OnFinish, _EditMode);
            }
            catch (OperationCanceledException)
            {
                //Cancellation is a normal exit: abandon staging without publishing or reporting success.
            }
            catch (Exception exception)
            {
                Debug.LogException(exception);
            }
            //the StagingTransaction's Dispose (via the `using` declaration above) already restored the construction
            //alias and discarded any unfinished op timer, even on early return/exception.
        }

        #endregion

        #region Service methods

        //Completion goes to the main thread (the caller's handler touches Unity objects), except on the edit-mode
        //baking path, which has no dispatcher pumping and runs the callback inline — a user handler throwing there
        //must not take the worker thread down with it.
        static void ReportFinished(Action _OnFinish, bool _EditMode)
        {
            if (_OnFinish == null)
                return;

            if (!_EditMode)
            {
                ThreadDispatcher.BeginInvoke(_OnFinish);

                return;
            }

            try
            {
                _OnFinish.Invoke();
            }
            catch (Exception exception)
            {
                Debug.LogException(exception);
            }
        }

        #endregion

    }
}
