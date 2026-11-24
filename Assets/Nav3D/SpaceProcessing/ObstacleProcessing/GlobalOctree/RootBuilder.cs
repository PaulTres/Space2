using Nav3D.Common;
using Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.Elements;
using Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.OperationsProgress;
using Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.SpatialStorages;
using System;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree
{
    //The pure node-construction primitive extracted out of the GlobalOctree god-partial — the only code that
    //constructs nodes. Still needs a
    //GlobalOctree back-reference: every Node/Fork/Leaf stores one for its whole lifetime and calls back into it
    //(CheckMaxLayer, GetBucketSizeOnLayer, GetBucketIndexOnLayer) for recursive operations long after initial
    //construction (e.g. Fork.ComplementAbsentChildren, Leaf.ComplementOrRebuild) — this coupling is structural to
    //Elements/, not something this extraction can remove. Everything that changes per-call (the active roots grid —
    //live vs. staging alias — and the per-op construction progress object) is taken as an explicit parameter rather
    //than a widened GlobalOctree field, keeping this close to a pure function of its inputs.
    internal sealed class RootBuilder
    {

        #region Nested types

        //One cell to rebuild in place of a whole root, and the slot it hangs in.
        internal readonly struct SubtreeRebuildTarget
        {
            public Vector3Int        RootIndex { get; }
            public Vector3Int        Index     { get; }
            public int               Layer     { get; }
            public float             Size      { get; }
            public Fork              Parent    { get; }
            public ForkChildOctIndex Slot      { get; }

            public SubtreeRebuildTarget(Vector3Int _RootIndex, Vector3Int _Index, int _Layer, float _Size, Fork _Parent, ForkChildOctIndex _Slot)
            {
                RootIndex = _RootIndex;
                Index     = _Index;
                Layer     = _Layer;
                Size      = _Size;
                Parent    = _Parent;
                Slot      = _Slot;
            }
        }

        #endregion

        #region Attributes

        readonly GlobalOctree  m_Octree;
        readonly SemaphoreSlim m_ConcurrencySemaphore;

        #endregion

        #region Constructors

        public RootBuilder(GlobalOctree _Octree, SemaphoreSlim _ConcurrencySemaphore)
        {
            m_Octree               = _Octree;
            m_ConcurrencySemaphore = _ConcurrencySemaphore;
        }

        #endregion

        #region Public methods

        public void ConstructRoots(
                Vector3Int[]                        _RootsIndices,
                Func<Vector3Int, IOccupancyStorage> _TriangleStorageProvider,
                Bounds[]                            _ConstructionBounds,
                LeavesSpatialGrid                   _LeavesSpatialGrid,
                CancellationToken                   _CancellationToken,
                Dictionary<Vector3Int, Node>        _PartiallyConstructedRoots,
                NodeSpatialGrid                     _ActiveRootsGrid,
                GraphConstructionProgress           _ConstructionProgress,
                //Was a direct private-field read (m_BaseLayersCount) while ConstructRoots lived inside GlobalOctree;
                //now an explicit parameter out of necessity (a private field on a different class is unreachable),
                //which incidentally also matches BuildRootsReusingCache's own existing _BaseLayersCount parameter —
                //a forced, minimal consequence of the physical split, not a deliberate signature cleanup.
                int _BaseLayersCount,
                //When false, the caller has already established the progress denominator over a wider root set (e.g. all
                //of an obstacle's cluster passes), so this per-call invocation must not reset it — otherwise the
                //construction percentage would dip on every pass instead of climbing monotonically to 100%.
                bool _SetProgressTotal = true
            )
        {
            //Progress is tracked at root granularity (completed roots / total considered roots). Per-leaf accounting
            //was abandoned: the build reuses whole subtrees (cached/complemented roots are not re-traversed), so the
            //per-leaf counters only ever saw the freshly-built leaves and the percentage stalled near 0 on reuse-heavy
            //obstacles. A root counts as done whether built fresh or reused, which is robust to that reuse.
            if (_SetProgressTotal)
                _ConstructionProgress?.SetTotalRootsCount(_RootsIndices.Length);

            List<Task>              taskSet = new List<Task>(_RootsIndices.Length);
            ConcurrentHashSet<Node> roots   = new ConcurrentHashSet<Node>();

            //Three brackets over this method: what the dispatch loop itself costs (it runs roots INLINE whenever the
            //concurrency semaphore is exhausted, so it is not merely a spawn loop), what the wait on the spawned
            //tasks costs, and what installing the finished roots costs.
            m_Octree.DevStepStartHook();

            foreach (Vector3Int rootIndex in _RootsIndices)
            {
                Bounds rootBounds = UtilsSpatialGrid.GetBucketBounds(rootIndex, m_Octree.RootSize);

                Action processRoot = () =>
                {
                    bool needCheckMaxLayer = m_Octree.CheckMaxLayer(rootBounds, _BaseLayersCount - 1, out int maxLayer);

                    //per-root storage: the full geometry crossing this root (see BuildRootCombinedStorage).
                    //Occupancy reduces to "does that geometry cross the root" — IsOccupy is exactly IsIntersect.
                    IOccupancyStorage triangleStorage = _TriangleStorageProvider(rootIndex);

                    bool needConstruct = false;

                    Node root;

                    if (_PartiallyConstructedRoots.TryGetValue(rootIndex, out Node partialRoot) && partialRoot != null)
                    {
                        if (partialRoot is Leaf rootLeaf)
                        {
                            _ActiveRootsGrid.RemoveNode(0, partialRoot.Index);
                            _LeavesSpatialGrid.RemoveLeaf(rootLeaf);

                            //clean neighbor connections to prevent orphaned references
                            foreach (Leaf neighbor in rootLeaf.FreeNeighbors)
                                neighbor.RemoveFreeNeighbor(rootLeaf);

                            rootLeaf.RemoveAllFreeNeighbors();

                            needConstruct = true;
                        }
                    }
                    else
                    {
                        needConstruct = true;
                    }

                    //construct subtree
                    if (needConstruct)
                    {
                        //Runs on this root's own task thread, so its per-operation total sums across threads — read it
                        //as the construction's CPU cost, never against the enclosing step's wall clock.
                        m_Octree.DevStepStartHook();

                        root = Node.Create(
                                m_Octree,
                                _LeavesSpatialGrid,
                                triangleStorage,
                                _ConstructionBounds,
                                m_Octree.RootSize,
                                rootIndex,
                                0,
                                triangleStorage.IsIntersect(rootBounds),
                                m_Octree.GetNextNodeID,
                                maxLayer,
                                needCheckMaxLayer,
                                _CancellationToken,
                                m_ConcurrencySemaphore,
                                null,
                                null
                            );

                        m_Octree.DevStepEndHook("construct: Node.Create (per root, CPU)");
                    }
                    else
                    {
                        m_Octree.DevStepStartHook();

                        root = partialRoot;
                        root.AttachNewIDsToSubtree(m_Octree.GetNextNodeID);

                        root.ComplementAbsentChildren(
                                maxLayer,
                                m_Octree.GetNextNodeID,
                                _LeavesSpatialGrid,
                                triangleStorage,
                                _ConstructionBounds,
                                _CancellationToken,
                                needCheckMaxLayer,
                                m_ConcurrencySemaphore,
                                null,
                                null
                            );

                        m_Octree.DevStepEndHook("construct: complement (per root, CPU)");
                    }

                    roots.TryAdd(root);

                    //root-granular progress: this considered root is fully built/reused (thread-safe Interlocked add)
                    _ConstructionProgress?.AddProcessedLeavesCount(1);
                };

                if (m_ConcurrencySemaphore.Wait(0))
                {
                    taskSet.Add(
                            Task.Run(
                                    () =>
                                    {
                                        try
                                        {
                                            GlobalOctree.DevTaskCpuStart();

                                            processRoot();
                                        }
                                        finally
                                        {
                                            GlobalOctree.DevTaskCpuEnd();
                                            m_ConcurrencySemaphore.Release();
                                        }
                                    },
                                    _CancellationToken
                                )
                        );
                }
                else
                {
                    processRoot();
                }
            }

            m_Octree.DevStepEndHook("construct: dispatch + inline roots");

            m_Octree.DevStepStartHook();

            Task allDone = Task.WhenAll(taskSet);
            WaitHandle.WaitAny(new[] { ((IAsyncResult) allDone).AsyncWaitHandle, _CancellationToken.WaitHandle });

            m_Octree.DevStepEndHook("construct: wait for root tasks");

            if (_CancellationToken.IsCancellationRequested)
            {
                JoinAfterCancellation(taskSet);

                return;
            }

            m_Octree.DevStepStartHook();

            foreach (Node root in roots)
                _ActiveRootsGrid.AddNode(root);

            m_Octree.DevStepEndHook("construct: install roots");
        }

        //Constructs SUBTREES in place of whole roots — the narrowed-rebuild primitive.
        //Same primitive as
        //ConstructRoots (Node.Create over the operation's geometry view, same parallel dispatch); the one difference
        //is that the result is grafted into its parent fork instead of installed into the roots grid, so everything
        //outside the rebuilt cells keeps the nodes it already had.
        //
        //The graft is deliberately SEQUENTIAL, after the parallel build has joined: a Fork's children map is a plain
        //Dictionary, so two tasks grafting into the same parent would race it. The build is where the time is, and
        //that half parallelises exactly as ConstructRoots does.
        public void ConstructSubtrees(
                IReadOnlyList<SubtreeRebuildTarget> _Targets,
                Func<Vector3Int, IOccupancyStorage> _TriangleStorageProvider,
                Bounds[]                            _ConstructionBounds,
                LeavesSpatialGrid                   _LeavesSpatialGrid,
                CancellationToken                   _CancellationToken,
                GraphConstructionProgress           _ConstructionProgress,
                int                                 _BaseLayersCount
            )
        {
            if (_Targets.Count == 0)
                return;

            Node[]     built   = new Node[_Targets.Count];
            List<Task> taskSet = new List<Task>(_Targets.Count);

            m_Octree.DevStepStartHook();

            for (int targetNum = 0; targetNum < _Targets.Count; targetNum++)
            {
                int                  slot   = targetNum;
                SubtreeRebuildTarget target = _Targets[slot];

                Action buildOne = () =>
                {
                    Bounds bounds = UtilsSpatialGrid.GetBucketBounds(target.Index, target.Size);

                    //A cell EXISTS only where some cluster REACHES it — the same test Fork.ProduceChildren applies to
                    //every child it considers, epsilon included, and skipping it is a defect Run A caught outright:
                    //a cell left by a removal, with no surviving cluster anywhere near it, was handed to Node.Create
                    //and came back as a free Leaf where the canonical tree has no node at all. Such a leaf touches no
                    //other free leaf, so it reads as ORPHANED_FREE_LEAF (35-41 per validator step) — and the deeper
                    //point is that a narrowed rebuild must reproduce the whole-root construction's own rules, not
                    //merely call the same primitive.
                    if (!Fork.IsInsideAnyCluster(bounds, _ConstructionBounds))
                    {
                        built[slot] = null;

                        return;
                    }

                    bool needCheckMaxLayer = m_Octree.CheckMaxLayer(bounds, _BaseLayersCount - 1, out int maxLayer);

                    IOccupancyStorage triangleStorage = _TriangleStorageProvider(target.RootIndex);

                    //Runs on this cell's own task thread, so its per-operation total sums across threads — the same
                    //CPU reading as ConstructRoots's own Node.Create bracket, and directly comparable to it.
                    m_Octree.DevStepStartHook();

                    built[slot] = Node.Create(
                            m_Octree,
                            _LeavesSpatialGrid,
                            triangleStorage,
                            _ConstructionBounds,
                            target.Size,
                            target.Index,
                            target.Layer,
                            triangleStorage.IsIntersect(bounds),
                            m_Octree.GetNextNodeID,
                            maxLayer,
                            needCheckMaxLayer,
                            _CancellationToken,
                            m_ConcurrencySemaphore,
                            null,
                            null
                        );

                    m_Octree.DevStepEndHook("construct: Node.Create (per subtree, CPU)");

                    _ConstructionProgress?.AddProcessedLeavesCount(1);
                };

                if (m_ConcurrencySemaphore.Wait(0))
                {
                    taskSet.Add(
                            Task.Run(
                                    () =>
                                    {
                                        try
                                        {
                                            GlobalOctree.DevTaskCpuStart();

                                            buildOne();
                                        }
                                        finally
                                        {
                                            GlobalOctree.DevTaskCpuEnd();
                                            m_ConcurrencySemaphore.Release();
                                        }
                                    },
                                    _CancellationToken
                                )
                        );
                }
                else
                {
                    buildOne();
                }
            }

            m_Octree.DevStepEndHook("construct: dispatch + inline subtrees");

            m_Octree.DevStepStartHook();

            Task allDone = Task.WhenAll(taskSet);
            WaitHandle.WaitAny(new[] { ((IAsyncResult) allDone).AsyncWaitHandle, _CancellationToken.WaitHandle });

            m_Octree.DevStepEndHook("construct: wait for subtree tasks");

            if (_CancellationToken.IsCancellationRequested)
            {
                JoinAfterCancellation(taskSet);

                return;
            }

            m_Octree.DevStepStartHook();

            //index-based: each graft pairs a target with the node built for it at the same position
            for (int targetNum = 0; targetNum < _Targets.Count; targetNum++)
            {
                SubtreeRebuildTarget target = _Targets[targetNum];

                //nothing built means the cell has no reason to exist any more (no cluster reaches it): the slot must
                //be EMPTIED, not left holding the node the rebuild retired
                if (built[targetNum] == null)
                {
                    target.Parent.ChildrenMap.Remove(target.Slot);

                    continue;
                }

                //the same canonical-leaf rule every child graft in Fork.ProduceChildren applies
                target.Parent.SetChild(target.Slot, Fork.AdoptCanonicalLeaf(built[targetNum], _LeavesSpatialGrid));
            }

            m_Octree.DevStepEndHook("construct: graft subtrees");
        }


        public void BuildRootsReusingCache(
                float             _RootSize,
                Vector3Int[]      _RootsIndices,
                IOccupancyStorage _TriangleStorage,
                Bounds[]          _ObstacleClusters,
                LeavesSpatialGrid _LeavesSpatialGrid,
                int               _BaseLayersCount,
                CancellationToken _CancellationToken,
                //completed subtrees that should be inserted instead of calculating new ones
                NodeSpatialGrid _CompletedSubtreesCache,
                //completed subtrees where can be presented excessive leaves located outside any of _ObstacleClusters
                NodeSpatialGrid _ExcessiveSubtreesCache,
                //subtrees where absent some children. The need to be constructed
                NodeSpatialGrid           _PartialSubtreesCache,
                NodeSpatialGrid           _ActiveRootsGrid,
                GraphConstructionProgress _ConstructionProgress
            )
        {
            try
            {
                _ConstructionProgress?.SetTotalRootsCount(_RootsIndices.Length);

                List<Task>              taskSet         = new List<Task>(_RootsIndices.Length);
                ConcurrentHashSet<Node> roots           = new ConcurrentHashSet<Node>();
                ConcurrentHashSet<Leaf> excessiveLeaves = new ConcurrentHashSet<Leaf>();

                foreach (Vector3Int rootIndex in _RootsIndices)
                {
                    Bounds rootBounds = UtilsSpatialGrid.GetBucketBounds(rootIndex, _RootSize);

                    Action processRoot = () =>
                    {
                        bool needCheckMaxLayer = m_Octree.CheckMaxLayer(rootBounds, _BaseLayersCount - 1, out int maxLayer);

                        Node root;

                        //try to get completed subtree from cache
                        if (_CompletedSubtreesCache != null && _CompletedSubtreesCache.TryGetNode(0, rootIndex, out Node cachedCompletedRoot))
                        {
                            root = cachedCompletedRoot;
                            root.AttachNewIDsToSubtree(m_Octree.GetNextNodeID);
                        }
                        else if (_ExcessiveSubtreesCache != null && _ExcessiveSubtreesCache.TryGetNode(0, rootIndex, out Node cachedExcessiveRoot))
                        {
                            root = cachedExcessiveRoot;
                            root.AttachNewIDsToSubtree(m_Octree.GetNextNodeID);
                            root.TryPruneExcessiveLeaves(_ObstacleClusters, excessiveLeaves);
                        }
                        //try to get partial subtree from cache
                        else if (_PartialSubtreesCache != null && _PartialSubtreesCache.TryGetNode(0, rootIndex, out Node cachedPartialRoot))
                        {
                            root = cachedPartialRoot;
                            root.AttachNewIDsToSubtree(m_Octree.GetNextNodeID);

                            root.ComplementAbsentChildren(
                                    maxLayer,
                                    m_Octree.GetNextNodeID,
                                    _LeavesSpatialGrid,
                                    _TriangleStorage,
                                    _ObstacleClusters,
                                    _CancellationToken,
                                    needCheckMaxLayer,
                                    m_ConcurrencySemaphore,
                                    _CompletedSubtreesCache,
                                    _PartialSubtreesCache
                                );
                        }
                        //construct subtree
                        else
                        {
                            root = Node.Create(
                                    m_Octree,
                                    _LeavesSpatialGrid,
                                    _TriangleStorage,
                                    _ObstacleClusters,
                                    _RootSize,
                                    rootIndex,
                                    0,
                                    true,
                                    m_Octree.GetNextNodeID,
                                    maxLayer,
                                    needCheckMaxLayer,
                                    _CancellationToken,
                                    m_ConcurrencySemaphore,
                                    _CompletedSubtreesCache,
                                    _PartialSubtreesCache
                                );
                        }

                        roots.TryAdd(root);

                        //root-granular progress: this considered root is fully built/reused (thread-safe Interlocked add)
                        _ConstructionProgress?.AddProcessedLeavesCount(1);
                    };

                    if (m_ConcurrencySemaphore.Wait(0))
                    {
                        taskSet.Add(
                                Task.Run(
                                        () =>
                                        {
                                            try
                                            {
                                                processRoot();
                                            }
                                            finally
                                            {
                                                m_ConcurrencySemaphore.Release();
                                            }
                                        },
                                        _CancellationToken
                                    )
                            );
                    }
                    else
                    {
                        processRoot();
                    }
                }

                Task allDone = Task.WhenAll(taskSet);
                WaitHandle.WaitAny(new[] { ((IAsyncResult) allDone).AsyncWaitHandle, _CancellationToken.WaitHandle });

                if (_CancellationToken.IsCancellationRequested)
                    return;

                //clean up excessive leaves (their refs were already removed from parents during TryPruneExcessiveLeaves.
                //Now we just should remove refs from neighboring leaves)
                foreach (Leaf excessiveLeaf in excessiveLeaves)
                {
                    foreach (Leaf neighbor in excessiveLeaf.FreeNeighbors)
                        neighbor.RemoveFreeNeighbor(excessiveLeaf);

                    _LeavesSpatialGrid.RemoveLeaf(excessiveLeaf);
                }

                foreach (Node root in roots)
                    _ActiveRootsGrid.AddNode(root);
            }
            catch (OperationCanceledException)
            {
                //Cancellation requested — silently interrupt execution
            }
            catch (Exception exception)
            {
                Debug.LogException(exception);
            }
        }

        #endregion

        #region Service methods

        //A cancelled build returns early, but its spawned tasks are still winding down (each fork checks the token
        //and bails, so this is milliseconds). They are joined here rather than abandoned because the operation's
        //buffers are now REUSED across operations (OperationScratch): a straggler writing into the construction
        //grid after the next operation took it would corrupt that build. The scratch is also DISCARDED after any
        //incomplete operation (StagingTransaction.Dispose), so this join is the first line and the discard the
        //second. Exceptions the tasks throw for the cancellation are theirs to have; nothing is published.
        static void JoinAfterCancellation(List<Task> _Tasks)
        {
            try
            {
                Task.WaitAll(_Tasks.ToArray());
            }
            catch (AggregateException)
            {
                //cancelled / faulted tasks — expected on this path
            }
        }

        #endregion

    }
}
