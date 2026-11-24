using Nav3D.API;
using Nav3D.Common;
using Nav3D.SpaceProcessing.ObstacleProcessing.UtilityTypes;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using Nav3D.Obstacles;
using UnityEngine;
using Node = Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.Elements.Node;
using Fork = Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.Elements.Fork;
using Leaf = Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.Elements.Leaf;
using NodeSpatialGrid = Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.SpatialStorages.NodeSpatialGrid;
using LeavesSpatialGrid = Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.SpatialStorages.LeavesSpatialGrid;
using TriangleStorage = Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.SpatialStorages.TriangleStorage;
using GraphConstructionProgress = Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.OperationsProgress.GraphConstructionProgress;
using ObstacleAdditionOperation = Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.Operations.ObstacleAdditionOperation;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree
{
    partial class GlobalOctree
    {

        #region Attributes

        //The roots grid the construction code (ConstructRoots / prune / complement / connect helpers) builds into.
        //Normally it aliases m_NavTree.LiveRootsGrid, so construction mutates the live tree directly. While an
        //obstacle operation is building, it is pointed at a private staging grid instead, so the heavy build never
        //touches the live tree; the finished result is installed by NavTree.PublishStagedRegion. Readers never use
        //this field.
        NodeSpatialGrid m_ActiveRootsGrid;

        //The octree's live read surface — m_RootsGrid, m_TreeLock, m_NavTree.TriangleClusterStorage, every
        //read-query method, and open-face classification (NavTree.cs).
        readonly NavTree m_NavTree;

        //all obstacle→geometry bookkeeping (GeometryRegistry.cs): raw/safe clusters per obstacle,
        //obstacle info, cluster→roots, root→cluster indices (safe + raw), cluster→owning obstacle IDs — behind
        //atomic Register/Unregister methods instead of seven parallel maps kept in sync by convention.
        readonly GeometryRegistry m_GeometryRegistry;

        //The pure node-construction primitive (RootBuilder.cs) — build/complement
        //one root's subtree from an explicit geometry view. Owns no mutable octree state itself; the active roots
        //grid and per-op construction progress are passed explicitly by the caller on every call.
        readonly RootBuilder m_RootBuilder;

        //the per-operation buffers every operation reuses in turn (see OperationScratch's header)
        readonly OperationScratch m_OperationScratch = new OperationScratch();

        //The four obstacle-operation workers — each a standalone internal sealed class holding this
        //GlobalOctree back-reference, owning one op's business logic (footprint/delta computation, the
        //StagingTransaction lifecycle, post-publish gizmo/notify). RebuildRoots (the shared rebuild primitive all
        //four call, 3 directly + PreBakedAdd via its own two helpers) stays on GlobalOctree itself — see
        //RuntimeOperations.cs's own note on why. Only the add worker lives in this shared file; the other three are
        //created on first use by their own partials (GlobalOctree.Pro.cs, GlobalOctree.ResolutionRegions.cs).
        readonly AddObstacleWorker m_AddObstacleWorker;

        readonly ObstacleOperationsExecutor m_OperationsExecutor = new ObstacleOperationsExecutor();

        readonly float m_MinBucketSizeBase;
        float          m_MinBucketSizeReal;

        //The finest bucket size any baked binary loaded into this octree was baked at (float.MaxValue = none loaded).
        //Baked subtrees are installed verbatim, so they can be DEEPER than the live region set justifies — a scene
        //baked with a resolution region and opened with that region gone carries leaves the current layer count has no
        //bucket size for, and installing them threw IndexOutOfRange out of GetBucketSizeOnLayer. Written by the
        //deserializer (GlobalOctree.Serialization.cs) before the pre-baked add runs, folded into the capacity floor by
        //UpdateMinBucketSize below.
        float m_MinBucketSizeBaked = float.MaxValue;

        //Per-operation frozen view of the registered resolution regions. Captured once at the start of each
        //operation (UpdateMinBucketSize) and read by the construction worker threads (CheckMaxLayer) instead of
        //re-querying the live ResolutionRegionManager per node, so
        //the whole build sees one consistent region set. Operations are serialized, so only one op writes it at a time;
        //within an op it is set before construction tasks spawn (a happens-before via Task scheduling) then only read.
        ResolutionRegionSnapshot m_RegionSnapshot = ResolutionRegionSnapshot.Empty;

        readonly float m_MaxAgentRadius;

        readonly int   m_BaseLayersCount;
        readonly float m_RootSize;
        int            m_LayersCount;

        volatile int m_CurNodeID = -1;

        float[] m_BucketSizes;

        readonly SemaphoreSlim m_ConcurrencySemaphore;

        #endregion

        #region Events

        //Raised on the main thread after a publish, carrying the volumes whose occupancy/connectivity changed. Higher
        //layers subscribe: pathfinding re-paths every stored path crossing any of them (stale trajectories through
        //newly-blocked space, missed newly-opened shortcuts) and local avoidance marks agents in them obstacle-dirty.
        //GlobalOctree itself stays decoupled from Pathfinding / LocalAvoidance — SpaceProcessing must not depend on
        //them — so it only emits the volumes and lets the orchestrator (ObstacleManager) wire the rest.
        //
        //A list, not one Bounds: an obstacle's geometry can be in spatially disjoint pieces (one component on a parent
        //with distant child meshes is an ordinary setup), and one AABB over all of them declares the empty space
        //between them changed too. See NotifyRegionChanged for the whole argument. Every element is already expanded
        //by the notification margin, so subscribers use them verbatim.
        public event Action<IReadOnlyList<Bounds>> OnRegionChanged;

        #endregion

        #region Properties

        public int NodeCount => m_CurNodeID + 1;

        //internal (not private) setter: AddObstacleWorker.cs (a standalone class) sets this once per
        //add; Remove/RegionUpdate workers only ever read it via the public getter.
        public GraphConstructionProgress ConstructionProgress { get; internal set; }

        //internal (not private): RootBuilder.cs / NavTree.cs (standalone classes) need these —
        //everything else that changes per-call (active roots grid, construction progress) stays an explicit
        //parameter instead of a widened field; these are the only members widened for that extraction.
        internal float RootSize          => m_RootSize;
        internal float MinBucketSizeReal => m_MinBucketSizeReal;

        //internal (not private): StagingTransaction.cs (a standalone class) needs these three.
        //ActiveRootsGrid originally had a set-only accessor — StagingTransaction only ever wrote the alias (point
        //construction at its staging grid on entry, restore it to the live tree on Dispose). The getter was added
        //for PreBakedAddWorker.cs, which reads it extensively (TryGetNode etc.) during its unlocked build.
        //ActiveLayersCount is not the same member as GlobalOctree.Editor.cs's public `LayersCount` — that one is
        //`#if UNITY_EDITOR`-only (editor/gizmo tooling), so StagingTransaction (shipped runtime code, compiled in
        //Player builds too) needs its own always-compiled accessor to the same backing field.
        internal NodeSpatialGrid ActiveRootsGrid
        {
            get => m_ActiveRootsGrid;
            set => m_ActiveRootsGrid = value;
        }

        internal NavTree NavTree           => m_NavTree;
        internal int     ActiveLayersCount => m_LayersCount;

        //internal (not private): AddObstacleWorker.cs/RemoveObstacleWorker.cs/RegionUpdateWorker.cs
        //(standalone classes) need these two (GeometryRegistry mirrors the NavTree property above; MaxAgentRadius is
        //read only by AddObstacleWorker for triangle clustering).
        internal GeometryRegistry GeometryRegistry => m_GeometryRegistry;
        internal float            MaxAgentRadius   => m_MaxAgentRadius;

        //internal (not private): PreBakedAddWorker.cs (a standalone class) needs these two.
        internal RootBuilder RootBuilder     => m_RootBuilder;
        internal int         BaseLayersCount => m_BaseLayersCount;

        #endregion

        //ON, and these two are BEHAVIOUR rather than options — no customer surface mentions them, nothing in the
        //inspector exposes them, and the only reason they are fields at all is that a dev probe must be able to
        //take the other arm of an A/B (`Dev/Tools/BuildPassArms`, the `-armNo*` arguments; the same convention as
        //`-cfpNarrowCloneOff` and `-allocProbeNarrowCloneOff`). Never write to them from shipped code.
        //
        //parallel open-FACE classification. `NavTree.ClassifyOpenFacesStaged` is the whole publish phase of a large
        //build (1 478 ms of a 6 969 ms Skyline add; the write-locked install itself is 0.2 ms) and it is
        //root-PARTITIONED by construction — a leaf belongs to exactly one root, so no leaf is visited twice, and
        //the only writes are a leaf's own open-face bits. Measured **4.3x on the pass** (1 478 -> 342 ms).
        internal static bool ParallelClassifyEnabled { get; set; } = true;

        //the repair PASS SKIPS ROOTS it CREATED. `RepairStagedRegionFreeGraph` verifies every free-neighbour ref
        //aimed into a rebuilt root, and "rebuilt" used to mean the operation's whole footprint — including roots
        //that did not exist in the live tree at all. A ref can only be stale if the leaf it points at was
        //DISCARDED, and nothing was discarded in a root that was CREATED, so those verifications can find nothing:
        //measured 1.5 s over 506 552 leaves and ~1.5 M tree descents producing zero edits on a first Skyline load.
        //The replaceable set is now the roots the live tree actually holds. Inert on an update, where the pass
        //keeps walking and keeps deferring its edits (Run A's op-cost rig: 55 / 87 / 145, unchanged).
        internal static bool RepairScopeSkipsCreatedRootsEnabled { get; set; } = true;

        #region Constructors

        public GlobalOctree(int _LayersCount, float _MinBucketSize, float _MaxAgentRadius, int _MaxConcurrency = 0)
        {
            m_BaseLayersCount   = _LayersCount;
            m_LayersCount       = _LayersCount;
            m_MinBucketSizeBase = _MinBucketSize;
            m_MinBucketSizeReal = _MinBucketSize;
            m_MaxAgentRadius    = _MaxAgentRadius;
            m_RootSize          = _MinBucketSize * Mathf.Pow(2, _LayersCount - 1);

            int maxConcurrency = _MaxConcurrency > 0 ? _MaxConcurrency : Environment.ProcessorCount;

            m_ConcurrencySemaphore = new SemaphoreSlim(maxConcurrency);

            m_NavTree          = new NavTree(this, _LayersCount);
            m_ActiveRootsGrid  = m_NavTree.LiveRootsGrid;
            m_GeometryRegistry = new GeometryRegistry(m_RootSize);
            m_RootBuilder      = new RootBuilder(this, m_ConcurrencySemaphore);

            m_AddObstacleWorker = new AddObstacleWorker(this);
        }

        #endregion

        #region Public methods

        public void AddObstacle(
                Func<ObstacleInfo>       _GetObstacleInfo,
                ObstacleAdditionProgress _AdditionProgress,
                CancellationToken        _CancellationToken,
                Action                   _OnFinish,
                bool                     _EditMode
            )
        {
            m_OperationsExecutor.EnqueueAdditionalOrder(
                    _GetObstacleInfo().ID,
                    new ObstacleAdditionOperation(_GetObstacleInfo, _AdditionProgress, AddObstacle, _CancellationToken, _OnFinish, _EditMode)
                );
        }

        public bool IsBoundsCrossAnyObstacleBody(Bounds _Bounds) => m_NavTree.IsBoundsCrossAnyObstacleBody(_Bounds);

        internal bool TryIsPointInsideSolid(Vector3 _Point, out bool _Result) => m_NavTree.TryIsPointInsideSolid(_Point, out _Result);

        public bool IsPointInsideOccupiedLeaf(Vector3 _Point) => m_NavTree.IsPointInsideOccupiedLeaf(_Point);

        //the per-tick main-thread forms: false = a publish holds or waits for the tree, keep last tick's answer
        public bool TryIsPointInsideOccupiedLeaf(Vector3 _Point, out bool _Result) => m_NavTree.TryIsPointInsideOccupiedLeaf(_Point, out _Result);

        public bool TrySegmentIntersectOccupiedLeaf(Segment3 _Segment, out bool _Result) => m_NavTree.TrySegmentIntersectOccupiedLeaf(_Segment, out _Result);

        //Leaf snapshot for debug visualization: copies the occupied leaves — and, when asked, the free ones —
        //out as plain structs (read-locked inside, see NavTree.SnapshotLeaves).
        public void SnapshotLeaves(List<NavLeafInfo> _Result, bool _IncludeFree = false, NavGraphSnapshot _Graph = null) =>
            m_NavTree.SnapshotLeaves(_Result, _IncludeFree, _Graph);

        public void SnapshotBoxes(NavBoxSnapshot _Result) => m_NavTree.SnapshotBoxes(_Result);

        public Triangle[] GetIntersectedObstaclesTriangles(Bounds _Bounds) => m_NavTree.GetIntersectedObstaclesTriangles(_Bounds);

        //the main thread's non-blocking form: false = a publish holds or waits for the tree, ask again next tick
        public bool TryGetIntersectedObstaclesTriangles(Bounds _Bounds, out Triangle[] _Result) => m_NavTree.TryGetIntersectedObstaclesTriangles(_Bounds, out _Result);

        public float GetBucketSizeOnLayer(int _Layer)
        {
            return m_BucketSizes![_Layer];
        }

        public Vector3Int GetBucketIndexOnLayer(int _Layer, Leaf _Leaf)
        {
            return UtilsSpatialGrid.GetBucketIndex(_Leaf.Bounds.center, GetBucketSizeOnLayer(_Layer));
        }

        public string GetStorageContentDescription()
        {
            return $"{nameof(GlobalOctree)}: {m_GeometryRegistry.GetStorageContentDescription()}";
        }

        #endregion

        #region Service methods

        //Collects every triangle that crosses the given root's bounds across all clusters touching it: the
        //clusters already registered in the global storage plus the in-flight operation clusters (which are not
        //registered until after construction). This gives a shared root its complete geometry, so it is built
        //against every obstacle that touches it — not just the cluster of the current construction pass. Without
        //it, a root shared by clusters built in separate passes is rebuilt with a storage that lacks another
        //cluster's crossing triangle, leaving a free leaf inside that obstacle's footprint (FREE_INTERSECTS).
        TriangleStorage BuildRootCombinedStorage(Vector3Int _RootIndex, IReadOnlyCollection<TriangleCluster> _OperationClusters)
        {
            Bounds            rootBounds = UtilsSpatialGrid.GetBucketBounds(_RootIndex, RootSize);
            HashSet<Triangle> triangles  = new HashSet<Triangle>();

            HashSet<TriangleCluster> existingClusters = null;

            if (m_NavTree.TriangleClusterStorage.TryGetCrossingBoundables(rootBounds, out existingClusters))
                foreach (TriangleCluster existingCluster in existingClusters)
                    triangles.UnionWith(existingCluster.GetCrossedTriangles(rootBounds));

            foreach (TriangleCluster operationCluster in _OperationClusters)
                if (operationCluster.Bounds.Intersects(rootBounds))
                    triangles.UnionWith(operationCluster.GetCrossedTriangles(rootBounds));

            TriangleStorage storage = new TriangleStorage(triangles.ToArray());

            //the bodies the root's free cells are classified against, whole — the triangle set above is cut to the root
            storage.Solids = CreateSolidRegions(
                    existingClusters == null
                        ? _OperationClusters
                        : existingClusters.Concat(_OperationClusters.Where(_Cluster => _Cluster.Bounds.Intersects(rootBounds))),
                    new GeometryDelta(null, _OperationClusters)
                );

            return storage;
        }

        //the seam UpdateMinBucketSize captures the live region set through — see its comment there
        partial void CaptureRegionSnapshot();

        //internal (not private): AddObstacleWorker/RemoveObstacleWorker/RegionUpdateWorker (standalone
        //classes) all call this at the start of their build.
        internal void UpdateMinBucketSize()
        {
            //the operation-start seam every worker already calls: an operation's closure verdicts start empty
            ResetClosureVerdicts();

            //Capture the current resolution-region snapshot for the whole operation. It is a volatile static the manager
            //republishes on the main thread whenever regions change, so this is one atomic immutable-reference read —
            //no worker-thread access to the manager's Unity object (which was unreliable and made an op intermittently
            //see no regions → wrong layer count). Captured once here at the op's start, so the whole build sees one
            //consistent region set reflecting the latest region change. The capture is a partial-void seam filled by
            //GlobalOctree.ResolutionRegions.cs; without that file the call compiles to nothing and m_RegionSnapshot stays
            //Empty, so every node stops at the base layer.
            CaptureRegionSnapshot();

            float snapshotMinBucketSize = m_MinBucketSizeBase;

            //Snap the region's requested min bucket to the base power-of-two grid (base / 2^k) before it drives the
            //layer count. m_RootSize is fixed at construction as base * 2^(baseLayers-1); the octree assumes
            //RootSize == m_BucketSizes[0] == effectiveMinBucket * 2^(layers-1). Feeding a raw region value whose ratio
            //to the base is not a power of two (e.g. base 3, region 1) makes GetLayersCount overshoot RootSize, so
            //m_BucketSizes[0] != RootSize — root bounds use RootSize while child cell sizes use the bucket table, which
            //pushes index>=1 roots' children outside the obstacle and leaves them as empty Forks (uncovered geometry).
            //Snapping (the same as CheckMaxLayer/TryGetEmbracingRegionsMinRes already do) keeps the invariant exact.
            if (m_RegionSnapshot.TryGetMinBucketSize(out float minBucketSize) && minBucketSize < m_MinBucketSizeBase)
                snapshotMinBucketSize = GetClosestBucketSize(minBucketSize, m_MinBucketSizeBase);

            //The construction grids (LeavesSpatialGrid / NodeSpatialGrid) are sized to m_LayersCount, so m_LayersCount
            //must never be smaller than the depth of the finest leaf currently living in the tree — otherwise
            //removing/coarsening such a leaf indexes a grid layer that does not exist (ArgumentOutOfRange). The region
            //snapshot can transiently lose a region (e.g. the disable→enable churn of a domain reload while playing),
            //which would otherwise shrink the layer count below leaves a region-present op already built. So the
            //effective min bucket only ever gets finer, never coarser: keep the finest value this octree has ever
            //constructed at. CheckMaxLayer still gates actual per-node refinement by the current snapshot, so coarse
            //regions / region-free areas stay coarse — only the grid capacity (layer count) is retained.
            //The same rule covers baked data: m_MinBucketSizeBaked is the finest resolution any loaded binary carries,
            //and those subtrees are installed as they were baked regardless of which regions exist now.
            m_MinBucketSizeReal = Mathf.Min(Mathf.Min(snapshotMinBucketSize, m_MinBucketSizeBaked), m_MinBucketSizeReal);

            m_LayersCount = GetLayersCount(m_RootSize, m_MinBucketSizeReal);
        }

        //internal (not private): RootBuilder.cs needs this (see RootSize's own comment above).
        internal int GetNextNodeID()
        {
            return Interlocked.Increment(ref m_CurNodeID);
        }

        //internal (not private): RegionUpdateWorker.cs (a standalone class) needs this.
        internal Dictionary<Vector3Int, Node> RootIndicesToInstancesDict(IEnumerable<Vector3Int> _RootIndices)
        {
            return _RootIndices.Select(_Index => m_ActiveRootsGrid.TryGetNode(0, _Index, out Node root) ? root : null)
                               .Where(_Root => _Root != null)
                               .ToDictionary(_Root => _Root.Index, _Root => _Root);
        }

        //Resolves the max subdivision layer for a node and whether its children must re-check. A node is refined to a
        //region's resolution when it merely TOUCHES the region — it need not be entirely embraced. Returns true
        //(children re-check) whenever a region is touched, because a child may leave every region (→ base) or enter a
        //finer one. The return value is the construction's needCheckMaxLayer flag.
        public bool CheckMaxLayer(Bounds _Bounds, int _CurMaxLayer, out int _ResultingMaxLayer)
        {
            //read the per-operation frozen region snapshot (captured in UpdateMinBucketSize) — safe to read from the
            //parallel construction worker threads, and consistent across the whole build
            return CheckMaxLayer(_Bounds, _CurMaxLayer, m_RegionSnapshot, out _ResultingMaxLayer);
        }

        //Same decision against an explicitly supplied region set. The construction path uses the overload above (the
        //operation's own frozen snapshot); a reader that must reproduce the decision without being the operation —
        //today only the dev occupancy validator — passes its own snapshot here instead of assigning m_RegionSnapshot,
        //which is the operation's state and is read per node by an in-flight build.
        public bool CheckMaxLayer(Bounds _Bounds, int _CurMaxLayer, ResolutionRegionSnapshot _RegionSnapshot, out int _ResultingMaxLayer)
        {
            //No region touches this node ⇒ neither it nor any descendant (a subset) can touch one. Cap at the base
            //layer and stop re-checking. (Cap explicitly rather than inheriting _CurMaxLayer, which may carry a finer
            //parent value now that touched nodes also keep re-checking their children.)
            if (!_RegionSnapshot.TryGetCrossingRegionsMinBucketSize(_Bounds, out float regionMinBucketSize))
            {
                _ResultingMaxLayer = m_BaseLayersCount - 1;
                return false;
            }

            float bucketSize = GetClosestBucketSize(regionMinBucketSize, m_MinBucketSizeBase);
            _ResultingMaxLayer = GetLayersCount(RootSize, bucketSize) - 1;

            return true;
        }

        int GetLayersCount(float _MaxBucketSize, float _MinBucketSize)
        {
            int   currentLayerNum   = 1;
            float currentBucketSize = _MinBucketSize;

            while (currentBucketSize < _MaxBucketSize && currentLayerNum < 10)
            {
                currentBucketSize *= 2f;
                currentLayerNum++;
            }

            return currentLayerNum;
        }

        //determines the number of the layer, that has the greatest bucket size less
        //than the smallest bucket size among all resolution regions that embraces the given bounds
        float GetClosestBucketSize(float _Resolution, float _MinBucketSize)
        {
            float minBucketSize = _MinBucketSize;
            float multiplier;

            if (_Resolution < minBucketSize)
            {
                multiplier = 0.5f;

                do
                {
                    minBucketSize *= multiplier;
                } while (minBucketSize > _Resolution);
            }
            else if (_Resolution > minBucketSize)
            {
                multiplier = 2f;

                while (minBucketSize < _Resolution)
                {
                    minBucketSize *= multiplier;
                }
            }

            return minBucketSize;
        }

        /// <summary>
        /// Filter out octree leaves leaving only those that does not intersect any of Bounds from set.
        /// The intersected leaves are cut off, their outside neighboring leaves are filled into HashSet to reconnect them with newly constructed leaves.
        /// The intersected leaf's parents are cut off as well, the same takes place for their parents and so on upper by tree.
        /// Returns:
        /// So at the result we have:
        /// 1)A set of unchanged roots (no any of their ancestors touched any of Bounds from set).
        /// 2)A set of partially pruned roots (some of their leaves or subnodes touched any of Bounds from set).
        /// 3)A set of leaves neighboring to pruned leaves (only outer leaves that did not relate to the reconstruction volume).
        /// 4)A set of triangles that should be used during construction. (This is all triangles which intersect any Bounds from set).
        /// Deliberately does not expose the pruned nodes' own bounds: node bounds must never gate construction.
        /// TryPruneTouchedSubtrees clears a fork whole (any Leaf child ⇒ up to a whole octant/root), so a pruned
        /// node's bounds cover volumes outside every obstacle shell — and, Bounds.Intersects being inclusive, touch
        /// every sibling octant — so a rebuild gated by them free-fills the pruned fork and spills whole-octant free
        /// leaves into adjacent absent octants (the ORPHANED_FREE_LEAF spill). Rebuild gating must come
        /// from cluster shells only (a root is tiled only within the obstacle + inflation shell).
        /// </summary>
        /// <param name="_Roots">All octree's roots that affected by subtrees recreation volume.</param>
        /// <param name="_Triangles">All surrounding triangles that will be filtered. Will be left only these, which crossed any of pruned nodes.</param>
        /// <param name="_TriangleBoundsSet">New triangles clusters.</param>
        /// <param name="_FormerBoundsSet">The set of the bounds of the former triangles clusters (with safe area).</param>
        //internal (not private): PreBakedAddWorker.cs's ProcessIntersectingCachedClusters
        //(a standalone class) is this method's only caller.
        internal (Node[] UnchangedRoots, Node[] PrunedRoots, HashSet<Leaf> NeighborLeaves, HashSet<Triangle> TrianglesToReUse) PruneOctreeForRegionsAddition(
                Node[]            _Roots,
                HashSet<Triangle> _Triangles,
                Bounds[]          _TriangleBoundsSet,
                Bounds[]          _FormerBoundsSet
            )
        {
            HashSet<Node> embracingMinimalNodes = new HashSet<Node>();

            List<Node> unchangedRoots = new List<Node>(_Roots.Length);
            List<Node> prunedRoots    = new List<Node>(_Roots.Length);

            //triangles related to other clusters which touches edge roots
            HashSet<Triangle> foreignTriangles = new HashSet<Triangle>();

            foreach (Node root in _Roots)
            {
                if (_TriangleBoundsSet.All(_Bounds => !root.Bounds.Intersects(_Bounds)))
                {
                    if (!_FormerBoundsSet.Any() || _FormerBoundsSet.Any(_Bounds => _Bounds.Embrace(root.Bounds)))
                    {
                        unchangedRoots.Add(root);
                        DevTraceRootPath(UtilsSpatialGrid.GetBucketIndex(root.Bounds.center, RootSize), "prune:unchanged(not-crossed)");
                    }
                    //root was crossed by any of former triangle cluster bounds and now it is need to be checked if some of missed children can be added
                    else
                    {
                        prunedRoots.Add(root);
                        DevTraceRootPath(UtilsSpatialGrid.GetBucketIndex(root.Bounds.center, RootSize), "prune:partial-cache(not-crossed,former)");
                    }

                    continue;
                }

                //If the root contains any leaf among it first level children, then it have to be reconstructed entirely
                if (root is Fork rootFork && rootFork.ChildrenMap.AnyLeafChild())
                {
                    if (m_NavTree.TriangleClusterStorage.TryGetCrossingBoundables(root.Bounds, out HashSet<TriangleCluster> crossingClusters))
                    {
                        foreignTriangles.AddRange(crossingClusters.SelectMany(_Cluster => _Cluster.Triangles));
                    }

                    embracingMinimalNodes.Add(root);
                    DevTraceRootPath(UtilsSpatialGrid.GetBucketIndex(root.Bounds.center, RootSize), "prune:whole-root(first-level-leaf)");
                    continue;
                }

                //the root was partially pruned
                if (root.TryPruneTouchedSubtrees(_TriangleBoundsSet, out HashSet<Node> cutNodes))
                {
                    embracingMinimalNodes.UnionWith(cutNodes);
                    prunedRoots.Add(root);
                    DevTraceRootPath(UtilsSpatialGrid.GetBucketIndex(root.Bounds.center, RootSize), $"prune:partial-cache(cut={cutNodes.Count})");
                }
                //root was entirely embraced by any of former triangle cluster bounds (for now it still entirely inside new cluster)
                else if (_FormerBoundsSet.Any(_Bounds => _Bounds.Embrace(root.Bounds)))
                {
                    unchangedRoots.Add(root);
                    DevTraceRootPath(UtilsSpatialGrid.GetBucketIndex(root.Bounds.center, RootSize), "prune:unchanged(embraced)");
                }
                //root was crossed by any of former triangle cluster bounds and now it is need to be checked if some of missed children can be added
                else
                {
                    prunedRoots.Add(root);
                    DevTraceRootPath(UtilsSpatialGrid.GetBucketIndex(root.Bounds.center, RootSize), "prune:partial-cache(no-cut)");
                }
            }

            HashSet<Triangle> restTriangles;

            if (embracingMinimalNodes.Count == 0)
            {
                restTriangles = foreignTriangles;
            }
            else
            {
                //compute compound bounds of all embracing nodes for cheap pre-filter
                Bounds[] nodeBoundsArray = embracingMinimalNodes.Select(_Node => _Node.Bounds).ToArray();

                DevRecordPruneCutBounds(nodeBoundsArray);

                Bounds compoundNodeBound = nodeBoundsArray[0];

                for (int i = 1; i < nodeBoundsArray.Length; i++)
                    compoundNodeBound = compoundNodeBound.Union(nodeBoundsArray[i]);

                restTriangles = new HashSet<Triangle>(foreignTriangles);

                foreach (Triangle triangle in _Triangles)
                {
                    //Quick reject: skip triangles that don't even touch the compound bounds
                    if (!triangle.Intersects(compoundNodeBound))
                        continue;

                    //Detailed check against individual node bounds (only for triangles passing pre-filter)
                    foreach (Bounds nodeBounds in nodeBoundsArray)
                    {
                        if (triangle.Intersects(nodeBounds))
                        {
                            restTriangles.Add(triangle);
                            break;
                        }
                    }
                }
            }

            DevTraceOpNote($"prune: embracing={embracingMinimalNodes.Count} foreign={foreignTriangles.Count} rest={restTriangles.Count} of considered={_Triangles.Count}");

            //DEV: the cut region's extent, because restTriangles above is scoped to exactly it while the rebuild it
            //feeds complements the whole pruned root — a triangle outside this box but inside a rebuilt cell is the
            //shape of the open [StagedCoverage/prebaked-add] hole.
            if (embracingMinimalNodes.Count > 0)
            {
                Bounds cutCompound = embracingMinimalNodes.First().Bounds;

                foreach (Node cutNode in embracingMinimalNodes)
                    cutCompound = cutCompound.Union(cutNode.Bounds);

                DevTraceOpNote($"prune: cut compound {cutCompound.min:F2}..{cutCompound.max:F2}");
            }

            foreach (Node prunedRoot in prunedRoots)
                DevTraceOpNote($"prune: prunedRoot {prunedRoot.Bounds.min:F2}..{prunedRoot.Bounds.max:F2}");

            HashSet<Leaf> allContainedLeaves = GetAllContainedFreeLeaves(embracingMinimalNodes);
            HashSet<Leaf> neighborLeaves     = GetAllNeighborsAndCleanRefs(allContainedLeaves);
            HashSet<Leaf> prunedRootsLeaves  = GetAllContainedFreeLeaves(prunedRoots.ToHashSet());

            neighborLeaves.UnionWith(prunedRootsLeaves);

            return (unchangedRoots.ToArray(), prunedRoots.ToArray(), neighborLeaves.ToHashSet(), restTriangles);
        }

        //AddToRootToClusterMap, RemoveFromRootToClusterMap, CollectObstacleIDsInRoots,
        //GetRawCrossedRootIndices, AddToRootToRawClusterMap, and RemoveFromRootToRawClusterMap all moved to
        //GeometryRegistry.cs (m_GeometryRegistry.*) — same bodies, same semantics (incl. the unguarded
        //m_RootToSafeClustersMap[rootIndex] indexer in the old RemoveFromRootToClusterMap, now
        //GeometryRegistry.RemoveFromRootToClusters).

        HashSet<Leaf> GetAllContainedFreeLeaves(HashSet<Node> _Nodes)
        {
            HashSet<Leaf> result = new HashSet<Leaf>();

            foreach (Node node in _Nodes)
            {
                node.GetAllContainedFreeLeaves(result);
            }

            return result;
        }

        HashSet<Leaf> GetAllNeighborsAndCleanRefs(HashSet<Leaf> _Leafs, LeavesSpatialGrid _Grid = null)
        {
            HashSet<Leaf> result = new HashSet<Leaf>();

            GetAllNeighborsAndCleanRefs(_Leafs, _Grid, result, new List<Leaf>());

            return result;
        }

        //The scratch-backed form the runtime prune uses: the neighbourhood lands in _Result (cleared first) and each
        //leaf's neighbour set is snapshotted into _NeighborBuffer instead of a fresh array per leaf
        //.
        void GetAllNeighborsAndCleanRefs(HashSet<Leaf> _Leafs, LeavesSpatialGrid _Grid, HashSet<Leaf> _Result, List<Leaf> _NeighborBuffer)
        {
            HashSet<Leaf> result = _Result;

            result.Clear();

            //Add all neighbors of the source leafs collection
            //Note: the members of source leafs collection will be presented in the resulting set.
            foreach (Leaf leaf in _Leafs)
            {
                leaf.CopyFreeNeighborsTo(_NeighborBuffer);

                foreach (Leaf neighborLeaf in _NeighborBuffer)
                {
                    result.Add(neighborLeaf);
                    neighborLeaf.RemoveFreeNeighbor(leaf);
                }

                leaf.RemoveAllFreeNeighbors();

                //A pruned leaf leaves the node tree, but it may have been seeded into the spatial grid as an
                //external neighbor by an earlier prune in the same operation. If left behind it becomes a
                //"ghost": FormLeavesConnections would wire live leaves to it, producing dangling references.
                //Evict it so the grid never holds a leaf that is no longer in the tree.
                _Grid?.RemoveLeaf(leaf);
            }

            //Remove all source leafs from result set
            foreach (Leaf leaf in _Leafs)
            {
                result.Remove(leaf);
            }
        }


        //Enforces the invariant FormLeavesConnections relies on: every leaf in the spatial grid must be the
        //tree's live leaf at its location. During an operation the grid accumulates leaves (seeded neighbors,
        //existing leaves re-registered during reconstruction, freshly built leaves) that subsequent reconstruction
        //can make obsolete — pruned, occupied, coarsened, or replaced by a different-resolution leaf. Such a stale
        //leaf is a "ghost"; if left in the grid, FormLeavesConnections wires live leaves to it, producing dangling
        //neighbor references. This removes every grid leaf that is no longer the tree's canonical leaf for its cell
        //(the tree and grid share the same leaf instance per cell, so a reference mismatch means the grid entry is
        //stale).
        //internal (not private): PreBakedAddWorker.cs's ProcessIntersectingCachedClusters
        //(a standalone class) calls this too, alongside RebuildRoots's own in-class call.
        internal void PurgeGridGhosts(LeavesSpatialGrid _Grid)
        {
            //walked in place (a ConcurrentDictionary enumeration tolerates removal of the entry it is on) rather than
            //through a snapshot array of every grid leaf — that array was one allocation per operation proportional
            //to the whole construction grid
            _Grid.RemoveWhere(_GridLeaf =>
                    {
                        Vector3 center = _GridLeaf.Bounds.center;

                        Leaf treeLeaf = m_ActiveRootsGrid.TryGetNode(0, UtilsSpatialGrid.GetBucketIndex(center, RootSize), out Node root)
                            ? root.GetClosestOrEmbracingLeaf(center)
                            : null;

                        return !ReferenceEquals(treeLeaf, _GridLeaf);
                    }
                );
        }


        //internal (not private): AddObstacleWorker/RemoveObstacleWorker/RegionUpdateWorker need this.
        internal void FillBucketSizesCache()
        {
            m_BucketSizes = new float[m_LayersCount];

            for (int layerNum = 0; layerNum < m_LayersCount; layerNum++)
            {
                m_BucketSizes[layerNum] = m_MinBucketSizeReal * Mathf.Pow(2, m_LayersCount - layerNum - 1);
            }
        }

        partial void ResetGizmosDataForObstacles(params int[] _IDs);

        //Internal, non-partial forwarder for the 3 workers (standalone classes): ResetGizmosDataForObstacles
        //is implemented in the `#if UNITY_EDITOR`-only GlobalOctree.Editor.cs, so it must stay a bare (implicitly
        //private) partial method: widening it would make it a C# 9 "extended" partial, which requires an
        //implementation and so breaks a Player build. This forwarder is always compiled and elides to nothing
        //outside the editor.
        internal void ResetGizmosDataForObstaclesHook(int[] _IDs) => ResetGizmosDataForObstacles(_IDs);

        //Raise OnRegionChanged on the main thread with the volumes the PATH/AGENT layer must re-examine.
        //Dispatched rather than invoked inline, because the publish runs on a worker while the subscribers are
        //main-thread only. A no-op when nothing changed or no one is listening.
        //
        //_OperationVolumes is the operation's own footprint, piece by piece — one volume per triangle cluster it
        //added or removed, or the single volume a region update re-resolved. Absent, the notification falls back to
        //the clone-set AABB.
        //
        //why A list and not one union BOX. Clusters are formed by AABB-distance clustering, so separate clusters are
        //exactly the obstacle's spatially disjoint pieces, and encapsulating them declares the empty space between
        //them changed too — one obstacle on a parent with two distant child meshes would re-path everything crossing
        //the gap, and a moving obstacle's union would span its whole travel. Firing per volume is safe because both
        //subscribers are idempotent, and the path handler takes the whole list at once so a path crossing two
        //volumes is still dispatched exactly once.
        //
        //why not the clone set. That is a publish concept — affected roots plus a one-root ring, because the publish
        //must repoint the ring's back-refs — and its AABB is enormously coarser than what changed: a 3x3x3 ring of
        //8 u roots is a 24 u box, so a slab moving a few centimetres invalidated every path in the playfield. The
        //ring is required for the publish and irrelevant to path validity: geometry there is unchanged.
        //
        //the margin, and why it has two terms — both measured rather than assumed:
        //  * quantisation — occupancy flips only within about one bucket of the geometry itself. The bucket that
        //    bounds it is the COARSEST one that can be a terminal occupied leaf here: normally m_MinBucketSizeReal,
        //    but a coarsening resolution region caps subdivision inside its volume, so a crossing region's own
        //    bucket size wins when it is coarser.
        //  * MaxAgentRadius — path relevance, which is what the subscribers test. Agent size is applied at query time,
        //    so geometry appearing within an agent's radius of a stored path invalidates it even though occupancy
        //    never changed on the path itself. Dropping this term is the one way to make this change unsafe.
        internal void NotifyRegionChanged(ICollection<Vector3Int> _ChangedRootIndices, IReadOnlyList<Bounds> _OperationVolumes = null)
        {
            //the hierarchy's region map answers for the tree as published; the publish marked the roots it rebuilt
            //(NavTree.NoteRebuild) and this starts the build that repaints them
            m_NavTree.ScheduleRegionMapRebuild();

            if (OnRegionChanged == null)
                return;

            List<Bounds> changedVolumes;

            if (_OperationVolumes != null && _OperationVolumes.Count > 0)
            {
                changedVolumes = new List<Bounds>(_OperationVolumes.Count);

                foreach (Bounds operationVolume in _OperationVolumes)
                    changedVolumes.Add(ExpandByNotificationMargin(operationVolume));

                CollapseToUnionIfCheaper(changedVolumes);
            }
            else
            {
                if (_ChangedRootIndices.Count == 0)
                    return;

                bool   hasBounds      = false;
                Bounds cloneSetBounds = default;

                foreach (Vector3Int rootIndex in _ChangedRootIndices)
                {
                    Bounds rootBounds = UtilsSpatialGrid.GetBucketBounds(rootIndex, RootSize);

                    if (!hasBounds)
                    {
                        cloneSetBounds = rootBounds;
                        hasBounds      = true;
                    }
                    else
                    {
                        cloneSetBounds.Encapsulate(rootBounds);
                    }
                }

                changedVolumes = new List<Bounds> { cloneSetBounds };
            }

            ThreadDispatcher.BeginInvoke(() => OnRegionChanged?.Invoke(changedVolumes));
        }

        //One volume per cluster — the operation's own footprint, piece by piece, for NotifyRegionChanged above (see
        //there for why the pieces must stay apart). Returns false for an empty set, in which case the caller must fall
        //back to the clone-set bounds rather than notify nothing.
        internal static bool TryGetClusterVolumes(IReadOnlyList<TriangleCluster> _Clusters, out List<Bounds> _Volumes)
        {
            _Volumes = null;

            if (_Clusters == null || _Clusters.Count == 0)
                return false;

            _Volumes = new List<Bounds>(_Clusters.Count);

            foreach (TriangleCluster cluster in _Clusters)
                _Volumes.Add(cluster.Bounds);

            return true;
        }

        //the margin's two terms are documented on NotifyRegionChanged; the reason it is applied per volume is the
        //quantisation term: it is derived from the volume itself, so a small cluster in a fine region must not inherit
        //a coarsening region's bucket size merely because a distant sibling cluster crosses that region.
        Bounds ExpandByNotificationMargin(Bounds _OperationVolume)
        {
            float quantisation = m_MinBucketSizeReal;

            if (m_RegionSnapshot != null && m_RegionSnapshot.TryGetCrossingRegionsMaxBucketSize(_OperationVolume, out float regionBucketSize))
                quantisation = Mathf.Max(quantisation, regionBucketSize);

            Bounds expanded = _OperationVolume;

            //Expand takes a TOTAL size increase, so this adds the margin on each side
            expanded.Expand(2f * (m_MaxAgentRadius + quantisation));

            return expanded;
        }

        //Union-vs-parts, decided by what the subscribers will actually walk rather than by a tuned threshold. Both
        //handlers sweep a spatial grid over each volume they are given, so overlapping parts make them walk the shared
        //buckets twice: the honest comparison is the union's volume against the sum of the parts' volumes, and the
        //cheaper one wins. Consequences worth stating, since they are the whole behaviour of this rule:
        //  * one part          -> union == that part, collapsed (identical to the pre-2026-08-12 single-box notify);
        //  * parts overlapping -> the union is no bigger than what we would walk anyway, so one query is strictly
        //                         better and the extra volume it covers is bounded by the overlap;
        //  * parts far apart   -> the union exceeds the sum (it spans the gap), so the parts are sent — which is
        //                         precisely the defect this exists to prevent.
        //No slack factor is applied: ties go to the union (one query, and it is the older behaviour), and anything
        //that inflates the notified volume materially fails the test by construction.
        static void CollapseToUnionIfCheaper(List<Bounds> _Volumes)
        {
            if (_Volumes.Count < 2)
                return;

            Bounds union       = _Volumes[0];
            float  partsVolume = 0f;

            foreach (Bounds volume in _Volumes)
            {
                union.Encapsulate(volume);

                Vector3 size = volume.size;

                partsVolume += size.x * size.y * size.z;
            }

            Vector3 unionSize   = union.size;
            float   unionVolume = unionSize.x * unionSize.y * unionSize.z;

            if (unionVolume > partsVolume)
                return;

            _Volumes.Clear();
            _Volumes.Add(union);
        }

        #endregion

    }
}
