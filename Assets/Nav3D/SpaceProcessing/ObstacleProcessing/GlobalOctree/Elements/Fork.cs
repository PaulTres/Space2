using Nav3D.Common;
using Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.SpatialStorages;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;
using LeavesSpatialGrid = Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.SpatialStorages.LeavesSpatialGrid;
using NodeSpatialGrid = Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.SpatialStorages.NodeSpatialGrid;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.Elements
{
    partial class Fork : Node
    {

        #region Attributes

        //Eight slots, addressed by ForkChildOctIndex's bit position — see ForkChildren for what this replaced and
        //why (a per-fork Dictionary that both weighed and hashed on every octree descent).
        readonly Node[] m_Children = new Node[ForkChildren.SLOTS_COUNT];

        #endregion

        #region Properties

        public ForkChildren ChildrenMap => new ForkChildren(m_Children);

        #endregion

        #region Constructors

        public Fork(
                GlobalOctree      _Octree,
                LeavesSpatialGrid _LeavesSpatialGrid,
                IOccupancyStorage _ObstacleTriangles,
                Bounds[]          _TriangleClusters,
                float             _Size,
                Vector3Int        _Index,
                int               _GridLayer,
                Func<int>         _GetID,
                int               _MaxGridLayer,
                bool              _NeedCheckMaxLayer,
                CancellationToken _CancellationToken,
                SemaphoreSlim     _ConcurrencySemaphore,
                NodeSpatialGrid   _CompletedSubtreesCache,
                NodeSpatialGrid   _PartialSubtreesCache
            ) : base(
                _Octree,
                _Size,
                _Index,
                _GridLayer,
                LeafState.CROSSED,
                _GetID
            )
        {
            if (_CancellationToken.IsCancellationRequested)
                return;

            ProduceChildren(
                    _MaxGridLayer,
                    _GetID,
                    _LeavesSpatialGrid,
                    _ObstacleTriangles,
                    _TriangleClusters,
                    _CancellationToken,
                    _NeedCheckMaxLayer,
                    _ConcurrencySemaphore,
                    _CompletedSubtreesCache,
                    _PartialSubtreesCache
                );
        }

        public Fork(int _ID, Vector3Int _Index, float _Size, int _GridLayer, LeafState _State)
            : base(_ID, _Index, _Size, _GridLayer, _State)
        {
        }

        #endregion

        #region Public methods

        public override bool TryPruneExcessiveLeaves(Bounds[] _BoundsSet, ConcurrentHashSet<Leaf> _ExcessiveLeaves)
        {
            //if fork is entirely inside any _BoundsSet then return
            if (_BoundsSet.Any(_Bounds => _Bounds.Embrace(Bounds)))
                return false;

            //removal happens in place: slots are indices, so clearing one cannot invalidate the walk (the
            //collect-then-remove list this used to build existed only because a Dictionary forbids it)
            for (int slot = 0; slot < ForkChildren.SLOTS_COUNT; slot++)
            {
                Node child = m_Children[slot];

                //try to remove only leaves in children
                if (child != null && child.TryPruneExcessiveLeaves(_BoundsSet, _ExcessiveLeaves))
                    m_Children[slot] = null;
            }

            //fork never removes
            return false;
        }

        public override Leaf GetClosestOrEmbracingLeaf(Vector3 _Point)
        {
            Vector3 center = Bounds.center;

            //the octant is a 3-bit number (x, y, z against the centre), which is the slot — the eight-way branch
            //forest this replaced computed the same thing and then hashed it
            int slot = (_Point.x >= center.x ? 4 : 0) | (_Point.y >= center.y ? 2 : 0) | (_Point.z >= center.z ? 1 : 0);

            Node child = m_Children[slot];

            return child?.GetClosestOrEmbracingLeaf(_Point);
        }

        public override void GetAllContainedFreeLeaves(HashSet<Leaf> _Result)
        {
            foreach (Node child in ChildrenMap.Values)
            {
                child.GetAllContainedFreeLeaves(_Result);
            }
        }

        public override void ChangeGridLayer(int _Delta)
        {
            m_GridLayer += _Delta;

            foreach (Node child in ChildrenMap.Values)
            {
                child.ChangeGridLayer(_Delta);
            }
        }

        public override void AttachNewIDsToSubtree(Func<int> _GetID)
        {
            base.AttachNewIDsToSubtree(_GetID);

            foreach (Node child in ChildrenMap.Values)
            {
                child.AttachNewIDsToSubtree(_GetID);
            }
        }

        public void SetChild(ForkChildOctIndex _ChildOctIndex, Node _Child)
        {
            m_Children[ForkChildren.SlotOf(_ChildOctIndex)] = _Child;
        }

        public override void SetOctreeReference(GlobalOctree _Octree)
        {
            base.SetOctreeReference(_Octree);

            foreach (Node child in ChildrenMap.Values)
            {
                child.SetOctreeReference(_Octree);
            }
        }

        public override Node CloneSubtree(Dictionary<Leaf, Leaf> _OriginalToClone)
        {
            Fork clone = new Fork(m_ID, m_Index, m_Size, m_GridLayer, m_State);
            clone.m_Octree = m_Octree;

            for (int slot = 0; slot < ForkChildren.SLOTS_COUNT; slot++)
                clone.m_Children[slot] = m_Children[slot]?.CloneSubtree(_OriginalToClone);

            return clone;
        }

        //A fork is copied only when something below it was: the copy exists to hold the new child reference, so a
        //branch that produced no clone is returned as `this` and its whole subtree stays shared. The result is a
        //spine down to the accepted leaves and nothing else — the two-pass shape (ask the children first, allocate
        //only if one of them changed) is what keeps an untouched branch allocation-free.
        public override Node CloneSubtreeScoped(Func<Node, bool> _NeedsOwnClone, Dictionary<Leaf, Leaf> _OriginalToClone)
        {
            //nothing under here is written to by this operation — share the whole subtree and stop descending
            if (!_NeedsOwnClone(this))
                return this;

            //slots make the two-pass shape allocation-free: the cloned children are staged positionally on the
            //stack instead of in a temporary dictionary, and a branch that produced nothing new still returns `this`
            Node[] clonedChildren = null;

            for (int slot = 0; slot < ForkChildren.SLOTS_COUNT; slot++)
            {
                Node child = m_Children[slot];

                if (child == null)
                    continue;

                Node clonedChild = child.CloneSubtreeScoped(_NeedsOwnClone, _OriginalToClone);

                if (ReferenceEquals(clonedChild, child))
                    continue;

                clonedChildren       ??= new Node[ForkChildren.SLOTS_COUNT];
                clonedChildren[slot] =   clonedChild;
            }

            if (clonedChildren == null)
                return this;

            Fork clone = new Fork(m_ID, m_Index, m_Size, m_GridLayer, m_State);
            clone.m_Octree = m_Octree;

            for (int slot = 0; slot < ForkChildren.SLOTS_COUNT; slot++)
                clone.m_Children[slot] = clonedChildren[slot] ?? m_Children[slot];

            return clone;
        }

        #endregion

        #region Service methods

        public override void ComplementAbsentChildren(
                int               _MaxGridLayer,
                Func<int>         _GetID,
                LeavesSpatialGrid _LeavesSpatialGrid,
                IOccupancyStorage _ObstacleTriangles,
                Bounds[]          _TriangleClusters,
                CancellationToken _CancellationToken,
                bool              _NeedCheckMaxLayer,
                SemaphoreSlim     _ConcurrencySemaphore,
                NodeSpatialGrid   _CompletedSubtreesCache,
                NodeSpatialGrid   _PartialSubtreesCache
            )
        {
            float childrenSize = m_Octree.GetBucketSizeOnLayer(m_GridLayer + 1);

            foreach ((ForkChildOctIndex OctIndex, Vector3Int GridIndex) childIndexData in UtilsSpatialGrid.GetChildrenIndices(m_Index))
            {
                //if child presented in dictionary then execute complement method for it
                Node node = m_Children[ForkChildren.SlotOf(childIndexData.OctIndex)];

                if (node != null)
                {
                    //recurse a Fork, or re-evaluate/rebuild a stale free leaf that the storage now occupies —
                    //dispatched polymorphically (ComplementOrRebuild) so this loop needs no node type checks.
                    //A reattached or rebuilt subtree leaf may duplicate a leaf already canonical in the grid
                    //(e.g. a retained neighbor seeded before reconstruction); adopt the grid's instance so the
                    //tree never keeps an orphaned duplicate.
                    Node updatedChild = node.ComplementOrRebuild(
                            _MaxGridLayer,
                            _GetID,
                            _LeavesSpatialGrid,
                            _ObstacleTriangles,
                            _TriangleClusters,
                            _CancellationToken,
                            _NeedCheckMaxLayer,
                            _ConcurrencySemaphore,
                            _CompletedSubtreesCache,
                            _PartialSubtreesCache
                        );

                    m_Children[ForkChildren.SlotOf(childIndexData.OctIndex)] = AdoptCanonicalLeaf(updatedChild, _LeavesSpatialGrid);
                }
                //if child is not presented in dictionary, but should be, then execute subtree construction method
                else
                {
                    Bounds childBounds = UtilsSpatialGrid.GetBucketBounds(childIndexData.GridIndex, childrenSize);

                    //check if bucket is out of obstacle bounds — the octant-skip test (IsInsideAnyCluster), never a
                    //local rewrite of it: the complement lane is routinely reached by move operations now, and a
                    //third definition without the epsilon shrink counted cells that merely touch a cluster bound —
                    //exactly the drift the single shared definition exists to prevent.
                    if (!IsInsideAnyCluster(childBounds, _TriangleClusters))
                        continue;

                    int  maxLayer          = _MaxGridLayer;
                    bool needCheckMaxLayer = _NeedCheckMaxLayer && m_Octree.CheckMaxLayer(childBounds, maxLayer, out maxLayer);

                    Node createdChild = Create(
                            m_Octree,
                            _LeavesSpatialGrid,
                            _ObstacleTriangles,
                            _TriangleClusters,
                            childrenSize,
                            childIndexData.GridIndex,
                            m_GridLayer + 1,
                            IsOccupy(_ObstacleTriangles, childBounds),
                            _GetID,
                            maxLayer,
                            needCheckMaxLayer,
                            _CancellationToken,
                            _ConcurrencySemaphore,
                            _CompletedSubtreesCache,
                            _PartialSubtreesCache
                        );

                    //a freshly built free leaf may collide with a leaf already canonical in the grid
                    //(its constructor reuses, not replaces); adopt the grid's instance into the tree.
                    m_Children[ForkChildren.SlotOf(childIndexData.OctIndex)] = AdoptCanonicalLeaf(createdChild, _LeavesSpatialGrid);
                }
            }
        }

        //A Fork re-processes itself by filling in its absent children; it stays in place.
        public override Node ComplementOrRebuild(
                int               _MaxGridLayer,
                Func<int>         _GetID,
                LeavesSpatialGrid _LeavesSpatialGrid,
                IOccupancyStorage _ObstacleTriangles,
                Bounds[]          _TriangleClusters,
                CancellationToken _CancellationToken,
                bool              _NeedCheckMaxLayer,
                SemaphoreSlim     _ConcurrencySemaphore,
                NodeSpatialGrid   _CompletedSubtreesCache,
                NodeSpatialGrid   _PartialSubtreesCache
            )
        {
            ComplementAbsentChildren(
                    _MaxGridLayer,
                    _GetID,
                    _LeavesSpatialGrid,
                    _ObstacleTriangles,
                    _TriangleClusters,
                    _CancellationToken,
                    _NeedCheckMaxLayer,
                    _ConcurrencySemaphore,
                    _CompletedSubtreesCache,
                    _PartialSubtreesCache
                );

            return this;
        }

        //Returns the canonical leaf the grid holds for _Child's cell, reusing it instead of a duplicate
        //object. Non-leaf or occupied children, and children that already are the grid's instance, are
        //returned unchanged. Keeps the node tree and LeavesSpatialGrid in sync for every free cell.
        //the octant-SKIP test — one definition, because two of them is how the narrowed rebuild first went wrong: a
        //cell exists only where some cluster REACHES it, and the cluster bounds are shrunk by an epsilon first so a
        //cell that merely touches one does not count (touching is what a neighbouring node's bounds do, and counting
        //it produces excessive leaves). Written as a loop rather than LINQ because it runs eight times per fork and
        //the closure + delegate + boxed enumerator LINQ needs for it were measured at >= 442 B per fork.
        //internal (not private): RootBuilder.ConstructSubtrees applies the identical test to a cell it rebuilds.
        internal static bool IsInsideAnyCluster(Bounds _Bounds, Bounds[] _ClusterBounds)
        {
            const float EPSILON = 0.00001f;

            foreach (Bounds clusterBounds in _ClusterBounds)
            {
                if (clusterBounds.Enlarge(-EPSILON).Intersects(_Bounds))
                    return true;
            }

            return false;
        }

        //internal (not private): RootBuilder.ConstructSubtrees grafts a freshly built cell into a parent fork and
        //owes the same canonical-leaf rule every child graft here applies — the tree must never point at a second
        //instance for a cell the construction grid already has a leaf for.
        internal static Node AdoptCanonicalLeaf(Node _Child, LeavesSpatialGrid _LeavesSpatialGrid)
        {
            if (_Child is Leaf { Occupied: false } leaf                                           &&
                _LeavesSpatialGrid.TryGetLeaf(leaf.GridLayer, leaf.Index, out Leaf canonicalLeaf) &&
                !ReferenceEquals(canonicalLeaf, leaf))
            {
                return canonicalLeaf;
            }

            return _Child;
        }

        Node ResolveOrCreateChild(
                Vector3Int        _ChildGridIndex,
                Bounds            _ChildBounds,
                float             _ChildrenSize,
                int               _MaxLayer,
                bool              _NeedCheckMaxLayer,
                Func<int>         _GetID,
                LeavesSpatialGrid _LeavesSpatialGrid,
                IOccupancyStorage _ObstacleTriangles,
                Bounds[]          _TriangleClusters,
                CancellationToken _CancellationToken,
                SemaphoreSlim     _ConcurrencySemaphore,
                NodeSpatialGrid   _CompletedSubtreesCache,
                NodeSpatialGrid   _PartialSubtreesCache
            )
        {
            //try get completed subtree from cache
            if (_CompletedSubtreesCache != null && _CompletedSubtreesCache.TryGetNode(m_GridLayer + 1, _ChildGridIndex, out Node cachedNode))
            {
                cachedNode.AttachNewIDsToSubtree(_GetID);
                return cachedNode;
            }

            //try get partial subtree from cache
            if (_PartialSubtreesCache != null && _PartialSubtreesCache.TryGetNode(m_GridLayer + 1, _ChildGridIndex, out Node cachedPartialRoot))
            {
                cachedPartialRoot.AttachNewIDsToSubtree(_GetID);

                cachedPartialRoot.ComplementAbsentChildren(
                        _MaxLayer,
                        _GetID,
                        _LeavesSpatialGrid,
                        _ObstacleTriangles,
                        _TriangleClusters,
                        _CancellationToken,
                        _NeedCheckMaxLayer,
                        _ConcurrencySemaphore,
                        _CompletedSubtreesCache,
                        _PartialSubtreesCache
                    );

                return cachedPartialRoot;
            }

            //generate subtree
            return Create(
                    m_Octree,
                    _LeavesSpatialGrid,
                    _ObstacleTriangles,
                    _TriangleClusters,
                    _ChildrenSize,
                    _ChildGridIndex,
                    m_GridLayer + 1,
                    IsOccupy(_ObstacleTriangles, _ChildBounds),
                    _GetID,
                    _MaxLayer,
                    _NeedCheckMaxLayer,
                    _CancellationToken,
                    _ConcurrencySemaphore,
                    _CompletedSubtreesCache,
                    _PartialSubtreesCache
                );
        }

        void ProduceChildren(
                int               _MaxGridLayer,
                Func<int>         _GetID,
                LeavesSpatialGrid _LeavesSpatialGrid,
                IOccupancyStorage _ObstacleTriangles,
                Bounds[]          _TriangleClusters,
                CancellationToken _CancellationToken,
                bool              _NeedCheckMaxLayer,
                SemaphoreSlim     _ConcurrencySemaphore,
                NodeSpatialGrid   _CompletedSubtreesCache,
                NodeSpatialGrid   _PartialSubtreesCache
            )
        {
            //allocation, not style: this method runs once per fork, and forks are an eighth of every node an
            //operation builds. Weighed per fork (ARM 0 of the allocation rig): the
            //ConcurrentDictionary this used to stage children in cost **928 B** — more than the 762 B children map
            //it exists to fill — the always-allocated task list 47 B, and the LINQ octant test below a display
            //class + a delegate + a boxed enumerator per child, >= 442 B a fork. Together >= 1.4 KB of garbage per
            //fork against 762 B kept, and a collection pause is the frame cost of an obstacle operation
            //().
            //
            //What replaces them changes nothing about the order of anything: children are staged POSITIONALLY in a
            //plain array over GetChildrenIndices' fixed 8 entries — parallel tasks write distinct slots, which is
            //all the concurrent map ever provided here — and the merge into the fork's own slots stays sequential, after
            //the join, exactly as before. AdoptCanonicalLeaf stays inside the task, where it reads the concurrent
            //grid at the same moment it always did.
            UtilsSpatialGrid.ChildrenIndices childrenIndices = UtilsSpatialGrid.GetChildrenIndices(m_Index);

            Node[] parallelChildren = null;

            List<Task> taskSet = null;

            float childrenSize = m_Octree.GetBucketSizeOnLayer(m_GridLayer + 1);

            const int MIN_REMAINING_LEVELS_FOR_PARALLEL = 2;

            bool allowParallel = _MaxGridLayer - m_GridLayer > MIN_REMAINING_LEVELS_FOR_PARALLEL;

            //index-based: a child's slot in the staging array must match its entry in childrenIndices
            for (int childNum = 0; childNum < childrenIndices.Length; childNum++)
            {
                (ForkChildOctIndex OctIndex, Vector3Int GridIndex) childIndexData = childrenIndices[childNum];

                Bounds childBounds = UtilsSpatialGrid.GetBucketBounds(childIndexData.GridIndex, childrenSize);

                //check if bucket is out of obstacle bounds
                //(It is needs to reduce cluster bounds size because instead of cluster bounds there may be neighboring node bounds and
                //intersection comparison can return true which is lead to excessive leaves creation)
                if (!IsInsideAnyCluster(childBounds, _TriangleClusters))
                    continue;

                int  maxLayer          = _MaxGridLayer;
                bool needCheckMaxLayer = _NeedCheckMaxLayer && m_Octree.CheckMaxLayer(childBounds, maxLayer, out maxLayer);

                if (allowParallel && _ConcurrencySemaphore.Wait(0))
                {
                    //allocated on the first spawned task: a fork within two layers of the bottom never spawns any
                    //(allowParallel is false there), and those are the numerous ones
                    parallelChildren ??= new Node[childrenIndices.Length];
                    taskSet          ??= new List<Task>(childrenIndices.Length);

                    int childSlot = childNum;

                    taskSet.Add(
                            Task.Run(
                                    () =>
                                    {
                                        try
                                        {
                                            GlobalOctree.DevTaskCpuStart();

                                            Node child = ResolveOrCreateChild(
                                                    childIndexData.GridIndex,
                                                    childBounds,
                                                    childrenSize,
                                                    maxLayer,
                                                    needCheckMaxLayer,
                                                    _GetID,
                                                    _LeavesSpatialGrid,
                                                    _ObstacleTriangles,
                                                    _TriangleClusters,
                                                    _CancellationToken,
                                                    _ConcurrencySemaphore,
                                                    _CompletedSubtreesCache,
                                                    _PartialSubtreesCache
                                                );

                                            parallelChildren[childSlot] = AdoptCanonicalLeaf(child, _LeavesSpatialGrid);
                                        }
                                        finally
                                        {
                                            GlobalOctree.DevTaskCpuEnd();
                                            _ConcurrencySemaphore.Release();
                                        }
                                    },
                                    _CancellationToken
                                )
                        );
                }
                else
                {
                    Node child = ResolveOrCreateChild(
                            childIndexData.GridIndex,
                            childBounds,
                            childrenSize,
                            maxLayer,
                            needCheckMaxLayer,
                            _GetID,
                            _LeavesSpatialGrid,
                            _ObstacleTriangles,
                            _TriangleClusters,
                            _CancellationToken,
                            _ConcurrencySemaphore,
                            _CompletedSubtreesCache,
                            _PartialSubtreesCache
                        );

                    m_Children[ForkChildren.SlotOf(childIndexData.OctIndex)] = AdoptCanonicalLeaf(child, _LeavesSpatialGrid);
                }
            }

            if (taskSet != null && taskSet.Count > 0)
            {
                Task.WaitAll(taskSet.ToArray(), _CancellationToken);

                for (int childNum = 0; childNum < parallelChildren.Length; childNum++)
                {
                    if (parallelChildren[childNum] != null)
                        m_Children[ForkChildren.SlotOf(childrenIndices[childNum].OctIndex)] = parallelChildren[childNum];
                }
            }
        }

        protected internal override void GetEdgeLeavesInternal(ForkChildOctIndex[] _EdgeIndices, HashSet<Leaf> _Result)
        {
            foreach (ForkChildOctIndex edgeIndex in _EdgeIndices)
            {
                Node child = m_Children[ForkChildren.SlotOf(edgeIndex)];

                if (child != null)
                {
                    child.GetEdgeLeavesInternal(_EdgeIndices, _Result);
                }
            }
        }

        protected internal override bool TryPruneTouchedSubtreesInternal(Bounds[] _BoundsSet, HashSet<Node> _Result)
        {
            if (ChildrenMap.AnyLeafChild())
            {
                _Result.Add(this);
                ChildrenMap.Clear();
                return true;
            }

            bool anyChildIsPruned = false;

            for (int slot = 0; slot < ForkChildren.SLOTS_COUNT; slot++)
            {
                Node child = m_Children[slot];

                if (child == null)
                    continue;

                if (_BoundsSet.All(_Bounds => !Bounds.Intersects(_Bounds)))
                    continue;

                if (child.TryPruneTouchedSubtreesInternal(_BoundsSet, _Result))
                {
                    m_Children[slot] = null;
                    anyChildIsPruned = true;
                }
            }

            return anyChildIsPruned;
        }

        #endregion

    }
}
