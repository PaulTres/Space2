using Nav3D.Common;
using Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.SpatialStorages;
using System;
using System.Collections.Generic;
using System.Threading;
using UnityEngine;
using LeavesSpatialGrid = Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.SpatialStorages.LeavesSpatialGrid;
using NodeSpatialGrid = Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.SpatialStorages.NodeSpatialGrid;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.Elements
{
    abstract partial class Node
    {

        #region Factory

        public static Node Create(
                GlobalOctree      _Octree,
                LeavesSpatialGrid _LeavesSpatialGrid,
                IOccupancyStorage _ObstacleTriangles,
                Bounds[]          _TriangleClusters,
                float             _Size,
                Vector3Int        _Index,
                int               _GridLayer,
                bool              _Crossed,
                Func<int>         _GetID,
                int               _MaxGridLayer,
                bool              _NeedCheckMaxLayer,
                CancellationToken _CancellationToken,
                SemaphoreSlim     _ConcurrencySemaphore,
                NodeSpatialGrid   _CompletedSubtreesCache,
                NodeSpatialGrid   _PartialSubtreesCache
            )
        {
            if (_Crossed && _GridLayer < _MaxGridLayer)
                return new Fork(
                        _Octree,
                        _LeavesSpatialGrid,
                        _ObstacleTriangles,
                        _TriangleClusters,
                        _Size,
                        _Index,
                        _GridLayer,
                        _GetID,
                        _MaxGridLayer,
                        _NeedCheckMaxLayer,
                        _CancellationToken,
                        _ConcurrencySemaphore,
                        _CompletedSubtreesCache,
                        _PartialSubtreesCache
                    );

            return new Leaf(_Octree, _LeavesSpatialGrid, _Size, _Index, _GridLayer, ClassifyLeaf(_Crossed, _ObstacleTriangles, _Index, _Size), _GetID);
        }

        public abstract void ComplementAbsentChildren(
                int               _MaxGridLayer,
                Func<int>         _GetID,
                LeavesSpatialGrid _LeavesSpatialGrid,
                IOccupancyStorage _ObstacleTriangles,
                //considered obstacle clusters bounds
                Bounds[]          _TriangleClusters,
                CancellationToken _CancellationToken,
                bool              _NeedCheckMaxLayer,
                SemaphoreSlim     _ConcurrencySemaphore,
                NodeSpatialGrid   _CompletedSubtreesCache,
                NodeSpatialGrid   _PartialSubtreesCache
            );

        //Re-process an already-present child during a parent's complement pass and return the node that
        //should occupy its slot. A Fork recurses into its own children (returns itself); a free Leaf that the
        //construction storage now occupies rebuilds itself as an occupied subtree (a Leaf cannot subdivide in
        //place), otherwise re-registers and returns itself. Lets the parent stay free of node type checks.
        public abstract Node ComplementOrRebuild(
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
            );

        #endregion

        #region Attributes

        protected          GlobalOctree m_Octree;
        protected readonly float        m_Size;
        protected          int          m_GridLayer;
        protected          Vector3Int   m_Index;
        protected          LeafState    m_State;

        //The ID unique within Octree instance
        protected int m_ID;

        Bounds? m_Bounds;

        #endregion

        #region Properties

        public int        GridLayer => m_GridLayer;
        public Vector3Int Index     => m_Index;
        public Bounds     Bounds    => m_Bounds ??= UtilsSpatialGrid.GetBucketBounds(m_Index, m_Size);
        public int        ID        => m_ID;
        public LeafState  State     => m_State;

        //Not traversable: crossed by geometry or enclosed by a body. This is the question navigation asks; the occupancy
        //law asks Crossed.
        public bool Occupied => m_State != LeafState.FREE;
        public bool Crossed  => m_State == LeafState.CROSSED;
        public bool Interior => m_State == LeafState.INTERIOR;

        //The layer-0 (root) cell this node lives in, derived from its own grid index by integer floor-division
        //instead of re-deriving it from the world position. Every grid layer is the root grid halved
        //`GridLayer` times about the same origin, so floor(p / rootSize) == floor(floor(p / layerSize) / 2^layer) —
        //the identity floor-division nesting gives for free. The staging passes ask this question ~150 k times per
        //publish (ComputeRingBackRefRepoints, RepairStagedRegionFreeGraph) and the old route cost a lazy Bounds
        //construction plus three float divisions each.
        public Vector3Int RootIndex => FloorDivide(m_Index, 1 << m_GridLayer);

        #endregion

        #region Constructors

        protected Node(
                GlobalOctree _Octree,
                float        _Size,
                Vector3Int   _Index,
                int          _GridLayer,
                LeafState    _State,
                Func<int>    _GetID
            )
        {
            m_Octree    = _Octree;
            m_Size      = _Size;
            m_Index     = _Index;
            m_GridLayer = _GridLayer;
            m_State     = _State;
            m_ID        = _GetID();
        }

        protected Node(int _ID, Vector3Int _Index, float _Size, int _GridLayer, LeafState _State)
        {
            m_ID        = _ID;
            m_Index     = _Index;
            m_Size      = _Size;
            m_GridLayer = _GridLayer;
            m_State     = _State;
        }

        #endregion

        #region Public methods

        public void GetEdgeLeaves(Vector3Int _EdgeDirection, HashSet<Leaf> _Result)
        {
            ForkChildOctIndex[] childrenOctIndices = UtilsSpatialGrid.ChildrenIndicesByDirection[_EdgeDirection];

            GetEdgeLeavesInternal(childrenOctIndices, _Result);
        }

        public abstract bool TryPruneExcessiveLeaves(Bounds[] _BoundsSet, ConcurrentHashSet<Leaf> _ExcessiveLeaves);

        public abstract Leaf GetClosestOrEmbracingLeaf(Vector3 _Point);

        public bool TryPruneTouchedSubtrees(Bounds[] _BoundsSet, out HashSet<Node> _Result)
        {
            HashSet<Node> cutNodes = new HashSet<Node>();

            bool result = TryPruneTouchedSubtreesInternal(_BoundsSet, cutNodes);

            _Result = cutNodes;
            return result;
        }

        public abstract void GetAllContainedFreeLeaves(HashSet<Leaf> _Result);

        public abstract void ChangeGridLayer(int _Delta);

        public virtual void AttachNewIDsToSubtree(Func<int> _GetID)
        {
            m_ID = _GetID();
        }

        public virtual void SetOctreeReference(GlobalOctree _Octree)
        {
            m_Octree = _Octree;
        }

        //Deep-clones this subtree into new node instances carrying the same ID/index/size/layer/occupancy and the
        //same octree reference. Used to build an operation's changes on the clone without mutating the live tree that
        //readers are traversing; the clone is installed in one step at the end (GlobalOctree.PublishStagedRegion).
        //Every original→clone leaf pair is recorded in _OriginalToClone so the caller can remap FreeNeighbors after
        //the whole affected set is cloned. This method itself copies no FreeNeighbors and touches no spatial grid.
        public abstract Node CloneSubtree(Dictionary<Leaf, Leaf> _OriginalToClone);

        //partial clone: copies only the leaves _NeedsOwnClone accepts (plus the fork spine leading to them) and
        //SHARES every other subtree with the caller's tree by returning the existing instance. A subtree with no
        //accepted leaf is returned as `this`, so an untouched region costs one reference and no allocation at all.
        //
        //Used for the staging clone's ring roots, where the copy exists only to keep an operation's writes off the
        //live tree and the operation provably writes to nothing but the ring's boundary layer (measured: 0 interior
        //mutations). The shared instances stay in both trees, which is sound
        //exactly while nothing writes to them: the caller owns that obligation, and the leaves it does write to are
        //the ones it asked to have cloned.
        //Like CloneSubtree, this copies no FreeNeighbors and touches no spatial grid; the caller remaps refs
        //afterwards, and a shared leaf's ref that aimed at a now-cloned leaf is the caller's to repoint.
        //_NeedsOwnClone is asked of every node, not only of leaves: a fork whose volume touches nothing this
        //operation writes to returns itself, so the descent skips its whole subtree instead of testing every leaf
        //under it. Sound because a leaf lies inside its fork — if a leaf must be cloned, its fork's volume qualifies
        //too. Measured when the test was leaf-only: the staging clone cost 105 ms against 40 for a full copy.
        public abstract Node CloneSubtreeScoped(Func<Node, bool> _NeedsOwnClone, Dictionary<Leaf, Leaf> _OriginalToClone);

        #endregion

        #region Service methods

        //Component-wise floor division — Vector3Int's own operator/ truncates toward zero, which is wrong for
        //negative indices (the grid extends into negative world space, so -1 / 2 must be -1, not 0).
        static Vector3Int FloorDivide(Vector3Int _Index, int _Divisor)
        {
            return new Vector3Int(
                    FloorDivide(_Index.x, _Divisor),
                    FloorDivide(_Index.y, _Divisor),
                    FloorDivide(_Index.z, _Divisor)
                );
        }

        static int FloorDivide(int _Value, int _Divisor)
        {
            int quotient = _Value / _Divisor;

            return _Value % _Divisor != 0 && (_Value < 0) != (_Divisor < 0) ? quotient - 1 : quotient;
        }

        protected bool IsOccupy(IOccupancyStorage _TriangleStorage, Bounds _Bounds)
        {
            return _TriangleStorage.IsIntersect(_Bounds);
        }

        //A free cell crosses no triangle, so it lies wholly on one side of every closed surface and its centre decides
        //for the whole cell; a sealed volume decides by holding the whole cell.
        protected static LeafState ClassifyLeaf(bool _Crossed, IOccupancyStorage _TriangleStorage, Vector3Int _Index, float _Size)
        {
            if (_Crossed)
                return LeafState.CROSSED;

            return _TriangleStorage.Solids != null && _TriangleStorage.Solids.IsInteriorCell(UtilsSpatialGrid.GetBucketBounds(_Index, _Size))
                ? LeafState.INTERIOR
                : LeafState.FREE;
        }

        protected internal abstract bool TryPruneTouchedSubtreesInternal(Bounds[] _BoundsSet, HashSet<Node> _Result);

        protected internal abstract void GetEdgeLeavesInternal(ForkChildOctIndex[] _EdgeIndices, HashSet<Leaf> _Result);

        #endregion

    }
}
