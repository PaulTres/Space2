using Nav3D.Common;
using Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.SpatialStorages;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using UnityEngine;
using LeavesSpatialGrid = Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.SpatialStorages.LeavesSpatialGrid;
using NodeSpatialGrid = Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.SpatialStorages.NodeSpatialGrid;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.Elements
{
    partial class Leaf : Node
    {

        #region Attributes

        //the set is lazy, and the lock object is gone. Both were allocated eagerly by every leaf, and a leaf is 87 %
        //of the nodes an operation builds — of which the OCCUPIED ones, 39 % of the tree, carry no free-neighbour
        //refs at all and never touched either. Weighed (ARM 0 of the allocation rig): a
        //bare leaf 223 B, of which the eager set and lock are most; a leaf with its six neighbours 451 B. The lock
        //is now the set itself once it exists, and its creation is published by compare-exchange (it
        //used to be guarded by a per-leaf lock object, 24 B on every leaf of the tree) — no object at all until the
        //graph actually wires this leaf.
        //volatile: the lazy init (EnsureNeighbors) and every lock-free read of the reference need the created set
        //PUBLISHED — without it a thread could observe the reference before the set's own fields.

        volatile HashSet<Leaf> m_FreeNeighbors;

        //the set'S first size: six face neighbours is the common case (equal-sized cells), and the default HashSet
        //starts at 3 slots and regrows through 7 — one throw-away bucket/slot pair per wired leaf per operation.
        //Sized for the common case it allocates once; the size-graded leaves with more neighbours still grow.
        const int FREE_NEIGHBORS_INITIAL_CAPACITY = 6;

        //Navigation-graph classification, used by pathfinding (a NavNode property kept on the free Leaf while the nav
        //graph is the octree's free leaves). Bit d is set iff face direction d (indexed as UtilsSpatialGrid.FacesDirections)
        //opens to open air — i.e. the cell across that face holds no leaf at all (its root is absent). Computed at
        //construction/publish time (GlobalOctree.ClassifyOpenFaces) and read by the pathfinder's free-space-edge gate;
        //never computed at query time. At most 3 bits can be set (a leaf touches at most 3 of its root's faces).
        byte m_OpenFaces;

        #endregion

        #region Properties

        public Leaf[] FreeNeighbors
        {
            get
            {
                HashSet<Leaf> neighbors = m_FreeNeighbors;

                if (neighbors == null)
                    return Array.Empty<Leaf>();

                lock (neighbors)
                {
                    return neighbors.ToArray();
                }
            }
        }

        //the pathfinder'S view of the same set, without the copy. FreeNeighbors above snapshots into a fresh array on
        //every read, which is one allocation per expansion — and an expansion is the hot loop of both search lanes
        //(a leaf has 13–90 neighbours, so the array is not small either).
        //
        //the contract, and it is what makes the copy unnecessary: the caller must hold the tree read lock. Under it
        //no writer touches a live leaf's neighbour set — construction writes only to the transaction's own clones,
        //and the publish applies every edit to live leaves inside the write lock
        // — so the set cannot be mutated while it is being walked.
        //Returns null when this leaf was never wired (the set is created on first use); `foreach` over the returned
        //HashSet uses its struct enumerator, so the walk allocates nothing either.
        //Any caller that MUTATES the graph while walking it (the complement lane severing refs) must keep using
        //FreeNeighbors and its copy.
        public HashSet<Leaf> FreeNeighborsLocked => m_FreeNeighbors;

        #endregion

        #region Constructors

        public Leaf(
                GlobalOctree      _Octree,
                LeavesSpatialGrid _LeavesSpatialGrid,
                float             _Size,
                Vector3Int        _Index,
                int               _GridLayer,
                LeafState         _State,
                Func<int>         _GetID
            ) : base(
                _Octree,
                _Size,
                _Index,
                _GridLayer,
                _State,
                _GetID
            )
        {
            //only a FREE leaf joins the construction grid, which is what wires the graph: a crossed or interior leaf
            //gets no FreeNeighbors and nothing refers to it
            if (!Occupied)
                _LeavesSpatialGrid.SetLeaf(this);
        }

        public Leaf(int _ID, Vector3Int _Index, float _Size, int _GridLayer, LeafState _State)
            : base(_ID, _Index, _Size, _GridLayer, _State)
        {
        }

        #endregion

        #region Public methods

        public override bool TryPruneExcessiveLeaves(Bounds[] _BoundsSet, ConcurrentHashSet<Leaf> _ExcessiveLeaves)
        {
            //leaf is outside any bounds
            if (!_BoundsSet.Any(_Bounds => _Bounds.Intersects(Bounds)))
            {
                _ExcessiveLeaves.TryAdd(this);

                return true;
            }

            return false;
        }

        public override Leaf GetClosestOrEmbracingLeaf(Vector3 _Point)
        {
            return this;
        }

        public override void GetAllContainedFreeLeaves(HashSet<Leaf> _Result)
        {
            if (Occupied)
                return;

            _Result.Add(this);
        }

        public override void ChangeGridLayer(int _Delta)
        {
            m_GridLayer += _Delta;
        }

        public void AddFreeNeighbor(Leaf _Adjacent)
        {
            DevNoteNeighborEdit("add");

            lock (EnsureNeighbors())
            {
                m_FreeNeighbors.Add(_Adjacent);
            }
        }

        //dev sentinel (implemented in the [Nav3DDevOnly] Leaf.DevValidation.cs partial; the calls compile out with
        //it): reports — with a stack trace — any free-neighbour write landing on a leaf that belongs to the pre-baked
        //cache, which nothing but the bake may own.
        partial void DevNoteNeighborEdit(string _Kind);

        //Creates the set on first use. Published with a compare-exchange rather than under a per-leaf lock object:
        //two construction threads can wire the same leaf concurrently (FormLeavesConnections and the connection
        //passes both run parallel), and the loser of the race discards its set and adopts the winner's — one set
        //per leaf, no second object per leaf to guard it (the guard object was 24 B on every leaf of the tree).
        HashSet<Leaf> EnsureNeighbors()
        {
            HashSet<Leaf> neighbors = m_FreeNeighbors;

            if (neighbors != null)
                return neighbors;

            HashSet<Leaf> created = new HashSet<Leaf>(FREE_NEIGHBORS_INITIAL_CAPACITY);

            #pragma warning disable 420

            return Interlocked.CompareExchange(ref m_FreeNeighbors, created, null) ?? created;

            #pragma warning restore 420
        }

        public Vector3 GetAdjacentContactPoint(Leaf _OtherLeaf)
        {
            return GetAdjacentContactPointInternal(_OtherLeaf);
        }

        public void RemoveFreeNeighbor(Leaf _OtherLeaf)
        {
            DevNoteNeighborEdit("remove");

            HashSet<Leaf> neighbors = m_FreeNeighbors;

            if (neighbors == null)
                return;

            lock (neighbors)
            {
                neighbors.Remove(_OtherLeaf);
            }
        }

        //A read with the same locking discipline as the mutations — used by StagingTransaction.EditFreeNeighbor to
        //drop no-op edits on SHARED leaves instead of queueing them for the publish's write lock. Sound unlocked
        //because operations are serialized: the only writer of this set during an operation is the operation itself.
        public bool ContainsFreeNeighbor(Leaf _Leaf)
        {
            HashSet<Leaf> neighbors = m_FreeNeighbors;

            if (neighbors == null)
                return false;

            lock (neighbors)
            {
                return neighbors.Contains(_Leaf);
            }
        }

        public void RemoveAllFreeNeighbors()
        {
            DevNoteNeighborEdit("remove-all");

            HashSet<Leaf> neighbors = m_FreeNeighbors;

            if (neighbors == null)
                return;

            lock (neighbors)
            {
                neighbors.Clear();
            }
        }

        //the dead leaf'S form of RemoveAllFreeNeighbors, for NavTree.TearDownDetachedSubtrees only: drops the set
        //object rather than clearing it. Clear() keeps the set's bucket and slot arrays allocated, so a torn-down
        //leaf that a stale stack slot still pins (the P28 mechanism) kept costing its set's ~200-300 B on top of its
        //own 144; nulled, the pin retains the bare leaf and nothing else. Only sound on a leaf no reader can reach —
        //a live leaf's readers walk FreeNeighborsLocked under the read lock and tolerate null, but they must never
        //see a set vanish mid-walk, which the teardown's post-lock, unreachable-only contract guarantees.
        public void DetachAllFreeNeighbors()
        {
            DevNoteNeighborEdit("detach");

            m_FreeNeighbors = null;
        }


        //Snapshot of the neighbour set into a caller-owned buffer (cleared first), under the same lock the
        //mutations take — the allocation-free form of FreeNeighbors for the op-thread passes that walk a set while
        //editing it (the free-graph repair, the prune, the clone wiring, the ring back-ref search each took a fresh
        //array per leaf, i.e. one allocation per leaf per pass per operation). Semantics are those of the array:
        //the buffer is a copy, so editing the set while walking the buffer is safe.
        public void CopyFreeNeighborsTo(List<Leaf> _Buffer)
        {
            _Buffer.Clear();

            HashSet<Leaf> neighbors = m_FreeNeighbors;

            if (neighbors == null)
                return;

            lock (neighbors)
            {
                foreach (Leaf neighbor in neighbors)
                    _Buffer.Add(neighbor);
            }
        }


        //Navigation-graph open-face classification (see m_OpenFaces). Written only under the octree write lock during
        //ClassifyOpenFaces, read only under the read lock by the pathfinder — no per-leaf lock needed.
        public bool IsEdgeNode => m_OpenFaces != 0;

        public bool IsFaceOpen(int _FaceBit)
        {
            return (m_OpenFaces & (1 << _FaceBit)) != 0;
        }

        public void SetFaceOpen(int _FaceBit)
        {
            m_OpenFaces |= (byte) (1 << _FaceBit);
        }

        public void ClearOpenFaces()
        {
            m_OpenFaces = 0;
        }

        public override void SetOctreeReference(GlobalOctree _Octree)
        {
            base.SetOctreeReference(_Octree);

            m_Index = m_Octree.GetBucketIndexOnLayer(m_GridLayer, this);
        }

        public override Node CloneSubtree(Dictionary<Leaf, Leaf> _OriginalToClone)
        {
            Leaf clone = new Leaf(m_ID, m_Index, m_Size, m_GridLayer, m_State);
            clone.m_Octree = m_Octree;

            //The open-face classification is part of the leaf's state, so a "faithful replica" has to carry it
            //. It used to be dropped here and rebuilt for the whole clone set by
            //NavTree.ClassifyOpenFacesStaged — which is why that pass had to sample all six faces of every leaf in
            //every cloned root, ring roots included, at ~99 ms per operation. Copying it costs one byte assignment
            //and lets the pass re-classify only the leaves whose adjacency this operation can actually have
            //changed.
            clone.m_OpenFaces = m_OpenFaces;

            //The volumetric clearance annotation travels too (implemented in Leaf.Volumetric.cs; compiles to
            //nothing when that file is absent). Unlike the open-face bits it cannot go stale on a clone: its
            //validity is derived from root rebuild versions at read time, so a copied value is exactly as valid
            //as the original's would have been.
            CopyVolumetricAnnotation(clone);

            //FreeNeighbors are intentionally not copied here — the caller remaps them once the whole affected
            //set is cloned (internal edges → clone neighbour; boundary edges → surviving live leaf). See
            //GlobalOctree.CloneRootsForStaging.
            _OriginalToClone[this] = clone;

            return clone;
        }

        //A leaf is the decision point of the partial clone: either the caller asked for this cell to be its own
        //copy, or the leaf stays SHARED with the live tree and is not recorded in _OriginalToClone at all — which is
        //what makes every later pass treat it as "not a clone" (the publish's back-ref search, the free-graph
        //repair), and that is the correct reading: a shared leaf is not replaced, so nothing pointing at it goes
        //stale.
        public override Node CloneSubtreeScoped(Func<Node, bool> _NeedsOwnClone, Dictionary<Leaf, Leaf> _OriginalToClone)
        {
            return _NeedsOwnClone(this) ? CloneSubtree(_OriginalToClone) : this;
        }

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
            if (!Occupied)
                _LeavesSpatialGrid.SetLeaf(this);
        }

        //A free leaf is never re-evaluated by ComplementAbsentChildren (it only re-registers itself). When the
        //construction storage now occupies this cell — geometry from another cluster sharing the root — and the
        //cell is within the pass's construction bounds, rebuild as an occupied subtree (a leaf cannot subdivide
        //in place) so it cannot survive free inside an obstacle's footprint (FREE_INTERSECTS). The stale leaf is
        //detached from the grid and the neighbor graph first (mirrors the root-leaf replacement in
        //GlobalOctree.ConstructRoots). Any other case keeps the prior behavior: re-register and stay.
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
            //the cluster-reach gate is the shared octant test (Fork.IsInsideAnyCluster, epsilon-shrunk), never a
            //local Intersects rewrite — same reasoning as in Fork.ComplementAbsentChildren. The state is re-derived, not
            //kept: new geometry may cross the cell, or close a body around it, or a body around it may be gone.
            bool      crossed = !Crossed && Fork.IsInsideAnyCluster(Bounds, _TriangleClusters) && IsOccupy(_ObstacleTriangles, Bounds);
            LeafState derived = Crossed ? LeafState.CROSSED : ClassifyLeaf(crossed, _ObstacleTriangles, m_Index, m_Size);

            if (derived != m_State)
            {
                _LeavesSpatialGrid.RemoveLeaf(this);

                foreach (Leaf neighbor in FreeNeighbors)
                    neighbor.RemoveFreeNeighbor(this);

                RemoveAllFreeNeighbors();

                int  maxLayer          = _MaxGridLayer;
                bool needCheckMaxLayer = _NeedCheckMaxLayer && m_Octree.CheckMaxLayer(Bounds, maxLayer, out maxLayer);

                return Create(
                        m_Octree,
                        _LeavesSpatialGrid,
                        _ObstacleTriangles,
                        _TriangleClusters,
                        m_Size,
                        m_Index,
                        m_GridLayer,
                        crossed,
                        _GetID,
                        maxLayer,
                        needCheckMaxLayer,
                        _CancellationToken,
                        _ConcurrencySemaphore,
                        _CompletedSubtreesCache,
                        _PartialSubtreesCache
                    );
            }

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

        #endregion

        #region Service methods

        //Seam to the volumetric extension (Leaf.Volumetric.cs, a Nav3D Pro file): copies the clearance annotation
        //onto the clone. An unimplemented partial void compiles to nothing, so a build without that file loses the
        //call, not the clone.
        partial void CopyVolumetricAnnotation(Leaf _Clone);

        Vector3 GetAdjacentContactPointInternal(Leaf _OtherLeaf)
        {
            Vector3 navigationExtents = Bounds.extents;
            Vector3 center            = Bounds.center;

            if (m_GridLayer == _OtherLeaf.m_GridLayer)
            {
                if (m_Index.x != _OtherLeaf.m_Index.x)
                {
                    if (m_Index.x < _OtherLeaf.m_Index.x)
                        return center + new Vector3(navigationExtents.x, 0, 0);

                    return center + new Vector3(-navigationExtents.x, 0, 0);
                }

                if (m_Index.y != _OtherLeaf.m_Index.y)
                {
                    if (m_Index.y < _OtherLeaf.m_Index.y)
                        return center + new Vector3(0, navigationExtents.y, 0);

                    return center + new Vector3(0, -navigationExtents.y, 0);
                }

                if (m_Index.z != _OtherLeaf.m_Index.z)
                {
                    if (m_Index.z < _OtherLeaf.m_Index.z)
                        return center + new Vector3(0, 0, navigationExtents.z);

                    return center + new Vector3(0, 0, -navigationExtents.z);
                }

                return center;
            }

            Leaf smallerLeaf;
            Leaf biggerLeaf;

            Vector3Int smallerIndex;
            Vector3Int biggerIndex;

            int layersDelta = Mathf.Abs(m_GridLayer - _OtherLeaf.m_GridLayer);

            if (m_GridLayer < _OtherLeaf.m_GridLayer)
            {
                smallerLeaf = _OtherLeaf;
                biggerLeaf  = this;
            }
            else
            {
                smallerLeaf = this;
                biggerLeaf  = _OtherLeaf;
            }

            smallerIndex = smallerLeaf.m_Index;
            biggerIndex  = biggerLeaf.m_Index;

            //equalize layers for leaves
            for (int i = 0; i < layersDelta; i++)
                smallerIndex = UtilsSpatialGrid.GetParentIndex(smallerIndex);

            Vector3 smallerLeafNavigationExtents = smallerLeaf.Bounds.extents;
            Vector3 smallerLeafCenter            = smallerLeaf.Bounds.center;

            if (smallerIndex.x != biggerIndex.x)
            {
                if (smallerIndex.x < biggerIndex.x)
                    return smallerLeafCenter + new Vector3(smallerLeafNavigationExtents.x, 0, 0);

                return smallerLeafCenter + new Vector3(-smallerLeafNavigationExtents.x, 0, 0);
            }

            if (smallerIndex.y != biggerIndex.y)
            {
                if (smallerIndex.y < biggerIndex.y)
                    return smallerLeafCenter + new Vector3(0, smallerLeafNavigationExtents.y, 0);

                return smallerLeafCenter + new Vector3(0, -smallerLeafNavigationExtents.y, 0);
            }

            if (smallerIndex.z != biggerIndex.z)
            {
                if (smallerIndex.z < biggerIndex.z)
                    return smallerLeafCenter + new Vector3(0, 0, smallerLeafNavigationExtents.z);

                return smallerLeafCenter + new Vector3(0, 0, -smallerLeafNavigationExtents.z);
            }

            Debug.LogError($"{nameof(GetAdjacentContactPointInternal)}: Positive infinity. {smallerLeaf.m_GridLayer} {biggerLeaf.m_GridLayer}, {smallerIndex} {biggerIndex}");

            return Vector3.positiveInfinity;
        }


        protected internal override bool TryPruneTouchedSubtreesInternal(Bounds[] _BoundsSet, HashSet<Node> _Result)
        {
            _Result.Add(this);
            return true;
        }

        protected internal override void GetEdgeLeavesInternal(ForkChildOctIndex[] _EdgeIndices, HashSet<Leaf> _Result)
        {
            if (!Occupied)
                _Result.Add(this);
        }

        #endregion

    }
}
