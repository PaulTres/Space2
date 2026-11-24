using Nav3D.Common;
using Nav3D.SpaceProcessing.ObstacleProcessing.UtilityTypes;
using System.Collections.Generic;
using UnityEngine;
using Node = Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.Elements.Node;
using Leaf = Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.Elements.Leaf;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree
{
    partial class GlobalOctree
    {

        #region Service methods

        //_ConnectInSetNeighbors: when false, neighbours inside the added set are skipped because every operation
        //builds in one shared grid and FormLeavesConnections already linked them. PIC reconstruction passes true:
        //it may reuse cached subtrees whose leaves are outside that grid, so even an in-set face needs stitching.
        //AddFreeNeighbor is idempotent; already connected pairs remain unchanged.
        //_Transaction: the "in-set faces are already wired" argument holds only while both roots of a face were
        //rebuilt whole. A root rebuilt cell by cell keeps its surviving leaves out of the construction grid, so a
        //survivor bordering no rebuilt cell but bordering a root this operation built whole is linked by nothing,
        //and the repair finds no stale ref to re-point because the neighbour root did not exist before. So an
        //in-set face is skipped only when neither root was narrowed; a mixed face is stitched here, with the writes
        //routed through the transaction so a survivor shared with the live tree is edited under the publish's write
        //lock rather than inline.
        //internal, not private: shared by RebuildRoots and by the standalone PreBakedAddWorker.
        internal void FormEdgeConnectionsForAddedRoots(HashSet<Vector3Int> _AddedRootIndices, bool _ConnectInSetNeighbors = false, StagingTransaction _Transaction = null)
        {
            //the pass's four per-face buffers are the operation's scratch (cleared per face) — two leaf sets and
            //two face-cell indices, which used to be four fresh containers per root face plus a List per leaf
            //; a transaction-less caller keeps local ones
            OperationScratch scratch = ActiveScratch;

            HashSet<Leaf> addedEdgeLeaves    = scratch?.EdgeLeavesA ?? new HashSet<Leaf>();
            HashSet<Leaf> neighborEdgeLeaves = scratch?.EdgeLeavesB ?? new HashSet<Leaf>();

            Dictionary<(float Size, Vector2Int Cell), Leaf> addedIndex        = scratch?.FaceIndexA ?? new Dictionary<(float, Vector2Int), Leaf>();
            Dictionary<(float Size, Vector2Int Cell), Leaf> neighborFaceIndex = scratch?.FaceIndexB ?? new Dictionary<(float, Vector2Int), Leaf>();

            Dictionary<(float Size, Vector2Int Cell), List<Leaf>> addedOverflow    = scratch?.FaceOverflowA ?? new Dictionary<(float, Vector2Int), List<Leaf>>();
            Dictionary<(float Size, Vector2Int Cell), List<Leaf>> neighborOverflow = scratch?.FaceOverflowB ?? new Dictionary<(float, Vector2Int), List<Leaf>>();

            foreach (Vector3Int rootIndex in _AddedRootIndices)
            {
                if (!m_ActiveRootsGrid.TryGetNode(0, rootIndex, out Node addedRoot))
                    continue;

                bool rootNarrowed = IsNarrowedRoot(_Transaction, rootIndex);

                foreach (Vector3Int direction in UtilsSpatialGrid.FacesDirections)
                {
                    Vector3Int neighborIndex = rootIndex + direction;

                    //skip in-batch neighbours only when the whole batch shares one grid (pre-baked) — and, for an
                    //operation that narrows, only when both sides of the face were rebuilt whole (see above)
                    if (!_ConnectInSetNeighbors && _AddedRootIndices.Contains(neighborIndex) && rootNarrowed == IsNarrowedRoot(_Transaction, neighborIndex))
                        continue;

                    if (!m_ActiveRootsGrid.TryGetNode(0, neighborIndex, out Node neighborRoot))
                        continue;

                    //Get edge leaves facing each other
                    DevStepStart();

                    addedEdgeLeaves.Clear();
                    neighborEdgeLeaves.Clear();

                    addedRoot.GetEdgeLeaves(direction, addedEdgeLeaves);
                    neighborRoot.GetEdgeLeaves(-direction, neighborEdgeLeaves);

                    DevStepEnd("edge conn: gather edge leaves");

                    //Connect free leaves from both sides that are spatially adjacent — via an index over the shared
                    //face, not by testing every pair. Every leaf on this face touches the boundary plane, so
                    //adjacency reduces to whether the two face RECTANGLES overlap, and octree cells either nest or
                    //are disjoint there: a partner is the leaf occupying this leaf's face cell at its own size or at
                    //any coarser one. AreLeavesAdjacent still decides every candidate, so the result is unchanged by
                    //construction — the index only shortens the candidate list.
                    DevStepStart();

                    int planeAxis = direction.x != 0 ? 0 : direction.y != 0 ? 1 : 2;

                    //Both directions are walked, because a pair is only found from the finer side (a coarser
                    //partner's cell CONTAINS the finer leaf's face cell, never the other way round). An equal-size
                    //pair is therefore found twice, which costs one extra idempotent AddFreeNeighbor.
                    BuildFaceCellIndex(neighborEdgeLeaves, planeAxis, neighborFaceIndex, neighborOverflow);
                    BuildFaceCellIndex(addedEdgeLeaves, planeAxis, addedIndex, addedOverflow);

                    ConnectAcrossFace(addedEdgeLeaves, neighborFaceIndex, neighborOverflow, planeAxis, _Transaction);
                    ConnectAcrossFace(neighborEdgeLeaves, addedIndex, addedOverflow, planeAxis, _Transaction);

                    DevStepEnd("edge conn: pairwise adjacency");

                    //A missing edge here is silent — it shows up much later as a route the pathfinder cannot find —
                    //so the index does not rest on the nesting argument alone: the oracle re-runs the full pairwise
                    //sweep and checks the tree's actual state against it.
                    DevCheckEdgeConnectionEquivalence(addedEdgeLeaves, neighborEdgeLeaves, _Transaction);

                    DevCountWork("edge conn: edge leaves gathered", addedEdgeLeaves.Count + neighborEdgeLeaves.Count);
                    DevCountWork("edge conn: root faces visited", 1);
                }
            }
        }

        static bool IsNarrowedRoot(StagingTransaction _Transaction, Vector3Int _RootIndex)
        {
            return _Transaction != null && _Transaction.RebuiltCellsByRoot.ContainsKey(_RootIndex);
        }

        //One face's leaves, keyed by their own cell size and their cell on the face plane. Keyed by size (never by
        //layer): every leaf size is the root size halved some number of times, and halving/doubling a float is exact,
        //so the doubling walk in ConnectAcrossFace lands on exactly the keys stored here. one leaf per key in the flat
        //map — two same-sized cells cannot share a face cell in an octree — with an overflow map for the second
        //instance a face can still present (the pre-baked lane can hold a duplicate leaf object for a cell), so the
        //flat form finds exactly what the old dictionary-of-dictionaries-of-lists found. Both maps are the
        //scratch's, cleared here; the overflow allocates a list only when a collision actually occurs.
        static void BuildFaceCellIndex(
                HashSet<Leaf>                                         _EdgeLeaves,
                int                                                   _PlaneAxis,
                Dictionary<(float Size, Vector2Int Cell), Leaf>       _Index,
                Dictionary<(float Size, Vector2Int Cell), List<Leaf>> _Overflow
            )
        {
            _Index.Clear();
            _Overflow.Clear();

            foreach (Leaf leaf in _EdgeLeaves)
            {
                float cellSize = leaf.Bounds.size.x;

                (float Size, Vector2Int Cell) key = (cellSize, GetFaceCell(leaf.Bounds.center, cellSize, _PlaneAxis));

                if (!_Index.ContainsKey(key))
                {
                    _Index[key] = leaf;

                    continue;
                }

                if (!_Overflow.TryGetValue(key, out List<Leaf> extra))
                {
                    extra = new List<Leaf>();

                    _Overflow[key] = extra;
                }

                extra.Add(leaf);
            }
        }

        //Each leaf's partners on the opposite side: the same face cell at its own size, then at every coarser size up
        //to the root. Quantisation goes through UtilsSpatialGrid.GetBucketIndex, the same one the tree itself uses,
        //so the cell a leaf lands in here is the cell it occupies everywhere else.
        void ConnectAcrossFace(
                HashSet<Leaf>                                         _EdgeLeaves,
                Dictionary<(float Size, Vector2Int Cell), Leaf>       _OppositeIndex,
                Dictionary<(float Size, Vector2Int Cell), List<Leaf>> _OppositeOverflow,
                int                                                   _PlaneAxis,
                StagingTransaction                                    _Transaction
            )
        {
            foreach (Leaf leaf in _EdgeLeaves)
            {
                for (float candidateSize = leaf.Bounds.size.x; candidateSize <= RootSize; candidateSize *= 2f)
                {
                    (float Size, Vector2Int Cell) key = (candidateSize, GetFaceCell(leaf.Bounds.center, candidateSize, _PlaneAxis));

                    if (!_OppositeIndex.TryGetValue(key, out Leaf candidate))
                        continue;

                    TryConnectAcrossFace(leaf, candidate, _Transaction);

                    if (_OppositeOverflow.Count > 0 && _OppositeOverflow.TryGetValue(key, out List<Leaf> extra))
                    {
                        foreach (Leaf extraCandidate in extra)
                            TryConnectAcrossFace(leaf, extraCandidate, _Transaction);
                    }
                }
            }
        }

        void TryConnectAcrossFace(Leaf _Leaf, Leaf _Candidate, StagingTransaction _Transaction)
        {
            DevCount("edge conn: candidate pairs TESTED", 1);

            if (!AreLeavesAdjacent(_Leaf, _Candidate))
                return;

            //routed by ownership when the operation has a transaction: inline on a clone or a leaf this
            //operation built, deferred to the publish's write lock on a leaf shared with the live tree
            if (_Transaction != null)
            {
                _Transaction.EditFreeNeighbor(_Leaf, _Candidate, true);
                _Transaction.EditFreeNeighbor(_Candidate, _Leaf, true);
            }
            else
            {
                _Leaf.AddFreeNeighbor(_Candidate);
                _Candidate.AddFreeNeighbor(_Leaf);
            }

            DevCount("edge conn: pairs ADJACENT", 1);
        }

        static Vector2Int GetFaceCell(Vector3 _Center, float _CellSize, int _PlaneAxis)

        {
            Vector3Int cellIndex = UtilsSpatialGrid.GetBucketIndex(_Center, _CellSize);

            if (_PlaneAxis == 0)
                return new Vector2Int(cellIndex.y, cellIndex.z);

            if (_PlaneAxis == 1)
                return new Vector2Int(cellIndex.x, cellIndex.z);

            return new Vector2Int(cellIndex.x, cellIndex.y);
        }

        bool AreLeavesAdjacent(Leaf _LeafA, Leaf _LeafB)
        {
            Bounds boundsA = _LeafA.Bounds;
            Bounds boundsB = _LeafB.Bounds;

            //Two leaves are adjacent if their bounds touch on exactly one face
            //They must overlap or touch in 2 axes and be exactly touching in the 3rd
            int touchingAxes    = 0;
            int overlappingAxes = 0;

            for (int axis = 0; axis < 3; axis++)
            {
                float aMin = boundsA.min[axis];
                float aMax = boundsA.max[axis];
                float bMin = boundsB.min[axis];
                float bMax = boundsB.max[axis];

                float overlap = Mathf.Min(aMax, bMax) - Mathf.Max(aMin, bMin);
                float epsilon = Mathf.Min(boundsA.size[axis], boundsB.size[axis]) * 0.01f;

                if (overlap > epsilon)
                    overlappingAxes++;
                else if (Mathf.Abs(overlap) <= epsilon)
                    touchingAxes++;
            }

            return touchingAxes == 1 && overlappingAxes == 2;
        }

        //True when a cluster's raw geometry fits within a single octree root cell on every axis. Clusters are never
        //united across obstacles (one owner per cluster), so this is no longer a union-eligibility gate; it is
        //purely a reconstruction-COST gate for ProcessIntersectingCachedClusters:
        //an existing cluster that merely happens to share bounds with a re-added obstacle (e.g. a scene-spanning
        //terrain) must not be pulled whole into `existingCrossedClusters`, or `trianglesToConsider` would union in
        //its entire triangle set — exactly the ~8.6s-construct/~92.8s-remove regression union-skip was originally
        //built to fix, this time on pre-baked re-enable. Measured on BoundsSource (raw geometry), not the inflated
        //safe Bounds — inflation adds a fixed shell that shouldn't by itself classify a genuinely small obstacle
        //as "large".
        //internal (not private): shared by RuntimeOperations.cs's IsRootComplementEligible (in-class
        //call) and PreBakedAddWorker.cs's ProcessIntersectingCachedClusters (standalone class).
        internal bool ClusterBoundedBySingleRoot(TriangleCluster _Cluster)
        {
            Vector3 size = _Cluster.BoundsSource.size;

            return size.x <= RootSize && size.y <= RootSize && size.z <= RootSize;
        }

        #endregion

    }
}
