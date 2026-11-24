using Nav3D.Common;
using Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.OperationsProgress;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Leaf = Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.Elements.Leaf;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.SpatialStorages
{
    class LeavesSpatialGrid
    {
        #region Attributes

        readonly int m_LayersCount;

        //0 is the lowest resolution, resp. the largest bucket size
        //(m_LayersCount - 1) is m_MinBucketSize resolution
        readonly List<ConcurrentDictionary<Vector3Int, Leaf>> m_Layers;

        #endregion

        #region Constructors

        public LeavesSpatialGrid(int _LayersCount, IEnumerable<Leaf> _InitialLeaves = null)
            : this(_LayersCount, null, _InitialLeaves)
        {
        }

        //_CapacityHints (optional, one entry per layer): the entry count each layer is expected to reach, from the
        //previous operations' grids (OperationScratch). A ConcurrentDictionary that grows re-creates every entry
        //node it holds, so a grid that starts at its final size allocates each entry once instead of ~twice.
        public LeavesSpatialGrid(int _LayersCount, int[] _CapacityHints, IEnumerable<Leaf> _InitialLeaves = null)
        {
            m_LayersCount = _LayersCount;
            m_Layers      = new List<ConcurrentDictionary<Vector3Int, Leaf>>(m_LayersCount);

            for (int i = 0; i < m_LayersCount; i++)
            {
                int hint = _CapacityHints != null && i < _CapacityHints.Length ? _CapacityHints[i] : 0;

                m_Layers.Add(hint > 0
                        ? new ConcurrentDictionary<Vector3Int, Leaf>(Environment.ProcessorCount, hint)
                        : new ConcurrentDictionary<Vector3Int, Leaf>());
            }

            if (_InitialLeaves != null)
                SetLeaves(_InitialLeaves);
        }

        //Records each layer's high-water entry count into _CapacityHints (grown, never shrunk).
        public void SaveCapacityHints(int[] _CapacityHints)
        {
            for (int i = 0; i < m_Layers.Count && i < _CapacityHints.Length; i++)
            {
                int count = m_Layers[i].Count;

                if (count > _CapacityHints[i])
                    _CapacityHints[i] = count;
            }
        }


        #endregion

        #region Public methods

        //hub severing: a finished operation's construction grid maps every leaf the
        //operation built or seeded, individually — under the conservative collector a single stale stack slot
        //pinning the grid would retain a subtree's worth of leaves past even the detached-subtree teardown
        //(which only unlinks the graph). Called when the operation is over; the grid must not be read after.
        public void Clear()
        {
            foreach (ConcurrentDictionary<Vector3Int, Leaf> layer in m_Layers)
                layer.Clear();
        }

        public void SetLeaves(IEnumerable<Leaf> _Leaves)
        {
            foreach (Leaf leaf in _Leaves)
            {
                //filter only not occupied leaves
                if (leaf.Occupied)
                    continue;

                SetLeaf(leaf);
            }
        }
        
        //Registers a free leaf for its (layer, index) and returns the canonical leaf for that cell.
        //If a leaf already exists at the slot it is REUSED (and _Leaf discarded), so the grid never
        //evicts a live leaf and the neighbor graph stays intact. Callers (Fork) must adopt the
        //returned instance into the tree so the node tree and the grid never diverge for the same
        //cell. GetOrAdd performs the get-or-insert atomically, which also makes this thread-safe
        //across the parallel root construction tasks.
        public Leaf SetLeaf(Leaf _Leaf)
        {
            return m_Layers[_Leaf.GridLayer].GetOrAdd(_Leaf.Index, _Leaf);
        }

        public bool TryGetLeaf(int _GridLayer, Vector3Int _Index, out Leaf _Leaf)
        {
            return m_Layers[_GridLayer].TryGetValue(_Index, out _Leaf);
        }

        public Leaf[] GetAllLeaves()
        {
            return m_Layers.SelectMany(_Layer => _Layer.Values).ToArray();
        }

        public void RemoveLeaf(Leaf _Leaf)
        {
            m_Layers[_Leaf.GridLayer]?.TryRemove(_Leaf.Index, out _);
        }

        //Evicts every leaf the predicate rejects, walking each layer in place. Single-threaded use only (the
        //post-construction passes on the op thread): the enumeration is safe against its own removals, not against
        //concurrent construction writes.
        public void RemoveWhere(Predicate<Leaf> _Reject)
        {
            foreach (ConcurrentDictionary<Vector3Int, Leaf> layer in m_Layers)
            {
                foreach (KeyValuePair<Vector3Int, Leaf> entry in layer)
                {
                    if (_Reject(entry.Value))
                        layer.TryRemove(entry.Key, out _);
                }
            }
        }


        //_Transaction routes every write by ownership (StagingTransaction.EditFreeNeighbor): inline on a leaf this
        //operation replaces or built, deferred to the publish's write lock on a leaf SHARED with the live tree. A
        //narrowed clone seeds shared leaves into this grid (a freed face's other side, a coarse across-face
        //neighbourhood), and wiring one of those inline is an unlocked write into the tree readers are traversing —
        //the exact write class the clone-anatomy sentinel trapped here . A caller whose grid can hold
        //no shared leaf (the pre-baked add: full clone, verbatim installs) passes null and keeps the plain writes.
        internal void FormLeavesConnections(GraphConstructionProgress _Progress, StagingTransaction _Transaction = null)
        {
            if (m_Layers.IsNullOrEmpty())
                return;

            void Wire(Leaf _A, Leaf _B)
            {
                if (_Transaction != null)
                {
                    _Transaction.EditFreeNeighbor(_A, _B, true);
                    _Transaction.EditFreeNeighbor(_B, _A, true);
                }
                else
                {
                    _A.AddFreeNeighbor(_B);
                    _B.AddFreeNeighbor(_A);
                }
            }

            //measured and not PARALLELISED, so the shape of this loop is a decision rather than an oversight.
            //It is the largest single step of a big build (1 885 ms of a 6 969 ms Skyline add) and it partitions
            //cleanly — a neighbour is found by lookup, never by mutation, and every pair is visited exactly once —
            //but a parallel form measured **3 772 ms, twice as slow**, at 9 cores busy and ~30 CPU-seconds against
            //1.9 s of work, because every edge writes the leaf on its far end through `Leaf.AddFreeNeighbor`'s
            //per-leaf lock ~3 M times and those writes convoy. the RULE that came out of it: a pass whose writes
            //are not partitioned with its reads cannot be parallelised by partitioning the reads. Making this one
            //faster means changing how an edge is written — or deleting the materialised graph altogether, which is
            //what the "leaf adjacency at QUERY time" direction proposes.
            //for each layer
            for (int i = 0; i < m_Layers.Count; i++)
            {
                ConcurrentDictionary<Vector3Int, Leaf> layer = m_Layers[i];

                int layerIndex = i;

                //for each leaf in layer
                foreach (KeyValuePair<Vector3Int, Leaf> kvp in layer)
                {
                    Vector3Int index = kvp.Key;
                    Leaf       leaf  = kvp.Value;

                    Vector3 cellCenter = leaf.Bounds.center;

                    foreach (Vector3Int direction in UtilsSpatialGrid.FacesDirections)
                    {
                        Vector3Int neighborIndex = index + direction;

                        //try to find a free adjacent leaf on the same layer
                        if (layer.TryGetValue(neighborIndex, out Leaf neighborLeaf))
                        {
                            //A same-layer pair is wired once, from its lower-index side: the edge is symmetric, so
                            //the leaf across a negative face has already wired this pair — or will, when the walk
                            //reaches it. This used to be a per-layer dictionary of per-leaf "already wired" direction
                            //lists (one List per leaf that had a neighbour), i.e. hundreds of bytes of garbage per
                            //leaf per operation for a rule the direction's sign expresses for free
                            //. A coarser neighbour (the branch below)
                            //is reached only from the finer side, so that lane was single-visit already.
                            if (direction.x + direction.y + direction.z > 0)
                                Wire(leaf, neighborLeaf);
                        }
                        else
                        {

                            float leafSize = leaf.Bounds.size.x;
                            Vector3 neighborCellCenter = leaf.Bounds.center + new Vector3(
                                    direction.x * leafSize,
                                    direction.y * leafSize,
                                    direction.z * leafSize
                                );

                            float curLeafSize   = leafSize * 2f;
                            int   curLayerIndex = layerIndex - 1;

                            Vector3Int curLeafIndex = UtilsSpatialGrid.GetBucketIndex(cellCenter, curLeafSize);
                            neighborIndex = UtilsSpatialGrid.GetBucketIndex(neighborCellCenter, curLeafSize);

                            while (curLayerIndex >= 0 && curLeafIndex != neighborIndex)
                            {
                                if (m_Layers[curLayerIndex].TryGetValue(neighborIndex, out neighborLeaf))
                                    break;

                                //go a higher level
                                curLayerIndex--;
                                curLeafSize *= 2f;

                                curLeafIndex  = UtilsSpatialGrid.GetBucketIndex(cellCenter, curLeafSize);
                                neighborIndex = UtilsSpatialGrid.GetBucketIndex(neighborCellCenter, curLeafSize);
                            }

                            if (neighborLeaf is null)
                                continue;

                            Wire(leaf, neighborLeaf);
                        }
                    }
                }

                //i + 1: the report sits at the end of layer i, so the layer just finished counts
                _Progress?.SetLayersProcessignProgress(i + 1, m_LayersCount);
            }
        }

        #endregion
    }
}
