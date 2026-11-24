using Nav3D.Common;
using Nav3D.Common.Extensions;
using Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.SpatialStorages;
using Nav3D.SpaceProcessing.ObstacleProcessing.UtilityTypes;
using System;
using System.Collections.Generic;
using UnityEngine;
using Leaf = Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.Elements.Leaf;
using Node = Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.Elements.Node;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree
{
    //The operation's reusable scratch, one instance per octree handed to every operation in turn: the per-operation
    //buffers every add, remove, move and region update used to allocate fresh. Reuse rather than more severing
    //because Unity's conservative collector pins dead objects through stale stack slots, and a pinned container
    //retains everything it maps; a buffer that lives across operations is never garbage, and is cleared between
    //operations with its capacity kept. Operations are serialized on the executor, so one scratch is never contended;
    //it is taken in the StagingTransaction constructor and released in its Dispose. The discard rule: a cancelled or
    //failed operation returns before its construction tasks have necessarily finished, and a straggler may still
    //write into the grid, so a transaction that did not complete its publish releases the scratch with _Discard and
    //the next operation allocates new instances.
    internal sealed class OperationScratch
    {
        #region Attributes

        bool m_InUse;

        //a snapshot of one leaf's neighbour set for the op-thread passes that edit a set while walking it
        List<Leaf> m_NeighborBuffer;

        //the prune's two per-root sets (contained leaves, their neighbourhood) and the narrowed cell's across-face set
        HashSet<Leaf> m_PruneContained;
        HashSet<Leaf> m_PruneNeighbors;
        HashSet<Leaf> m_PruneAcrossFaces;

        //the free-graph repair's walk set, and the classification's per-root leaf set
        HashSet<Leaf> m_RepairLeaves;
        HashSet<Leaf> m_ClassifyLeaves;

        //the edge-connection pass: the two faces' leaf sets and the face-cell index of each side
        HashSet<Leaf>                                    m_EdgeLeavesA;
        HashSet<Leaf>                                    m_EdgeLeavesB;
        Dictionary<(float Size, Vector2Int Cell), Leaf>  m_FaceIndexA;
        Dictionary<(float Size, Vector2Int Cell), Leaf>  m_FaceIndexB;

        //the rare second leaf on the same face cell (a duplicate instance the pre-baked lane can present) — kept
        //so the flat index loses nothing the old list-per-cell index found
        Dictionary<(float Size, Vector2Int Cell), List<Leaf>> m_FaceOverflowA;
        Dictionary<(float Size, Vector2Int Cell), List<Leaf>> m_FaceOverflowB;

        //the transaction's own hubs (P28: all severed at op end; now retained empty instead of dropped)
        Dictionary<Leaf, Leaf>                        m_OriginalToClone;
        HashSet<Leaf>                                 m_Clones;
        Dictionary<Leaf, bool>                        m_SharedOwnerAnswers;
        List<(Leaf Owner, Leaf Neighbour, bool Add)>  m_DeferredSharedGraphEdits;
        List<Action>                                  m_DeferredRegistryChanges;
        List<Action>                                  m_RegistryRollbacks;
        List<(Leaf Outer, Leaf Original, Leaf Clone)> m_RingBackRefRepoints;
        List<Node>                                    m_ReplacedOriginals;
        Stack<Node>                                   m_TeardownStack;

        //per-layer entry counts of the last operations' construction grids — the next grid is sized to them, so a
        //ConcurrentDictionary (whose growth re-creates every entry node) never grows during construction
        int[] m_GridCapacityHints = Array.Empty<int>();

        //the pre-transaction BUFFERS: the add / move workers cluster the obstacle's triangles before they open their
        //transaction (the footprint is cut from the clusters), so these two live outside Begin/End. Both hold only
        //what one clustering needs and are emptied on the way out of it — the clustering scratch keeps integers
        //(its own Clear), the triangle list is cleared by the worker once the clusters own their sets.
        readonly ExtensionIBoundable.ClusteringScratch m_Clustering   = new ExtensionIBoundable.ClusteringScratch();
        readonly List<Triangle>                        m_TriangleList = new List<Triangle>();

        //the cluster-storage query set (TriangleClusterStorage.CollectCrossingBoundables per footprint root)
        HashSet<TriangleCluster> m_ClusterQuery;

        #endregion

        #region Properties

        public bool InUse => m_InUse;

        public ExtensionIBoundable.ClusteringScratch Clustering   => m_Clustering;
        public List<Triangle>                        TriangleList => m_TriangleList;

        public HashSet<TriangleCluster> ClusterQuery => m_ClusterQuery;

        public List<Leaf> NeighborBuffer => m_NeighborBuffer;

        public HashSet<Leaf> PruneContained   => m_PruneContained;
        public HashSet<Leaf> PruneNeighbors   => m_PruneNeighbors;
        public HashSet<Leaf> PruneAcrossFaces => m_PruneAcrossFaces;
        public HashSet<Leaf> RepairLeaves     => m_RepairLeaves;
        public HashSet<Leaf> ClassifyLeaves   => m_ClassifyLeaves;

        public HashSet<Leaf>                                   EdgeLeavesA => m_EdgeLeavesA;
        public HashSet<Leaf>                                   EdgeLeavesB => m_EdgeLeavesB;
        public Dictionary<(float Size, Vector2Int Cell), Leaf> FaceIndexA  => m_FaceIndexA;
        public Dictionary<(float Size, Vector2Int Cell), Leaf> FaceIndexB  => m_FaceIndexB;

        public Dictionary<(float Size, Vector2Int Cell), List<Leaf>> FaceOverflowA => m_FaceOverflowA;
        public Dictionary<(float Size, Vector2Int Cell), List<Leaf>> FaceOverflowB => m_FaceOverflowB;

        public Dictionary<Leaf, Leaf>                        OriginalToClone          => m_OriginalToClone;
        public HashSet<Leaf>                                 Clones                   => m_Clones;
        public Dictionary<Leaf, bool>                        SharedOwnerAnswers       => m_SharedOwnerAnswers;
        public List<(Leaf Owner, Leaf Neighbour, bool Add)>  DeferredSharedGraphEdits => m_DeferredSharedGraphEdits;
        public List<Action>                                  DeferredRegistryChanges  => m_DeferredRegistryChanges;
        public List<Action>                                  RegistryRollbacks        => m_RegistryRollbacks;
        public List<(Leaf Outer, Leaf Original, Leaf Clone)> RingBackRefRepoints      => m_RingBackRefRepoints;
        public List<Node>                                    ReplacedOriginals        => m_ReplacedOriginals;
        public Stack<Node>                                   TeardownStack            => m_TeardownStack;

        #endregion

        #region Public methods

        //Takes the scratch for one operation. A second Begin while in use is a programming error — operations are
        //serialized and each owns one transaction — and is reported rather than silently shared.
        public void Begin()
        {
            if (m_InUse)
                Debug.LogError($"[Nav3D] {nameof(OperationScratch)}: Begin while an operation still holds the scratch — operations must be serialized.");

            m_InUse = true;

            m_NeighborBuffer ??= new List<Leaf>();
            m_ClusterQuery   ??= new HashSet<TriangleCluster>();

            m_PruneContained   ??= new HashSet<Leaf>();
            m_PruneNeighbors   ??= new HashSet<Leaf>();
            m_PruneAcrossFaces ??= new HashSet<Leaf>();
            m_RepairLeaves     ??= new HashSet<Leaf>();
            m_ClassifyLeaves   ??= new HashSet<Leaf>();

            m_EdgeLeavesA ??= new HashSet<Leaf>();
            m_EdgeLeavesB ??= new HashSet<Leaf>();
            m_FaceIndexA  ??= new Dictionary<(float, Vector2Int), Leaf>();
            m_FaceIndexB  ??= new Dictionary<(float, Vector2Int), Leaf>();
            m_FaceOverflowA ??= new Dictionary<(float, Vector2Int), List<Leaf>>();
            m_FaceOverflowB ??= new Dictionary<(float, Vector2Int), List<Leaf>>();

            m_OriginalToClone          ??= new Dictionary<Leaf, Leaf>();
            m_Clones                   ??= new HashSet<Leaf>();
            m_SharedOwnerAnswers       ??= new Dictionary<Leaf, bool>();
            m_DeferredSharedGraphEdits ??= new List<(Leaf, Leaf, bool)>();
            m_DeferredRegistryChanges  ??= new List<Action>();
            m_RegistryRollbacks        ??= new List<Action>();
            m_RingBackRefRepoints      ??= new List<(Leaf, Leaf, Leaf)>();
            m_ReplacedOriginals        ??= new List<Node>();
            m_TeardownStack            ??= new Stack<Node>();
        }

        //Releases the scratch. Cleared (contents dropped, capacity kept) after a completed operation; DROPPED after
        //one that did not complete — see the discard rule in the header.
        public void End(bool _Discard)
        {
            m_InUse = false;

            if (_Discard)
            {
                Drop();

                return;
            }

            m_NeighborBuffer.Clear();
            m_ClusterQuery.Clear();

            m_PruneContained.Clear();
            m_PruneNeighbors.Clear();
            m_PruneAcrossFaces.Clear();
            m_RepairLeaves.Clear();
            m_ClassifyLeaves.Clear();

            m_EdgeLeavesA.Clear();
            m_EdgeLeavesB.Clear();
            m_FaceIndexA.Clear();
            m_FaceIndexB.Clear();
            m_FaceOverflowA.Clear();
            m_FaceOverflowB.Clear();

            m_OriginalToClone.Clear();
            m_Clones.Clear();
            m_SharedOwnerAnswers.Clear();
            m_DeferredSharedGraphEdits.Clear();
            m_DeferredRegistryChanges.Clear();
            m_RegistryRollbacks.Clear();
            m_RingBackRefRepoints.Clear();
            m_ReplacedOriginals.Clear();
            m_TeardownStack.Clear();
        }

        //A construction grid sized to what the previous operations' grids held, so it never grows mid-build. The grid
        //object itself is per operation (its layer count and progress object are the operation's); the reuse is
        //the sizing, because a ConcurrentDictionary cannot keep its capacity across a Clear.
        public LeavesSpatialGrid RentConstructionGrid(int _LayersCount)
        {
            if (m_GridCapacityHints.Length != _LayersCount)
                Array.Resize(ref m_GridCapacityHints, _LayersCount);

            return new LeavesSpatialGrid(_LayersCount, m_GridCapacityHints);
        }

        //Called with the finished grid before it is severed: remembers how big each layer got.
        public void ReturnConstructionGrid(LeavesSpatialGrid _Grid)
        {
            _Grid.SaveCapacityHints(m_GridCapacityHints);
        }

        #endregion

        #region Service methods

        void Drop()
        {
            m_NeighborBuffer = null;
            m_ClusterQuery   = null;

            m_PruneContained   = null;
            m_PruneNeighbors   = null;
            m_PruneAcrossFaces = null;
            m_RepairLeaves     = null;
            m_ClassifyLeaves   = null;

            m_EdgeLeavesA = null;
            m_EdgeLeavesB = null;
            m_FaceIndexA  = null;
            m_FaceIndexB  = null;
            m_FaceOverflowA = null;
            m_FaceOverflowB = null;


            m_OriginalToClone          = null;
            m_Clones                   = null;
            m_SharedOwnerAnswers       = null;
            m_DeferredSharedGraphEdits = null;
            m_DeferredRegistryChanges  = null;
            m_RegistryRollbacks        = null;
            m_RingBackRefRepoints      = null;
            m_ReplacedOriginals        = null;
            m_TeardownStack            = null;
        }

        #endregion
    }
}
