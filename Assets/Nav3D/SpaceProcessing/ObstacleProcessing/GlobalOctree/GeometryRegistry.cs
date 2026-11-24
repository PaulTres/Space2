using Nav3D.Common;
using Nav3D.SpaceProcessing.ObstacleProcessing.UtilityTypes;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree
{
    //Consolidates all obstacle->geometry bookkeeping (raw/safe clusters per obstacle, ObstacleInfo, cluster->roots,
    //root->cluster indices, cluster->owning-obstacle) behind atomic Register/Unregister methods.
    //
    //Two atomic compound units:
    //  - obstacle-identity: RawClustersByID[id] + the root->raw-cluster index + ObstacleInfo[id];
    //  - cluster: RootsByCluster[cluster] + the root->safe-cluster index + ObstaclesByCluster[cluster] + each
    //    owner's SafeClustersByID membership.
    //An obstacle's clusters are a separate, independently-sized set of cluster units, unregistered one at a time
    //before its own identity is touched.
    //
    //Deliberately not a changeset/published split — StagingTransaction.DeferredRegistryChanges already is that
    //mechanism. This registry only exposes correct, synchronous, atomic methods; each caller decides whether to
    //invoke them immediately (build-only, unlocked, not yet reader-visible) or through the transaction's deferred
    //list (reader-visible only at publish).
    internal sealed class GeometryRegistry
    {
        readonly float m_RootSize;

        //Boundary-inclusion epsilon for raw-bounds→root derivations (see GetRawCrossedRootIndices). World units;
        //deliberately a small constant (not tied to the region-dependent min bucket size) so every op derives the
        //same root set for the same bounds regardless of the resolution-region state at the time.
        const float RAW_ROOT_BOUNDARY_EPSILON = 0.001f;

        //ObstacleControllerID -> Source (raw) clusters. Keep source clusters for every added obstacle.
        readonly Dictionary<int, TriangleCluster[]> m_IDToRawClusters = new Dictionary<int, TriangleCluster[]>();

        //ObstacleControllerID -> Safe (enlarged) clusters. One obstacle may correspond to one or more clusters.
        readonly Dictionary<int, List<TriangleCluster>> m_IDToSafeClusters = new Dictionary<int, List<TriangleCluster>>();
        readonly Dictionary<int, ObstacleInfo>          m_IDToObstacleInfo = new Dictionary<int, ObstacleInfo>();

        //Cluster -> roots it crosses.
        readonly Dictionary<TriangleCluster, Vector3Int[]>     m_ClusterToRoots     = new Dictionary<TriangleCluster, Vector3Int[]>();
        readonly Dictionary<Vector3Int, List<TriangleCluster>> m_RootToSafeClusters = new Dictionary<Vector3Int, List<TriangleCluster>>();
        readonly Dictionary<Vector3Int, List<TriangleCluster>> m_RootToRawClusters  = new Dictionary<Vector3Int, List<TriangleCluster>>();

        //Cluster -> owning ObstacleControllerID. Always a singleton set: clusters are never united across obstacles
        //(one owner per cluster). Kept as HashSet<int> rather than a scalar int to avoid touching the ~15+ call
        //sites across this file and three others that only ever enumerate/join it.
        readonly Dictionary<TriangleCluster, HashSet<int>> m_ClusterToObstacles = new Dictionary<TriangleCluster, HashSet<int>>();

        //ObstacleControllerID -> the chunks its registered geometry was built from; what the next update diffs against.
        //Only an obstacle the chunked builder clustered has one (the bake cache and the Lite capture register none).
        readonly Dictionary<int, ObstacleChunkTable> m_IDToChunks = new Dictionary<int, ObstacleChunkTable>();

        public GeometryRegistry(float _RootSize)
        {
            m_RootSize = _RootSize;
        }

        #region Read-only views and queries

        public IReadOnlyDictionary<int, TriangleCluster[]>        RawClustersByID    => m_IDToRawClusters;
        public IReadOnlyDictionary<int, List<TriangleCluster>>    SafeClustersByID   => m_IDToSafeClusters;
        public IReadOnlyDictionary<TriangleCluster, Vector3Int[]> RootsByCluster     => m_ClusterToRoots;
        public IReadOnlyDictionary<TriangleCluster, HashSet<int>> ObstaclesByCluster => m_ClusterToObstacles;

        public int  ObstacleCount                 => m_IDToObstacleInfo.Count;
        public bool IsObstacleRegistered(int _ID) => m_IDToObstacleInfo.ContainsKey(_ID);

        public bool TryGetRawClusters(int _ID, out TriangleCluster[] _Clusters) => m_IDToRawClusters.TryGetValue(_ID, out _Clusters);

        public bool TryGetSafeClusters(int _ID, out List<TriangleCluster> _Clusters) => m_IDToSafeClusters.TryGetValue(_ID, out _Clusters);

        public bool TryGetClustersAtRoot(Vector3Int _Root, out List<TriangleCluster> _Clusters) => m_RootToSafeClusters.TryGetValue(_Root, out _Clusters);

        public bool TryGetRawClustersAtRoot(Vector3Int _Root, out List<TriangleCluster> _Clusters) => m_RootToRawClusters.TryGetValue(_Root, out _Clusters);

        public bool TryGetRootsOfCluster(TriangleCluster _Cluster, out Vector3Int[] _Roots) => m_ClusterToRoots.TryGetValue(_Cluster, out _Roots);

        public bool TryGetObstaclesOfCluster(TriangleCluster _Cluster, out HashSet<int> _Owners) => m_ClusterToObstacles.TryGetValue(_Cluster, out _Owners);

        public bool TryGetChunkTable(int _ID, out ObstacleChunkTable _Table) => m_IDToChunks.TryGetValue(_ID, out _Table);

        //Root (layer-0) cells a cluster's raw geometry can occupy. not a plain GetCrossedBucketsIndices(BoundsSource):
        //octree occupancy uses inclusive bounds/triangle intersection, so geometry lying exactly on a root boundary
        //plane (e.g. a ground quad at y = 0) also occupies boundary-touching leaves in the NEIGHBOURING root — while
        //the floor-based half-open bucket walk excludes that root when the bounds' face sits exactly on the plane.
        //Every raw-bounds→roots derivation (the root→raw-cluster map, the removal's prune-branch classification, the
        //pre-baked re-add's reconstruct scoping) must use this epsilon-enlarged form; otherwise the neighbouring
        //root's boundary leaves fall outside every prune set and survive their obstacle's removal as stale occupancy
        //(the toggler-rig ground-slab OCCUPIED_WITHOUT_TRIANGLE: stale leaves whose top face was exactly y = 0, left
        //by clusters whose BoundsSource bottom was exactly y = 0).
        public Vector3Int[] GetRawCrossedRootIndices(Bounds _BoundsSource)
        {
            return UtilsSpatialGrid.GetCrossedBucketsIndices(_BoundsSource.Enlarge(RAW_ROOT_BOUNDARY_EPSILON), m_RootSize);
        }

        //Every obstacle ID owning any cluster currently registered in the given roots. When an add/remove touches a
        //set of roots, every obstacle sharing those roots (not only the one whose clusters were re-built) may have
        //different occupied leaves afterward, so its cached per-obstacle gizmo must be invalidated.
        public HashSet<int> CollectObstacleIDsInRoots(IEnumerable<Vector3Int> _RootIndices)
        {
            HashSet<int> ids = new HashSet<int>();

            foreach (Vector3Int rootIndex in _RootIndices)
            {
                if (!m_RootToSafeClusters.TryGetValue(rootIndex, out List<TriangleCluster> clusters))
                    continue;

                foreach (TriangleCluster cluster in clusters)
                    if (m_ClusterToObstacles.TryGetValue(cluster, out HashSet<int> owners))
                        ids.UnionWith(owners);
            }

            return ids;
        }

        public string GetStorageContentDescription()
        {
            return $"Obstacles count: {m_IDToObstacleInfo.Count}, Clusters count: {m_ClusterToObstacles.Count}";
        }

        #endregion

        #region Obstacle-identity atomic unit

        //Registers an obstacle's raw (source) clusters and its ObstacleInfo together. ContainsKey backstop mirrors
        //AddObstacleFromCache's pre-registry behavior: cached source-cluster objects are identical across re-enables
        //of the same pre-baked obstacle, so a stray pre-existing entry must not append duplicates into the
        //root→raw-cluster index.
        //The epoch's death only (GlobalOctree.ReleaseWorld): every map names clusters, and a cluster names a whole
        //obstacle's geometry, so an emptied registry is what keeps one pinned cluster from costing all of them
        public void Clear()
        {
            m_IDToRawClusters.Clear();
            m_IDToSafeClusters.Clear();
            m_IDToObstacleInfo.Clear();
            m_ClusterToRoots.Clear();
            m_RootToSafeClusters.Clear();
            m_RootToRawClusters.Clear();
            m_ClusterToObstacles.Clear();
            m_IDToChunks.Clear();
        }

        //The partial update's identity swap: the raw clusters and the ObstacleInfo are replaced while the obstacle stays
        //registered — its unchanged safe clusters keep their registration, only the changed ones were unregistered
        //and their replacements registered beside them. RemoveRawRootIndexEntries must already have run (inline).
        public void ReplaceObstacleRaw(int _ID, TriangleCluster[] _RawClusters, ObstacleInfo _Info)
        {
            m_IDToRawClusters[_ID] = _RawClusters;
            AddToRootToRawClusters(_RawClusters);

            m_IDToObstacleInfo[_ID] = _Info;
        }

        //The chunk table moves with the identity: registered in the same deferred block as RegisterObstacleRaw, dropped by
        //UnregisterObstacleIdentity.
        public void RegisterChunkTable(int _ID, ObstacleChunkTable _Table)
        {
            m_IDToChunks[_ID] = _Table;
        }

        public void RegisterObstacleRaw(int _ID, TriangleCluster[] _RawClusters, ObstacleInfo _Info)
        {
            if (!m_IDToRawClusters.ContainsKey(_ID))
            {
                m_IDToRawClusters[_ID] = _RawClusters;
                AddToRootToRawClusters(_RawClusters);
            }

            m_IDToObstacleInfo[_ID] = _Info;
        }

        //Drops _ID's entries from the root→raw-cluster index only — build-only, needed current by other in-op
        //readers (e.g. PurgeGridGhosts) before the identity removal below is committed. Call this inline/immediately;
        //RawClustersByID/ObstacleInfo/SafeClustersByID are untouched here (read to derive which roots to drop, not
        //removed) — call UnregisterObstacleIdentity to actually commit those, typically deferred to publish.
        public void RemoveRawRootIndexEntries(int _ID)
        {
            if (m_IDToRawClusters.TryGetValue(_ID, out TriangleCluster[] raw))
                RemoveFromRootToRawClusters(raw);
        }

        //The inverse of RemoveRawRootIndexEntries, for an operation that does not publish (the same rule
        //as RestoreRootIndexAndOwnership below: an inline registry change must be undone by an abandoned operation).
        //Sound because the identity removal that would drop m_IDToRawClusters is deferred to the publish, so on this
        //path the obstacle's raw clusters are still there to re-index.
        public void RestoreRawRootIndexEntries(int _ID)
        {
            if (m_IDToRawClusters.TryGetValue(_ID, out TriangleCluster[] raw))
                AddToRootToRawClusters(raw);
        }

        //Commits an obstacle's identity removal: RawClustersByID, ObstacleInfo, and the SafeClustersByID backstop
        //entry — the ID-map atomicity pair (safe + raw registrations move together), plus ObstacleInfo for
        //full consistency (a build-phase exception between this and publish must not leave the obstacle
        //half-torn-down). In the normal removal flow every one of the obstacle's clusters is already unregistered
        //per-cluster (UnregisterCluster empties and removes the owner's safe-list entry as its last owner leaves),
        //so the SafeClustersByID removal here is a backstop, not the primary path. RemoveRawRootIndexEntries must
        //already have run (inline, earlier) — this does not touch the root index.
        public void UnregisterObstacleIdentity(int _ID)
        {
            m_IDToRawClusters.Remove(_ID);
            m_IDToObstacleInfo.Remove(_ID);
            m_IDToSafeClusters.Remove(_ID);
            m_IDToChunks.Remove(_ID);
        }

        //Convenience: both steps together, immediately — for the "orphan reap" case (an obstacle whose clusters
        //were never registered at all, so it was never reader-visible; nothing to retry-protect by deferring).
        public void UnregisterObstacleRaw(int _ID)
        {
            RemoveRawRootIndexEntries(_ID);
            UnregisterObstacleIdentity(_ID);
        }

        #endregion

        #region Cluster atomic unit

        public void RegisterCluster(TriangleCluster _Cluster, Vector3Int[] _Roots, HashSet<int> _OwnerIDs)
        {
            AddToRootToClusters(_Roots, _Cluster);
            m_ClusterToObstacles.Add(_Cluster, _OwnerIDs);

            m_ClusterToRoots.Add(_Cluster, _Roots);

            foreach (int ownerID in _OwnerIDs)
                m_IDToSafeClusters.GetOrAdd(ownerID).Add(_Cluster);
        }

        //Split of UnregisterCluster's two build-only-vs-reader-visible halves, for call sites whose surrounding op
        //needs the root index (and ObstaclesByCluster) current INLINE before later in-op code reads it — e.g.
        //RemoveObstacleInternal's remainingFootprintClusters, computed right after this runs, reads the root index
        //expecting the removed cluster already gone from it — while the storage/roots/safe-cluster commit stays
        //deferred to publish. Capture ObstaclesByCluster[_Cluster] (via TryGetObstaclesOfCluster) before calling
        //this — it removes that entry — then pass the captured owners to CommitClusterUnregistration.
        public void RemoveFromRootIndexAndOwnership(TriangleCluster _Cluster)
        {
            if (m_ClusterToRoots.TryGetValue(_Cluster, out Vector3Int[] roots))
                RemoveFromRootToClusters(roots, _Cluster);

            m_ClusterToObstacles.Remove(_Cluster);
        }

        //the inverse of RemoveFromRootIndexAndOwnership, for an operation that does not reach its publish.
        //The pair above is deliberately split across the publish — the root index must be current INLINE because the
        //construction reads it, while the reader-visible half waits for the write lock — and that split is only sound
        //if an abandoned operation puts the inline half back. It did not, and the state it left is exactly the
        //KeyNotFoundException the owner met: the cluster gone from ObstaclesByCluster while SafeClustersByID still
        //lists it, so the next removal of that obstacle threw on the first line that reads the owners, was caught and
        //logged as "Removal error", and every later removal of that ID repeated it.
        public void RestoreRootIndexAndOwnership(TriangleCluster _Cluster, HashSet<int> _Owners)
        {
            if (m_ClusterToRoots.TryGetValue(_Cluster, out Vector3Int[] roots))
                AddToRootToClusters(roots, _Cluster);

            if (_Owners != null && !m_ClusterToObstacles.ContainsKey(_Cluster))
                m_ClusterToObstacles.Add(_Cluster, _Owners);
        }

        public void CommitClusterUnregistration(TriangleCluster _Cluster, HashSet<int> _Owners)
        {
            m_ClusterToRoots.Remove(_Cluster);

            if (_Owners == null)
                return;

            foreach (int ownerID in _Owners)
            {
                if (!m_IDToSafeClusters.TryGetValue(ownerID, out List<TriangleCluster> list))
                    continue;

                list.Remove(_Cluster);

                if (!list.Any())
                    m_IDToSafeClusters.Remove(ownerID);
            }
        }

        #endregion

        #region Root-index helpers (private — only reachable through the atomic units above)

        void AddToRootToClusters(Vector3Int[] _RootIndices, TriangleCluster _Cluster)
        {
            foreach (Vector3Int rootIndex in _RootIndices)
            {
                if (m_RootToSafeClusters.TryGetValue(rootIndex, out List<TriangleCluster> clusters))
                    clusters.Add(_Cluster);
                else
                    m_RootToSafeClusters.Add(rootIndex, new List<TriangleCluster> { _Cluster });
            }
        }

        void RemoveFromRootToClusters(Vector3Int[] _RootIndices, TriangleCluster _Cluster)
        {
            foreach (Vector3Int rootIndex in _RootIndices)
            {
                List<TriangleCluster> clusters = m_RootToSafeClusters[rootIndex];
                clusters.Remove(_Cluster);

                if (!clusters.Any())
                    m_RootToSafeClusters.Remove(rootIndex);
            }
        }

        void AddToRootToRawClusters(TriangleCluster[] _RawClusters)
        {
            foreach (TriangleCluster rawCluster in _RawClusters)
            {
                Vector3Int[] rootIndices = GetRawCrossedRootIndices(rawCluster.BoundsSource);

                foreach (Vector3Int rootIndex in rootIndices)
                {
                    if (m_RootToRawClusters.TryGetValue(rootIndex, out List<TriangleCluster> clusters))
                        clusters.Add(rawCluster);
                    else
                        m_RootToRawClusters.Add(rootIndex, new List<TriangleCluster> { rawCluster });
                }
            }
        }

        void RemoveFromRootToRawClusters(TriangleCluster[] _RawClusters)
        {
            foreach (TriangleCluster rawCluster in _RawClusters)
            {
                //same epsilon-enlarged derivation as AddToRootToRawClusters — entries must be removed from exactly
                //the cells they were added to
                Vector3Int[] rootIndices = GetRawCrossedRootIndices(rawCluster.BoundsSource);

                foreach (Vector3Int rootIndex in rootIndices)
                {
                    if (!m_RootToRawClusters.TryGetValue(rootIndex, out List<TriangleCluster> clusters))
                        continue;

                    clusters.Remove(rawCluster);

                    if (!clusters.Any())
                        m_RootToRawClusters.Remove(rootIndex);
                }
            }
        }

        #endregion

    }
}
