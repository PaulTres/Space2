using Nav3D.Common;
using Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.SpatialStorages;
using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.UtilityTypes
{
    /// <summary>
    /// Data type proposed for storaging combined volume which occupied by one or more obstacles.
    /// </summary>
    //A cluster is a set of PARTS (ClusterPart, one per sub-cluster of one chunk), each with its own BVH; the cluster's
    //queries walk the parts whose bounds the query reaches — a forest, not one tree — so an update that keeps a part
    //keeps its BVH. A cluster built from one triangle set (the bake cache, the Lite capture, the legacy clustering)
    //is ONE part made on first use, and its queries are exactly what they were.
    class TriangleCluster : IBoundable
    {

        #region Attributes

        readonly object m_SpatialHashLock = new object();

        //the parts; null until the legacy single-set cluster makes its one part on first use
        ClusterPart[] m_Parts;

        //the index over the parts' bounds, made with them (PartForest); every query below goes through it
        PartForest m_Forest;

        //the legacy single set, or the lazy union of the parts' triangles
        HashSet<Triangle> m_Triangles;

        readonly object       m_SolidLock = new object();
        volatile ClusterSolid m_Solid;

        #endregion

        #region Properties

        //A set of all Triangles that containing in all obstacles that occupies that cluster's volume.
        public HashSet<Triangle> Triangles
        {
            get
            {
                HashSet<Triangle> triangles = m_Triangles;

                if (triangles != null)
                    return triangles;

                lock (m_SpatialHashLock)
                {
                    if (m_Triangles != null)
                        return m_Triangles;

                    HashSet<Triangle> union = new HashSet<Triangle>();

                    foreach (ClusterPart part in m_Parts)
                        union.UnionWith(part.Triangles);

                    m_Triangles = union;

                    return union;
                }
            }
        }

        //The parts the cluster is made of; a legacy cluster becomes one WHOLE part on the first ask.
        public IReadOnlyList<ClusterPart> Parts
        {
            get
            {
                ClusterPart[] parts = m_Parts;

                if (parts != null)
                    return parts;

                lock (m_SpatialHashLock)
                {
                    if (m_Parts == null)
                    {
                        ClusterPart[] made = { new ClusterPart(m_Triangles.ToArray(), BoundsSource, ChunkKey.Whole) };

                        m_Forest = new PartForest(made);
                        m_Parts  = made;
                    }

                    return m_Parts;
                }
            }
        }

        //the parts' index; a legacy cluster makes its one part on the first ask
        public PartForest Forest
        {
            get
            {
                PartForest forest = m_Forest;

                if (forest != null)
                    return forest;

                _ = Parts;

                return m_Forest;
            }
        }

        //Bounds represents spatial bounds of area occupied by triangles set.
        public Bounds BoundsSource { get; private set; }

        //Bounds enlarged to the 2*m_MaxAgentRadius in all axes.
        //Such enlargement guarantees that there are an obstacle-free space on the borders of cluster.
        public Bounds Bounds { get; private set; }

        //Accepts the triangles of an obstacle marked hollow: a shell holding one is never solid. Assigned before the
        //cluster is registered and never after.
        public Predicate<Triangle> HollowTriangles { get; set; }

        #endregion

        #region Constructors

        public TriangleCluster(HashSet<Triangle> _Triangles, Bounds _BoundsSource, Bounds _Bounds)
        {
            m_Triangles  = _Triangles;
            BoundsSource = _BoundsSource;
            Bounds       = _Bounds;
        }

        //A cluster over parts, which may be shared with the cluster versions this one replaces.
        public TriangleCluster(ClusterPart[] _Parts, Bounds _BoundsSource, Bounds _Bounds)
        {
            m_Parts      = _Parts;
            m_Forest     = new PartForest(_Parts);
            BoundsSource = _BoundsSource;
            Bounds       = _Bounds;
        }

        #endregion

        #region Public methods

        //The cluster's shells, analysed on first use (thread-safe) and kept for the cluster's life — the triangle set
        //never changes once the cluster exists, and the gap tolerance is the world's, fixed for the octree's life.
        public ClusterSolid GetSolid(float _GapTolerance)
        {
            ClusterSolid solid = m_Solid;

            if (solid != null)
                return solid;

            //a parts cluster offers its own BVHs as the closed surface; a legacy single-set cluster builds the closed
            //storage as before
            lock (m_SolidLock)
                return m_Solid ??= ClusterSolid.Build(Triangles, HollowTriangles, _GapTolerance, m_Forest, m_Forest != null ? BoundsSource : (Bounds?) null);
        }

        //Eagerly build the spatial storage (BVH) now instead of on the first lazy query. Used to warm prebaked
        //clusters' BVHs on a background thread after load, so the first runtime obstacle op that queries this cluster
        //does not pay the one-time O(N log N) SAH build (multiple seconds for a scene-spanning terrain). No-op if
        //already built; thread-safe via the same lock GetCrossedTriangles uses.
        public void WarmStorage()
        {
            foreach (ClusterPart part in Parts)
                _ = part.Storage;
        }

        //Widens both bounds to also cover another version's: a replaced cluster never lets its bounds shrink while the
        //obstacle lives, or the free leaves tiled along the retreated side of its box would stand outside every cluster.
        //A bound only permits tiling of empty octants; occupancy does not depend on it.
        public void WidenBounds(Bounds _BoundsSource, Bounds _Bounds)
        {
            Bounds source = BoundsSource;
            Bounds safe   = Bounds;

            source.Encapsulate(_BoundsSource);
            safe.Encapsulate(_Bounds);

            BoundsSource = source;
            Bounds       = safe;
        }

        //Does any triangle of this cluster cross the box? The BVH answers it with an early exit; the linear scan it
        //replaces in the dev validator (every triangle of every crossing cluster, per leaf) was the reason a
        //terrain-scale validation never finished — a scene-spanning cluster crosses every leaf's bounds query, so
        //each leaf paid the whole 525 312-triangle set.
        public bool CrossesAnyTriangle(Bounds _Bounds)
        {
            return Forest.AnyTriangleIntersects(_Bounds);
        }

        //Does a triangle outside _Except cross the box — "is there other geometry here", asked about an open shell's holes.
        public bool CrossesAnyTriangleExcept(Bounds _Bounds, HashSet<Triangle> _Except)
        {
            return Forest.AnyTriangleIntersectsExcept(_Bounds, _Except);
        }

        public bool TryRaycastFirst(Vector3 _Origin, Vector3 _Direction, float _MaxDistance, HashSet<Triangle> _Except, out Triangle _Hit, out float _Distance)
        {
            Predicate<Triangle> ignore = _Except == null ? null : _Except.Contains;

            return Forest.TryRaycastFirst(_Origin, _Direction, _MaxDistance, ignore, out _Hit, out _Distance);
        }

        public HashSet<Triangle> GetCrossedTriangles(Bounds _CrossingBounds)
        {
            HashSet<Triangle> result = new HashSet<Triangle>();

            Forest.CollectIntersectedTriangles(_CrossingBounds, result);

            return result;
        }

        public HashSet<Triangle> GetCrossedTriangles(Bounds[] _CrossingBounds)
        {
            HashSet<Triangle> result = new HashSet<Triangle>();

            CollectCrossedTriangles(_CrossingBounds, result);

            return result;
        }

        //The allocation-free form of GetCrossedTriangles(Bounds[]): the crossing triangles land in a caller-owned
        //set (the operation's reusable geometry-view set) instead of a fresh List
        //per bounds plus a fresh HashSet per call.
        public void CollectCrossedTriangles(Bounds[] _CrossingBounds, HashSet<Triangle> _Result)
        {
            PartForest forest = Forest;

            foreach (Bounds bounds in _CrossingBounds)
            {
                if (Bounds.Intersects(bounds))
                    forest.CollectIntersectedTriangles(bounds, _Result);
            }
        }

        //Set by the runtime add / move workers on the clusters they cluster from a fresh capture — the only clusters
        //whose geometry may be released when they leave the octree. A cluster the bake cache owns is installed by
        //reference on every pre-baked re-enable (P22 clones the NODES, not the clusters), so releasing it on the
        //disable's removal would hand the next enable a cluster with no triangles — measured on the protocol's
        //op-cost rig as every class's counters shrinking 20–80 % while the validator stayed clean (nothing left to be
        //uncovered), the exact blind spot the work-counter gate exists for.
        public bool GeometryReleasable { get; private set; }

        public void MarkGeometryReleasable()
        {
            GeometryReleasable = true;
        }

        //Severs the cluster's geometry once it has left the octree for good (the removed obstacle's clusters, a
        //moved obstacle's old clusters): the triangle set and the BVH map thousands of triangle objects, and under
        //the conservative collector a single stale pin on the dead cluster kept all of them (the P28 hub rule,
        //). Only after the publish — no reader can reach an unregistered
        //cluster — and only on a cluster the operation path itself created (GeometryReleasable); a cache-owned
        //cluster is left exactly as it is. _IncludeParts: false keeps the parts alive — a replaced cluster whose
        //parts a newer version still holds releases its own set only.
        public void ReleaseGeometry(bool _IncludeParts = true)
        {
            if (!GeometryReleasable)
                return;

            lock (m_SpatialHashLock)
            {
                m_Triangles?.Clear();

                if (_IncludeParts && m_Parts != null)
                {
                    foreach (ClusterPart part in m_Parts)
                        part.ReleaseGeometry();
                }
            }

            m_Solid = ClusterSolid.Empty;
        }

        #endregion

    }
}
