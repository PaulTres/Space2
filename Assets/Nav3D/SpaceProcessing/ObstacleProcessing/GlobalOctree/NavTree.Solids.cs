using Nav3D.Common;
using Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.SpatialStorages;
using Nav3D.SpaceProcessing.ObstacleProcessing.UtilityTypes;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System;
using UnityEngine;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree
{
    internal sealed partial class NavTree
    {
        //Published geometry only. Cleared under the publish write lock: closure can change at a distant door,
        //not just at the queried root. Lazy prevents duplicate construction by concurrent readers.
        readonly ConcurrentDictionary<Vector3Int, Lazy<SolidRegions>> m_PointSolids =
            new ConcurrentDictionary<Vector3Int, Lazy<SolidRegions>>();

        float m_PublishedSolidPitch;

        void ResetPointSolids()
        {
            m_PointSolids.Clear();
            m_PublishedSolidPitch = m_Octree.MinBucketSizeReal;
        }

        //Unlike occupied-leaf queries, this asks whether the CENTRE is inside geometric solid, including
        //open shells closed by other geometry. Sealed navigation volumes have no collision surface.
        internal bool TryIsPointInsideSolid(Vector3 _Point, out bool _Result)
        {
            _Result = false;

            if (!m_TreeLock.TryEnterReadLock(0))
            {
                DevReadDeclined();
                return false;
            }

            DevReadHoldStart();
            DevReadHoldLabel("TryIsPointInsideSolid");

            try
            {
                Vector3Int index = UtilsSpatialGrid.GetBucketIndex(_Point, m_Octree.RootSize);

                if (!m_PointSolids.TryGetValue(index, out Lazy<SolidRegions> cached))
                    cached = m_PointSolids.GetOrAdd(index, _Key => new Lazy<SolidRegions>(() => CreatePointSolids(_Key)));

                SolidRegions solids = cached.Value;
                _Result = solids != null && solids.Contains(_Point);
                return true;
            }
            finally
            {
                ReleaseReadLock();
            }
        }

        //Caller holds the live read lock; lazily evaluated while that same hold is active.
        SolidRegions CreatePointSolids(Vector3Int _Index)
        {
            HashSet<TriangleCluster> clusters = new HashSet<TriangleCluster>();

            m_TriangleClusterStorage.CollectCrossingBoundables(
                    UtilsSpatialGrid.GetBucketBounds(_Index, m_Octree.RootSize),
                    clusters
                );

            float pitch = m_PublishedSolidPitch > 0f ? m_PublishedSolidPitch : m_Octree.ShellGapTolerance;

            return SolidRegions.Create(
                    clusters,
                    m_Octree.ShellGapTolerance,
                    (_Shell, _Cluster) => m_Octree.IsClosedByOtherGeometry(
                            _Shell,
                            _Cluster,
                            GlobalOctree.GeometryDelta.None,
                            _Pitch: pitch
                        )
                );
        }
    }
}