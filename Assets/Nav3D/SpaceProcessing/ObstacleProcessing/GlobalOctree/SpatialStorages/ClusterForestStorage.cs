using Nav3D.SpaceProcessing.ObstacleProcessing.UtilityTypes;
using System.Collections.Generic;
using UnityEngine;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.SpatialStorages
{
    //The operation's geometry view as the surviving clusters' own parts under one index: the construction's occupancy
    //question goes to the parts' BVHs, which the clusters already hold, so the operation collects no triangles and
    //builds no BVH of its own. A cell the construction asks about lies inside the view's roots, and a triangle crossing
    //it crosses the root, so the answer is the one the per-operation storage over the collected triangles gave.
    internal sealed class ClusterForestStorage : IOccupancyStorage
    {

        #region Attributes

        readonly PartForest m_Forest;

        #endregion

        #region Properties

        public SolidRegions Solids { get; set; }

        public int PartCount => m_Forest.Count;

        #endregion

        #region Constructors

        public ClusterForestStorage(IReadOnlyList<TriangleCluster> _Clusters)
        {
            List<ClusterPart> parts = new List<ClusterPart>();

            foreach (TriangleCluster cluster in _Clusters)
                parts.AddRange(cluster.Parts);

            m_Forest = new PartForest(parts.ToArray());
        }

        #endregion

        #region Public methods

        public bool IsIntersect(Bounds _Bounds)
        {
            return m_Forest.AnyTriangleIntersects(_Bounds);
        }

        #endregion

    }
}