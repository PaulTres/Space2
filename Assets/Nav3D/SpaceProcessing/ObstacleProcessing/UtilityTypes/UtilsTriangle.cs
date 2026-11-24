using Nav3D.Common.Extensions;
using Nav3D.SpaceProcessing.ObstacleProcessing.UtilityTypes;
using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

//The namespace is Common's, the assembly is SpaceProcessing's: clustering an obstacle's triangles is obstacle
//processing (its only consumers are the three octree workers), and it is written in terms of TriangleCluster,
//which owns a BVH and cannot be lowered.
namespace Nav3D.Common
{
    static class UtilsTriangle
    {

        #region Public methods

        public static TriangleCluster[] ClusterTriangles(Triangle[] _Triangles, float _ThresholdDistance, float _FinalEnlargement)
        {
            if (_FinalEnlargement > _ThresholdDistance * 2)
                throw new ArgumentException(
                        $"{nameof(UtilsTriangle)}.{nameof(ClusterTriangles)}: _FinalEnlargement " +
                        $"({_FinalEnlargement}) must be at most twice _ThresholdDistance ({_ThresholdDistance * 2})"
                    );

            return WeldClustering.Groups(_Triangles, _ThresholdDistance, null)
                                 .Select(_Cluster =>
                                          {
                                              Bounds bounds = ExtensionBounds.TrianglesBounds(_Cluster);
                                              return new TriangleCluster(_Cluster.ToHashSet(), bounds, bounds.Enlarge(_FinalEnlargement));
                                          }
                                      )
                                 .ToArray();
        }

        /// <summary>
        /// Clusters triangles once and produces two cluster sets with different enlargement values,
        /// avoiding redundant ClusterByAABBDistance computation.
        /// </summary>
        //_Scratch (optional): the clustering's reusable buffers — see ExtensionIBoundable.ClusteringScratch.
        public static (TriangleCluster[] RawClusters, TriangleCluster[] EnlargedClusters) ClusterTrianglesDual(
                IList<Triangle>                       _Triangles,
                float                                 _ThresholdDistance,
                float                                 _FinalEnlargement,
                ExtensionIBoundable.ClusteringScratch _Scratch = null
            )
        {
            if (_FinalEnlargement > _ThresholdDistance * 2)
                throw new ArgumentException(
                        $"{nameof(UtilsTriangle)}.{nameof(ClusterTrianglesDual)}: _FinalEnlargement " +
                        $"({_FinalEnlargement}) must be at most twice _ThresholdDistance ({_ThresholdDistance * 2})"
                    );

            Triangle[][] groups = WeldClustering.Groups(_Triangles, _ThresholdDistance, _Scratch);

            TriangleCluster[] rawClusters = new TriangleCluster[groups.Length];

            TriangleCluster[] enlargedClusters = new TriangleCluster[groups.Length];

            for (int i = 0; i < groups.Length; i++)
            {
                HashSet<Triangle> triangleSet = groups[i].ToHashSet();
                Bounds            bounds      = ExtensionBounds.TrianglesBounds(groups[i]);

                rawClusters[i]      = new TriangleCluster(triangleSet, bounds, bounds);
                enlargedClusters[i] = new TriangleCluster(triangleSet, bounds, bounds.Enlarge(_FinalEnlargement));
            }

            return (rawClusters, enlargedClusters);
        }

        #endregion

    }
}