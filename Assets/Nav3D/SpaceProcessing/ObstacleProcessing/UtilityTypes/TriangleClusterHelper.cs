using Nav3D.Common;
using System.Collections.Generic;
using UnityEngine;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.UtilityTypes
{
    static class TriangleClusterHelper
    {
        public static HashSet<Triangle> GetCrossedTriangles(
                BoundablesSpatialHashMap<TriangleCluster> _TriangleClusterStorage,
                Bounds[]                                  _CrossingBounds
            )
        {
            HashSet<Triangle> result = new HashSet<Triangle>();

            //Collect all unique clusters across all bounds first, then batch-query each cluster once
            Dictionary<TriangleCluster, List<Bounds>> clusterToBounds = new Dictionary<TriangleCluster, List<Bounds>>();

            for (int i = 0; i < _CrossingBounds.Length; i++)
            {
                if (!_TriangleClusterStorage.TryGetCrossingBoundables(_CrossingBounds[i], out HashSet<TriangleCluster> crossingClusters))
                    continue;

                foreach (TriangleCluster cluster in crossingClusters)
                {
                    if (!clusterToBounds.TryGetValue(cluster, out List<Bounds> boundsList))
                    {
                        boundsList               = new List<Bounds>();
                        clusterToBounds[cluster] = boundsList;
                    }
                    boundsList.Add(_CrossingBounds[i]);
                }
            }

            foreach (KeyValuePair<TriangleCluster, List<Bounds>> kvp in clusterToBounds)
            {
                result.UnionWith(kvp.Key.GetCrossedTriangles(kvp.Value.ToArray()));
            }

            return result;
        }
    }
}