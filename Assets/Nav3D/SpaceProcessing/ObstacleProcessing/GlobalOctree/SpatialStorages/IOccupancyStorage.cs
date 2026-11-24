using UnityEngine;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.SpatialStorages
{
    //What a construction asks of the geometry it builds against: whether a triangle crosses a cell, and the bodies a free
    //cell is classified against. A TriangleStorage answers it over one triangle set; a ClusterForestStorage over the
    //surviving clusters' own parts, with no triangle set of the operation's own.
    internal interface IOccupancyStorage
    {
        bool IsIntersect(Bounds _Bounds);

        SolidRegions Solids { get; }
    }
}