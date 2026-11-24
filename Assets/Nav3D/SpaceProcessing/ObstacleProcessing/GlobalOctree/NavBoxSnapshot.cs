using System.Collections.Generic;
using UnityEngine;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree
{
    //The box graph copied out (NavTree.SnapshotBoxes): every free box, and every portal as the edge a route over boxes
    //takes — box centre, the centre of the rectangle the two boxes share, the other box's centre — with that rectangle.
    //Scope limits the copy to the roots it intersects; null means every root.
    sealed class NavBoxSnapshot
    {
        public NavBoxSnapshot(Bounds? _Scope)
        {
            Scope = _Scope;
        }

        public Bounds? Scope { get; }

        public List<Bounds>       Boxes       { get; } = new List<Bounds>();
        public List<NavGraphEdge> Portals     { get; } = new List<NavGraphEdge>();
        public List<Bounds>       PortalRects { get; } = new List<Bounds>();
    }
}
