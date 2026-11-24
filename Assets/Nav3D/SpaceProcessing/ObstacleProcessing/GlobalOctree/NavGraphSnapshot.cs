using System.Collections.Generic;
using UnityEngine;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree
{
    //One stored connection of the navigation graph, copied out: a free leaf's centre, the contact point on the face it
    //shares with a free neighbour, and that neighbour's centre — the two segments the point-lane search reconstructs
    //for an in-region edge. Query-time successors (the straight shot toward a goal, radius-aware portals) are not
    //stored anywhere and so are never in a snapshot.
    readonly struct NavGraphEdge
    {
        public Vector3 From    { get; }
        public Vector3 Contact { get; }
        public Vector3 To      { get; }

        public NavGraphEdge(Vector3 _From, Vector3 _Contact, Vector3 _To)
        {
            From    = _From;
            Contact = _Contact;
            To      = _To;
        }
    }

    //The graph half of a leaf snapshot (NavTree.SnapshotLeaves), filled under the same per-root read-lock holds.
    //Each undirected pair appears once. Scope limits the walk to the roots it intersects — the whole point of a scoped
    //graph, since the edges outnumber the leaves several times over; null means every root.
    sealed class NavGraphSnapshot
    {
        public NavGraphSnapshot(Bounds? _Scope)
        {
            Scope = _Scope;
        }

        public Bounds? Scope { get; }

        public List<NavGraphEdge> Edges { get; } = new List<NavGraphEdge>();

        //free leaves with a face opening onto open air (Leaf.IsEdgeNode): where the search may leave for a straight
        //shot toward its goal. not NavLeafInfo.OpenFacesMask, which also counts faces onto free leaves.
        public List<Bounds> EdgeNodes { get; } = new List<Bounds>();
    }
}
