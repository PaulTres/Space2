using UnityEngine;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.Boxes
{
    //One free box, in cells of its root's lattice, root-local, lower corner inclusive and upper exclusive.
    internal readonly struct BoxCells
    {
        public Vector3Int Min { get; }
        public Vector3Int Max { get; }

        public BoxCells(Vector3Int _Min, Vector3Int _Max)
        {
            Min = _Min;
            Max = _Max;
        }
    }

    //The rectangle two boxes share. A and B are local box indices: within one root for an inner portal, in the lower
    //root and in the upper root of a face table. The rectangle is in world space and flat along Axis.
    internal readonly struct BoxPortal
    {
        public int    A    { get; }
        public int    B    { get; }
        public int    Axis { get; }
        public Bounds Rect { get; }

        public BoxPortal(int _A, int _B, int _Axis, Bounds _Rect)
        {
            A    = _A;
            B    = _B;
            Axis = _Axis;
            Rect = _Rect;
        }
    }

    //The free space of one octree root partitioned into boxes: every box an exact union of free cells of the root's own
    //lattice (the size of its smallest leaf), so it is convex and free and never crosses the root. Inner portals join
    //two of its boxes; the boxes touching each of its six faces are what the face tables between neighbouring roots
    //are built from. Immutable once built; a publish that rebuilds the root replaces the record whole.
    internal sealed class BoxRootRecord
    {
        public BoxRootRecord(Vector3Int _RootIndex, int _Layer, float _RootSize, BoxCells[] _Boxes, BoxPortal[] _InnerPortals, int[][] _FaceBoxes)
        {
            RootIndex    = _RootIndex;
            Layer        = _Layer;
            RootSize     = _RootSize;
            Boxes        = _Boxes;
            InnerPortals = _InnerPortals;
            FaceBoxes    = _FaceBoxes;
            PortalsOfBox = IndexPortals(_Boxes.Length, _InnerPortals);
        }

        public Vector3Int  RootIndex    { get; }
        public int         Layer        { get; }
        public float       RootSize     { get; }
        public BoxCells[]  Boxes        { get; }
        public BoxPortal[] InnerPortals { get; }

        //per face, in the order -x, +x, -y, +y, -z, +z (face = axis * 2 + (upper ? 1 : 0)): the boxes lying on it
        public int[][] FaceBoxes { get; }

        //per box, the indices of the inner portals it takes part in — the search's adjacency inside the root
        public int[][] PortalsOfBox { get; }

        public int   Cells => 1 << Layer;
        public float Pitch => RootSize / Cells;

        public Vector3 Origin => (Vector3) RootIndex * RootSize;

        //the box holding the point (on a shared face, the first found), -1 when the point is outside the root or in no box
        public int Locate(Vector3 _Point)
        {
            Vector3 local = (_Point - Origin) / Pitch;

            for (int axis = 0; axis < 3; axis++)
            {
                if (local[axis] < 0f || local[axis] > Cells)
                    return -1;
            }

            for (int box = 0; box < Boxes.Length; box++)
            {
                BoxCells cells = Boxes[box];

                if (local.x >= cells.Min.x && local.x <= cells.Max.x &&
                    local.y >= cells.Min.y && local.y <= cells.Max.y &&
                    local.z >= cells.Min.z && local.z <= cells.Max.z)
                    return box;
            }

            return -1;
        }

        public Bounds WorldBounds(int _Box)
        {
            BoxCells box = Boxes[_Box];
            Vector3  min = Origin + (Vector3) box.Min * Pitch;
            Vector3  max = Origin + (Vector3) box.Max * Pitch;

            return new Bounds((min + max) * 0.5f, max - min);
        }

        static int[][] IndexPortals(int _Boxes, BoxPortal[] _Portals)
        {
            int[] counts = new int[_Boxes];

            foreach (BoxPortal portal in _Portals)
            {
                counts[portal.A]++;
                counts[portal.B]++;
            }

            int[][] result = new int[_Boxes][];

            for (int box = 0; box < _Boxes; box++)
                result[box] = new int[counts[box]];

            for (int portal = 0; portal < _Portals.Length; portal++)
            {
                BoxPortal p = _Portals[portal];

                result[p.A][--counts[p.A]] = portal;
                result[p.B][--counts[p.B]] = portal;
            }

            return result;
        }
    }
}
