using Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.Elements;
using System;
using System.Collections.Generic;
using UnityEngine;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.Boxes
{
    //Partitions a root's free space into boxes, straight from its leaves: a bit mask of the root's lattice, one bit a cell
    //and a row along z packed into ulongs, is opened whole and closed under every occupied leaf; the open cells — free
    //leaves and the void where the root holds no node alike — are then grown greedily into boxes (a run along z, rows
    //along y, slabs along x). Portals are found by a sweep per face plane. Pure: reads the subtree it is given and
    //nothing else, so it runs on a staged root outside the tree lock.
    internal static class BoxPartition
    {
        #region Nested types

        //per-thread buffers, reused across builds: the mask of a 128-cell lattice alone is 256 KB
        sealed class Scratch
        {
            public ulong[]                                 Mask        { get; set; } = Array.Empty<ulong>();
            public List<(Vector3Int Index, int Layer)>     Occupied    { get; } = new List<(Vector3Int, int)>();
            public List<BoxCells>                          Boxes       { get; } = new List<BoxCells>();
            public List<BoxPortal>                         Portals     { get; } = new List<BoxPortal>();
            public List<(int Start, bool Upper, int Box)>  Events      { get; } = new List<(int, bool, int)>();
            public List<int>                               ActiveUpper { get; } = new List<int>();
            public List<int>                               ActiveLower { get; } = new List<int>();
            public Stack<Node>                             Walk        { get; } = new Stack<Node>();
        }

        #endregion

        #region Constants

        const int FACES = 6;

        static readonly int[] s_DeBruijn =
        {
            0, 1, 48, 2, 57, 49, 28, 3, 61, 58, 50, 42, 38, 29, 17, 4, 62, 55, 59, 36, 53, 51, 43, 22, 45, 39, 33, 30, 24, 18, 12, 5,
            63, 47, 56, 27, 60, 41, 37, 16, 54, 35, 52, 21, 44, 32, 23, 11, 46, 26, 40, 15, 34, 20, 31, 10, 25, 14, 19, 9, 13, 8, 7, 6,
        };

        static readonly Comparison<(int Start, bool Upper, int Box)> s_ByStart = (_A, _B) => _A.Start.CompareTo(_B.Start);

        #endregion

        #region Attributes

        [ThreadStatic] static Scratch t_Scratch;

        #endregion

        #region Public methods

        public static BoxRootRecord Build(Node _Root, float _RootSize)
        {
            Scratch scratch = t_Scratch ??= new Scratch();

            Vector3Int rootIndex = _Root.Index;
            int        layer     = CollectOccupied(_Root, scratch);
            int        n         = 1 << layer;
            int        w         = (n + 63) / 64;
            int        words     = n * n * w;

            if (scratch.Mask.Length < words)
                scratch.Mask = new ulong[words];

            ulong[] mask = scratch.Mask;

            for (int row = 0; row < n * n; row++)
                SetRange(mask, row * w, 0, n);

            foreach ((Vector3Int index, int leafLayer) in scratch.Occupied)
            {
                int        shift = layer - leafLayer;
                int        span  = 1 << shift;
                Vector3Int min   = (index - rootIndex * (1 << leafLayer)) * span;

                for (int x = min.x; x < min.x + span; x++)
                for (int y = min.y; y < min.y + span; y++)
                    ClearRange(mask, (x * n + y) * w, min.z, min.z + span);
            }

            List<BoxCells> boxes = scratch.Boxes;

            boxes.Clear();

            for (int x = 0; x < n; x++)
            for (int y = 0; y < n; y++)
            {
                int row = (x * n + y) * w;

                for (int z0 = FirstSet(mask, row, 0, n); z0 >= 0; z0 = FirstSet(mask, row, z0, n))
                {
                    int z1 = FirstClear(mask, row, z0, n);
                    int y1 = y + 1;

                    while (y1 < n && HasRange(mask, (x * n + y1) * w, z0, z1))
                        y1++;

                    int x1 = x + 1;

                    while (x1 < n && SlabOpen(mask, n, w, x1, y, y1, z0, z1))
                        x1++;

                    for (int xx = x; xx < x1; xx++)
                    for (int yy = y; yy < y1; yy++)
                        ClearRange(mask, (xx * n + yy) * w, z0, z1);

                    boxes.Add(new BoxCells(new Vector3Int(x, y, z0), new Vector3Int(x1, y1, z1)));
                }
            }

            BoxCells[] boxArray = boxes.ToArray();
            float      pitch    = _RootSize / n;
            Vector3    origin   = (Vector3) rootIndex * _RootSize;

            return new BoxRootRecord(rootIndex, layer, _RootSize, boxArray, InnerPortals(boxArray, n, pitch, origin, scratch), FaceBoxes(boxArray, n));
        }

        //the portals across the face between _Lower and the root above it along _Axis; A indexes _Lower's boxes, B _Upper's
        public static BoxPortal[] FacePortals(BoxRootRecord _Lower, BoxRootRecord _Upper, int _Axis)
        {
            int[] lowerFace = _Lower.FaceBoxes[_Axis * 2 + 1];
            int[] upperFace = _Upper.FaceBoxes[_Axis * 2];

            if (lowerFace.Length == 0 || upperFace.Length == 0)
                return Array.Empty<BoxPortal>();

            //both sides on the finer of the two lattices
            int   layer      = Math.Max(_Lower.Layer, _Upper.Layer);
            int   lowerShift = layer - _Lower.Layer;
            int   upperShift = layer - _Upper.Layer;
            float pitch      = _Lower.RootSize / (1 << layer);
            int   u          = (_Axis + 1) % 3, v = (_Axis + 2) % 3;
            float plane      = (_Lower.RootIndex[_Axis] + 1) * _Lower.RootSize;

            //the upper root's face cells, offset into the lower root's frame (the two share u and v)
            List<BoxPortal> portals = new List<BoxPortal>();

            foreach (int a in lowerFace)
            {
                BoxCells lower = _Lower.Boxes[a];

                foreach (int b in upperFace)
                {
                    BoxCells upper = _Upper.Boxes[b];

                    int uLo = Math.Max(lower.Min[u] << lowerShift, upper.Min[u] << upperShift);
                    int uHi = Math.Min(lower.Max[u] << lowerShift, upper.Max[u] << upperShift);

                    if (uLo >= uHi)
                        continue;

                    int vLo = Math.Max(lower.Min[v] << lowerShift, upper.Min[v] << upperShift);
                    int vHi = Math.Min(lower.Max[v] << lowerShift, upper.Max[v] << upperShift);

                    if (vLo >= vHi)
                        continue;

                    portals.Add(new BoxPortal(a, b, _Axis, Rect(_Axis, plane, _Lower.Origin, pitch, uLo, uHi, vLo, vHi)));
                }
            }

            return portals.ToArray();
        }

        #endregion

        #region Service methods

        //the occupied leaves as (grid index, layer), and the layer of the root's smallest leaf — its lattice
        static int CollectOccupied(Node _Root, Scratch _Scratch)
        {
            _Scratch.Occupied.Clear();
            _Scratch.Walk.Clear();
            _Scratch.Walk.Push(_Root);

            int layer = 0;

            while (_Scratch.Walk.Count > 0)
            {
                Node node = _Scratch.Walk.Pop();

                if (node is Fork fork)
                {
                    foreach (Node child in fork.ChildrenMap.Values)
                        _Scratch.Walk.Push(child);

                    continue;
                }

                if (!(node is Leaf leaf))
                    continue;

                layer = Math.Max(layer, leaf.GridLayer);

                if (leaf.Occupied)
                    _Scratch.Occupied.Add((leaf.Index, leaf.GridLayer));
            }

            return layer;
        }

        //per axis, the boxes whose upper face lies in a plane against those whose lower face does, swept along u, so a
        //pair is looked at only while both u-intervals are open. Planes on the root's own faces are left to the face tables
        static BoxPortal[] InnerPortals(BoxCells[] _Boxes, int _Cells, float _Pitch, Vector3 _Origin, Scratch _Scratch)
        {
            List<BoxPortal> portals = _Scratch.Portals;

            portals.Clear();

            for (int axis = 0; axis < 3; axis++)
            {
                int u = (axis + 1) % 3, v = (axis + 2) % 3;

                Dictionary<int, (List<int> Upper, List<int> Lower)> planes = new Dictionary<int, (List<int>, List<int>)>();

                (List<int> Upper, List<int> Lower) PlaneOf(int _Plane)
                {
                    if (!planes.TryGetValue(_Plane, out (List<int> Upper, List<int> Lower) lists))
                        planes[_Plane] = lists = (new List<int>(), new List<int>());

                    return lists;
                }

                for (int i = 0; i < _Boxes.Length; i++)
                {
                    if (_Boxes[i].Max[axis] < _Cells)
                        PlaneOf(_Boxes[i].Max[axis]).Upper.Add(i);

                    if (_Boxes[i].Min[axis] > 0)
                        PlaneOf(_Boxes[i].Min[axis]).Lower.Add(i);
                }

                foreach (KeyValuePair<int, (List<int> Upper, List<int> Lower)> plane in planes)
                {
                    if (plane.Value.Upper.Count == 0 || plane.Value.Lower.Count == 0)
                        continue;

                    List<(int Start, bool Upper, int Box)> events = _Scratch.Events;

                    events.Clear();
                    _Scratch.ActiveUpper.Clear();
                    _Scratch.ActiveLower.Clear();

                    foreach (int box in plane.Value.Upper)
                        events.Add((_Boxes[box].Min[u], true, box));

                    foreach (int box in plane.Value.Lower)
                        events.Add((_Boxes[box].Min[u], false, box));

                    events.Sort(s_ByStart);

                    foreach ((int Start, bool Upper, int Box) entering in events)
                    {
                        List<int> opposite = entering.Upper ? _Scratch.ActiveLower : _Scratch.ActiveUpper;

                        for (int k = opposite.Count - 1; k >= 0; k--)
                        {
                            int other = opposite[k];

                            //closed before this one opens, so it meets nothing that opens later either
                            if (_Boxes[other].Max[u] <= entering.Start)
                            {
                                opposite[k] = opposite[opposite.Count - 1];

                                opposite.RemoveAt(opposite.Count - 1);

                                continue;
                            }

                            BoxCells a = _Boxes[entering.Box], b = _Boxes[other];

                            int vLo = Math.Max(a.Min[v], b.Min[v]), vHi = Math.Min(a.Max[v], b.Max[v]);

                            if (vLo >= vHi)
                                continue;

                            int uLo = Math.Max(a.Min[u], b.Min[u]), uHi = Math.Min(a.Max[u], b.Max[u]);

                            int lowerBox = entering.Upper ? entering.Box : other;
                            int upperBox = entering.Upper ? other : entering.Box;

                            portals.Add(new BoxPortal(lowerBox, upperBox, axis, Rect(axis, _Origin[axis] + plane.Key * _Pitch, _Origin, _Pitch, uLo, uHi, vLo, vHi)));
                        }

                        (entering.Upper ? _Scratch.ActiveUpper : _Scratch.ActiveLower).Add(entering.Box);
                    }
                }
            }

            return portals.ToArray();
        }

        static int[][] FaceBoxes(BoxCells[] _Boxes, int _Cells)
        {
            List<int>[] faces = new List<int>[FACES];

            for (int face = 0; face < FACES; face++)
                faces[face] = new List<int>();

            for (int i = 0; i < _Boxes.Length; i++)
            {
                for (int axis = 0; axis < 3; axis++)
                {
                    if (_Boxes[i].Min[axis] == 0)
                        faces[axis * 2].Add(i);

                    if (_Boxes[i].Max[axis] == _Cells)
                        faces[axis * 2 + 1].Add(i);
                }
            }

            int[][] result = new int[FACES][];

            for (int face = 0; face < FACES; face++)
                result[face] = faces[face].ToArray();

            return result;
        }

        static Bounds Rect(int _Axis, float _Plane, Vector3 _Origin, float _Pitch, int _ULo, int _UHi, int _VLo, int _VHi)
        {
            int     u   = (_Axis + 1) % 3, v = (_Axis + 2) % 3;
            Vector3 min = Vector3.zero, max = Vector3.zero;

            min[_Axis] = max[_Axis] = _Plane;
            min[u]     = _Origin[u] + _ULo * _Pitch;
            max[u]     = _Origin[u] + _UHi * _Pitch;
            min[v]     = _Origin[v] + _VLo * _Pitch;
            max[v]     = _Origin[v] + _VHi * _Pitch;

            return new Bounds((min + max) * 0.5f, max - min);
        }

        static bool SlabOpen(ulong[] _Mask, int _Cells, int _Words, int _X, int _Y0, int _Y1, int _Z0, int _Z1)
        {
            for (int y = _Y0; y < _Y1; y++)
            {
                if (!HasRange(_Mask, (_X * _Cells + y) * _Words, _Z0, _Z1))
                    return false;
            }

            return true;
        }

        static void SetRange(ulong[] _Bits, int _Row, int _From, int _To)
        {
            for (int word = _From >> 6; word <= (_To - 1) >> 6; word++)
                _Bits[_Row + word] |= WordMask(word, _From, _To);
        }

        static void ClearRange(ulong[] _Bits, int _Row, int _From, int _To)
        {
            for (int word = _From >> 6; word <= (_To - 1) >> 6; word++)
                _Bits[_Row + word] &= ~WordMask(word, _From, _To);
        }

        static bool HasRange(ulong[] _Bits, int _Row, int _From, int _To)
        {
            for (int word = _From >> 6; word <= (_To - 1) >> 6; word++)
            {
                ulong wanted = WordMask(word, _From, _To);

                if ((_Bits[_Row + word] & wanted) != wanted)
                    return false;
            }

            return true;
        }

        //the bits of one word that fall in [_From, _To)
        static ulong WordMask(int _Word, int _From, int _To)
        {
            int lo = Math.Max(_From - (_Word << 6), 0);
            int hi = Math.Min(_To - (_Word << 6), 64);

            ulong upper = hi == 64 ? ulong.MaxValue : (1UL << hi) - 1UL;
            ulong lower = (1UL << lo) - 1UL;

            return upper & ~lower;
        }

        static int FirstSet(ulong[] _Bits, int _Row, int _From, int _Cells)
        {
            for (int word = _From >> 6; word <= (_Cells - 1) >> 6; word++)
            {
                ulong bits = _Bits[_Row + word] & WordMask(word, _From, _Cells);

                if (bits != 0UL)
                    return (word << 6) + LowestBit(bits);
            }

            return -1;
        }

        static int FirstClear(ulong[] _Bits, int _Row, int _From, int _Cells)
        {
            for (int word = _From >> 6; word <= (_Cells - 1) >> 6; word++)
            {
                ulong bits = ~_Bits[_Row + word] & WordMask(word, _From, _Cells);

                if (bits != 0UL)
                    return (word << 6) + LowestBit(bits);
            }

            return _Cells;
        }

        static int LowestBit(ulong _Bits)
        {
            return s_DeBruijn[((_Bits & (ulong) -(long) _Bits) * 0x03F79D71B4CB0A89UL) >> 58];
        }

        #endregion
    }
}
