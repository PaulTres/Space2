using Nav3D.Common;
using System;
using System.Collections.Generic;
using UnityEngine;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.UtilityTypes
{
    //Closes the cracks of a welded mesh before its shells are judged: where a boundary runs back alongside a boundary closer
    //than the tolerance — the two sides of a crack, an unwelded seam, a T-junction — the two are joined, vertices merged and
    //an edge split where the other side has a vertex along it. No navigable cell fits through a gap that narrow, so the
    //mesh encloses what it would enclose without it.
    //Only boundaries running in OPPOSITE directions are joined, and never two vertices one edge already connects: a hole's
    //rim runs one way all round, so a terrain's rim or a doorway never folds onto itself, and a wide opening has no
    //opposite side within reach.
    static class ShellGapZipper
    {
        #region Constants

        //the cosine below which two boundary directions count as running against each other
        const float ANTIPARALLEL_COS = -0.5f;

        //a split point this close to an edge's end is that end, which the merge handles
        const float SPLIT_END_MARGIN = 0.02f;

        //the hash reach in cells around a vertex; edges are sampled at half a cell, so two cells cover the tolerance
        const int HASH_REACH = 2;

        #endregion

        #region Nested types

        struct BoundaryEdge
        {
            public int From;
            public int To;
            public int Triangle;
        }

        //Merged vertices as disjoint sets whose spread stays under the tolerance, so a chain of near vertices cannot pull a
        //whole boundary into one point.
        sealed class WeldGroups
        {
            readonly Dictionary<int, int>                   m_Parent = new Dictionary<int, int>();
            readonly Dictionary<int, (Vector3 Min, Vector3 Max)> m_Extent = new Dictionary<int, (Vector3, Vector3)>();
            readonly List<Vector3>                          m_Positions;

            public bool Changed { get; private set; }

            public WeldGroups(List<Vector3> _Positions)
            {
                m_Positions = _Positions;
            }

            public void TryUnion(int _A, int _B, float _Tolerance)
            {
                int rootA = Find(_A);
                int rootB = Find(_B);

                if (rootA == rootB)
                    return;

                (Vector3 minA, Vector3 maxA) = ExtentOf(rootA);
                (Vector3 minB, Vector3 maxB) = ExtentOf(rootB);

                Vector3 min = Vector3.Min(minA, minB);
                Vector3 max = Vector3.Max(maxA, maxB);
                Vector3 size = max - min;

                if (Mathf.Max(size.x, Mathf.Max(size.y, size.z)) >= _Tolerance)
                    return;

                m_Parent[rootA] = rootB;
                m_Extent[rootB] = (min, max);
                Changed         = true;
            }

            //every corner to its group's root, and each root to the mean of its group
            public void Apply(List<Vector3> _Positions, List<int> _Corners)
            {
                if (!Changed)
                    return;

                Dictionary<int, (Vector3 Sum, int Count)> means = new Dictionary<int, (Vector3, int)>();

                foreach (int vertex in new List<int>(m_Parent.Keys))
                {
                    int root = Find(vertex);

                    means.TryGetValue(root, out (Vector3 Sum, int Count) mean);
                    means[root] = (mean.Sum + m_Positions[vertex], mean.Count + 1);
                }

                for (int i = 0; i < _Corners.Count; i++)
                    _Corners[i] = Find(_Corners[i]);

                //a final root is never a child, so its own position is added here
                foreach (KeyValuePair<int, (Vector3 Sum, int Count)> mean in means)
                    _Positions[mean.Key] = (mean.Value.Sum + m_Positions[mean.Key]) / (mean.Value.Count + 1);
            }

            int Find(int _Vertex)
            {
                while (m_Parent.TryGetValue(_Vertex, out int parent) && parent != _Vertex)
                {
                    if (m_Parent.TryGetValue(parent, out int grand) && grand != parent)
                        m_Parent[_Vertex] = grand;

                    _Vertex = parent;
                }

                return _Vertex;
            }

            (Vector3 Min, Vector3 Max) ExtentOf(int _Root)
            {
                return m_Extent.TryGetValue(_Root, out (Vector3 Min, Vector3 Max) extent) ? extent : (m_Positions[_Root], m_Positions[_Root]);
            }
        }

        #endregion

        #region Public methods

        //_Corners holds three vertex ids per triangle, _Origins the source triangle of each, _Positions the welded vertex
        //positions; _BoundaryEdges every edge walked more often one way than the other, in its unmatched direction, with a
        //triangle that walks it so. On a change the lists are rewritten in place — split triangles replaced by fans,
        //merged vertices renumbered to one id at their mean position — and degenerate triangles are left for the caller.
        public static bool Zip(
                List<Vector3>      _Positions,
                List<Triangle>     _Origins,
                List<int>          _Corners,
                List<(int From, int To, int Triangle)> _BoundaryEdges,
                Func<int, int, bool> _AreConnected,
                float              _Tolerance
            )
        {
            if (_BoundaryEdges.Count == 0 || _Tolerance <= 0f)
                return false;

            List<BoundaryEdge> edges = new List<BoundaryEdge>(_BoundaryEdges.Count);

            foreach ((int from, int to, int triangle) in _BoundaryEdges)
                edges.Add(new BoundaryEdge { From = from, To = to, Triangle = triangle });

            Dictionary<Vector3Int, List<int>> hash      = HashEdges(edges, _Positions, _Tolerance);
            Dictionary<int, Vector3>          tangents  = Tangents(edges, _Positions);
            WeldGroups                        groups    = new WeldGroups(_Positions);
            Dictionary<int, List<(int Corner, float T, int Vertex)>> splits = new Dictionary<int, List<(int, float, int)>>();

            int[] stamps = new int[edges.Count];
            int   stamp  = 0;

            List<int> vertices = new List<int>(tangents.Keys);
            vertices.Sort();

            foreach (int vertex in vertices)
            {
                Vector3 tangent = tangents[vertex];

                //a vertex where the boundary turns back on itself (a crack's closed end) has no direction to match
                if (tangent.sqrMagnitude < 0.25f)
                    continue;

                tangent.Normalize();

                Vector3 point = _Positions[vertex];

                int   weldTo       = -1;
                float weldDistance = _Tolerance;
                int   splitEdge    = -1;
                float splitT       = 0f;
                float splitDistance = _Tolerance;

                stamp++;

                Vector3Int cell = CellOf(point, _Tolerance);

                for (int x = -HASH_REACH; x <= HASH_REACH; x++)
                for (int y = -HASH_REACH; y <= HASH_REACH; y++)
                for (int z = -HASH_REACH; z <= HASH_REACH; z++)
                {
                    if (!hash.TryGetValue(cell + new Vector3Int(x, y, z), out List<int> candidates))
                        continue;

                    foreach (int index in candidates)
                    {
                        if (stamps[index] == stamp)
                            continue;

                        stamps[index] = stamp;

                        BoundaryEdge edge = edges[index];

                        if (edge.From == vertex || edge.To == vertex)
                            continue;

                        Vector3 a = _Positions[edge.From];
                        Vector3 b = _Positions[edge.To];

                        Vector3 along  = b - a;
                        float   length = along.magnitude;

                        if (length <= 0f || Vector3.Dot(tangent, along / length) > ANTIPARALLEL_COS)
                            continue;

                        float   t       = Mathf.Clamp01(Vector3.Dot(point - a, along) / (length * length));
                        float   distance = Vector3.Distance(point, a + along * t);

                        if (distance >= _Tolerance)
                            continue;

                        foreach (int end in new[] { edge.From, edge.To })
                        {
                            float endDistance = Vector3.Distance(point, _Positions[end]);

                            if (endDistance < weldDistance && !_AreConnected(vertex, end))
                            {
                                weldTo       = end;
                                weldDistance = endDistance;
                            }
                        }

                        if (t > SPLIT_END_MARGIN && t < 1f - SPLIT_END_MARGIN && distance < splitDistance)
                        {
                            splitEdge     = index;
                            splitT        = t;
                            splitDistance = distance;
                        }
                    }
                }

                if (weldTo >= 0)
                {
                    groups.TryUnion(vertex, weldTo, _Tolerance);

                    continue;
                }

                if (splitEdge < 0)
                    continue;

                BoundaryEdge split = edges[splitEdge];

                if (!splits.TryGetValue(split.Triangle, out List<(int Corner, float T, int Vertex)> list))
                    splits.Add(split.Triangle, list = new List<(int, float, int)>());

                list.Add((CornerOf(_Corners, split.Triangle, split.From), splitT, vertex));
            }

            if (!groups.Changed && splits.Count == 0)
                return false;

            ApplySplits(_Positions, _Origins, _Corners, splits);
            groups.Apply(_Positions, _Corners);

            return true;
        }

        #endregion

        #region Service methods

        //each edge under every cell its half-cell samples fall in, so a query two cells round a vertex meets every edge
        //passing within the tolerance of it
        static Dictionary<Vector3Int, List<int>> HashEdges(List<BoundaryEdge> _Edges, List<Vector3> _Positions, float _Tolerance)
        {
            Dictionary<Vector3Int, List<int>> hash = new Dictionary<Vector3Int, List<int>>();

            for (int index = 0; index < _Edges.Count; index++)
            {
                Vector3 a = _Positions[_Edges[index].From];
                Vector3 b = _Positions[_Edges[index].To];

                int steps = Mathf.Max(1, Mathf.CeilToInt(Vector3.Distance(a, b) / (_Tolerance * 0.5f)));

                for (int step = 0; step <= steps; step++)
                {
                    Vector3Int cell = CellOf(Vector3.Lerp(a, b, (float) step / steps), _Tolerance);

                    if (!hash.TryGetValue(cell, out List<int> list))
                        hash.Add(cell, list = new List<int>());

                    if (list.Count == 0 || list[list.Count - 1] != index)
                        list.Add(index);
                }
            }

            return hash;
        }

        //the direction the boundary runs through each of its vertices: in along the incoming edges, out along the outgoing
        static Dictionary<int, Vector3> Tangents(List<BoundaryEdge> _Edges, List<Vector3> _Positions)
        {
            Dictionary<int, Vector3> tangents = new Dictionary<int, Vector3>();

            foreach (BoundaryEdge edge in _Edges)
            {
                Vector3 direction = (_Positions[edge.To] - _Positions[edge.From]).normalized;

                tangents.TryGetValue(edge.From, out Vector3 from);
                tangents[edge.From] = from + direction;

                tangents.TryGetValue(edge.To, out Vector3 to);
                tangents[edge.To] = to + direction;
            }

            return tangents;
        }

        static Vector3Int CellOf(Vector3 _Point, float _Size)
        {
            return new Vector3Int(Mathf.FloorToInt(_Point.x / _Size), Mathf.FloorToInt(_Point.y / _Size), Mathf.FloorToInt(_Point.z / _Size));
        }

        static int CornerOf(List<int> _Corners, int _Triangle, int _Vertex)
        {
            for (int k = 0; k < 3; k++)
                if (_Corners[_Triangle * 3 + k] == _Vertex)
                    return k;

            return 0;
        }

        //A triangle with split points on its edges becomes a fan from its centroid over the polygon they make — the same
        //surface, walked the same way round, with the other side's vertices now on its boundary.
        static void ApplySplits(
                List<Vector3>  _Positions,
                List<Triangle> _Origins,
                List<int>      _Corners,
                Dictionary<int, List<(int Corner, float T, int Vertex)>> _Splits
            )
        {
            List<int> triangles = new List<int>(_Splits.Keys);
            triangles.Sort();

            foreach (int triangle in triangles)
            {
                List<(int Corner, float T, int Vertex)> points = _Splits[triangle];
                points.Sort((_A, _B) => _A.Corner != _B.Corner ? _A.Corner.CompareTo(_B.Corner) : _A.T.CompareTo(_B.T));

                List<int> polygon = new List<int>();
                Vector3   centroid = Vector3.zero;

                for (int k = 0; k < 3; k++)
                {
                    int corner = _Corners[triangle * 3 + k];

                    polygon.Add(corner);
                    centroid += _Positions[corner];

                    foreach ((int edgeCorner, float _, int vertex) in points)
                        if (edgeCorner == k)
                            polygon.Add(vertex);
                }

                int apex = _Positions.Count;
                _Positions.Add(centroid / 3f);

                for (int i = 0; i < polygon.Count; i++)
                {
                    int from = polygon[i];
                    int to   = polygon[(i + 1) % polygon.Count];

                    if (i == 0)
                    {
                        _Corners[triangle * 3]     = from;
                        _Corners[triangle * 3 + 1] = to;
                        _Corners[triangle * 3 + 2] = apex;

                        continue;
                    }

                    _Corners.Add(from);
                    _Corners.Add(to);
                    _Corners.Add(apex);
                    _Origins.Add(_Origins[triangle]);
                }
            }
        }

        #endregion
    }
}
