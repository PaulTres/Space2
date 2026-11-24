using Nav3D.Common;
using System;
using System.Collections.Generic;
using UnityEngine;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.UtilityTypes
{
    //An index over the raw bounds of a set of parts: a small flat BVH, so a box or a ray reaches the few parts it crosses
    //instead of every part of a body. A pavilion's cluster is ~200 parts, and construction asks per cell and the interior
    //winding per free cell, so the per-part loop was the cost; the index is built once per part set, in microseconds, and
    //never reorders the array it indexes (cluster versions share their part arrays by reference).
    internal sealed class PartForest
    {

        #region Constants

        const int MAX_LEAF_PARTS = 4;

        #endregion

        #region Nested types

        struct ForestNode
        {
            public Vector3 Min { get; set; }
            public Vector3 Max { get; set; }

            //a leaf holds parts by their position in m_Order; an inner node its two children
            public int StartOrLeft  { get; set; }
            public int CountOrRight { get; set; }

            public bool IsLeaf     => CountOrRight > 0;
            public int  Start      => StartOrLeft;
            public int  Count      => CountOrRight;
            public int  LeftChild  => StartOrLeft;
            public int  RightChild => -CountOrRight;
        }

        #endregion

        #region Attributes

        readonly ClusterPart[] m_Parts;
        readonly int[]         m_Order;
        readonly ForestNode[]  m_Nodes;

        int m_NodeCount;

        #endregion

        #region Properties

        public IReadOnlyList<ClusterPart> Parts => m_Parts;

        public int Count => m_Parts.Length;

        #endregion

        #region Constructors

        public PartForest(ClusterPart[] _Parts)
        {
            m_Parts = _Parts;
            m_Order = new int[_Parts.Length];

            for (int i = 0; i < m_Order.Length; i++)
                m_Order[i] = i;

            m_Nodes = new ForestNode[Math.Max(1, 2 * _Parts.Length)];

            if (_Parts.Length > 0)
                Build(0, _Parts.Length);
        }

        #endregion

        #region Public methods

        //Does a triangle of any part cross the box — the construction's occupancy question.
        public bool AnyTriangleIntersects(Bounds _Bounds)
        {
            if (m_Parts.Length == 0)
                return false;

            Vector3 queryMin = _Bounds.min;
            Vector3 queryMax = _Bounds.max;

            Span<int> stack    = stackalloc int[64];
            int       stackPtr = 0;

            stack[stackPtr++] = 0;

            while (stackPtr > 0)
            {
                ref ForestNode node = ref m_Nodes[stack[--stackPtr]];

                if (Misses(node, queryMin, queryMax))
                    continue;

                if (!node.IsLeaf)
                {
                    stack[stackPtr++] = node.LeftChild;
                    stack[stackPtr++] = node.RightChild;

                    continue;
                }

                int end = node.Start + node.Count;

                for (int i = node.Start; i < end; i++)
                {
                    ClusterPart part = m_Parts[m_Order[i]];

                    if (part.Bounds.Intersects(_Bounds) && part.Storage.IsIntersect(_Bounds))
                        return true;
                }
            }

            return false;
        }

        //Every triangle of every part that crosses the box, appended to a caller-owned collection.
        public void CollectIntersectedTriangles(Bounds _Bounds, ICollection<Triangle> _Result)
        {
            if (m_Parts.Length == 0)
                return;

            Vector3 queryMin = _Bounds.min;
            Vector3 queryMax = _Bounds.max;

            Span<int> stack    = stackalloc int[64];
            int       stackPtr = 0;

            stack[stackPtr++] = 0;

            while (stackPtr > 0)
            {
                ref ForestNode node = ref m_Nodes[stack[--stackPtr]];

                if (Misses(node, queryMin, queryMax))
                    continue;

                if (!node.IsLeaf)
                {
                    stack[stackPtr++] = node.LeftChild;
                    stack[stackPtr++] = node.RightChild;

                    continue;
                }

                int end = node.Start + node.Count;

                for (int i = node.Start; i < end; i++)
                {
                    ClusterPart part = m_Parts[m_Order[i]];

                    if (part.Bounds.Intersects(_Bounds))
                        part.Storage.CollectIntersectedTriangles(_Bounds, _Result);
                }
            }
        }

        //Does a triangle outside _Except cross the box.
        public bool AnyTriangleIntersectsExcept(Bounds _Bounds, HashSet<Triangle> _Except)
        {
            if (_Except == null)
                return AnyTriangleIntersects(_Bounds);

            if (m_Parts.Length == 0)
                return false;

            Vector3 queryMin = _Bounds.min;
            Vector3 queryMax = _Bounds.max;

            Span<int> stack    = stackalloc int[64];
            int       stackPtr = 0;

            stack[stackPtr++] = 0;

            while (stackPtr > 0)
            {
                ref ForestNode node = ref m_Nodes[stack[--stackPtr]];

                if (Misses(node, queryMin, queryMax))
                    continue;

                if (!node.IsLeaf)
                {
                    stack[stackPtr++] = node.LeftChild;
                    stack[stackPtr++] = node.RightChild;

                    continue;
                }

                int end = node.Start + node.Count;

                for (int i = node.Start; i < end; i++)
                {
                    ClusterPart part = m_Parts[m_Order[i]];

                    if (!part.Bounds.Intersects(_Bounds))
                        continue;

                    foreach (Triangle triangle in part.Storage.GetIntersectedTriangles(_Bounds))
                        if (!_Except.Contains(triangle))
                            return true;
                }
            }

            return false;
        }

        //The nearest triangle the ray meets within _MaxDistance over every part, skipping those _Ignore accepts.
        public bool TryRaycastFirst(Vector3 _Origin, Vector3 _Direction, float _MaxDistance, Predicate<Triangle> _Ignore, out Triangle _Hit, out float _Distance)
        {
            _Hit      = null;
            _Distance = _MaxDistance;

            if (m_Parts.Length == 0)
                return false;

            double ox = _Origin.x,    oy = _Origin.y,    oz = _Origin.z;
            double dx = _Direction.x, dy = _Direction.y, dz = _Direction.z;

            double invX = 1.0 / dx, invY = 1.0 / dy, invZ = 1.0 / dz;

            Span<int> stack    = stackalloc int[64];
            int       stackPtr = 0;

            stack[stackPtr++] = 0;

            while (stackPtr > 0)
            {
                ref ForestNode node = ref m_Nodes[stack[--stackPtr]];

                //a node the ray does not enter within the best distance so far cannot hold a nearer hit
                if (!RayEntersBox(ox, oy, oz, invX, invY, invZ, node.Min, node.Max, out double entry) || entry > _Distance)
                    continue;

                if (!node.IsLeaf)
                {
                    stack[stackPtr++] = node.LeftChild;
                    stack[stackPtr++] = node.RightChild;

                    continue;
                }

                int end = node.Start + node.Count;

                for (int i = node.Start; i < end; i++)
                {
                    ClusterPart part = m_Parts[m_Order[i]];

                    if (!RayEntersBox(ox, oy, oz, invX, invY, invZ, part.Bounds.min, part.Bounds.max, out double partEntry) || partEntry > _Distance)
                        continue;

                    if (part.Storage.TryRaycastFirst(_Origin, _Direction, _Distance, _Ignore, out Triangle hit, out float distance) && distance < _Distance)
                    {
                        _Hit      = hit;
                        _Distance = distance;
                    }
                }
            }

            return _Hit != null;
        }

        //The signed crossings of the ray summed over every part its line reaches; false when any part reports a hit
        //too near an edge to count, so the caller tries the next direction. A part the ray's box test culls contributes
        //nothing — its own storage would have culled it at its root the same way.
        public bool TryCountSignedCrossings(Vector3 _Origin, Vector3 _Direction, out int _Winding)
        {
            _Winding = 0;

            if (m_Parts.Length == 0)
                return true;

            double ox = _Origin.x,    oy = _Origin.y,    oz = _Origin.z;
            double dx = _Direction.x, dy = _Direction.y, dz = _Direction.z;

            double invX = 1.0 / dx, invY = 1.0 / dy, invZ = 1.0 / dz;

            Span<int> stack    = stackalloc int[64];
            int       stackPtr = 0;

            stack[stackPtr++] = 0;

            while (stackPtr > 0)
            {
                ref ForestNode node = ref m_Nodes[stack[--stackPtr]];

                if (!RayEntersBox(ox, oy, oz, invX, invY, invZ, node.Min, node.Max, out _))
                    continue;

                if (!node.IsLeaf)
                {
                    stack[stackPtr++] = node.LeftChild;
                    stack[stackPtr++] = node.RightChild;

                    continue;
                }

                int end = node.Start + node.Count;

                for (int i = node.Start; i < end; i++)
                {
                    ClusterPart part = m_Parts[m_Order[i]];

                    if (!RayEntersBox(ox, oy, oz, invX, invY, invZ, part.Bounds.min, part.Bounds.max, out _))
                        continue;

                    if (!part.Storage.TryCountSignedCrossings(_Origin, _Direction, out int partWinding))
                        return false;

                    _Winding += partWinding;
                }
            }

            return true;
        }

        #endregion

        #region Service methods

        //median split on the longest axis of the parts' centres, leaves of a few parts
        int Build(int _Start, int _End)
        {
            int nodeIndex = m_NodeCount++;
            int count     = _End - _Start;

            Vector3 min = m_Parts[m_Order[_Start]].Bounds.min;
            Vector3 max = m_Parts[m_Order[_Start]].Bounds.max;

            for (int i = _Start + 1; i < _End; i++)
            {
                Bounds bounds = m_Parts[m_Order[i]].Bounds;

                min = Vector3.Min(min, bounds.min);
                max = Vector3.Max(max, bounds.max);
            }

            if (count <= MAX_LEAF_PARTS)
            {
                m_Nodes[nodeIndex] = new ForestNode { Min = min, Max = max, StartOrLeft = _Start, CountOrRight = count };

                return nodeIndex;
            }

            Vector3 extent = max - min;

            int axis = extent.x >= extent.y && extent.x >= extent.z ? 0 : extent.y >= extent.z ? 1 : 2;

            Array.Sort(m_Order, _Start, count, Comparer<int>.Create((_A, _B) => m_Parts[_A].Bounds.center[axis].CompareTo(m_Parts[_B].Bounds.center[axis])));

            int mid = _Start + count / 2;

            int left  = Build(_Start, mid);
            int right = Build(mid, _End);

            m_Nodes[nodeIndex] = new ForestNode { Min = min, Max = max, StartOrLeft = left, CountOrRight = -right };

            return nodeIndex;
        }

        static bool Misses(in ForestNode _Node, Vector3 _QueryMin, Vector3 _QueryMax)
        {
            return _QueryMax.x < _Node.Min.x || _QueryMin.x > _Node.Max.x ||
                   _QueryMax.y < _Node.Min.y || _QueryMin.y > _Node.Max.y ||
                   _QueryMax.z < _Node.Min.z || _QueryMin.z > _Node.Max.z;
        }

        //the slab test of TriangleStorage.Rays, with the same slack, and the entry distance for the raycast's cull
        static bool RayEntersBox(double _Ox, double _Oy, double _Oz, double _InvX, double _InvY, double _InvZ, Vector3 _Min, Vector3 _Max, out double _Entry)
        {
            double t1   = (_Min.x - _Ox) * _InvX, t2   = (_Max.x - _Ox) * _InvX;
            double tMin = Math.Min(t1, t2),       tMax = Math.Max(t1, t2);

            t1   = (_Min.y - _Oy) * _InvY;
            t2   = (_Max.y - _Oy) * _InvY;
            tMin = Math.Max(tMin, Math.Min(t1, t2));
            tMax = Math.Min(tMax, Math.Max(t1, t2));

            t1   = (_Min.z - _Oz) * _InvZ;
            t2   = (_Max.z - _Oz) * _InvZ;
            tMin = Math.Max(tMin, Math.Min(t1, t2));
            tMax = Math.Min(tMax, Math.Max(t1, t2));

            _Entry = Math.Max(tMin, 0.0);

            return tMax >= _Entry - 1e-6;
        }

        #endregion

    }
}