using System;
using System.Collections.Generic;
using UnityEngine;

namespace Nav3D.Common.Extensions
{
    static class ExtensionIBoundable
    {
        #region Nested types

        internal readonly struct CellKey : IEquatable<CellKey>
        {
            #region Properties

            public int X { get; }
            public int Y { get; }
            public int Z { get; }

            #endregion

            #region Constructors

            public CellKey(int _X, int _Y, int _Z)
            {
                X = _X;
                Y = _Y;
                Z = _Z;
            }

            #endregion

            #region Public methods

            public static CellKey FromPosition(Vector3 _P, float _CellSize)
            {
                //Floor to cell coordinates (works with negatives too)
                int cx = Mathf.FloorToInt(_P.x / _CellSize);
                int cy = Mathf.FloorToInt(_P.y / _CellSize);
                int cz = Mathf.FloorToInt(_P.z / _CellSize);
                return new CellKey(cx, cy, cz);
            }

            public override int GetHashCode()
            {
                unchecked
                {
                    //Simple, decent hash for 3 ints
                    int h = 17;
                    h = h * 31 + X;
                    h = h * 31 + Y;
                    h = h * 31 + Z;
                    return h;
                }
            }

            //IEquatable is what keeps the Dictionary<CellKey, ...> lookups below allocation-free: without it
            //EqualityComparer<CellKey>.Default falls back to ObjectEqualityComparer, which BOXES both keys on every
            //single comparison — and this grid is probed once per AABB per overlapped cell.
            public bool Equals(CellKey _Other) => _Other.X == X && _Other.Y == Y && _Other.Z == Z;

            public override bool Equals(object _Obj) => _Obj is CellKey other && Equals(other);

            #endregion
        }

        #endregion

        #region Public methods

        //Returns squared AABB distance between bounds
        public static float SqrAABBDistance(this IBoundable _First, IBoundable _Second)
        {
            Bounds boundsA = _First.Bounds;
            Bounds boundsB = _Second.Bounds;

            float dx = Mathf.Max(0.0f, Mathf.Max(boundsA.min.x - boundsB.max.x, boundsB.min.x - boundsA.max.x));
            float dy = Mathf.Max(0.0f, Mathf.Max(boundsA.min.y - boundsB.max.y, boundsB.min.y - boundsA.max.y));
            float dz = Mathf.Max(0.0f, Mathf.Max(boundsA.min.z - boundsB.max.z, boundsB.min.z - boundsA.max.z));

            return dx * dx + dy * dy + dz * dz;
        }

        //The clustering's reusable buffers: the spatial hash, the visited flags, the
        //BFS queue and the cluster index lists — without them a clustering allocated ~4.6 MB per obstacle add.
        //Everything is cleared on the way out, so the scratch never keeps a caller's objects alive. Not thread-safe.
        public sealed class ClusteringScratch
        {
            internal readonly Dictionary<CellKey, List<int>> Grid     = new Dictionary<CellKey, List<int>>();
            internal readonly List<List<int>>                ListPool = new List<List<int>>();
            internal readonly Queue<int>                     Queue    = new Queue<int>(256);
            internal readonly List<List<int>>                Clusters = new List<List<int>>();

            internal bool[] Visited = System.Array.Empty<bool>();
            internal int    ListsInUse;

            internal List<int> RentList()
            {
                if (ListsInUse == ListPool.Count)
                    ListPool.Add(new List<int>());

                List<int> list = ListPool[ListsInUse++];

                list.Clear();

                return list;
            }

            internal void Clear()
            {
                Grid.Clear();
                Queue.Clear();
                Clusters.Clear();

                for (int i = 0; i < ListsInUse; i++)
                    ListPool[i].Clear();

                ListsInUse = 0;

                System.Array.Clear(Visited, 0, Visited.Length);
            }
        }

        /// <summary>
        /// Chain-merge clustering for AABBs (Unity Bounds).
        /// Two items are connected if distance between their AABBs <= _MergeDistance.
        /// Returns clusters as arrays of the items. _Scratch (optional) supplies the reusable buffers — see
        /// ClusteringScratch; without it every call allocates its own.
        /// </summary>
        public static T[][] ClusterByAABBDistance<T>(IList<T> _AABBs, float _MergeDistance, ClusteringScratch _Scratch = null) where T : IBoundable
        {
            if (_AABBs == null || _AABBs.Count == 0)
                return new T[][] { };

            ClusteringScratch scratch = _Scratch ?? new ClusteringScratch();

            try
            {
                return ClusterByAABBDistanceInternal(_AABBs, _MergeDistance, scratch);
            }
            finally
            {
                scratch.Clear();
            }
        }

        static T[][] ClusterByAABBDistanceInternal<T>(IList<T> _AABBs, float _MergeDistance, ClusteringScratch _Scratch) where T : IBoundable
        {
            List<List<int>> clusters = _Scratch.Clusters;

            float sqrMergeDistance = _MergeDistance * _MergeDistance;

            //Grid cell size: _MergeDistance works well; you can tweak if you like.
            //If _MergeDistance is 0, avoid division by zero: use a small value.
            float cellSize = Mathf.Max(_MergeDistance, 0.00001f);

            //1) Build spatial hash: insert each AABB into all cells overlapped by (AABB expanded by _MergeDistance)
            Dictionary<CellKey, List<int>> grid = _Scratch.Grid;

            for (int i = 0; i < _AABBs.Count; i++)
            {
                Bounds b        = _AABBs[i].Bounds;
                Bounds expanded = b; // insert into the AABB's OWN cells only; the merge slack is added on the QUERY side
                                     //(enlarging at both insert and query over-inserts each AABB into ~(2*merge/cell)^3
                                     //cells, which for dense geometry inflates per-node scans by ~30x)

                CellKey minK = CellKey.FromPosition(expanded.min, cellSize);
                CellKey maxK = CellKey.FromPosition(expanded.max, cellSize);

                for (int x = minK.X; x <= maxK.X; x++)
                    for (int y = minK.Y; y <= maxK.Y; y++)
                        for (int z = minK.Z; z <= maxK.Z; z++)
                        {
                            CellKey key = new CellKey(x, y, z);
                            if (!grid.TryGetValue(key, out List<int> list))
                            {
                                list      = _Scratch.RentList();
                                grid[key] = list;
                            }
                            list.Add(i);
                        }
            }

            //2) BFS connected components (chain merges)
            if (_Scratch.Visited.Length < _AABBs.Count)
                _Scratch.Visited = new bool[Mathf.Max(_AABBs.Count, _Scratch.Visited.Length * 2)];

            bool[]     visited = _Scratch.Visited;
            Queue<int> queue   = _Scratch.Queue;

            for (int start = 0; start < _AABBs.Count; start++)
            {
                if (visited[start])
                    continue;

                List<int> cluster = _Scratch.RentList();
                visited[start] = true;
                queue.Enqueue(start);

                while (queue.Count > 0)
                {
                    int a = queue.Dequeue();
                    cluster.Add(a);

                    //Find unvisited neighbours within merge distance from the grid cells A's expanded bounds touch.
                    //As triangles get visited they are swap-REMOVED from the cell lists, so each grid entry is scanned
                    //O(1) amortized over the whole run. Without this, dense geometry (thousands of AABBs per cell)
                    //re-scans every already-clustered triangle for every node → O(N) per node → O(N^2) overall.
                    Bounds  expandedA = _AABBs[a].Bounds.Enlarge(_MergeDistance);

                    CellKey minK = CellKey.FromPosition(expandedA.min, cellSize);
                    CellKey maxK = CellKey.FromPosition(expandedA.max, cellSize);

                    for (int x = minK.X; x <= maxK.X; x++)
                        for (int y = minK.Y; y <= maxK.Y; y++)
                            for (int z = minK.Z; z <= maxK.Z; z++)
                            {
                                CellKey key = new CellKey(x, y, z);
                                if (!grid.TryGetValue(key, out List<int> list))
                                    continue;

                                //iterate backwards so swap-remove never skips an unexamined entry
                                for (int li = list.Count - 1; li >= 0; li--)
                                {
                                    int b = list[li];

                                    //already consumed (A itself, or a triangle visited via another cell): drop it so no
                                    //later scan of this cell re-examines it.
                                    if (visited[b])
                                    {
                                        list[li] = list[list.Count - 1];
                                        list.RemoveAt(list.Count - 1);
                                        continue;
                                    }

                                    if (_AABBs[a].SqrAABBDistance(_AABBs[b]) <= sqrMergeDistance)
                                    {
                                        visited[b] = true;
                                        queue.Enqueue(b);

                                        list[li] = list[list.Count - 1];
                                        list.RemoveAt(list.Count - 1);
                                    }
                                    //else: B is unvisited but too far from A — leave it for another node's scan.
                                }
                            }
                }

                clusters.Add(cluster);
            }

            T[][] result = new T[clusters.Count][];

            for (int clusterNum = 0; clusterNum < clusters.Count; clusterNum++)
            {
                List<int> cluster = clusters[clusterNum];
                T[]       items   = new T[cluster.Count];

                for (int itemNum = 0; itemNum < cluster.Count; itemNum++)
                    items[itemNum] = _AABBs[cluster[itemNum]];

                result[clusterNum] = items;
            }

            return result;
        }

        #endregion
    }
}
