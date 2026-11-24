using System;
using System.Collections.Generic;
using Node = Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.Elements.Node;
using Leaf = Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.Elements.Leaf;
using Fork = Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.Elements.Fork;
using UnityEngine;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.Hierarchy
{
    //Builds a RegionMap from the live tree: the field painted from the occupied leaves under one read-lock hold per
    //root (a whole-world reader takes one hold per chunk, never one per walk),
    //the distance transform, then the core-and-skin watershed: cores are the components of clearance >= T, skins the cells a flood from the cores
    //reaches within T of path length, what is left forms passages between the regions it touches (a piece touching one
    //region joins it, one touching none is isolated), and a doorway is a connected set of contact cells between two
    //regions — a passage and a room, or two rooms whose skins meet. Rule and thresholds measured on TUNNEL, the
    //pavilion and SKYLINE before being fixed.
    internal static class RegionMapBuilder
    {
        #region Constants

        //cells per axis are capped so a large world stays at a few million cells; the cell then grows past the radius
        //and the finest passages stop reading as passages, which is the honest degradation
        const int MAX_CELLS_PER_AXIS = 192;

        #endregion

        #region Public methods

        //null when cancelled or when the world holds no root. _Previous with _ChangedRoots: the previous map's field is
        //repainted only inside those roots (the publish rebuilt nothing else) when the world's extent is unchanged;
        //the transform and the watershed still run whole.
        public static RegionMap Build(NavTree _Tree, float _Threshold, Func<bool> _Cancelled, RegionMap _Previous = null, ICollection<Vector3Int> _ChangedRoots = null)
        {
            System.Diagnostics.Stopwatch watch = System.Diagnostics.Stopwatch.StartNew();
            System.Diagnostics.Stopwatch wait  = new System.Diagnostics.Stopwatch();

            GlobalOctree octree = _Tree.Octree;

            //the extent, the roots, and the epoch the map answers for, under one hold
            Bounds           extent;
            int              epoch;
            List<Vector3Int> rootIndices = new List<Vector3Int>();

            wait.Start();
            _Tree.EnterReadLock("RegionMap build: extent");
            wait.Stop();
            try
            {
                Node[] roots = _Tree.LiveRootsGrid.GetAllAtLevel(0);

                if (roots.Length == 0)
                    return null;

                extent = roots[0].Bounds;

                foreach (Node root in roots)
                {
                    extent.Encapsulate(root.Bounds);
                    rootIndices.Add(root.Index);
                }

                epoch = _Tree.PublishEpoch;
            }
            finally
            {
                _Tree.ExitReadLock();
            }

            //the cell divides the root, so a root owns whole cells and a repaint of one root touches no other's
            float largest = Mathf.Max(extent.size.x, Mathf.Max(extent.size.y, extent.size.z));
            float wanted  = Mathf.Max(octree.MaxAgentRadius, Mathf.Max(2f * octree.MinBucketSizeReal, largest / MAX_CELLS_PER_AXIS));
            int   perRoot = Mathf.Max(1, Mathf.FloorToInt(octree.RootSize / wanted + 1e-4f));
            float cell    = octree.RootSize / perRoot;

            Vector3Int origin = Nav3D.Common.UtilsSpatialGrid.GetBucketIndex(extent.min + Vector3.one * 0.01f, cell);
            Vector3Int last   = Nav3D.Common.UtilsSpatialGrid.GetBucketIndex(extent.max - Vector3.one * 0.01f, cell);

            RegionField field       = new RegionField(cell, origin, last - origin + Vector3Int.one);
            bool        incremental = _Previous != null && _ChangedRoots != null && _Previous.Field.Cell == cell && _Previous.Field.Origin == origin && _Previous.Field.Dims == field.Dims;
            int         repainted   = 0;

            //the sample lattice the painting reproduces: a cell is open when no point of it at the min-bucket pitch lies
            //in an occupied leaf, so no crust leaf falls between two samples
            int   sub  = Mathf.Max(1, Mathf.CeilToInt(cell / octree.MinBucketSizeReal - 0.001f));
            float step = cell / sub;

            if (incremental)
            {
                Array.Copy(_Previous.Field.Open, field.Open, field.Open.Length);

                foreach (Vector3Int rootIndex in _ChangedRoots)
                {
                    if (_Cancelled())
                        return null;

                    repainted += ClearRoot(field, rootIndex, perRoot);

                    PaintRoot(_Tree, field, rootIndex, sub, step, wait);
                }
            }
            else
            {
                for (int index = 0; index < field.Open.Length; index++)
                    field.Open[index] = true;

                foreach (Vector3Int rootIndex in rootIndices)
                {
                    if (_Cancelled())
                        return null;

                    PaintRoot(_Tree, field, rootIndex, sub, step, wait);
                }

                repainted = field.Count;
            }

            double fieldMilliseconds = watch.Elapsed.TotalMilliseconds;

            field.ComputeDistances();

            if (_Cancelled())
                return null;

            double transformMilliseconds = watch.Elapsed.TotalMilliseconds - fieldMilliseconds;

            List<RegionMap.Region> regions = new List<RegionMap.Region> { null };
            RegionMap              map     = new RegionMap(field, regions, epoch, _Threshold);

            Segment(map);

            map.Incremental           = incremental;
            map.RepaintedCells        = repainted;
            map.FieldMilliseconds     = fieldMilliseconds;
            map.HoldWaitMilliseconds  = wait.Elapsed.TotalMilliseconds;
            map.TransformMilliseconds = transformMilliseconds;
            map.BuildMilliseconds     = watch.Elapsed.TotalMilliseconds;

            return _Cancelled() ? null : map;
        }

        #endregion

        #region Service methods — the field

        //the cells of one root, back to open; returns how many
        static int ClearRoot(RegionField _Field, Vector3Int _RootIndex, int _PerRoot)
        {
            Vector3Int min = _RootIndex * _PerRoot - _Field.Origin;
            Vector3Int max = min + Vector3Int.one * (_PerRoot - 1);

            min = Vector3Int.Max(min, Vector3Int.zero);
            max = Vector3Int.Min(max, _Field.Dims - Vector3Int.one);

            int count = 0;

            for (int x = min.x; x <= max.x; x++)
            for (int y = min.y; y <= max.y; y++)
            for (int z = min.z; z <= max.z; z++)
            {
                _Field.Open[_Field.IndexOf(x, y, z)] = true;

                count++;
            }

            return count;
        }

        //Closes every cell one of the root's occupied leaves holds a sample of, under one hold: the walk down the
        //root never crosses a release, and a root that is gone (dropped by the publish) leaves its cells open.
        static void PaintRoot(NavTree _Tree, RegionField _Field, Vector3Int _RootIndex, int _Sub, float _Step, System.Diagnostics.Stopwatch _Wait)
        {
            _Wait.Start();
            _Tree.EnterReadLock("RegionMap build: root");
            _Wait.Stop();
            try
            {
                if (_Tree.LiveRootsGrid.TryGetNode(0, _RootIndex, out Node root) && root != null)
                    Paint(_Field, root, _Sub, _Step);
            }
            finally
            {
                _Tree.ExitReadLock();
            }
        }

        static void Paint(RegionField _Field, Node _Node, int _Sub, float _Step)
        {
            if (_Node is Leaf leaf)
            {
                if (leaf.Occupied)
                    Close(_Field, leaf.Bounds, _Sub, _Step);

                return;
            }

            if (_Node is Fork fork)
            {
                foreach (Node child in fork.ChildrenMap.Values)
                    Paint(_Field, child, _Sub, _Step);
            }
        }

        //The cells whose sample points a box contains. Samples sit at (k + 0.5) * step along each axis, step dividing
        //the cell, and a point on a leaf's lower face belongs to that leaf (the octant walk takes >= against the
        //centre), so the box [a, b) holds the samples with a <= (k + 0.5) * step < b.
        static void Close(RegionField _Field, Bounds _Box, int _Sub, float _Step)
        {
            Vector3Int min = _Field.Dims, max = -Vector3Int.one;

            for (int axis = 0; axis < 3; axis++)
            {
                int kMin = Mathf.CeilToInt(_Box.min[axis] / _Step - 0.5f);
                int kMax = Mathf.CeilToInt(_Box.max[axis] / _Step - 0.5f) - 1;

                if (kMax < kMin)
                    return;

                min[axis] = Mathf.Max(FloorDivide(kMin, _Sub) - _Field.Origin[axis], 0);
                max[axis] = Mathf.Min(FloorDivide(kMax, _Sub) - _Field.Origin[axis], _Field.Dims[axis] - 1);

                if (max[axis] < min[axis])
                    return;
            }

            for (int x = min.x; x <= max.x; x++)
            for (int y = min.y; y <= max.y; y++)
            for (int z = min.z; z <= max.z; z++)
                _Field.Open[_Field.IndexOf(x, y, z)] = false;
        }

        static int FloorDivide(int _Value, int _Divisor) => _Value >= 0 ? _Value / _Divisor : -((-_Value + _Divisor - 1) / _Divisor);

        #endregion

        #region Service methods — the watershed

        static void Segment(RegionMap _Map)
        {
            RegionField field     = _Map.Field;
            int[]       regionOf  = field.RegionOf;
            float       T         = _Map.Threshold;
            Queue<int>  frontier  = new Queue<int>();
            int[]       neighbors = new int[6];

            //cores: components of d >= T, each a room
            for (int seed = 0; seed < regionOf.Length; seed++)
            {
                if (!field.Open[seed] || field.Distance[seed] < T || regionOf[seed] != 0)
                    continue;

                RegionMap.Region room = new RegionMap.Region(_Map.Regions.Count, false);

                _Map.Regions.Add(room);

                regionOf[seed] = room.Id;

                frontier.Enqueue(seed);

                while (frontier.Count > 0)
                {
                    int around = Neighbors(field, frontier.Dequeue(), neighbors);

                    for (int n = 0; n < around; n++)
                    {
                        int next = neighbors[n];

                        if (field.Open[next] && field.Distance[next] >= T && regionOf[next] == 0)
                        {
                            regionOf[next] = room.Id;

                            frontier.Enqueue(next);
                        }
                    }
                }
            }

            //skins: a breadth-first flood from every core cell, at most T of path length
            int   reach = Mathf.CeilToInt(T / field.Cell);
            int[] depth = new int[regionOf.Length];

            for (int index = 0; index < regionOf.Length; index++)
            {
                if (regionOf[index] != 0)
                    frontier.Enqueue(index);
            }

            while (frontier.Count > 0)
            {
                int current = frontier.Dequeue();

                if (depth[current] >= reach)
                    continue;

                int around = Neighbors(field, current, neighbors);

                for (int n = 0; n < around; n++)
                {
                    int next = neighbors[n];

                    if (field.Open[next] && regionOf[next] == 0)
                    {
                        regionOf[next] = regionOf[current];
                        depth[next]    = depth[current] + 1;

                        frontier.Enqueue(next);
                    }
                }
            }

            //the rest: components; touching two or more regions = a passage, one = absorbed, none = isolated
            List<int> piece = new List<int>();

            for (int seed = 0; seed < regionOf.Length; seed++)
            {
                if (!field.Open[seed] || regionOf[seed] != 0)
                    continue;

                piece.Clear();

                HashSet<int> touched = new HashSet<int>();

                regionOf[seed] = -1;

                frontier.Enqueue(seed);

                while (frontier.Count > 0)
                {
                    int current = frontier.Dequeue();

                    piece.Add(current);

                    int around = Neighbors(field, current, neighbors);

                    for (int n = 0; n < around; n++)
                    {
                        int next = neighbors[n];

                        if (!field.Open[next])
                            continue;

                        if (regionOf[next] == 0)
                        {
                            regionOf[next] = -1;

                            frontier.Enqueue(next);
                        }
                        else if (regionOf[next] > 0)
                        {
                            touched.Add(regionOf[next]);
                        }
                    }
                }

                int id;

                if (touched.Count == 1)
                {
                    id = 0;

                    foreach (int room in touched)
                        id = room;
                }
                else
                {
                    RegionMap.Region region = new RegionMap.Region(_Map.Regions.Count, touched.Count >= 2);

                    _Map.Regions.Add(region);

                    id = region.Id;
                }

                foreach (int index in piece)
                    regionOf[index] = id;
            }

            foreach (RegionMap.Region region in _Map.Regions)
            {
                if (region != null)
                    region.Cells = 0;
            }

            for (int index = 0; index < regionOf.Length; index++)
            {
                if (regionOf[index] > 0)
                    _Map.Regions[regionOf[index]].Cells++;
            }

            //contacts: every open cell next to an open cell of another region, recorded on the lower id's side — a passage
            //meeting a room, and two rooms whose skins meet across a slot with no passage between (TIGHT's lattice)
            List<(int Cell, int RegionA, int RegionB)> contacts = new List<(int, int, int)>();

            for (int index = 0; index < regionOf.Length; index++)
            {
                int region = regionOf[index];

                if (region <= 0)
                    continue;

                int around = Neighbors(field, index, neighbors);

                for (int n = 0; n < around; n++)
                {
                    int next  = neighbors[n];
                    int other = regionOf[next];

                    if (other > region)
                        contacts.Add((index, region, other));
                }
            }

            GroupDoorways(_Map, contacts);
        }

        //Contact cells between the same two regions, connected through 26-adjacency, form one doorway. Its point in
        //region A is the contact cell nearest the group's centroid; its point in region B is that cell's neighbour
        //across — both navigable cell centres, so a search restricted to either region can arrive there. Its
        //clearance is the largest among its cells.
        static void GroupDoorways(RegionMap _Map, List<(int Cell, int RegionA, int RegionB)> _Contacts)
        {
            RegionField field     = _Map.Field;
            int[]       neighbors = new int[6];

            Dictionary<(int RegionA, int RegionB), HashSet<int>> byPair = new Dictionary<(int, int), HashSet<int>>();

            foreach ((int cell, int regionA, int regionB) in _Contacts)
            {
                if (!byPair.TryGetValue((regionA, regionB), out HashSet<int> cells))
                    byPair[(regionA, regionB)] = cells = new HashSet<int>();

                cells.Add(cell);
            }

            foreach (KeyValuePair<(int RegionA, int RegionB), HashSet<int>> pair in byPair)
            {
                HashSet<int> unvisited = new HashSet<int>(pair.Value);
                Queue<int>   frontier  = new Queue<int>();

                while (unvisited.Count > 0)
                {
                    int seed = -1;

                    foreach (int cell in unvisited)
                    {
                        seed = cell;

                        break;
                    }

                    List<int> group = new List<int> { seed };

                    unvisited.Remove(seed);
                    frontier.Enqueue(seed);

                    while (frontier.Count > 0)
                    {
                        Vector3Int cell = field.CellOf(frontier.Dequeue());

                        //26-connectivity along the contact sheet
                        for (int dx = -1; dx <= 1; dx++)
                        for (int dy = -1; dy <= 1; dy++)
                        for (int dz = -1; dz <= 1; dz++)
                        {
                            if (!field.Inside(cell.x + dx, cell.y + dy, cell.z + dz))
                                continue;

                            int next = field.IndexOf(cell.x + dx, cell.y + dy, cell.z + dz);

                            if (unvisited.Remove(next))
                            {
                                group.Add(next);
                                frontier.Enqueue(next);
                            }
                        }
                    }

                    Vector3 centroid  = Vector3.zero;
                    float   clearance = 0f;

                    foreach (int cell in group)
                    {
                        centroid  += field.CentreOf(cell);
                        clearance  = Mathf.Max(clearance, field.Distance[cell]);
                    }

                    centroid /= group.Count;

                    int   nearest    = group[0];
                    float nearestSqr = float.PositiveInfinity;
                    float extentSqr  = 0f;

                    foreach (int cell in group)
                    {
                        float sqr = (field.CentreOf(cell) - centroid).sqrMagnitude;

                        if (sqr < nearestSqr)
                        {
                            nearestSqr = sqr;
                            nearest    = cell;
                        }

                        if (sqr > extentSqr)
                            extentSqr = sqr;
                    }

                    int across = -1;
                    int around = Neighbors(field, nearest, neighbors);

                    for (int n = 0; n < around; n++)
                    {
                        if (field.RegionOf[neighbors[n]] == pair.Key.RegionB)
                        {
                            across = neighbors[n];

                            break;
                        }
                    }

                    if (across < 0)
                        continue;

                    RegionMap.Doorway doorway = new RegionMap.Doorway(_Map.Doorways.Count, pair.Key.RegionA, pair.Key.RegionB, field.CentreOf(nearest), field.CentreOf(across), clearance,
                                                                      Mathf.Min(field.Distance[nearest], field.Distance[across]), Mathf.Sqrt(extentSqr));

                    _Map.Doorways.Add(doorway);
                    _Map.Regions[pair.Key.RegionA].Doorways.Add(doorway);
                    _Map.Regions[pair.Key.RegionB].Doorways.Add(doorway);
                }
            }
        }

        //the 6-neighbours of a cell into _Buffer (length 6), returning how many; no allocation, since the floods below
        //ask this once per cell per pass over a field of millions
        static int Neighbors(RegionField _Field, int _Index, int[] _Buffer)
        {
            Vector3Int cell  = _Field.CellOf(_Index);
            int        count = 0;
            int        strideY = _Field.Dims.z;
            int        strideX = _Field.Dims.z * _Field.Dims.y;

            if (cell.x > 0)                 _Buffer[count++] = _Index - strideX;
            if (cell.x < _Field.Dims.x - 1) _Buffer[count++] = _Index + strideX;
            if (cell.y > 0)                 _Buffer[count++] = _Index - strideY;
            if (cell.y < _Field.Dims.y - 1) _Buffer[count++] = _Index + strideY;
            if (cell.z > 0)                 _Buffer[count++] = _Index - 1;
            if (cell.z < _Field.Dims.z - 1) _Buffer[count++] = _Index + 1;

            return count;
        }

        #endregion
    }
}
