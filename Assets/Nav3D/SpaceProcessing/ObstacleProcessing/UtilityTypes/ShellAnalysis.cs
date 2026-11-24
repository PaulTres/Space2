using Nav3D.Common;
using Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.SpatialStorages;
using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.UtilityTypes
{
    //Splits a triangle set into edge-connected shells and tells the closed ones from the open ones, which is what decides
    //where a body has an inside at all. Vertices are welded by exact position — a mesh's seams repeat a position computed
    //by the same transform — and a crack narrower than the gap tolerance is zipped, since no cell passes through it.
    static class ShellAnalysis
    {

        #region Nested types

        //A shell with holes. Capping every hole with a fan wound against the shell's own boundary closes the chain, so the
        //shell plus its caps has a winding number again — which side is "inside" — whatever other geometry does about
        //the holes. The caps are virtual: they count in that winding and never make a cell crossed.
        public sealed class OpenShell
        {
            readonly object          m_StorageLock = new object();
            volatile TriangleStorage m_ShellStorage;
            volatile TriangleStorage m_CappedStorage;

            //the surface as the winding counts it (zipped where a crack was closed)
            public Triangle[] Triangles { get; }

            //the source triangles, as the cluster holds them: what "other geometry" excludes
            public HashSet<Triangle> Members { get; }

            //each hole as its boundary loop, in the direction the shell's own triangles walk it
            public List<Vector3[]> Holes { get; }

            public Triangle[] CapTriangles { get; }

            //the hole each cap triangle covers
            public int[] CapHoles { get; }

            //a hole no cell fits through — it spans less than the gap tolerance, or its loop has no area — closes itself
            public bool[] HoleClosedByItself { get; }

            public bool ClosedByItself { get; }

            public Bounds CappedBounds { get; }

            //where the holes that other geometry must close run: that geometry lies within reach of these
            public Bounds HolesBounds { get; }

            public OpenShell(Triangle[] _Triangles, HashSet<Triangle> _Members, List<Vector3[]> _Holes, float _GapTolerance)
            {
                Triangles = _Triangles;
                Members   = _Members;
                Holes     = _Holes;

                List<Triangle> caps     = new List<Triangle>();
                List<int>      capHoles = new List<int>();

                HoleClosedByItself = new bool[_Holes.Count];
                ClosedByItself     = true;

                Bounds? holes = null;

                for (int h = 0; h < _Holes.Count; h++)
                {
                    int before = caps.Count;

                    AddCap(_Holes[h], caps);

                    for (int c = before; c < caps.Count; c++)
                        capHoles.Add(h);

                    Bounds loop = new Bounds(_Holes[h][0], Vector3.zero);

                    foreach (Vector3 vertex in _Holes[h])
                        loop.Encapsulate(vertex);

                    Vector3 size = loop.size;

                    HoleClosedByItself[h] = caps.Count == before || Mathf.Max(size.x, Mathf.Max(size.y, size.z)) < _GapTolerance;

                    if (HoleClosedByItself[h])
                        continue;

                    ClosedByItself = false;

                    if (holes.HasValue)
                        loop.Encapsulate(holes.Value);

                    holes = loop;
                }

                CapTriangles = caps.ToArray();
                CapHoles     = capHoles.ToArray();

                CappedBounds = ExtensionBounds.TrianglesBounds(_Triangles);

                if (CapTriangles.Length > 0)
                    CappedBounds.Encapsulate(ExtensionBounds.TrianglesBounds(CapTriangles));

                HolesBounds = holes ?? new Bounds(_Holes[0][0], Vector3.zero);
            }

            //the shell alone, for "which surface does a ray from a cap meet first"
            public TriangleStorage ShellStorage => EnsureStorages().Shell;

            //the shell and its caps: a surface without boundary, whose winding number is the capped shell's inside
            public TriangleStorage CappedStorage => EnsureStorages().Capped;

            (TriangleStorage Shell, TriangleStorage Capped) EnsureStorages()
            {
                if (m_CappedStorage == null)
                    lock (m_StorageLock)
                    {
                        if (m_CappedStorage == null)
                        {
                            m_ShellStorage  = new TriangleStorage(Triangles);
                            m_CappedStorage = new TriangleStorage(Triangles.Concat(CapTriangles).ToArray());
                        }
                    }

                return (m_ShellStorage, m_CappedStorage);
            }

            //a fan from the loop's centroid, walking every boundary edge the other way round
            static void AddCap(Vector3[] _Hole, List<Triangle> _Caps)
            {
                if (_Hole.Length < 3)
                    return;

                Vector3 centroid = Vector3.zero;

                foreach (Vector3 vertex in _Hole)
                    centroid += vertex;

                centroid /= _Hole.Length;

                for (int i = 0; i < _Hole.Length; i++)
                {
                    Vector3 from = _Hole[i];
                    Vector3 to   = _Hole[(i + 1) % _Hole.Length];

                    //the shell walks from→to, so the cap walks to→from
                    if (Vector3.Cross(from - centroid, to - centroid).sqrMagnitude > 1e-12f)
                        _Caps.Add(new Triangle(centroid, to, from));
                }
            }
        }

        public sealed class Result
        {
            //every triangle of every closed shell, as one set: winding numbers add over shells, so one ray answers all
            public List<Triangle> ClosedTriangles { get; } = new List<Triangle>();

            public List<OpenShell> OpenShells { get; } = new List<OpenShell>();

            //closed shells that enclose something: their faces point out of it
            public int ClosedShellsCount { get; set; }

            //closed shells facing inward (or enclosing nothing, a double-sided face): never a body
            public int InwardShellsCount { get; set; }

            //their triangles — what a winding counted over the whole surface must give back (ClusterSolid.ClosedForest)
            public List<Triangle> InwardTriangles { get; } = new List<Triangle>();

            //each inward shell's own box: the volume a partial update must re-derive when that cavity appears or opens
            public List<Bounds> InwardShellsBounds { get; } = new List<Bounds>();

            //whether a crack was zipped — then some closed triangles are rebuilt surfaces, not the source objects
            public bool Zipped { get; set; }
        }

        //Exact equality with a hash that mixes all three components' bits. Vector3's own hash xors shifted component hashes,
        //which collides wholesale on a regular grid — a terrain's vertices — and turns the weld quadratic.
        internal sealed class ExactPositionComparer : IEqualityComparer<Vector3>
        {
            public static ExactPositionComparer Instance { get; } = new ExactPositionComparer();

            public bool Equals(Vector3 _A, Vector3 _B)
            {
                return _A.x.Equals(_B.x) && _A.y.Equals(_B.y) && _A.z.Equals(_B.z);
            }

            public int GetHashCode(Vector3 _Position)
            {
                unchecked
                {
                    long hash = _Position.x.GetHashCode();
                    hash = hash * 0x9E3779B1L + _Position.y.GetHashCode();
                    hash = hash * 0x9E3779B1L + _Position.z.GetHashCode();

                    return (int) (hash ^ (hash >> 32));
                }
            }
        }

        //long's own hash xors its halves, and an edge key's halves are two vertex indices numbered in sequence — a ^ b
        //collides wholesale on any real mesh
        sealed class EdgeKeyComparer : IEqualityComparer<long>
        {
            public static EdgeKeyComparer Instance { get; } = new EdgeKeyComparer();

            public bool Equals(long _A, long _B) => _A == _B;

            public int GetHashCode(long _Key)
            {
                unchecked
                {
                    ulong hash = (ulong) _Key * 0x9E3779B97F4A7C15UL;

                    return (int) (hash >> 32);
                }
            }
        }

        struct EdgeRecord
        {
            //walks a→b minus walks b→a, with a < b
            public int Net           { get; set; }
            public int FirstTriangle { get; set; }

            //a triangle walking a→b and one walking b→a, as index + 1 (0: none)
            public int ForwardWalker  { get; set; }
            public int BackwardWalker { get; set; }
        }

        #endregion

        #region Public methods

        //A shell is closed when every edge is walked as often in one direction as in the other: its chain has no
        //boundary, which is exactly when the winding number is an integer everywhere off the surface. That covers seamed
        //meshes, two boxes sharing an edge and a double-sided face (whose two windings cancel, so it encloses nothing).
        //_IsHollow: a shell holding any triangle it accepts is dropped whole — never a candidate, closed or not.
        //_GapTolerance: gaps narrower than this do not open a shell — cracks, unwelded seams and T-junctions are zipped
        //(ShellGapZipper) and a hole that fits inside it is closed by the shell itself; 0 keeps the exact topology.
        //A closed shell facing inward (a negative volume) encloses nothing: it is counted, never a body, so an inward
        //cavity inside a body is solid with the body.
        public static Result Analyze(ICollection<Triangle> _Triangles, Predicate<Triangle> _IsHollow = null, float _GapTolerance = 0f)
        {
            Result result = new Result();

            if (_Triangles == null || _Triangles.Count == 0)
                return result;

            Dictionary<Vector3, int> vertexIds = new Dictionary<Vector3, int>(_Triangles.Count, ExactPositionComparer.Instance);
            List<Vector3>            positions = new List<Vector3>(_Triangles.Count);
            List<Triangle>           origins   = new List<Triangle>(_Triangles.Count);
            List<int>                corners   = new List<int>(_Triangles.Count * 3);

            foreach (Triangle triangle in _Triangles)
            {
                int a = WeldId(vertexIds, positions, triangle.V1);
                int b = WeldId(vertexIds, positions, triangle.V2);
                int c = WeldId(vertexIds, positions, triangle.V3);

                //a degenerate triangle has no side, so it adds nothing to a winding number and closes nothing
                if (a == b || b == c || c == a)
                    continue;

                origins.Add(triangle);
                corners.Add(a);
                corners.Add(b);
                corners.Add(c);
            }

            Dictionary<long, EdgeRecord> edges = BuildEdges(corners);

            bool zipped = _GapTolerance > 0f &&
                          ShellGapZipper.Zip(positions, origins, corners, BoundaryEdges(edges, corners), (_A, _B) => edges.ContainsKey(EdgeKey(_A, _B)), _GapTolerance);

            if (zipped)
            {
                RemoveDegenerate(origins, corners);
                edges = BuildEdges(corners);
            }

            result.Zipped = zipped;

            int count = origins.Count;

            //the surface the winding counts: the source triangle where zipping moved none of its corners
            Triangle[] geometry = new Triangle[count];

            for (int i = 0; i < count; i++)
            {
                Vector3 a = positions[corners[i * 3]], b = positions[corners[i * 3 + 1]], c = positions[corners[i * 3 + 2]];

                Triangle origin = origins[i];

                geometry[i] = !zipped || (origin.V1 == a && origin.V2 == b && origin.V3 == c) ? origin : new Triangle(a, b, c);
            }

            int[] parent = new int[count];

            for (int i = 0; i < count; i++)
                parent[i] = i;

            //edge-connected: every triangle joins the first one that walked each of its edges
            for (int i = 0; i < count; i++)
                for (int k = 0; k < 3; k++)
                    Union(parent, i, edges[EdgeKey(corners[i * 3 + k], corners[i * 3 + (k + 1) % 3])].FirstTriangle);

            Dictionary<int, List<int>> shells = new Dictionary<int, List<int>>();

            for (int i = 0; i < count; i++)
            {
                int root = Find(parent, i);

                if (!shells.TryGetValue(root, out List<int> members))
                    shells.Add(root, members = new List<int>());

                members.Add(i);
            }

            Dictionary<int, bool> shellIsClosed = new Dictionary<int, bool>(shells.Count);

            foreach (int root in shells.Keys)
                shellIsClosed.Add(root, true);

            foreach (EdgeRecord record in edges.Values)
                if (record.Net != 0)
                    shellIsClosed[Find(parent, record.FirstTriangle)] = false;

            foreach (KeyValuePair<int, List<int>> shell in shells)
            {
                if (_IsHollow != null && shell.Value.Exists(_Index => _IsHollow(origins[_Index])))
                    continue;

                if (shellIsClosed[shell.Key])
                {
                    AddBody(result, shell.Value.Select(_Index => geometry[_Index]).ToList());

                    continue;
                }

                OpenShell open = BuildOpenShell(shell.Value, geometry, origins, corners, edges, positions, _GapTolerance);

                //every hole too narrow for a cell: the shell and its caps are a body of their own
                if (open.ClosedByItself)
                {
                    AddBody(result, open.Triangles.Concat(open.CapTriangles).ToList());

                    continue;
                }

                result.OpenShells.Add(open);
            }

            return result;
        }

        #endregion

        #region Service methods

        static void AddBody(Result _Result, List<Triangle> _Chain)
        {
            if (SignedVolume(_Chain) > 0.0)
            {
                _Result.ClosedTriangles.AddRange(_Chain);
                _Result.ClosedShellsCount++;
            }
            else
            {
                _Result.InwardShellsCount++;
                _Result.InwardTriangles.AddRange(_Chain);
                _Result.InwardShellsBounds.Add(ExtensionBounds.TrianglesBounds(_Chain.ToArray()));
            }
        }

        //the volume a closed chain encloses, positive when its faces point out of it; near zero (a double-sided face) is
        //not positive
        static double SignedVolume(List<Triangle> _Chain)
        {
            Vector3 origin = _Chain[0].V1;
            Bounds  bounds = new Bounds(origin, Vector3.zero);
            double  sum    = 0.0;

            foreach (Triangle triangle in _Chain)
            {
                Vector3 a = triangle.V1 - origin, b = triangle.V2 - origin, c = triangle.V3 - origin;

                sum += (double) a.x   * ((double) b.y * c.z - (double) b.z * c.y)
                       - (double) a.y * ((double) b.x * c.z - (double) b.z * c.x)
                       + (double) a.z * ((double) b.x * c.y - (double) b.y * c.x);

                bounds.Encapsulate(triangle.V2);
                bounds.Encapsulate(triangle.V3);
            }

            double extent = Mathf.Max(bounds.size.x, Mathf.Max(bounds.size.y, bounds.size.z));
            double volume = sum / 6.0;

            return volume > 1e-9 * extent * extent * extent ? volume : 0.0;
        }

        static Dictionary<long, EdgeRecord> BuildEdges(List<int> _Corners)
        {
            int count = _Corners.Count / 3;

            Dictionary<long, EdgeRecord> edges = new Dictionary<long, EdgeRecord>(count * 2, EdgeKeyComparer.Instance);

            for (int i = 0; i < count; i++)
            {
                for (int k = 0; k < 3; k++)
                {
                    int from = _Corners[i * 3 + k];
                    int to   = _Corners[i * 3 + (k + 1) % 3];

                    long key = EdgeKey(from, to);

                    if (edges.TryGetValue(key, out EdgeRecord record))
                        record.Net += from < to ? 1 : -1;
                    else
                        record = new EdgeRecord { Net = from < to ? 1 : -1, FirstTriangle = i };

                    if (from < to)
                        record.ForwardWalker = i + 1;
                    else
                        record.BackwardWalker = i + 1;

                    edges[key] = record;
                }
            }

            return edges;
        }

        //every unbalanced edge in its unmatched direction, with a triangle that walks it so
        static List<(int From, int To, int Triangle)> BoundaryEdges(Dictionary<long, EdgeRecord> _Edges, List<int> _Corners)
        {
            List<(int From, int To, int Triangle)> boundary = new List<(int, int, int)>();

            foreach (KeyValuePair<long, EdgeRecord> edge in _Edges)
            {
                if (edge.Value.Net == 0)
                    continue;

                int low  = (int) (edge.Key >> 32);
                int high = (int) (edge.Key & 0xFFFFFFFFL);

                boundary.Add(edge.Value.Net > 0 ? (low, high, edge.Value.ForwardWalker - 1) : (high, low, edge.Value.BackwardWalker - 1));
            }

            return boundary;
        }

        static void RemoveDegenerate(List<Triangle> _Origins, List<int> _Corners)
        {
            int kept = 0;

            for (int i = 0; i < _Origins.Count; i++)
            {
                int a = _Corners[i * 3], b = _Corners[i * 3 + 1], c = _Corners[i * 3 + 2];

                if (a == b || b == c || c == a)
                    continue;

                _Origins[kept]         = _Origins[i];
                _Corners[kept * 3]     = a;
                _Corners[kept * 3 + 1] = b;
                _Corners[kept * 3 + 2] = c;
                kept++;
            }

            _Origins.RemoveRange(kept, _Origins.Count     - kept);
            _Corners.RemoveRange(kept * 3, _Corners.Count - kept * 3);
        }

        static OpenShell BuildOpenShell(
                List<int>                    _Members,
                Triangle[]                   _Geometry,
                List<Triangle>               _Origins,
                List<int>                    _Corners,
                Dictionary<long, EdgeRecord> _Edges,
                List<Vector3>                _Positions,
                float                        _GapTolerance
            )
        {
            Triangle[]        shellTriangles = new Triangle[_Members.Count];
            HashSet<Triangle> sources        = new HashSet<Triangle>();

            //the unmatched direction of every unbalanced edge, once per unit of imbalance
            Dictionary<int, List<int>> boundaryNext = new Dictionary<int, List<int>>();
            HashSet<long>              visitedKeys  = new HashSet<long>(EdgeKeyComparer.Instance);
            int                        edgesLeft    = 0;

            for (int m = 0; m < _Members.Count; m++)
            {
                int i = _Members[m];

                shellTriangles[m] = _Geometry[i];
                sources.Add(_Origins[i]);

                for (int k = 0; k < 3; k++)
                {
                    int  from = _Corners[i * 3 + k];
                    int  to   = _Corners[i * 3 + (k + 1) % 3];
                    long key  = EdgeKey(from, to);

                    if (!visitedKeys.Add(key))
                        continue;

                    int net = _Edges[key].Net;

                    if (net == 0)
                        continue;

                    int low  = Math.Min(from, to);
                    int high = Math.Max(from, to);

                    (int start, int end) = net > 0 ? (low, high) : (high, low);

                    for (int n = 0; n < Math.Abs(net); n++)
                    {
                        if (!boundaryNext.TryGetValue(start, out List<int> nexts))
                            boundaryNext.Add(start, nexts = new List<int>());

                        nexts.Add(end);
                        edgesLeft++;
                    }
                }
            }

            List<Vector3[]> holes = new List<Vector3[]>();

            //chain the directed boundary edges into loops; at a pinch vertex any continuation closes some loop, and the
            //caps only need the loops' union to be the boundary. The starts come off a stack rather than a scan of the
            //map, so a shell whose boundary is most of its edges (an inconsistently wound sheet) stays linear.
            Stack<int> starts = new Stack<int>(boundaryNext.Keys);

            while (edgesLeft > 0 && starts.Count > 0)
            {
                int start = starts.Peek();

                if (boundaryNext[start].Count == 0)
                {
                    starts.Pop();

                    continue;
                }

                List<Vector3> loop    = new List<Vector3>();
                int           current = start;

                do
                {
                    loop.Add(_Positions[current]);

                    List<int> nexts = boundaryNext[current];
                    int       next  = nexts[nexts.Count - 1];

                    nexts.RemoveAt(nexts.Count - 1);
                    edgesLeft--;

                    current = next;
                } while (current != start && boundaryNext.TryGetValue(current, out List<int> more) && more.Count > 0);

                holes.Add(loop.ToArray());
            }

            return new OpenShell(shellTriangles, sources, holes, _GapTolerance);
        }

        static int WeldId(Dictionary<Vector3, int> _VertexIds, List<Vector3> _Positions, Vector3 _Position)
        {
            if (!_VertexIds.TryGetValue(_Position, out int id))
            {
                id = _Positions.Count;
                _VertexIds.Add(_Position, id);
                _Positions.Add(_Position);
            }

            return id;
        }

        static long EdgeKey(int _A, int _B)
        {
            return _A < _B ? ((long) _A << 32) | (uint) _B : ((long) _B << 32) | (uint) _A;
        }

        static int Find(int[] _Parent, int _Index)
        {
            while (_Parent[_Index] != _Index)
            {
                _Parent[_Index] = _Parent[_Parent[_Index]];
                _Index          = _Parent[_Index];
            }

            return _Index;
        }

        static void Union(int[] _Parent, int _A, int _B)
        {
            int rootA = Find(_Parent, _A);
            int rootB = Find(_Parent, _B);

            if (rootA != rootB)
                _Parent[rootA] = rootB;
        }

        #endregion

    }
}