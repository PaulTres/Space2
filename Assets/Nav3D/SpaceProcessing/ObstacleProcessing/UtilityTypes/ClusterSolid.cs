using Nav3D.Common;
using Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.SpatialStorages;
using System.Collections.Generic;
using UnityEngine;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.UtilityTypes
{
    //What a cluster encloses: its closed shells as one ray-queryable store, plus its open shells with their holes. A shell
    //never spans two clusters (it is edge-connected, and clustering links anything that close), so a cluster is the whole
    //answer for every shell it holds.
    sealed class ClusterSolid
    {

        #region Constants

        //Directions that no grid-aligned mesh edge or face diagonal runs along; a ray that still lands on an edge moves on
        //to the next one.
        static readonly Vector3[] s_RayDirections =
        {
            new Vector3(0.3141593f, 0.7071068f, 0.6335811f).normalized,
            new Vector3(-0.5772157f, 0.2718282f, 0.7692309f).normalized,
            new Vector3(0.6931472f, -0.4142136f, 0.5890486f).normalized,
            new Vector3(-0.2236068f, -0.6180340f, -0.7536287f).normalized,
            new Vector3(0.8414710f, 0.1411200f, -0.5210953f).normalized
        };

        public static ClusterSolid Empty { get; } = new ClusterSolid(null, 0, new List<ShellAnalysis.OpenShell>(), new List<Bounds>());

        #endregion

        #region Properties

        //null when the cluster holds no closed shell, or when the closed surface is the cluster's own parts (ClosedParts)
        public TriangleStorage ClosedStorage { get; }

        //the closed surface as the cluster's own parts, when every triangle of the cluster is on a closed outward shell
        //and nothing was zipped: no second BVH over the body, the winding sums the counts of the parts a ray reaches
        public PartForest ClosedForest { get; }

        //with ClosedParts: the inward shells among them (a sealed cavity's surface), whose count the winding gives back —
        //an inward shell is dropped from a body, and inside a cavity it would otherwise cancel the body around it
        public TriangleStorage InwardStorage { get; }

        public bool HasClosedShells => ClosedStorage != null || ClosedForest != null;

        public Bounds ClosedBounds { get; }

        public int ClosedShellsCount { get; }

        //the sealed cavities among the closed shells, each as its own box: an inward shell merging into the outward one (a
        //door through a wall) leaves the closed count and bounds as they were, and this is what tells the two bodies apart
        public IReadOnlyList<Bounds> InwardShellsBounds { get; }

        public int InwardShellsCount => InwardShellsBounds.Count;

        public IReadOnlyList<ShellAnalysis.OpenShell> OpenShells { get; }

        #endregion

        #region Constructors

        ClusterSolid(List<Triangle> _ClosedTriangles, int _ClosedShellsCount, List<ShellAnalysis.OpenShell> _OpenShells, List<Bounds> _InwardShellsBounds)
        {
            ClosedShellsCount  = _ClosedShellsCount;
            OpenShells         = _OpenShells;
            InwardShellsBounds = _InwardShellsBounds;

            if (_ClosedTriangles == null || _ClosedTriangles.Count == 0)
                return;

            Triangle[] closed = _ClosedTriangles.ToArray();

            ClosedStorage = new TriangleStorage(closed);
            ClosedBounds  = ExtensionBounds.TrianglesBounds(closed);
        }

        ClusterSolid(
                PartForest                    _ClosedForest,
                Bounds                        _ClosedBounds,
                int                           _ClosedShellsCount,
                List<ShellAnalysis.OpenShell> _OpenShells,
                List<Triangle>                _InwardTriangles,
                List<Bounds>                  _InwardShellsBounds
            )
        {
            ClosedShellsCount  = _ClosedShellsCount;
            OpenShells         = _OpenShells;
            InwardShellsBounds = _InwardShellsBounds;
            ClosedForest       = _ClosedForest;
            ClosedBounds       = _ClosedBounds;
            InwardStorage      = _InwardTriangles.Count == 0 ? null : new TriangleStorage(_InwardTriangles.ToArray());
        }

        #endregion

        #region Public methods

        //_Forest / _PartsBounds: the cluster's own parts (indexed) and raw bounds, when it has them — a cluster whose every
        //triangle lies on a closed outward shell, with nothing zipped, counts its winding through the parts' BVHs and
        //builds no second storage over the body (the roadmap's "reuse the solids BVH where equivalence is proved").
        public static ClusterSolid Build(
                ICollection<Triangle>      _Triangles,
                System.Predicate<Triangle> _IsHollow,
                float                      _GapTolerance = 0f,
                PartForest                 _Forest       = null,
                Bounds?                    _PartsBounds  = null
            )
        {
            System.Diagnostics.Stopwatch watch = System.Diagnostics.Stopwatch.StartNew();

            ShellAnalysis.Result shells = ShellAnalysis.Analyze(_Triangles, _IsHollow, _GapTolerance);

            GlobalOctree.GlobalOctree.DevWorkSink?.Invoke("solids: shell analysis ms", watch.ElapsedMilliseconds);
            GlobalOctree.GlobalOctree.DevWorkSink?.Invoke("solids: triangles analysed", _Triangles.Count);

            if (shells.ClosedTriangles.Count == 0 && shells.OpenShells.Count == 0)
                return Empty;

            //every triangle on a closed shell, outward or inward, none rebuilt by zipping: the parts ARE the surface
            if (_Forest                                                     != null && _PartsBounds.HasValue && !shells.Zipped && shells.OpenShells.Count == 0 &&
                shells.ClosedTriangles.Count + shells.InwardTriangles.Count == _Triangles.Count)
            {
                GlobalOctree.GlobalOctree.DevWorkSink?.Invoke("solids: closed surface = the cluster's parts", 1);

                return new ClusterSolid(_Forest, _PartsBounds.Value, shells.ClosedShellsCount, shells.OpenShells, shells.InwardTriangles, shells.InwardShellsBounds);
            }

            watch.Restart();

            ClusterSolid solid = new ClusterSolid(shells.ClosedTriangles, shells.ClosedShellsCount, shells.OpenShells, shells.InwardShellsBounds);

            GlobalOctree.GlobalOctree.DevWorkSink?.Invoke("solids: closed storage build ms", watch.ElapsedMilliseconds);

            return solid;
        }

        //The winding number of _Point with respect to the closed shells; 0 outside their bounds. A point every direction
        //grazes counts as outside — the side that leaves a cell navigable, as it was before interiors existed.
        public int ClosedWinding(Vector3 _Point)
        {
            if (!HasClosedShells || !ClosedBounds.Contains(_Point))
                return 0;

            if (ClosedForest == null)
                return Winding(ClosedStorage, _Point);

            //the whole surface minus the inward shells: inside a sealed cavity the body's +1 and the cavity's −1 cancel
            //over the parts, and giving the −1 back makes the cavity solid with the body, as required by the solid-body rule
            return WindingOfParts(ClosedForest, _Point) - (InwardStorage == null ? 0 : Winding(InwardStorage, _Point));
        }

        //the same count summed over the storages of the parts the ray reaches: one direction for all of them, the next
        //when any part grazes
        static int WindingOfParts(PartForest _Forest, Vector3 _Point)
        {
            foreach (Vector3 direction in s_RayDirections)
                if (_Forest.TryCountSignedCrossings(_Point, direction, out int winding))
                    return winding;

            return 0;
        }

        //the winding number of _Point with respect to a surface without boundary, counted along the first direction no
        //hit grazes; 0 when every one does
        public static int Winding(TriangleStorage _Storage, Vector3 _Point)
        {
            foreach (Vector3 direction in s_RayDirections)
                if (_Storage.TryCountSignedCrossings(_Point, direction, out int winding))
                    return winding;

            return 0;
        }

        #endregion

    }
}