using Nav3D.Common;
using Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.SpatialStorages;
using Nav3D.SpaceProcessing.ObstacleProcessing.UtilityTypes;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree
{
    //Which bodies have an inside: every closed shell, and an open shell whose every hole other obstacle geometry closes.
    partial class GlobalOctree
    {

        #region Properties

        //The A/B switch of interior classification: off, every free cell is FREE as before interiors existed. It exists so a
        //gate can measure both arms on one build (a target inside a body answered at the endpoint against the whole
        //component exhausted), not as a setting.
        internal static bool InteriorClassificationEnabled { get; set; } = true;

        //The widest gap that does not open a shell: no cell of the world's min bucket size fits through a narrower one.
        //The world's setting, not the finest region's, so a cluster's analysis is the same at bake time and at load.
        internal float ShellGapTolerance => m_MinBucketSizeBase;

        #endregion

        #region Attributes

        //Whether other geometry closes an open shell, decided once per operation: the geometry does not change while an
        //operation builds, and a scene-spanning shell (a terrain) would otherwise be judged again for every root the
        //complement lane builds with a storage of its own. Cleared when an operation starts (ResetClosureVerdicts).
        readonly System.Collections.Concurrent.ConcurrentDictionary<ShellAnalysis.OpenShell, System.Lazy<bool>> m_ClosureVerdicts =
            new System.Collections.Concurrent.ConcurrentDictionary<ShellAnalysis.OpenShell, System.Lazy<bool>>();

        #endregion

        #region Internal methods

        //The solids an operation's construction classifies its free cells against, with open shells judged against the
        //geometry as it stands once this operation is published.
        internal SolidRegions CreateSolidRegions(IEnumerable<TriangleCluster> _Clusters, GeometryDelta _Delta)
        {
            return SolidRegions.Create(
                    _Clusters,
                    ShellGapTolerance,
                    (_Shell, _Cluster) => m_ClosureVerdicts.GetOrAdd(
                                                                _Shell,
                                                                _Key => new System.Lazy<bool>(() => IsClosedByOtherGeometry(_Key, _Cluster, _Delta ?? GeometryDelta.None))
                                                            )
                                                           .Value
                );
        }

        internal void ResetClosureVerdicts()
        {
            m_ClosureVerdicts.Clear();
        }

        //Whether other geometry closes every hole of _Shell that the shell does not close itself. Each hole is capped; the
        //cap is closed where each of its sample points lies within a min bucket of other geometry (no cell fits between),
        //inside another closed body, or behind another surface — the first thing a ray from it meets, going into the
        //shell, is other geometry seen from its back side, the way a building's sunk base sits under the terrain. A point
        //it misses is an opening. Hole boundaries are sampled first, so a hole nothing is near (a terrain's rim, an open
        //gate) answers after one sample.
        internal bool IsClosedByOtherGeometry(ShellAnalysis.OpenShell _Shell, TriangleCluster _Own, GeometryDelta _Delta, bool _BoundaryOnly = false, float? _Pitch = null)
        {
            float pitch = _Pitch ?? m_MinBucketSizeReal;

            List<TriangleCluster> others = OtherGeometryAround(_Shell.CappedBounds, pitch, _Delta);

            float reach = _Shell.CappedBounds.size.magnitude;

            foreach ((Vector3 point, Vector3 inward) in CapSamples(_Shell, pitch, _BoundaryOnly))
                if (!IsSealed(point, inward, _Shell, others, pitch, reach))
                    return false;

            return true;
        }

        //An operation that adds, moves or removes geometry at an open shell's hole can open or close that shell, which
        //flips every cell of its inside — cells the operation's own cut does not reach. Such a shell's capped bounds join
        //the cut and its roots the footprint, so those cells are rebuilt and derived again. A shell whose hole boundary is
        //sealed neither before nor after the operation cannot be closed by it (a terrain's rim) and is left alone.
        internal GeometryDelta WithClosureChanges(GeometryDelta _Delta, HashSet<Vector3Int> _Footprint)
        {
            if (!InteriorClassificationEnabled)
                return _Delta;

            float pitch = m_MinBucketSizeReal;

            List<TriangleCluster> changed = _Delta.AddedClusters.Concat(_Delta.RemovedClusters).ToList();

            //where the geometry changed: the operation's own cut when it knows one (the partial update), else the
            //changed clusters' whole bounds
            List<Bounds> changedVolumes = _Delta.CutVolumes != null ? _Delta.CutVolumes.ToList() : changed.Select(_Cluster => _Cluster.Bounds).ToList();

            HashSet<TriangleCluster> neighbours = new HashSet<TriangleCluster>();

            foreach (Bounds volume in changedVolumes)
                m_NavTree.TriangleClusterStorage.CollectCrossingBoundables(volume, neighbours);

            neighbours.ExceptWith(changed);

            List<Bounds> closureChanges = new List<Bounds>();

            int footprintBefore = _Footprint.Count;

            foreach (TriangleCluster cluster in neighbours)
            foreach (ShellAnalysis.OpenShell shell in cluster.GetSolid(ShellGapTolerance).OpenShells)
            {
                if (shell.CapTriangles.Length == 0 || !changedVolumes.Any(_Volume => HoleReaches(shell, _Volume, pitch)))
                    continue;

                if (!IsClosedByOtherGeometry(shell, cluster, GeometryDelta.None, true) && !IsClosedByOtherGeometry(shell, cluster, _Delta, true))
                    continue;

                closureChanges.Add(shell.CappedBounds);
                _Footprint.UnionWith(UtilsSpatialGrid.GetCrossedBucketsIndices(shell.CappedBounds, RootSize));
            }

            DevCountWork("closure: open shells widened into the cut", closureChanges.Count);

            //The partial update replaces a cluster with a cut narrower than its bounds, so a cavity of the obstacle's OWN
            //body that the edit opens or seals (a door through a hollow box's wall) has its cells outside the cut — the
            //whole replacement never had this, its cut being the cluster's bounds. A solid volume present on one side of
            //the replacement only (a closed shell, or an open shell other geometry seals) joins the cut.
            if (_Delta.CutVolumes != null)
            {
                int ownBefore = closureChanges.Count;

                List<(Bounds Volume, int Shape)> before = new List<(Bounds, int)>();
                List<(Bounds Volume, int Shape)> after  = new List<(Bounds, int)>();

                foreach (TriangleCluster removed in _Delta.RemovedClusters)
                    CollectSolidVolumes(removed, GeometryDelta.None, before);

                foreach (TriangleCluster added in _Delta.AddedClusters)
                    CollectSolidVolumes(added, _Delta, after);

                foreach ((Bounds volume, int shape) in before)
                    if (!after.Any(_Other => _Other.Shape == shape && SameVolume(volume, _Other.Volume, pitch)))
                        closureChanges.Add(volume);

                foreach ((Bounds volume, int shape) in after)
                    if (!before.Any(_Other => _Other.Shape == shape && SameVolume(volume, _Other.Volume, pitch)))
                        closureChanges.Add(volume);

                for (int i = ownBefore; i < closureChanges.Count; i++)
                    _Footprint.UnionWith(UtilsSpatialGrid.GetCrossedBucketsIndices(closureChanges[i], RootSize));

                DevCountWork("closure: own solids widened into the cut", closureChanges.Count - ownBefore);
            }

            DevCountWork("closure: footprint roots added", _Footprint.Count - footprintBefore);

            return closureChanges.Count == 0 ? _Delta : new GeometryDelta(_Delta.RemovedClusters, _Delta.AddedClusters, closureChanges, _Delta.CutVolumes);
        }

        #endregion

        #region Service methods

        //The volumes whose cells the replacement can flip outside the cut, each with its shape. NOT the outward closed
        //shells: a closed surface edited within a box cannot change the winding of a point beyond it, however many shells
        //the edit splits it into or merges, so a crater on a pavilion — even one that frees a clump of its ragged rim as a
        //shell of its own — flips nothing beyond its cut. What can: each sealed cavity (an inward shell, ignored by the
        //winding so that its inside stays solid) as its own box — a door through a hollow wall merges the cavity's inward
        //shell into the outward one and the cavity's cells go free, a wall sealed again makes them interior, and a cavity
        //carved inside a solid widens by its own small box; and each capped open shell that other geometry seals, judged
        //on the geometry _Delta leaves (None = as registered).
        void CollectSolidVolumes(TriangleCluster _Cluster, GeometryDelta _Delta, List<(Bounds Volume, int Shape)> _Result)
        {
            ClusterSolid solid = _Cluster.GetSolid(ShellGapTolerance);

            foreach (Bounds cavity in solid.InwardShellsBounds)
                _Result.Add((cavity, -1));

            foreach (ShellAnalysis.OpenShell shell in solid.OpenShells)
            {
                if (shell.CapTriangles.Length > 0 && IsClosedByOtherGeometry(shell, _Cluster, _Delta, true))
                    _Result.Add((shell.CappedBounds, 0));
            }
        }

        //within a min bucket is the same volume
        static bool SameVolume(Bounds _A, Bounds _B, float _Pitch)
        {
            return (_A.min - _B.min).sqrMagnitude <= _Pitch * _Pitch && (_A.max - _B.max).sqrMagnitude <= _Pitch * _Pitch;
        }

        //whether a hole's boundary passes within reach of a box — a change there can close or open it
        static bool HoleReaches(ShellAnalysis.OpenShell _Shell, Bounds _Box, float _Pitch)
        {
            Bounds reach = _Box;
            reach.Expand(4f * _Pitch);

            if (!reach.Intersects(_Shell.HolesBounds))
                return false;

            for (int h = 0; h < _Shell.Holes.Count; h++)
            {
                if (_Shell.HoleClosedByItself[h])
                    continue;

                Vector3[] hole = _Shell.Holes[h];

                for (int i = 0; i < hole.Length; i++)
                    if (reach.IntersectRay(new Ray(hole[i], hole[(i + 1) % hole.Length] - hole[i]), out float distance) &&
                        distance <= Vector3.Distance(hole[i], hole[(i + 1) % hole.Length]))
                        return true;
            }

            return false;
        }

        //the geometry around a shell as this operation leaves it: registered clusters minus the ones it removes, plus the
        //ones it adds — the shell's own cluster included, its own triangles excluded by the queries themselves
        List<TriangleCluster> OtherGeometryAround(Bounds _Bounds, float _Pitch, GeometryDelta _Delta)
        {
            Bounds area = _Bounds;
            area.Expand(2f * _Pitch);

            HashSet<TriangleCluster> clusters = new HashSet<TriangleCluster>();

            m_NavTree.TriangleClusterStorage.CollectCrossingBoundables(area, clusters);

            clusters.ExceptWith(_Delta.RemovedClusters);

            foreach (TriangleCluster added in _Delta.AddedClusters)
                if (added.Bounds.Intersects(area))
                    clusters.Add(added);

            return clusters.ToList();
        }

        bool IsSealed(Vector3 _Point, Vector3 _Inward, ShellAnalysis.OpenShell _Shell, List<TriangleCluster> _Others, float _Pitch, float _Reach)
        {
            //a box reaching a min bucket each way: geometry that close leaves no room for a cell
            Bounds cell = new Bounds(_Point, Vector3.one * (2f * _Pitch));

            int winding = 0;

            foreach (TriangleCluster cluster in _Others)
            {
                if (cluster.CrossesAnyTriangleExcept(cell, _Shell.Members))
                    return true;

                winding += cluster.GetSolid(ShellGapTolerance).ClosedWinding(_Point);
            }

            if (winding > 0)
                return true;

            Triangle nearest         = null;
            float    nearestDistance = _Reach;

            foreach (TriangleCluster cluster in _Others)
            {
                if (!cluster.TryRaycastFirst(_Point, _Inward, nearestDistance, _Shell.Members, out Triangle hit, out float distance) || distance >= nearestDistance)
                    continue;

                nearest         = hit;
                nearestDistance = distance;
            }

            if (nearest == null || Vector3.Dot(nearest.Normal, _Inward) <= 0f)
                return false;

            //the other surface counts only if the ray meets it before the shell's own
            return !_Shell.ShellStorage.TryRaycastFirst(_Point, _Inward, nearestDistance, null, out _, out _);
        }

        //Points over every cap of a hole the shell does not close itself, each with the direction into the capped shell
        //(against the cap's normal, which points out of it): the hole's boundary first, nudged off the edge onto the cap,
        //then the cap's interior at the cell pitch.
        static IEnumerable<(Vector3 Point, Vector3 Inward)> CapSamples(ShellAnalysis.OpenShell _Shell, float _Pitch, bool _BoundaryOnly)
        {
            float nudge = _Pitch * 0.05f;

            List<Triangle> caps = new List<Triangle>();

            for (int c = 0; c < _Shell.CapTriangles.Length; c++)
                if (!_Shell.HoleClosedByItself[_Shell.CapHoles[c]])
                    caps.Add(_Shell.CapTriangles[c]);

            foreach (Triangle cap in caps)
            {
                Vector3 inward = -cap.Normal;

                //the fan's apex is V1, the hole's boundary edge is V2–V3
                int steps = Mathf.Max(1, Mathf.CeilToInt(Vector3.Distance(cap.V2, cap.V3) / _Pitch));

                for (int i = 0; i <= steps; i++)
                {
                    Vector3 onEdge = Vector3.Lerp(cap.V2, cap.V3, (float) i / steps);

                    yield return (Vector3.MoveTowards(onEdge, cap.V1, nudge) + inward * nudge, inward);
                }
            }

            if (_BoundaryOnly)
                yield break;

            foreach (Triangle cap in caps)
            {
                Vector3 inward = -cap.Normal;

                int steps = Mathf.Max(1, Mathf.CeilToInt(Mathf.Max(Vector3.Distance(cap.V1, cap.V2), Vector3.Distance(cap.V1, cap.V3)) / _Pitch));

                for (int i = 0; i < steps; i++)
                    for (int j = 0; i + j < steps; j++)
                    {
                        float a = (i + 0.33f) / steps;
                        float b = (j + 0.33f) / steps;

                        yield return (cap.V1 + (cap.V2 - cap.V1) * a + (cap.V3 - cap.V1) * b + inward * nudge, inward);
                    }
            }
        }

        #endregion

    }
}