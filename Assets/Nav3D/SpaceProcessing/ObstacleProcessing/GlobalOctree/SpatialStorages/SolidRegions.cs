using Nav3D.Obstacles;
using Nav3D.SpaceProcessing.ObstacleProcessing.UtilityTypes;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using UnityEngine;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.SpatialStorages
{
    //The bodies one operation's construction classifies free cells against: the solids of every cluster in its geometry
    //view, uncut — a ray from a cell has to meet the whole surface, not the part crossing one root. A closed shell facing
    //outward is a body outright; an open one only where other geometry closes every hole it does not close itself, which
    //is asked the first time a cell falls inside its capped bounds and remembered for the operation. The parallel
    //construction tasks read it freely.
    sealed class SolidRegions
    {

        #region Attributes

        readonly ClusterSolid[] m_Solids;

        readonly (ShellAnalysis.OpenShell Shell, TriangleCluster Cluster)[] m_OpenShells;

        readonly Func<ShellAnalysis.OpenShell, TriangleCluster, bool> m_IsClosed;

        readonly ConcurrentDictionary<ShellAnalysis.OpenShell, Lazy<bool>> m_Closed = new ConcurrentDictionary<ShellAnalysis.OpenShell, Lazy<bool>>();

        //the authored sealed volumes as they stood when the operation began
        readonly SealedVolumeSnapshot m_SealedVolumes;

        #endregion

        #region Constructors

        SolidRegions(
                ClusterSolid[]                                             _Solids,
                (ShellAnalysis.OpenShell Shell, TriangleCluster Cluster)[] _OpenShells,
                Func<ShellAnalysis.OpenShell, TriangleCluster, bool>       _IsClosed,
                SealedVolumeSnapshot                                       _SealedVolumes
            )
        {
            m_Solids        = _Solids;
            m_OpenShells    = _OpenShells;
            m_IsClosed      = _IsClosed;
            m_SealedVolumes = _SealedVolumes;
        }

        #endregion

        #region Public methods

        //null when none of the clusters can enclose anything, so a world without bodies pays one null check per cell.
        //_GapTolerance: the world's min bucket size — a gap narrower than it does not open a shell.
        //_IsClosed: whether other geometry closes every hole of an open shell; null leaves every open shell open.
        public static SolidRegions Create(
                IEnumerable<TriangleCluster>                         _Clusters,
                float                                                _GapTolerance,
                Func<ShellAnalysis.OpenShell, TriangleCluster, bool> _IsClosed = null
            )
        {
            if (!GlobalOctree.InteriorClassificationEnabled)
                return null;

            List<ClusterSolid>                                             solids     = new List<ClusterSolid>();
            List<(ShellAnalysis.OpenShell Shell, TriangleCluster Cluster)> openShells = new List<(ShellAnalysis.OpenShell, TriangleCluster)>();

            foreach (TriangleCluster cluster in _Clusters)
            {
                ClusterSolid solid = cluster.GetSolid(_GapTolerance);

                if (solid.HasClosedShells)
                    solids.Add(solid);

                if (_IsClosed == null)
                    continue;

                foreach (ShellAnalysis.OpenShell shell in solid.OpenShells)
                    if (shell.CapTriangles.Length > 0 && HasVolume(shell.CappedBounds))
                        openShells.Add((shell, cluster));
            }

            SealedVolumeSnapshot sealedVolumes = SealedVolumeSnapshot.Current;

            return solids.Count == 0 && openShells.Count == 0 && sealedVolumes.IsEmpty
                ? null
                : new SolidRegions(solids.ToArray(), openShells.ToArray(), _IsClosed, sealedVolumes);
        }

        //A free cell is interior when a sealed volume holds it whole, or its centre is inside a body.
        public bool IsInteriorCell(Bounds _Cell)
        {
            return m_SealedVolumes.Embraces(_Cell) || Contains(_Cell.center);
        }

        //Winding numbers add over bodies, so overlapping solids stay solid. Nothing subtracts: closed shells facing inward
        //are no bodies at all, and a capped open shell adds only where its own winding is positive (a terrain capped at
        //its rim is negative over its valleys, which must not carve a body standing in one).
        public bool Contains(Vector3 _Point)
        {
            int winding = 0;

            foreach (ClusterSolid solid in m_Solids)
                winding += solid.ClosedWinding(_Point);

            foreach ((ShellAnalysis.OpenShell shell, TriangleCluster cluster) in m_OpenShells)
            {
                //the verdict first: it is asked once per shell and operation, answers after one sample for a hole nothing
                //is near, and spares an open shell's capped store — for a terrain, a BVH over the whole of it
                if (!shell.CappedBounds.Contains(_Point) || !IsClosed(shell, cluster))
                    continue;

                winding += Math.Max(0, ClusterSolid.Winding(shell.CappedStorage, _Point));
            }

            return winding > 0;
        }

        #endregion

        #region Service methods

        bool IsClosed(ShellAnalysis.OpenShell _Shell, TriangleCluster _Cluster)
        {
            return m_Closed.TryGetValue(_Shell, out Lazy<bool> cached) ? cached.Value : ResolveClosed(_Shell, _Cluster);
        }

        bool ResolveClosed(ShellAnalysis.OpenShell _Shell, TriangleCluster _Cluster)
        {
            return m_Closed.GetOrAdd(_Shell, _Key => new Lazy<bool>(() => m_IsClosed(_Key, _Cluster))).Value;
        }

        static bool HasVolume(Bounds _Bounds)
        {
            Vector3 size = _Bounds.size;

            return size.x > 1e-4f && size.y > 1e-4f && size.z > 1e-4f;
        }

        #endregion

    }
}