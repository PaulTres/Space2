using System.Collections.Generic;
using UnityEngine;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree
{
    //The publish epoch and the per-root rebuild versions: the one authority every stored annotation derives its
    //validity from — the volumetric clearance and per-edge stores (NavTree.Volumetric.cs) and the hierarchy's region
    //map (NavTree.Hierarchy.cs). A value stamped with an epoch is trusted iff no root of its support was rebuilt after
    //that epoch; nothing is ever cleared by a pass. Lifted out of the volumetric partial on 2026-09-27, because the
    //hierarchy ships in both editions and that partial does not.
    internal sealed partial class NavTree
    {

        #region Attributes

        //incremented once per PublishStagedRegion under the write lock; starts at 1 so that a stamp of 0 can only ever
        //mean "never filled". Read by searches under the read lock
        int m_PublishEpoch = 1;

        //the epoch of the publish that last rebuilt / narrowed / dropped / created the root (absent = never, i.e. 0).
        //Written only under the write lock; an entry outlives its root on purpose — a dropped root changed occupancy too
        readonly Dictionary<Vector3Int, int> m_RootRebuildVersions = new Dictionary<Vector3Int, int>();

        #endregion

        #region Properties

        internal int PublishEpoch => m_PublishEpoch;

        #endregion

        #region Service methods

        //called from PublishStagedRegion under the write lock, before any reader can see the new tree
        void NoteRebuild(HashSet<Vector3Int> _RebuiltRootIndices)
        {
            m_PublishEpoch++;
            ResetPointSolids();

            foreach (Vector3Int rootIndex in _RebuiltRootIndices)
                m_RootRebuildVersions[rootIndex] = m_PublishEpoch;

            MarkRegionMapRoots(_RebuiltRootIndices);
        }

        #endregion

    }
}
