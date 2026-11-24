#if UNITY_EDITOR

namespace Nav3D.Obstacles
{
    partial class ObstacleStorage
    {
        #region Properties

        public int OctreeLayersCount => m_Octree.LayersCount;

        #endregion

        #region Public methods

        public void FillGizmosDrawDataForAll(
                Common.Debug.GizmosDrawData _GizmosDrawData,
                bool                        _DrawOccupiedLeaves,
                bool                        _DrawFreeLeaves,
                bool                        _DrawGraph,
                bool                        _DrawRoots,
                int                         _DrawOccupiedLeavesAll,
                int                         _DrawOccupiedLeavesLayerNumber,
                int                         _DrawFreeLeavesAll,
                int                         _DrawFreeLeavesLayerNumber,
                int                         _DrawGraphNodesAll,
                int                         _DrawGraphLayerNumber
            )
        {
            m_Octree.FillGizmosDrawDataForAll(
                    _GizmosDrawData,
                    _DrawOccupiedLeaves,
                    _DrawFreeLeaves,
                    _DrawGraph,
                    _DrawRoots,
                    _DrawOccupiedLeavesAll,
                    _DrawOccupiedLeavesLayerNumber,
                    _DrawFreeLeavesAll,
                    _DrawFreeLeavesLayerNumber,
                    _DrawGraphNodesAll,
                    _DrawGraphLayerNumber
                );
        }

        public void FillGizmosDrawData(
                Common.Debug.GizmosDrawData _GizmosDrawData,
                int                         _ObstacleID,
                bool                        _DrawOccupiedLeaves,
                bool                        _DrawFreeLeaves,
                bool                        _DrawGraph,
                bool                        _DrawRoots,
                int                         _DrawOccupiedLeavesAll,
                int                         _DrawOccupiedLeavesLayerNumber,
                int                         _DrawFreeLeavesAll,
                int                         _DrawFreeLeavesLayerNumber,
                int                         _DrawGraphNodesAll,
                int                         _DrawGraphLayerNumber
            )
        {
            m_Octree.FillGizmosDrawData(
                    _GizmosDrawData,
                    _ObstacleID,
                    _DrawOccupiedLeaves,
                    _DrawFreeLeaves,
                    _DrawGraph,
                    _DrawRoots,
                    _DrawOccupiedLeavesAll,
                    _DrawOccupiedLeavesLayerNumber,
                    _DrawFreeLeavesAll,
                    _DrawFreeLeavesLayerNumber,
                    _DrawGraphNodesAll,
                    _DrawGraphLayerNumber
                );
        }

        public void FillOpenFacesGizmos(Common.Debug.GizmosDrawData _GizmosDrawData)
        {
            m_Octree.FillOpenFacesGizmos(_GizmosDrawData);
        }

        #endregion
    }
}

#endif