#if UNITY_EDITOR

using UnityEngine;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.Elements
{
    partial class Node
    {
        public virtual void FillGizmosDrawData(
                Common.Debug.GizmosDrawData _GizmosDrawData,
                bool                        _DrawOccupiedLeaves,
                bool                        _DrawFreeLeaves,
                bool                        _DrawGraph,
                int                         _DrawOccupiedLeavesAll,
                int                         _DrawOccupiedLeavesLayerNumber,
                int                         _DrawFreeLeavesAll,
                int                         _DrawFreeLeavesLayerNumber,
                int                         _DrawGraphNodesAll,
                int                         _DrawGraphLayerNumber
            )
        {
            if (_DrawOccupiedLeaves && Occupied)
            {
                if (_DrawOccupiedLeavesAll == 0 || _DrawOccupiedLeavesLayerNumber == m_GridLayer)
                    _GizmosDrawData.Add(new Common.Debug.GizmosWireCube(Bounds.center, Bounds.size, Color.red));

                return;
            }

            if (_DrawFreeLeaves && !Occupied)
            {
                if (_DrawFreeLeavesAll == 0 || _DrawFreeLeavesLayerNumber == m_GridLayer)
                    _GizmosDrawData.Add(new Common.Debug.GizmosWireCube(Bounds.center, Bounds.size, Color.green));
            }
        }
    }
}

#endif