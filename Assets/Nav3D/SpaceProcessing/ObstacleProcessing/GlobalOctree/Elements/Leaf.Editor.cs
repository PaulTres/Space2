#if UNITY_EDITOR

using Nav3D.Common;
using UnityEngine;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.Elements
{
    partial class Leaf
    {
        public override void FillGizmosDrawData(
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
            base.FillGizmosDrawData(
                    _GizmosDrawData,
                    _DrawOccupiedLeaves,
                    _DrawFreeLeaves,
                    _DrawGraph,
                    _DrawOccupiedLeavesAll,
                    _DrawOccupiedLeavesLayerNumber,
                    _DrawFreeLeavesAll,
                    _DrawFreeLeavesLayerNumber,
                    _DrawGraphNodesAll,
                    _DrawGraphLayerNumber
                );

            if (Occupied || !_DrawGraph)
                return;

            if (_DrawGraphNodesAll == 0 || _DrawGraphLayerNumber == m_GridLayer)
            {
                _GizmosDrawData.Add(new Common.Debug.GizmosSphereSolid(Bounds.center, Bounds.GetMinSize() * 0.05f, Color.magenta));

                //the set is created on first use and the gizmo may run before anything wired this leaf; FreeNeighbors
                //answers that with an empty array and takes the set's own lock when there is one
                foreach (Leaf leaf in FreeNeighbors)
                {
                    Vector3 contactPoint = GetAdjacentContactPointInternal(leaf);

                    _GizmosDrawData.Add(new Common.Debug.GizmosLine(Bounds.center, contactPoint, Color.magenta));
                    _GizmosDrawData.Add(new Common.Debug.GizmosLine(contactPoint, leaf.Bounds.center, Color.magenta));
                }
            }
        }
    }
}

#endif