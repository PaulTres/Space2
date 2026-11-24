#if UNITY_EDITOR

using System;
using System.Collections.Generic;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.Elements
{
    partial class Fork
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
            try
            {
                foreach (KeyValuePair<ForkChildOctIndex, Node> childData in ChildrenMap)
                {
                    childData.Value.FillGizmosDrawData(
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
                }
            }
            //Crutch way to suppress "Collection was modified" error for edit mode.
            catch (Exception)
            {
            }
        }
    }
}

#endif