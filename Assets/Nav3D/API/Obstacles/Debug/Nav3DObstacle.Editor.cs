#if UNITY_EDITOR

using Nav3D.Obstacles;
using System.Diagnostics;
using UnityEngine;

namespace Nav3D.API
{
    public partial class Nav3DObstacle : IObstacleGizmosCache
    {

        #region Serialized fields

        [SerializeField] bool m_DrawOccupiedLeaves;
        [SerializeField] bool m_DrawFreeLeaves;
        [SerializeField] bool m_DrawGraph;
        [SerializeField] bool m_DrawRoots;

        [SerializeField] int m_OccupiedLeavesDrawingMode; //0-all layers, 1-specific layer
        [SerializeField] int m_OccupiedLeavesDrawingLayerNumber;

        [SerializeField] int m_FreeLeavesDrawingMode; //0-all layers, 1-specific layer
        [SerializeField] int m_FreeLeavesDrawingLayerNumber;

        [SerializeField] int m_GraphNodesDrawingMode; //0-all layers, 1-specific layer
        [SerializeField] int m_GraphNodesDrawingLayerNumber;

        [SerializeField] float m_LastProposedRootSize;

        #endregion

        #region Attributes

        Common.Debug.GizmosDrawData m_GizmosData;

        #endregion

        #region Properties

        public int OctreeLayersCount => ObstacleManager.Instance.GetOctreeLayersMaxCount();

        #endregion

        #region Public methods

        [Conditional("UNITY_EDITOR")]
        internal void ClearGizmosCache()
        {
            m_GizmosData?.Clear();
        }

        //Explicit, and it has to be: a [Conditional] method cannot implement an interface member (CS0629), and the
        //attribute is what makes every call site above compile away. This is the octree's entry point (it resolves
        //obstacle IDs to IObstacleGizmosCache through the registry rather than to this component, so it names nothing
        //above itself).
        void IObstacleGizmosCache.ClearGizmosCache()
        {
            ClearGizmosCache();
        }

        partial void RegisterGizmosCacheEditor()
        {
            ObstacleGizmosCacheRegistry.Register(m_InstanceID, this);
        }

        partial void UnregisterGizmosCacheEditor()
        {
            ObstacleGizmosCacheRegistry.Unregister(m_InstanceID, this);
        }

        partial void OnObstacleAddedEditor()
        {
            ClearGizmosCache();
        }

        partial void OnObstacleRemovedEditor()
        {
            //Removal published: drop the stale gizmo cache so the next OnDrawGizmos re-queries (now-empty) data.
            ClearGizmosCache();
        }

        partial void CheckEditorDoDisableGuard(ref bool _Skip)
        {
            //Gate on play mode only. Unlike OnEnable, by the time OnDisable runs (component checkbox unticked,
            //enabled=false) the "!enabled" test is always true, which used to suppress removal at play time and left the
            //obstacle in the navigation after it was disabled. In edit mode the "!Application.isPlaying" check still
            //keeps this a no-op (ExecuteInEditMode fires OnDisable there too).
            if (!Application.isPlaying)
                _Skip = true;
        }

        #endregion

        #region Unity methods

        void OnDrawGizmos()
        {
            if (!enabled)
                return;

            if (!Application.isPlaying)
                return;

            if (m_GizmosData == null)
                m_GizmosData = new Common.Debug.GizmosDrawData();

            if (m_DrawOccupiedLeaves || m_DrawFreeLeaves || m_DrawGraph || m_DrawRoots)
            {
                if (m_GizmosData.IsClean)
                {
                    //add data
                    ObstacleManager.Instance.FillGizmosDrawData(
                            m_GizmosData,
                            m_InstanceID,
                            m_DrawOccupiedLeaves,
                            m_DrawFreeLeaves,
                            m_DrawGraph,
                            m_DrawRoots,
                            m_OccupiedLeavesDrawingMode,
                            m_OccupiedLeavesDrawingLayerNumber,
                            m_FreeLeavesDrawingMode,
                            m_FreeLeavesDrawingLayerNumber,
                            m_GraphNodesDrawingMode,
                            m_GraphNodesDrawingLayerNumber
                        );
                }
                else
                {
                    m_GizmosData.Draw();
                }
            }
        }

        #endregion

    }
}

#endif