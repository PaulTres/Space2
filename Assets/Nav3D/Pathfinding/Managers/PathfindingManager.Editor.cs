#if UNITY_EDITOR

using UnityEngine;

namespace Nav3D.Pathfinding
{
    public partial class PathfindingManager
    {
        #region Serialized fields

        [SerializeField] bool m_DrawPathsStorage;
        [SerializeField] bool m_DisplayExecutorStats;

        #endregion

        #region Service methods

        // volumetric-paths overview seam (all registered capsule paths as one batched wireframe pass) —
        //implemented in PathfindingManager.Volumetric.Editor.cs; no-op in player builds and in the Lite build.
        partial void DrawVolumetricOverviewHook();

        void DrawStorage()
        {
            if (!m_DrawPathsStorage)
                return;

            m_PathsStorage.Draw();
        }

        void DisplayOrdersExecutorStats()
        {
            GUILayout.Label($"Executor alive orders: {m_PathfindingOrdersExecutor.CurrentAliveOrders}/{m_PathfindingOrdersExecutor.MaxConcurrencyLevel}");

            GUILayout.Label(m_PathfindingOrdersExecutor.GetRunningOrdersInfo());
        }

        #endregion

        #region Unity methods

        void OnDrawGizmos()
        {
            if (!Application.isPlaying || !enabled)
                return;

            DrawStorage();

            DrawVolumetricOverviewHook();
        }

        void OnGUI()
        {
            if (!Application.isPlaying || !enabled)
                return;

            if (m_DisplayExecutorStats)
                DisplayOrdersExecutorStats();
        }

        #endregion
    }
}

#endif
