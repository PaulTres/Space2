#if UNITY_EDITOR

using UnityEditor;
using UnityEngine;
using GizmosDrawData = Nav3D.Common.Debug.GizmosDrawData;

namespace Nav3D.Obstacles
{
    partial class ObstacleManager
    {
        #region Constants

        const string PREFS_PREFIX = "Nav3D.ObstacleManager.";

        #endregion

        #region Serialized fields

        [SerializeField] bool m_DrawOccupiedLeaves;
        [SerializeField] bool m_DrawFreeLeaves;
        [SerializeField] bool m_DrawGraph;
        [SerializeField] bool m_DrawRoots;
        [SerializeField] bool m_DrawOpenFaces;

        [SerializeField] int m_OccupiedLeavesDrawingMode; //0-all layers, 1-specific layer
        [SerializeField] int m_OccupiedLeavesDrawingLayerNumber;

        [SerializeField] int m_FreeLeavesDrawingMode; //0-all layers, 1-specific layer
        [SerializeField] int m_FreeLeavesDrawingLayerNumber;

        [SerializeField] int m_GraphNodesDrawingMode; //0-all layers, 1-specific layer
        [SerializeField] int m_GraphNodesDrawingLayerNumber;

        #endregion

        #region Attributes

        GizmosDrawData m_GizmosData;

        #endregion

        #region Public methods

        public void FillGizmosDrawData(
                GizmosDrawData _GizmosDrawData,
                int            _ObstacleID,
                bool           _DrawOccupiedLeaves,
                bool           _DrawFreeLeaves,
                bool           _DrawGraph,
                bool           _DrawRoots,
                int            _DrawOccupiedLeavesAll,
                int            _DrawOccupiedLeavesLayerNumber,
                int            _DrawFreeLeavesAll,
                int            _DrawFreeLeavesLayerNumber,
                int            _DrawGraphNodesAll,
                int            _DrawGraphLayerNumber
            )
        {
            if (!Inited)
                return;

            m_ObstaclesStorage.FillGizmosDrawData(
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

        public int GetOctreeLayersMaxCount()
        {
            if (!Inited)
                return 0;

            return m_ObstaclesStorage.OctreeLayersCount;
        }

        #endregion

        #region Unity methods

        void OnEnable()
        {
            RestoreDrawingSettingsFromEditorPrefs();
        }

        void OnDrawGizmos()
        {
            if (!Application.isPlaying)
                return;

            if (m_GizmosData == null)
                m_GizmosData = new Common.Debug.GizmosDrawData();

            if (m_DrawOccupiedLeaves || m_DrawFreeLeaves || m_DrawGraph || m_DrawRoots || m_DrawOpenFaces)
            {
                m_GizmosData.Clear();

                if (m_DrawOccupiedLeaves || m_DrawFreeLeaves || m_DrawGraph || m_DrawRoots)
                    m_ObstaclesStorage.FillGizmosDrawDataForAll(
                            m_GizmosData,
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

                if (m_DrawOpenFaces)
                    m_ObstaclesStorage.FillOpenFacesGizmos(m_GizmosData);

                m_GizmosData.Draw();

            }
        }

        #endregion

        #region EditorPrefs persistence

        void RestoreDrawingSettingsFromEditorPrefs()
        {
            m_DrawOccupiedLeaves               = EditorPrefs.GetBool(PREFS_PREFIX + "DrawOccupiedLeaves",               false);
            m_DrawFreeLeaves                   = EditorPrefs.GetBool(PREFS_PREFIX + "DrawFreeLeaves",                   false);
            m_DrawGraph                        = EditorPrefs.GetBool(PREFS_PREFIX + "DrawGraph",                        false);
            m_DrawRoots                        = EditorPrefs.GetBool(PREFS_PREFIX + "DrawRoots",                        false);
            m_DrawOpenFaces                    = EditorPrefs.GetBool(PREFS_PREFIX + "DrawOpenFaces",                    false);
            m_OccupiedLeavesDrawingMode        = EditorPrefs.GetInt(PREFS_PREFIX  + "OccupiedLeavesDrawingMode",        0);
            m_OccupiedLeavesDrawingLayerNumber = EditorPrefs.GetInt(PREFS_PREFIX  + "OccupiedLeavesDrawingLayerNumber", 0);
            m_FreeLeavesDrawingMode            = EditorPrefs.GetInt(PREFS_PREFIX  + "FreeLeavesDrawingMode",            0);
            m_FreeLeavesDrawingLayerNumber     = EditorPrefs.GetInt(PREFS_PREFIX  + "FreeLeavesDrawingLayerNumber",     0);
            m_GraphNodesDrawingMode            = EditorPrefs.GetInt(PREFS_PREFIX  + "GraphNodesDrawingMode",            0);
            m_GraphNodesDrawingLayerNumber     = EditorPrefs.GetInt(PREFS_PREFIX  + "GraphNodesDrawingLayerNumber",     0);
        }

        #endregion
    }
}

#endif