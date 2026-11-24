using Nav3D.Obstacles;
using UnityEditor;
using UnityEngine;

namespace Nav3D.API.Editor
{
    [CustomEditor(typeof(ObstacleManager))]
    class ObstacleManagerInspector : UnityEditor.Editor
    {
        #region Constants

        const string PREFS_PREFIX = "Nav3D.ObstacleManager.";

        const string DRAW_ALL_BUTTON          = "All";
        const string DRAW_SPECIFIC_ONE_BUTTON = "Specific";

        const string TIP_DRAW_OCCUPIED_LEAVES_SET_UP = "Set up drawing occupied leaves:";
        const string TIP_DRAW_FREE_LEAVES_SET_UP     = "Set up drawing free leaves:";
        const string TIP_DRAW_GRAPH_NODES_SET_UP     = "Set up drawing graph nodes:";

        const string TIP_DRAW_OCCUPIED_LEAVES = "Occupied leaves";
        const string TIP_DRAW_FREE_LEAVES     = "Free leaves";
        const string TIP_DRAW_GRAPH_NODES     = "Navigation nodes";
        const string TIP_DRAW_ROOTS           = "Root nodes";
        const string TIP_DRAW_OPEN_FACES      = "Nav node open faces";

        #endregion

        #region Attributes

        SerializedProperty m_DrawOccupiedLeaves;
        SerializedProperty m_DrawFreeLeaves;
        SerializedProperty m_DrawGraph;
        SerializedProperty m_DrawRoots;
        SerializedProperty m_DrawOpenFaces;

        SerializedProperty m_OccupiedLeavesDrawingMode;
        SerializedProperty m_OccupiedLeavesDrawingLayerNumber;
        SerializedProperty m_FreeLeavesDrawingMode;
        SerializedProperty m_FreeLeavesDrawingLayerNumber;
        SerializedProperty m_GraphNodesDrawingMode;
        SerializedProperty m_GraphNodesDrawingLayerNumber;

        #endregion

        #region Unity methods

        void OnEnable()
        {
            m_DrawOccupiedLeaves               = serializedObject.FindProperty("m_DrawOccupiedLeaves");
            m_DrawFreeLeaves                   = serializedObject.FindProperty("m_DrawFreeLeaves");
            m_DrawGraph                        = serializedObject.FindProperty("m_DrawGraph");
            m_DrawRoots                        = serializedObject.FindProperty("m_DrawRoots");
            m_DrawOpenFaces                    = serializedObject.FindProperty("m_DrawOpenFaces");
            m_OccupiedLeavesDrawingMode        = serializedObject.FindProperty("m_OccupiedLeavesDrawingMode");
            m_OccupiedLeavesDrawingLayerNumber = serializedObject.FindProperty("m_OccupiedLeavesDrawingLayerNumber");
            m_FreeLeavesDrawingMode            = serializedObject.FindProperty("m_FreeLeavesDrawingMode");
            m_FreeLeavesDrawingLayerNumber     = serializedObject.FindProperty("m_FreeLeavesDrawingLayerNumber");
            m_GraphNodesDrawingMode            = serializedObject.FindProperty("m_GraphNodesDrawingMode");
            m_GraphNodesDrawingLayerNumber     = serializedObject.FindProperty("m_GraphNodesDrawingLayerNumber");
        }

        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            EditorGUILayout.LabelField("Debug features:", EditorStyles.boldLabel);
            EditorGUILayout.Space();

            DebugDrawing();

            if (serializedObject.ApplyModifiedProperties())
                SaveAllToEditorPrefs();
        }

        #endregion

        #region Service methods

        void DebugDrawing()
        {
            OccupiedLeavesDrawing();
            FreeLeavesDrawing();
            GraphNodesDrawing();
            RootsDrawing();
            OpenFacesDrawing();

            EditorGUILayout.EndFoldoutHeaderGroup();
        }

        void OccupiedLeavesDrawing()
        {
            m_DrawOccupiedLeaves.boolValue = EditorGUILayout.Toggle(TIP_DRAW_OCCUPIED_LEAVES, m_DrawOccupiedLeaves.boolValue);

            if (!m_DrawOccupiedLeaves.boolValue)
                return;

            EditorGUILayout.LabelField(TIP_DRAW_OCCUPIED_LEAVES_SET_UP);

            m_OccupiedLeavesDrawingMode.intValue =
                GUILayout.SelectionGrid(m_OccupiedLeavesDrawingMode.intValue, new[] { DRAW_ALL_BUTTON, DRAW_SPECIFIC_ONE_BUTTON }, 2);

            if (m_OccupiedLeavesDrawingMode.intValue == 1)
            {
                int maxLayers = GetOctreeLayersMaxCount();

                if (maxLayers > 0)
                    m_OccupiedLeavesDrawingLayerNumber.intValue =
                        EditorGUILayout.IntSlider(m_OccupiedLeavesDrawingLayerNumber.intValue, 0, maxLayers - 1);
            }
        }

        void FreeLeavesDrawing()
        {
            m_DrawFreeLeaves.boolValue = EditorGUILayout.Toggle(TIP_DRAW_FREE_LEAVES, m_DrawFreeLeaves.boolValue);

            if (!m_DrawFreeLeaves.boolValue)
                return;

            EditorGUILayout.LabelField(TIP_DRAW_FREE_LEAVES_SET_UP);

            m_FreeLeavesDrawingMode.intValue =
                GUILayout.SelectionGrid(m_FreeLeavesDrawingMode.intValue, new[] { DRAW_ALL_BUTTON, DRAW_SPECIFIC_ONE_BUTTON }, 2);

            if (m_FreeLeavesDrawingMode.intValue == 1)
            {
                int maxLayers = GetOctreeLayersMaxCount();

                if (maxLayers > 0)
                    m_FreeLeavesDrawingLayerNumber.intValue =
                        EditorGUILayout.IntSlider(m_FreeLeavesDrawingLayerNumber.intValue, 0, maxLayers - 1);
            }
        }

        void GraphNodesDrawing()
        {
            m_DrawGraph.boolValue = EditorGUILayout.Toggle(TIP_DRAW_GRAPH_NODES, m_DrawGraph.boolValue);

            if (!m_DrawGraph.boolValue)
                return;

            EditorGUILayout.LabelField(TIP_DRAW_GRAPH_NODES_SET_UP);

            m_GraphNodesDrawingMode.intValue =
                GUILayout.SelectionGrid(m_GraphNodesDrawingMode.intValue, new[] { DRAW_ALL_BUTTON, DRAW_SPECIFIC_ONE_BUTTON }, 2);

            if (m_GraphNodesDrawingMode.intValue == 1)
            {
                int maxLayers = GetOctreeLayersMaxCount();

                if (maxLayers > 0)
                    m_GraphNodesDrawingLayerNumber.intValue =
                        EditorGUILayout.IntSlider(m_GraphNodesDrawingLayerNumber.intValue, 0, maxLayers - 1);
            }
        }

        void RootsDrawing()
        {
            m_DrawRoots.boolValue = EditorGUILayout.Toggle(TIP_DRAW_ROOTS, m_DrawRoots.boolValue);
        }

        void OpenFacesDrawing()
        {
            m_DrawOpenFaces.boolValue = EditorGUILayout.Toggle(TIP_DRAW_OPEN_FACES, m_DrawOpenFaces.boolValue);
        }

        int GetOctreeLayersMaxCount()
        {
            ObstacleManager obstacleManager = (ObstacleManager)target;
            return obstacleManager.GetOctreeLayersMaxCount();
        }

        #endregion

        #region EditorPrefs persistence

        void SaveAllToEditorPrefs()
        {
            EditorPrefs.SetBool(PREFS_PREFIX + "DrawOccupiedLeaves",               m_DrawOccupiedLeaves.boolValue);
            EditorPrefs.SetBool(PREFS_PREFIX + "DrawFreeLeaves",                   m_DrawFreeLeaves.boolValue);
            EditorPrefs.SetBool(PREFS_PREFIX + "DrawGraph",                        m_DrawGraph.boolValue);
            EditorPrefs.SetBool(PREFS_PREFIX + "DrawRoots",                        m_DrawRoots.boolValue);
            EditorPrefs.SetBool(PREFS_PREFIX + "DrawOpenFaces",                    m_DrawOpenFaces.boolValue);
            EditorPrefs.SetInt(PREFS_PREFIX  + "OccupiedLeavesDrawingMode",        m_OccupiedLeavesDrawingMode.intValue);
            EditorPrefs.SetInt(PREFS_PREFIX  + "OccupiedLeavesDrawingLayerNumber", m_OccupiedLeavesDrawingLayerNumber.intValue);
            EditorPrefs.SetInt(PREFS_PREFIX  + "FreeLeavesDrawingMode",            m_FreeLeavesDrawingMode.intValue);
            EditorPrefs.SetInt(PREFS_PREFIX  + "FreeLeavesDrawingLayerNumber",     m_FreeLeavesDrawingLayerNumber.intValue);
            EditorPrefs.SetInt(PREFS_PREFIX  + "GraphNodesDrawingMode",            m_GraphNodesDrawingMode.intValue);
            EditorPrefs.SetInt(PREFS_PREFIX  + "GraphNodesDrawingLayerNumber",     m_GraphNodesDrawingLayerNumber.intValue);
        }

        #endregion
    }
}
