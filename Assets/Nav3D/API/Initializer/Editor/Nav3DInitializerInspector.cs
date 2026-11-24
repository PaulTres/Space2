using UnityEditor;
using UnityEngine;

namespace Nav3D.API.Editor
{
    [CustomEditor(typeof(Nav3DInitializer))]
    class Nav3DInitializerInspector : UnityEditor.Editor
    {
        #region Constants

        const string VISUALIZERS_HEADER = "Debug visualizers";

        #endregion

        #region Attributes

        SerializedProperty m_Settings;
        SerializedProperty m_InitObstacleDrawer;
        SerializedProperty m_InitPathsDrawer;

        UnityEditor.Editor m_SettingsEditor;

        #endregion

        #region Unity methods

        void OnEnable()
        {
            m_Settings           = serializedObject.FindProperty("m_Settings");
            m_InitObstacleDrawer = serializedObject.FindProperty("m_InitObstacleDrawer");
            m_InitPathsDrawer    = serializedObject.FindProperty("m_InitPathsDrawer");
        }

        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            SettingsHeader();
            
            if (m_Settings.objectReferenceValue == null)
                m_Settings.objectReferenceValue = CreateInstance<Nav3DWorldSettings>();
            
            CreateCachedEditor(m_Settings.objectReferenceValue, null, ref m_SettingsEditor);

            UtilsEditor.SeparatorHorizontal();

            if (m_SettingsEditor != null)
            {
                m_SettingsEditor.OnInspectorGUI();
            }

            UtilsEditor.SeparatorHorizontal();

            bool isPersistent = EditorUtility.IsPersistent(m_Settings.objectReferenceValue);
            
            //Settings embedded in 
            if (!isPersistent)
            {
                if (GUILayout.Button("Save", GUILayout.Width(100)))
                    SaveSettings((Nav3DWorldSettings) m_Settings.objectReferenceValue);

                Nav3DWorldSettings selectedSettings = EditorGUILayout.ObjectField(null, typeof(Nav3DWorldSettings), false) as Nav3DWorldSettings;

                if (selectedSettings != null)
                    m_Settings.objectReferenceValue = selectedSettings;
            }
            else
            {
                EditorGUILayout.BeginHorizontal();
                EditorGUILayout.PropertyField(m_Settings, new GUIContent("Settings ref."));
                if (m_Settings.objectReferenceValue != null && GUILayout.Button("X", GUILayout.Width(20)))
                    m_Settings.objectReferenceValue = null;
                EditorGUILayout.EndHorizontal();
            }

            VisualizersSettings();

            serializedObject.ApplyModifiedProperties();
        }

        #endregion

        #region Service methods

        void SettingsHeader()
        {
            EditorGUILayout.LabelField("Configure world settings:", EditorStyles.boldLabel);
            EditorGUILayout.Space();
        }

        //the scene's own choice, not the world settings': a shared settings asset would switch the drawers on in every scene using it
        void VisualizersSettings()
        {
            UtilsEditor.SeparatorHorizontal();

            EditorGUILayout.LabelField(VISUALIZERS_HEADER, EditorStyles.boldLabel);
            EditorGUILayout.Space();

            EditorGUILayout.PropertyField(m_InitObstacleDrawer, new GUIContent("Init obstacle drawer", m_InitObstacleDrawer.tooltip));
            EditorGUILayout.PropertyField(m_InitPathsDrawer, new GUIContent("Init paths drawer", m_InitPathsDrawer.tooltip));
        }

        void SaveSettings(Nav3DWorldSettings _Settings)
        {
            UnityEngine.SceneManagement.Scene scene = UnityEngine.SceneManagement.SceneManager.GetActiveScene();

            string filePath = EditorUtility.SaveFilePanelInProject(
                    "Save settings",
                    scene.name,
                    "asset",
                    "Select settings path",
                    scene.path
                );

            if (!string.IsNullOrEmpty(filePath))
            {

                AssetDatabase.CreateAsset(_Settings, filePath);
                AssetDatabase.SaveAssets();

                EditorUtility.FocusProjectWindow();
            }
        }

        #endregion
    }
}