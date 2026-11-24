using UnityEditor;

namespace Nav3D.API.Editor
{
    [CustomEditor(typeof(Nav3DObstacleRuntime))]
    class Nav3DObstacleRuntimeInspector : Nav3DObstacleInspector
    {
        #region Constants
        
        const string TIP_AUTO_UPDATE_ENABLED = "Auto-update on change";
        const string TIP_AUTO_UPDATE_PERIOD  = "Update min period(s)";

        #if LITE_VERSION

        const string LITE_NOTICE = "The settings below are available only in Nav3D Pro. In Nav3D Lite the scene's obstacles are captured once, when Nav3D initializes.";

        #endif

        #endregion

        #region Attributes

        Nav3DObstacleRuntime m_Obstacle;
        
        SerializedProperty m_AutoUpdateIfChanged;
        SerializedProperty m_AutoUpdateMinPeriod;

        #endregion
        
        #region Properties

        protected override Nav3DObstacle Obstacle => m_Obstacle;

        #endregion

        #region Unity methods

        protected override void OnEnable()
        {
            base.OnEnable();

            m_AutoUpdateIfChanged = serializedObject.FindProperty("m_AutoUpdateIfChanged");
            m_AutoUpdateMinPeriod = serializedObject.FindProperty("m_AutoUpdateMinPeriod");
        }

        public override void OnInspectorGUI()
        {
            m_Obstacle = (Nav3DObstacleRuntime) target;

            serializedObject.Update();

            if (CheckComponentDuplicated())
            {
                EditorGUILayout.HelpBox(DUPLICATED_COMPONENT_ERROR, MessageType.Error);
                return;
            }
            
            base.OnInspectorGUI();

            BehaviorSettings();

            DebugSection();

            serializedObject.ApplyModifiedProperties();
        }

        #endregion

        #region Service methods

        protected override void ChildrenProcessing()
        {
            UtilsEditor.SeparatorHorizontal();
            
            EditorGUILayout.LabelField(TRANSFORM_PROCESSING_OPTIONS, EditorStyles.boldLabel);
            EditorGUILayout.Space();
            
            m_ProcessChildren.boolValue = EditorGUILayout.Toggle(TIP_PROCESS_CHILDREN, m_ProcessChildren.boolValue);
            m_HollowInterior.boolValue  = EditorGUILayout.Toggle(TIP_HOLLOW_INTERIOR, m_HollowInterior.boolValue);

            GeometrySources();
        }

        void BehaviorSettings()
        {
            UtilsEditor.SeparatorHorizontal();

            #if LITE_VERSION

            EditorGUILayout.HelpBox(LITE_NOTICE, MessageType.Info);
            EditorGUI.BeginDisabledGroup(true);

            #endif

            m_AutoUpdateIfChanged.boolValue = EditorGUILayout.Toggle(TIP_AUTO_UPDATE_ENABLED, m_AutoUpdateIfChanged.boolValue);

            if (m_AutoUpdateIfChanged.boolValue)
                m_AutoUpdateMinPeriod.floatValue = EditorGUILayout.FloatField(TIP_AUTO_UPDATE_PERIOD, m_AutoUpdateMinPeriod.floatValue);

            #if LITE_VERSION

            EditorGUI.EndDisabledGroup();

            #endif
        }

        #endregion
    }
}