using UnityEditor;
using UnityEngine;

namespace Nav3D.API.Editor
{
    [CustomEditor(typeof(Nav3DAgentPathDrawer))]
    class Nav3DAgentPathDrawerInspector : UnityEditor.Editor
    {

        #region Constants

        const string DONT_DESTROY_ON_LOAD_SCENE = "DontDestroyOnLoad";

        const string SCENE_OBJECT_WARNING = "This drawer lives and dies with this object, and while Active it draws the path of every agent, "   +
                                            "not only this object's. To see one agent's path, tick Draw path on that agent; to have the drawer " +
                                            "in every run, tick Init paths drawer on Nav3DInitializer.";

        #if LITE_VERSION

        const string LITE_NOTICE = "The settings below are available only in Nav3D Pro.";

        #endif

        #endregion

        #region Attributes

        SerializedProperty m_RefreshPeriod;
        SerializedProperty m_PathColor;
        SerializedProperty m_TimedOutColor;
        SerializedProperty m_Width;
        SerializedProperty m_Outline;
        SerializedProperty m_OccludedFade;
        SerializedProperty m_TraveledFade;
        SerializedProperty m_DegradedColor;
        SerializedProperty m_BodyAlpha;
        SerializedProperty m_BodyBudget;

        #endregion

        #region Unity methods

        void OnEnable()
        {
            m_RefreshPeriod = serializedObject.FindProperty("m_RefreshPeriod");
            m_PathColor     = serializedObject.FindProperty("m_PathColor");
            m_TimedOutColor = serializedObject.FindProperty("m_TimedOutColor");
            m_Width         = serializedObject.FindProperty("m_Width");
            m_Outline       = serializedObject.FindProperty("m_Outline");
            m_OccludedFade  = serializedObject.FindProperty("m_OccludedFade");
            m_TraveledFade  = serializedObject.FindProperty("m_TraveledFade");
            m_DegradedColor = serializedObject.FindProperty("m_DegradedColor");
            m_BodyAlpha     = serializedObject.FindProperty("m_BodyAlpha");
            m_BodyBudget    = serializedObject.FindProperty("m_BodyBudget");
        }

        //the status line follows the agents
        public override bool RequiresConstantRepaint()
        {
            return Application.isPlaying;
        }

        public override void OnInspectorGUI()
        {
            Nav3DAgentPathDrawer drawer = (Nav3DAgentPathDrawer) target;

            if (IsSceneObject(drawer))
                EditorGUILayout.HelpBox(SCENE_OBJECT_WARNING, MessageType.Warning);

            ActiveToggle(drawer);

            serializedObject.Update();

            LinesSection();
            BodiesSection();

            serializedObject.ApplyModifiedProperties();

            if (Application.isPlaying)
                StatusLine(drawer);
        }

        #endregion

        #region Service methods

        //through the property, so switching off hides at once what was drawn
        void ActiveToggle(Nav3DAgentPathDrawer _Drawer)
        {
            EditorGUI.BeginChangeCheck();

            bool active = EditorGUILayout.Toggle(
                    new GUIContent("Active", "Draw the path of every agent. While inactive only the agents with Draw path ticked are drawn."),
                    _Drawer.Active
                );

            if (!EditorGUI.EndChangeCheck())
                return;

            foreach (Object candidate in targets)
            {
                if (candidate is not Nav3DAgentPathDrawer drawer)
                    continue;

                Undo.RecordObject(drawer, "Toggle path drawer");

                drawer.Active = active;

                EditorUtility.SetDirty(drawer);
            }
        }

        void LinesSection()
        {
            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Lines", EditorStyles.boldLabel);

            EditorGUILayout.PropertyField(m_PathColor, new GUIContent("Path color"));
            EditorGUILayout.PropertyField(m_TimedOutColor, new GUIContent("Timed out color", m_TimedOutColor.tooltip));
            EditorGUILayout.PropertyField(m_Width);
            EditorGUILayout.PropertyField(m_Outline, new GUIContent("Outline", m_Outline.tooltip));
            EditorGUILayout.PropertyField(m_TraveledFade, new GUIContent("Traveled fade", m_TraveledFade.tooltip));
            EditorGUILayout.PropertyField(m_OccludedFade, new GUIContent("Occluded fade", m_OccludedFade.tooltip));
            EditorGUILayout.PropertyField(m_RefreshPeriod, new GUIContent("Refresh period", m_RefreshPeriod.tooltip));

            m_Width.floatValue         = Mathf.Max(0.001f, m_Width.floatValue);
            m_RefreshPeriod.floatValue = Mathf.Max(0f, m_RefreshPeriod.floatValue);
        }

        void BodiesSection()
        {
            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Bodies", EditorStyles.boldLabel);

            #if LITE_VERSION

            EditorGUILayout.HelpBox(LITE_NOTICE, MessageType.Info);
            EditorGUI.BeginDisabledGroup(true);

            #endif

            EditorGUILayout.PropertyField(m_DegradedColor, new GUIContent("Degraded color", m_DegradedColor.tooltip));
            EditorGUILayout.PropertyField(m_BodyAlpha, new GUIContent("Body alpha", m_BodyAlpha.tooltip));
            EditorGUILayout.PropertyField(m_BodyBudget, new GUIContent("Body budget", m_BodyBudget.tooltip));

            m_BodyBudget.intValue = Mathf.Max(0, m_BodyBudget.intValue);

            #if LITE_VERSION

            EditorGUI.EndDisabledGroup();

            #endif

        }

        static void StatusLine(Nav3DAgentPathDrawer _Drawer)
        {
            EditorGUILayout.Space();

            string status = _Drawer.DrawnPathsCount == 0
                ? _Drawer.Active ? "Active: no agent is following a path." : "Inactive: no agent has Draw path ticked."
                : $"Drawing {_Drawer.DrawnPathsCount} path(s), {_Drawer.DrawnBodiesCount} with a body.";

            EditorGUILayout.HelpBox(status, MessageType.None);
        }

        //the drawer Nav3D creates sits in DontDestroyOnLoad; one authored into a scene shares its object's fate
        static bool IsSceneObject(Nav3DAgentPathDrawer _Drawer)
        {
            return !EditorUtility.IsPersistent(_Drawer) && _Drawer.gameObject.scene.name != DONT_DESTROY_ON_LOAD_SCENE;
        }

        #endregion

    }
}
