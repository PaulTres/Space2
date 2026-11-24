using UnityEditor;
using UnityEngine;

namespace Nav3D.API.Editor
{
    [CustomEditor(typeof(Nav3DPathTester))]
    class Nav3DPathTesterInspector : UnityEditor.Editor
    {
        #region Constants

        //the radius a switch to Volumetric starts from when the field still says point
        const float DEFAULT_VOLUMETRIC_RADIUS = 0.5f;

        const string CONFIG_NOTE = "Radius, pathfinding mode, timeout, smoothing and both repositioning settings come from the config, " +
                                   "so the tester asks exactly what an agent with this config asks. Loop and Skip unpassable targets " +
                                   "are the request's own.";

        const string OPTIMALITY_TOOLTIP = "FAST: a fast search; the route may come out a little longer. " +
                                          "QUALITY (the default): a quality route; the search costs more than FAST. " +
                                          "HIGH_QUALITY: a still shorter route; the search costs more still.";

        #if LITE_VERSION

        const string LITE_NOTICE = "Volumetric pathfinding is available only in Nav3D Pro.";

        #endif

        #endregion

        #region Attributes

        static readonly EditorFoldoutState s_RequestFoldout = new EditorFoldoutState("PathTester.Request", true);
        static readonly EditorFoldoutState s_ResultFoldout  = new EditorFoldoutState("PathTester.Result", true);
        static readonly EditorFoldoutState s_AdvancedFoldout = new EditorFoldoutState("PathTester.Advanced");

        SerializedProperty m_Targets;
        SerializedProperty m_Config;
        SerializedProperty m_Loop;
        SerializedProperty m_SkipUnpassableTargets;
        SerializedProperty m_TryRepositionStartIfOccupied;
        SerializedProperty m_TryRepositionTargetIfOccupied;
        SerializedProperty m_Timeout;
        SerializedProperty m_TimeoutMode;
        SerializedProperty m_Smooth;
        SerializedProperty m_SmoothRatio;
        SerializedProperty m_Radius;
        SerializedProperty m_VolumetricMode;
        SerializedProperty m_Optimality;
        SerializedProperty m_AutoUpdate;
        SerializedProperty m_Draw;

        #endregion

        #region Unity methods

        void OnEnable()
        {
            m_Targets                       = serializedObject.FindProperty("m_Targets");
            m_Config                        = serializedObject.FindProperty("m_Config");
            m_Loop                          = serializedObject.FindProperty("m_Loop");
            m_SkipUnpassableTargets         = serializedObject.FindProperty("m_SkipUnpassableTargets");
            m_TryRepositionStartIfOccupied  = serializedObject.FindProperty("m_TryRepositionStartIfOccupied");
            m_TryRepositionTargetIfOccupied = serializedObject.FindProperty("m_TryRepositionTargetIfOccupied");
            m_Timeout                       = serializedObject.FindProperty("m_Timeout");
            m_TimeoutMode                   = serializedObject.FindProperty("m_TimeoutMode");
            m_Smooth                        = serializedObject.FindProperty("m_Smooth");
            m_SmoothRatio                   = serializedObject.FindProperty("m_SmoothRatio");
            m_Radius                        = serializedObject.FindProperty("m_Radius");
            m_VolumetricMode                = serializedObject.FindProperty("m_VolumetricMode");
            m_Optimality                    = serializedObject.FindProperty("m_Optimality");
            m_AutoUpdate                    = serializedObject.FindProperty("m_AutoUpdate");
            m_Draw                          = serializedObject.FindProperty("m_Draw");
        }

        //the result box follows a search that ends on another frame
        public override bool RequiresConstantRepaint()
        {
            return Application.isPlaying;
        }

        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            EditorGUILayout.PropertyField(m_Targets, true);

            EditorGUILayout.Space();

            OrderSection();
            RequestSection();
            RunSection();
            DebugSection();

            serializedObject.ApplyModifiedProperties();

            if (Application.isPlaying)
                ResultSection();
        }

        #endregion

        #region Service methods

        void OrderSection()
        {
            EditorGUILayout.LabelField("Order", EditorStyles.boldLabel);

            EditorGUILayout.PropertyField(m_Loop);
            EditorGUILayout.PropertyField(m_SkipUnpassableTargets, new GUIContent("Skip unpassable targets"));

            EditorGUILayout.Space();
        }

        void RequestSection()
        {
            //ReSharper disable once AssignmentInConditionalExpression
            if (s_RequestFoldout.Value = EditorGUILayout.BeginFoldoutHeaderGroup(s_RequestFoldout.Value, "Request"))
            {
                EditorGUILayout.PropertyField(m_Config, new GUIContent("Config"));

                Nav3DAgentConfig config = m_Config.objectReferenceValue as Nav3DAgentConfig;

                if (config != null)
                {
                    EditorGUILayout.HelpBox(CONFIG_NOTE, MessageType.Info);

                    AgentRadiusWorldCheck.DrawHelpBox(config);
                }
                else
                {
                    OwnRequestFields();
                }
            }

            EditorGUILayout.EndFoldoutHeaderGroup();

            EditorGUILayout.Space();
        }

        void OwnRequestFields()
        {
            EditorGUILayout.PropertyField(m_Timeout, new GUIContent("Timeout (ms)"));

            m_Timeout.intValue = Mathf.Max(1, m_Timeout.intValue);

            EditorGUILayout.PropertyField(m_TimeoutMode, new GUIContent("On timeout"));
            EditorGUILayout.PropertyField(m_Smooth);

            if (m_Smooth.boolValue)
            {
                EditorGUILayout.PropertyField(m_SmoothRatio, new GUIContent("Smooth ratio"));

                m_SmoothRatio.intValue = Mathf.Max(1, m_SmoothRatio.intValue);
            }

            EditorGUILayout.Space();

            PathfindingModeSelector();

            EditorGUILayout.Space();

            EditorGUI.indentLevel++;

            //ReSharper disable once AssignmentInConditionalExpression
            if (s_AdvancedFoldout.Value = EditorGUILayout.Foldout(s_AdvancedFoldout.Value, "Advanced", true))
            {
                EditorGUILayout.PropertyField(m_TryRepositionStartIfOccupied, new GUIContent("Try reposition start if occupied"));
                EditorGUILayout.PropertyField(m_TryRepositionTargetIfOccupied, new GUIContent("Try reposition target if occupied"));
            }

            EditorGUI.indentLevel--;
        }

        void PathfindingModeSelector()
        {
            #if LITE_VERSION

            EditorGUILayout.HelpBox(LITE_NOTICE, MessageType.Info);
            EditorGUI.BeginDisabledGroup(true);

            #endif

            EditorGUILayout.LabelField("Pathfinding mode", EditorStyles.boldLabel);

            bool volumetric = m_Radius.floatValue > 0f;

            int selectedMode = GUILayout.Toolbar(volumetric ? 1 : 0, new[] { "Point", "Volumetric" });

            if (selectedMode == 1 && !volumetric)
                m_Radius.floatValue = DEFAULT_VOLUMETRIC_RADIUS;
            else if (selectedMode == 0 && volumetric)
                m_Radius.floatValue = 0f;

            if (m_Radius.floatValue > 0f)
            {
                EditorGUI.indentLevel++;

                EditorGUILayout.PropertyField(m_Radius);

                m_Radius.floatValue = Mathf.Max(0f, m_Radius.floatValue);

                EditorGUILayout.PropertyField(m_VolumetricMode, new GUIContent("On too-narrow space"));
                UtilsEditor.OptimalityField(m_Optimality, new GUIContent("Optimality", OPTIMALITY_TOOLTIP));

                EditorGUILayout.HelpBox(
                        $"The path keeps at least {m_Radius.floatValue:F3} clear of obstacles and refuses passages narrower " +
                        "than twice that. The radius is clamped to the world settings' Max agent radius at runtime.",
                        MessageType.Info
                    );

                AgentRadiusWorldCheck.DrawHelpBox(m_Radius.floatValue);

                EditorGUI.indentLevel--;
            }

            #if LITE_VERSION

            EditorGUI.EndDisabledGroup();

            #endif
        }

        void RunSection()
        {
            EditorGUILayout.LabelField("Run", EditorStyles.boldLabel);

            EditorGUILayout.PropertyField(m_AutoUpdate, new GUIContent("Auto update", "Search again whenever a target moves or a setting changes."));

            EditorGUI.BeginDisabledGroup(!Application.isPlaying);

            if (GUILayout.Button("Find path"))
            {
                foreach (Object candidate in targets)
                {
                    if (candidate is Nav3DPathTester tester)
                        tester.FindPath();
                }
            }

            EditorGUI.EndDisabledGroup();

            EditorGUILayout.Space();
        }

        void DebugSection()
        {
            EditorGUILayout.LabelField("Debug", EditorStyles.boldLabel);

            EditorGUILayout.PropertyField(m_Draw, new GUIContent("Draw", "Gizmos in Play mode: the route, its targets, the capsule of a volumetric route, the point a failed search names."));

            EditorGUILayout.Space();
        }

        void ResultSection()
        {
            Nav3DPathTester tester = (Nav3DPathTester) target;

            //ReSharper disable once AssignmentInConditionalExpression
            if (!(s_ResultFoldout.Value = EditorGUILayout.BeginFoldoutHeaderGroup(s_ResultFoldout.Value, "Result")))
            {
                EditorGUILayout.EndFoldoutHeaderGroup();

                return;
            }

            PathfindingResult result = tester.LastResult;

            if (tester.IsSearching)
                EditorGUILayout.HelpBox("Searching…", MessageType.None);
            else if (result == null)
                EditorGUILayout.HelpBox(Nav3DManager.Inited ? "No search yet: the tester needs at least two targets." : "Waiting for Nav3D to initialize.", MessageType.None);

            if (result != null)
            {
                if (result.Failed)
                {
                    EditorGUILayout.HelpBox($"{result.Result}\n{result.ErrorMessage}", MessageType.Error);

                    string failedPoint = result.GetFailedPointInfo();

                    if (!string.IsNullOrEmpty(failedPoint))
                        EditorGUILayout.LabelField("Failed point", failedPoint);
                }
                else
                {
                    string flags = (result.Degraded ? " · DEGRADED" : string.Empty) + (result.TimedOut ? " · TIMED OUT" : string.Empty);

                    EditorGUILayout.HelpBox($"{result.Result}{flags}", result.Degraded || result.TimedOut ? MessageType.Warning : MessageType.Info);

                    EditorGUILayout.LabelField("Points", $"{result.PathFinal?.Length ?? 0}");
                    EditorGUILayout.LabelField("Length", $"{tester.LastPathLength:F2} u");
                }

                EditorGUILayout.LabelField("Search", $"{tester.LastSearchSeconds * 1000f:F0} ms");
                EditorGUILayout.LabelField(
                        "Stages",
                        $"pathfinding {result.PathfindingDuration.TotalMilliseconds:F0} ms · optimizing {result.OptimizingDuration.TotalMilliseconds:F0} ms · smoothing {result.SmoothingDuration.TotalMilliseconds:F0} ms"
                    );

                PathfindingRequest request = tester.LastRequest;

                if (request != null && request.Radius > 0f)
                    EditorGUILayout.LabelField("Radius", result.Failed ? $"requested {request.Radius:F2}" : $"requested {request.Radius:F2} · certified {result.CertifiedRadius:F2}");
            }

            EditorGUILayout.EndFoldoutHeaderGroup();
        }

        #endregion
    }
}
