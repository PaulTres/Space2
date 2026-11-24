using System;
using UnityEditor;
using UnityEngine;

namespace Nav3D.API.Editor
{
    [CustomEditor(typeof(Nav3DWorldSettings))]
    class Nav3DWorldSettingsInspector : UnityEditor.Editor
    {
        #region Constants

        const string WORLD_SETTINGS_HEADER = "Navigation space settings";
        const string AGENTS_SETTINGS       = "Agents settings";
        const string INIT_SETTINGS         = "Nav3D init. settings";
        const string PATHFINDING_SETTINGS  = "Pathfinding settings";

        const string SNAP_BUTTON_FORMAT    = "Snap to {0:0.###}";
        const string SNAP_BUTTON_TOOLTIP   = "Set the min bucket size to the nearest power of two (… 0.25, 0.5, 1, 2, 4 …). Every size " +
                                             "derived from it — the root, the resolution a region rounds to — is then a round number.";
        const float  SNAP_BUTTON_WIDTH     = 96f;

        const string ROOT_SIZE_LABEL       = "Root size";
        const string ROOT_SIZE_TOOLTIP     = "Edge of the largest octree cell. The octree is a grid of independent trees of this size, " +
                                             "and every obstacle operation rebuilds only the roots it crosses. A layer is a doubling of " +
                                             "the min bucket size, so the root is always min bucket × 2^(layers − 1).";
        const string ROOT_GROW_TOOLTIP     = "Double the root size (one more layer)";
        const string ROOT_SHRINK_TOOLTIP   = "Halve the root size (one layer less)";
        const string LAYERS_COUNT_FORMAT   = "Layers count: {0}  ({1:0.###} × 2^{2})";
        const float  STEP_BUTTON_WIDTH     = 24f;

        const string MAX_SEARCHES_HINT     = "0 = automatic: one search per processor core, less one";
        const string MIN_SEARCHES_ERROR    = "Min concurrent searches must be at least 1";
        //"Adaptive Search Concurrency", the label the field name yields, is heavier than the row needs; the property
        //keeps its full name.
        const string ADAPTIVE_LABEL        = "Adaptive Concurrency";
        const string ADAPTIVE_HINT         = "Adaptive: fewer searches run at once while Nav3D's own main-thread " +
                                             "time spikes, so queued searches take longer and agents may wait for " +
                                             "their route";

        #endregion

        #region Attributes

        SerializedProperty m_OctreeMinBucketSize;
        SerializedProperty m_OctreeLayersCount;
        SerializedProperty m_MaxAgentRadius;
        SerializedProperty m_Nav3DInitOnAwake;
        SerializedProperty m_Nav3DDisposeOnDestroy;
        SerializedProperty m_MaxConcurrentSearches;
        SerializedProperty m_AdaptiveSearchConcurrency;
        SerializedProperty m_MinConcurrentSearches;

        #endregion

        #region Unity methods

        void OnEnable()
        {
            try
            {
                m_OctreeMinBucketSize   = serializedObject.FindProperty("m_OctreeMinBucketSize");
                m_OctreeLayersCount     = serializedObject.FindProperty("m_OctreeLayersCount");
                m_MaxAgentRadius          = serializedObject.FindProperty("m_MaxAgentRadius");
                m_Nav3DInitOnAwake      = serializedObject.FindProperty("m_Nav3DInitOnAwake");
                m_Nav3DDisposeOnDestroy = serializedObject.FindProperty("m_Nav3DDisposeOnDestroy");
                m_MaxConcurrentSearches     = serializedObject.FindProperty("m_MaxConcurrentSearches");
                m_AdaptiveSearchConcurrency = serializedObject.FindProperty("m_AdaptiveSearchConcurrency");
                m_MinConcurrentSearches     = serializedObject.FindProperty("m_MinConcurrentSearches");
            }
            catch (Exception)
            {
            }
        }

        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            bool isSettingsValid = true;
            NavigationSpaceSettings(ref isSettingsValid);
            AgentsSettings(ref isSettingsValid);
            PathfindingSettings();
            InitializationSettings();

            serializedObject.ApplyModifiedProperties();
        }

        #endregion

        #region Service methods

        void NavigationSpaceSettings(ref bool _IsSettingsValid)
        {
            EditorGUILayout.LabelField(WORLD_SETTINGS_HEADER, EditorStyles.boldLabel);
            EditorGUILayout.Space();

            EditorGUI.BeginChangeCheck();

            MinBucketSizeField();

            if (!Nav3DWorldSettings.TryValidateOctreeMinBucketSize(m_OctreeMinBucketSize.floatValue, out string errorMessage))
            {
                _IsSettingsValid = false;
                EditorGUILayout.HelpBox(errorMessage, MessageType.Error);
            }
            else
            {
                RootSizeField();
            }

            //Arm the transient root-grid scene preview when the octree resolution settings change.
            if (EditorGUI.EndChangeCheck())
                Nav3DInitializer.PingRootGridPreview();
        }

        //The field plus a snap button that appears only while the value is off the power-of-two ladder, naming the
        //value it would set so the user sees the change before making it.
        void MinBucketSizeField()
        {
            EditorGUILayout.BeginHorizontal();

            EditorGUILayout.PropertyField(m_OctreeMinBucketSize);

            float value = m_OctreeMinBucketSize.floatValue;

            if (value > 0f && !Nav3DWorldSettings.IsPowerOfTwo(value))
            {
                float snapped = Nav3DWorldSettings.GetNearestPowerOfTwo(value);

                if (GUILayout.Button(new GUIContent(string.Format(SNAP_BUTTON_FORMAT, snapped), SNAP_BUTTON_TOOLTIP), EditorStyles.miniButton, GUILayout.Width(SNAP_BUTTON_WIDTH)))
                {
                    m_OctreeMinBucketSize.floatValue = snapped;
                    GUI.changed                      = true;
                }
            }

            EditorGUILayout.EndHorizontal();
        }

        //The user picks the root size; the layer count is what the pair implies and is only reported. The root can
        //only be the min bucket doubled a whole number of times, so the control is a pair of step buttons rather
        //than a free field, and a min-bucket edit keeps the layer count (the old root is unreachable from the new
        //bucket anyway) clamped into the range the new bucket allows.
        void RootSizeField()
        {
            float minBucketSize = m_OctreeMinBucketSize.floatValue;
            int   minLayers     = Nav3DWorldSettings.OCTREE_LAYERS_COUNT_MIN;
            int   maxLayers     = Nav3DWorldSettings.GetMaxLayersCount(minBucketSize);
            int   layersCount   = Mathf.Clamp(m_OctreeLayersCount.intValue, minLayers, maxLayers);

            EditorGUILayout.BeginHorizontal();

            EditorGUILayout.PrefixLabel(new GUIContent(ROOT_SIZE_LABEL, ROOT_SIZE_TOOLTIP));

            EditorGUI.BeginDisabledGroup(true);
            EditorGUILayout.FloatField(Nav3DWorldSettings.GetMaxBucketSize(minBucketSize, layersCount));
            EditorGUI.EndDisabledGroup();

            EditorGUI.BeginDisabledGroup(layersCount >= maxLayers);
            if (GUILayout.Button(new GUIContent("\u2227", ROOT_GROW_TOOLTIP), EditorStyles.miniButtonLeft, GUILayout.Width(STEP_BUTTON_WIDTH)))
                layersCount++;
            EditorGUI.EndDisabledGroup();

            EditorGUI.BeginDisabledGroup(layersCount <= minLayers);
            if (GUILayout.Button(new GUIContent("\u2228", ROOT_SHRINK_TOOLTIP), EditorStyles.miniButtonRight, GUILayout.Width(STEP_BUTTON_WIDTH)))
                layersCount--;
            EditorGUI.EndDisabledGroup();

            EditorGUILayout.EndHorizontal();

            //A clamp after a min-bucket edit is a change too: it moves the root, and the preview must follow.
            if (layersCount != m_OctreeLayersCount.intValue)
            {
                m_OctreeLayersCount.intValue = layersCount;
                GUI.changed                  = true;
            }

            EditorGUILayout.LabelField(string.Format(LAYERS_COUNT_FORMAT, layersCount, minBucketSize, layersCount - 1), EditorStyles.miniLabel);
        }

        void AgentsSettings(ref bool _IsSettingsValid)
        {
            UtilsEditor.SeparatorHorizontal();
            EditorGUILayout.LabelField(AGENTS_SETTINGS, EditorStyles.boldLabel);
            EditorGUILayout.Space();

            //the ceiling is the octree's root size, derived from the min bucket size — the number the octree section shows
            float maxBucketSize = Nav3DWorldSettings.GetMaxBucketSize(m_OctreeMinBucketSize.floatValue, m_OctreeLayersCount.intValue);

            EditorGUILayout.PropertyField(m_MaxAgentRadius);
            if (!Nav3DWorldSettings.TryValidateMaxAgentRadius(m_MaxAgentRadius.floatValue, maxBucketSize, out string maxAgentRadiusError))
            {
                _IsSettingsValid = false;
                EditorGUILayout.HelpBox(maxAgentRadiusError, MessageType.Error);
            }
        }

        void PathfindingSettings()
        {
            UtilsEditor.SeparatorHorizontal();
            EditorGUILayout.LabelField(PATHFINDING_SETTINGS, EditorStyles.boldLabel);
            EditorGUILayout.Space();

            EditorGUILayout.PropertyField(m_MaxConcurrentSearches);
            EditorGUILayout.LabelField(MAX_SEARCHES_HINT, EditorStyles.miniLabel);

            if (m_MaxConcurrentSearches.intValue < 0)
                m_MaxConcurrentSearches.intValue = 0;

            EditorGUILayout.PropertyField(m_AdaptiveSearchConcurrency, new GUIContent(ADAPTIVE_LABEL));

            if (m_AdaptiveSearchConcurrency.boolValue)
            {
                EditorGUILayout.HelpBox(ADAPTIVE_HINT, MessageType.Info);
                EditorGUILayout.PropertyField(m_MinConcurrentSearches);

                if (m_MinConcurrentSearches.intValue < 1)
                    EditorGUILayout.HelpBox(MIN_SEARCHES_ERROR, MessageType.Error);
            }
        }

        void InitializationSettings()
        {
            UtilsEditor.SeparatorHorizontal();
            EditorGUILayout.LabelField(INIT_SETTINGS, EditorStyles.boldLabel);
            EditorGUILayout.Space();
            EditorGUILayout.PropertyField(m_Nav3DInitOnAwake);
            EditorGUILayout.PropertyField(m_Nav3DDisposeOnDestroy);
        }

        #endregion
    }
}