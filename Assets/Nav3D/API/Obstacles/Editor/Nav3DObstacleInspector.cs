using System.Linq;
using Nav3D.Common;
using UnityEngine;
using UnityEditor;

namespace Nav3D.API.Editor
{
    abstract class Nav3DObstacleInspector : UnityEditor.Editor
    {
        #region Constants

        #if !LITE_VERSION

        protected readonly string DUPLICATED_COMPONENT_ERROR =
            $"There is another {nameof(Nav3DObstaclePreBaked)} or {nameof(Nav3DObstacleRuntime)} instance attached " +
            "to game object. It is allowed to attach only one instance per game object.";

        #else

        protected readonly string DUPLICATED_COMPONENT_ERROR =
            $"There is another {nameof(Nav3DObstacleRuntime)} instance attached to game object. It is allowed to attach only one instance per game object.";

        #endif

        const string DRAW_ALL_BUTTON          = "All";
        const string DRAW_SPECIFIC_ONE_BUTTON = "Specific";

        protected const string TRANSFORM_PROCESSING_OPTIONS = "Transform processing options:";

        protected static readonly GUIContent TIP_HOLLOW_INTERIOR = new GUIContent(
                "Hollow interior",
                "Off: the inside of a closed mesh is solid — no path enters it, and a point inside it moves to the nearest " +
                "free point outside. On: the enclosed space stays navigable, for a body agents live inside."
            );

        protected static readonly GUIContent TIP_PROCESS_CHILDREN = new GUIContent(
                "Process children",
                "Include the children's colliders, meshes and terrains in this obstacle. A child GameObject carrying its own " +

                #if !LITE_VERSION

                $"{nameof(Nav3DObstacleRuntime)}/{nameof(Nav3DObstaclePreBaked)} component owns its subtree exclusively — " +

                #else

                $"{nameof(Nav3DObstacleRuntime)} component owns its subtree exclusively — " +

                #endif

                "that subtree is excluded here and is registered (or not) by the nested component itself."
            );

        protected static readonly GUIContent TIP_GEOMETRY_SOURCE = new GUIContent(
                "Geometry source",
                "Which components give the obstacle its geometry. With one kind present it is the one used; with several, " +
                "tick the ones to use. An obstacle added here starts with the first by priority: collider, mesh, terrain."
            );

        static readonly (ObstacleGeometrySources Kind, GUIContent Label)[] GEOMETRY_SOURCE_TOGGLES =
        {
            (ObstacleGeometrySources.COLLIDER, new GUIContent("Collider", "Enabled box, sphere, capsule, mesh and terrain colliders that are not triggers.")),
            (ObstacleGeometrySources.MESH, new GUIContent("Mesh", "MeshFilter meshes.")),
            (ObstacleGeometrySources.TERRAIN, new GUIContent("Terrain", "Enabled Terrain components."))
        };

        const string NO_GEOMETRY_SOURCE = "No enabled collider, mesh or terrain to take the obstacle's geometry from.";

        const float TOGGLE_BOX_WIDTH = 18f;
        const float TOGGLE_SPACING   = 8f;
        const float PREFIX_PADDING   = 2f;

        const string TIP_DRAW_OCCUPIED_LEAVES_SET_UP = "Set up drawing occupied leaves:";
        const string TIP_DRAW_FREE_LEAVES_SET_UP     = "Set up drawing free leaves:";
        const string TIP_DRAW_GRAPH_NODES_SET_UP     = "Set up drawing graph nodes:";

        const string TIP_DRAW_OCCUPIED_LEAVES = "Occupied leaves";
        const string TIP_DRAW_FREE_LEAVES     = "Free leaves";
        const string TIP_DRAW_GRAPH_NODES     = "Navigation nodes";
        const string TIP_DRAW_ROOTS           = "Root nodes";

        #endregion

        #region Attributes

        protected SerializedProperty m_ProcessChildren;
        protected SerializedProperty m_HollowInterior;

        SerializedProperty m_GeometrySources;

        SerializedProperty m_DrawOccupiedLeaves;
        SerializedProperty m_DrawFreeLeaves;
        SerializedProperty m_DrawGraph;
        SerializedProperty m_DrawRoots;

        SerializedProperty m_OccupiedLeavesDrawingMode;
        SerializedProperty m_OccupiedLeavesDrawingLayerNumber;
        SerializedProperty m_FreeLeavesDrawingMode;
        SerializedProperty m_FreeLeavesDrawingLayerNumber;
        SerializedProperty m_GraphNodesDrawingMode;
        SerializedProperty m_GraphNodesDrawingLayerNumber;

        //foldout state outlives this inspector instance (see EditorFoldoutState) — a plain field would
        //collapse the group on every selection change. A second, never-read foldout field stood beside this
        //one and went with the migration
        static readonly EditorFoldoutState s_DebugDrawingFoldout = new EditorFoldoutState("Obstacle.DebugDrawing");

        #endregion

        #region Properties

        protected abstract Nav3DObstacle Obstacle { get; }

        #endregion

        #region Unity methods

        protected virtual void OnEnable()
        {
            m_ProcessChildren = serializedObject.FindProperty("m_ProcessChildren");
            m_HollowInterior  = serializedObject.FindProperty("m_HollowInterior");
            m_GeometrySources = serializedObject.FindProperty("m_GeometrySources");

            m_DrawOccupiedLeaves               = serializedObject.FindProperty("m_DrawOccupiedLeaves");
            m_DrawFreeLeaves                   = serializedObject.FindProperty("m_DrawFreeLeaves");
            m_DrawGraph                        = serializedObject.FindProperty("m_DrawGraph");
            m_DrawRoots                        = serializedObject.FindProperty("m_DrawRoots");
            m_OccupiedLeavesDrawingMode        = serializedObject.FindProperty("m_OccupiedLeavesDrawingMode");
            m_OccupiedLeavesDrawingLayerNumber = serializedObject.FindProperty("m_OccupiedLeavesDrawingLayerNumber");
            m_FreeLeavesDrawingMode            = serializedObject.FindProperty("m_FreeLeavesDrawingMode");
            m_FreeLeavesDrawingLayerNumber     = serializedObject.FindProperty("m_FreeLeavesDrawingLayerNumber");
            m_GraphNodesDrawingMode            = serializedObject.FindProperty("m_GraphNodesDrawingMode");
            m_GraphNodesDrawingLayerNumber     = serializedObject.FindProperty("m_GraphNodesDrawingLayerNumber");
        }

        public override void OnInspectorGUI()
        {
            EditorGUILayout.BeginVertical();

            ProcessingSettings();

            EditorGUILayout.EndVertical();
        }

        #endregion

        #region Service methods

        protected void DebugSection()
        {
            UtilsEditor.SeparatorHorizontal();

            EditorGUILayout.LabelField("Debug features:", EditorStyles.boldLabel);
            EditorGUILayout.Space();

            DebugDrawing();
        }

        protected bool CheckComponentDuplicated()
        {
            Nav3DObstacleRuntime[] runtimeObstacles = Obstacle.gameObject.GetComponents<Nav3DObstacleRuntime>();

            bool duplicated = runtimeObstacles.Any(_RuntimeObstacle => _RuntimeObstacle != Obstacle);

            #if !LITE_VERSION

            Nav3DObstaclePreBaked[] preBakedObstacles = Obstacle.gameObject.GetComponents<Nav3DObstaclePreBaked>();

            duplicated |= preBakedObstacles.Any(_PreBakedObstacle => _PreBakedObstacle != Obstacle);

            #endif

            return duplicated;
        }

        protected virtual void ProcessingSettings()
        {
            ChildrenProcessing();
        }

        protected abstract void ChildrenProcessing();

        //One kind present is shown ticked and locked, several as ticks of which the last left cannot be cleared. Returns
        //whether the selection changed. Kinds not present keep their stored bits for when they appear.
        protected bool GeometrySources()
        {
            ObstacleGeometrySources present   = Obstacle.PresentGeometrySources;
            ObstacleGeometrySources selection = (ObstacleGeometrySources) m_GeometrySources.intValue;
            ObstacleGeometrySources used      = ExtensionTransform.ResolveGeometrySources(selection, present);

            if (present == ObstacleGeometrySources.AUTO)
            {
                EditorGUILayout.HelpBox(NO_GEOMETRY_SOURCE, MessageType.Warning);

                return false;
            }

            ObstacleGeometrySources newUsed = used;

            //a plain label: a prefix label takes the enabled state of the control after it, and the first toggle may be locked
            Rect row   = EditorGUILayout.GetControlRect();
            Rect field = new Rect(row.x + EditorGUIUtility.labelWidth + PREFIX_PADDING, row.y, row.width, row.height);

            EditorGUI.LabelField(new Rect(row.x, row.y, EditorGUIUtility.labelWidth, row.height), TIP_GEOMETRY_SOURCE);

            foreach ((ObstacleGeometrySources Kind, GUIContent Label) toggle in GEOMETRY_SOURCE_TOGGLES)
            {
                if ((present & toggle.Kind) == 0)
                    continue;

                bool  isUsed = (used & toggle.Kind) != 0;
                float width  = EditorStyles.label.CalcSize(toggle.Label).x + TOGGLE_BOX_WIDTH;

                EditorGUI.BeginDisabledGroup(used == toggle.Kind);

                bool ticked = EditorGUI.ToggleLeft(new Rect(field.x, field.y, width, field.height), toggle.Label, isUsed);

                EditorGUI.EndDisabledGroup();

                field.x += width + TOGGLE_SPACING;

                if (ticked != isUsed)
                    newUsed = ticked ? newUsed | toggle.Kind : newUsed & ~toggle.Kind;
            }

            if (newUsed == used)
                return false;

            m_GeometrySources.intValue = (int) ((selection & ~present) | newUsed);

            return true;
        }

        void OccupiedLeavesDrawing()
        {
            bool oldDrawOccupiedLeaves = m_DrawOccupiedLeaves.boolValue;

            m_DrawOccupiedLeaves.boolValue = EditorGUILayout.Toggle(TIP_DRAW_OCCUPIED_LEAVES, m_DrawOccupiedLeaves.boolValue);

            if (m_DrawOccupiedLeaves.boolValue != oldDrawOccupiedLeaves)
                Obstacle.ClearGizmosCache();

            if (!m_DrawOccupiedLeaves.boolValue)
                return;

            EditorGUILayout.LabelField(TIP_DRAW_OCCUPIED_LEAVES_SET_UP);

            int oldMode = m_OccupiedLeavesDrawingMode.intValue;

            m_OccupiedLeavesDrawingMode.intValue =
                GUILayout.SelectionGrid(m_OccupiedLeavesDrawingMode.intValue, new[] { DRAW_ALL_BUTTON, DRAW_SPECIFIC_ONE_BUTTON }, 2);

            if (m_OccupiedLeavesDrawingMode.intValue != oldMode)
                Obstacle.ClearGizmosCache();

            if (m_OccupiedLeavesDrawingMode.intValue == 1)
            {
                int octreeLevelCount = Obstacle.OctreeLayersCount;

                int oldLayerNumber = m_OccupiedLeavesDrawingLayerNumber.intValue;

                m_OccupiedLeavesDrawingLayerNumber.intValue =
                    EditorGUILayout.IntSlider(m_OccupiedLeavesDrawingLayerNumber.intValue, 0, octreeLevelCount - 1);

                if (m_OccupiedLeavesDrawingLayerNumber.intValue != oldLayerNumber)
                    Obstacle.ClearGizmosCache();
            }
        }

        void FreeLeavesDrawing()
        {
            bool oldDrawFreeLeaves = m_DrawFreeLeaves.boolValue;

            m_DrawFreeLeaves.boolValue = EditorGUILayout.Toggle(TIP_DRAW_FREE_LEAVES, m_DrawFreeLeaves.boolValue);

            if (m_DrawFreeLeaves.boolValue != oldDrawFreeLeaves)
                Obstacle.ClearGizmosCache();

            if (!m_DrawFreeLeaves.boolValue)
                return;

            EditorGUILayout.LabelField(TIP_DRAW_FREE_LEAVES_SET_UP);

            int oldMode = m_FreeLeavesDrawingMode.intValue;

            m_FreeLeavesDrawingMode.intValue =
                GUILayout.SelectionGrid(m_FreeLeavesDrawingMode.intValue, new[] { DRAW_ALL_BUTTON, DRAW_SPECIFIC_ONE_BUTTON }, 2);

            if (m_FreeLeavesDrawingMode.intValue != oldMode)
                Obstacle.ClearGizmosCache();

            if (m_FreeLeavesDrawingMode.intValue == 1)
            {
                int octreeLevelCount = Obstacle.OctreeLayersCount;

                int oldLayerNumber = m_FreeLeavesDrawingLayerNumber.intValue;

                m_FreeLeavesDrawingLayerNumber.intValue =
                    EditorGUILayout.IntSlider(m_FreeLeavesDrawingLayerNumber.intValue, 0, octreeLevelCount - 1);

                if (m_FreeLeavesDrawingLayerNumber.intValue != oldLayerNumber)
                    Obstacle.ClearGizmosCache();
            }
        }

        void GraphNodesDrawing()
        {
            bool oldDrawGraph = m_DrawGraph.boolValue;

            m_DrawGraph.boolValue = EditorGUILayout.Toggle(TIP_DRAW_GRAPH_NODES, m_DrawGraph.boolValue);

            if (m_DrawGraph.boolValue != oldDrawGraph)
                Obstacle.ClearGizmosCache();

            if (!m_DrawGraph.boolValue)
                return;

            EditorGUILayout.LabelField(TIP_DRAW_GRAPH_NODES_SET_UP);

            int oldMode = m_GraphNodesDrawingMode.intValue;

            m_GraphNodesDrawingMode.intValue =
                GUILayout.SelectionGrid(m_GraphNodesDrawingMode.intValue, new[] { DRAW_ALL_BUTTON, DRAW_SPECIFIC_ONE_BUTTON }, 2);

            if (m_GraphNodesDrawingMode.intValue != oldMode)
                Obstacle.ClearGizmosCache();

            if (m_GraphNodesDrawingMode.intValue == 1)
            {
                int octreeLevelCount = Obstacle.OctreeLayersCount;

                int oldLayerNumber = m_GraphNodesDrawingLayerNumber.intValue;

                m_GraphNodesDrawingLayerNumber.intValue =
                    EditorGUILayout.IntSlider(m_GraphNodesDrawingLayerNumber.intValue, 0, octreeLevelCount - 1);

                if (m_GraphNodesDrawingLayerNumber.intValue != oldLayerNumber)
                    Obstacle.ClearGizmosCache();
            }
        }

        void RootsSettings()
        {
            bool oldDrawRoots = m_DrawRoots.boolValue;

            m_DrawRoots.boolValue = EditorGUILayout.Toggle(TIP_DRAW_ROOTS, m_DrawRoots.boolValue);

            if (m_DrawRoots.boolValue != oldDrawRoots)
                Obstacle.ClearGizmosCache();

            if (!m_DrawRoots.boolValue)
                return;
        }

        void DebugDrawing()
        {
            //ReSharper disable once AssignmentInConditionalExpression
            if (s_DebugDrawingFoldout.Value = EditorGUILayout.BeginFoldoutHeaderGroup(s_DebugDrawingFoldout.Value, new GUIContent("Draw nav. graph:")))
            {
                OccupiedLeavesDrawing();
                FreeLeavesDrawing();
                GraphNodesDrawing();
                RootsSettings();
            }

            EditorGUILayout.EndFoldoutHeaderGroup();
        }

        #endregion
    }
}