using UnityEditor;

namespace Nav3D.API.Editor
{
    //The drawer's default inspector with the octree-leaf view and its options gathered into a folded "Advanced (debug)"
    //group at the bottom: the free boxes are the picture a user wants, the leaves they are merged from are for the curious.
    [CustomEditor(typeof(Nav3DOctreeDrawer))]
    class Nav3DOctreeDrawerInspector : UnityEditor.Editor
    {
        #region Constants

        const string SCRIPT_PROPERTY = "m_Script";

        static readonly string[] s_AdvancedProperties =
        {
            "m_DrawOctreeLeaves",
            "m_DrawFreeCellsTouchingObstaclesOnly",
            "m_DrawEdgeNodes",
            "m_EdgeNodeColor",
        };

        static readonly EditorFoldoutState s_AdvancedFoldout = new EditorFoldoutState("Nav3DOctreeDrawerInspector.Advanced");

        #endregion

        #region Unity methods

        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            SerializedProperty property = serializedObject.GetIterator();

            for (bool enter = true; property.NextVisible(enter); enter = false)
            {
                if (property.name == SCRIPT_PROPERTY || IsAdvanced(property.name))
                    continue;

                EditorGUILayout.PropertyField(property, true);
            }

            EditorGUILayout.Space();

            s_AdvancedFoldout.Value = EditorGUILayout.Foldout(s_AdvancedFoldout.Value, "Advanced (debug)", true);

            if (s_AdvancedFoldout.Value)
            {
                using (new EditorGUI.IndentLevelScope())
                {
                    foreach (string name in s_AdvancedProperties)
                        EditorGUILayout.PropertyField(serializedObject.FindProperty(name), true);
                }
            }

            serializedObject.ApplyModifiedProperties();
        }

        #endregion

        #region Service methods

        static bool IsAdvanced(string _Name)
        {
            foreach (string name in s_AdvancedProperties)
            {
                if (name == _Name)
                    return true;
            }

            return false;
        }

        #endregion
    }
}
