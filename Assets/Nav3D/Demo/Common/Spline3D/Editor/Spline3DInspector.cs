using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

namespace Nav3D.Demo.Editor
{
    [CustomEditor(typeof(Spline3D))]
    public class Spline3DInspector : UnityEditor.Editor
    {
        #region Constants

        //EditorPrefs key: keeps the toggle state across selections and editor sessions
        const string DRAW_HANDLES_PREF_KEY = "Nav3D.Spline3DInspector.DrawHandles";

        #endregion

        #region Attributes

        bool m_DrawHandles = true;

        SerializedProperty m_PointsSourceProperty;
        SerializedProperty m_PointsProperty;
        SerializedProperty m_TransformsProperty;
        SerializedProperty m_UseChildTransformsProperty;
        SerializedProperty m_DistanceStepProperty;
        SerializedProperty m_ParamStepProperty;
        SerializedProperty m_UniformDistributionProperty;
        SerializedProperty m_LoopProperty;

        #endregion

        #region Unity events

        void OnEnable()
        {
            m_DrawHandles = EditorPrefs.GetBool(DRAW_HANDLES_PREF_KEY, true);

            m_PointsSourceProperty        = serializedObject.FindProperty("m_PointsSource");
            m_PointsProperty              = serializedObject.FindProperty("m_Points");
            m_TransformsProperty          = serializedObject.FindProperty("m_Transforms");
            m_UseChildTransformsProperty  = serializedObject.FindProperty("m_UseChildTransforms");
            m_DistanceStepProperty        = serializedObject.FindProperty("m_DistanceStep");
            m_ParamStepProperty           = serializedObject.FindProperty("m_ParamStep");
            m_UniformDistributionProperty = serializedObject.FindProperty("m_UniformDistribution");
            m_LoopProperty                = serializedObject.FindProperty("m_Loop");
        }

        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            EditorGUILayout.PropertyField(m_PointsSourceProperty);

            if (IsTransformsSource)
            {
                EditorGUILayout.PropertyField(
                        m_UseChildTransformsProperty,
                        new GUIContent(
                                "Use Child Transforms",
                                "Use this object's direct children as control points instead of a manually attached list"
                            )
                    );

                if (!m_UseChildTransformsProperty.boolValue)
                    EditorGUILayout.PropertyField(m_TransformsProperty);
            }
            else
            {
                EditorGUILayout.PropertyField(m_PointsProperty);
            }

            EditorGUILayout.PropertyField(m_LoopProperty);
            EditorGUILayout.PropertyField(m_UniformDistributionProperty);

            if (m_UniformDistributionProperty.boolValue)
                EditorGUILayout.PropertyField(m_DistanceStepProperty, new GUIContent("Distance Step", "Max distance between sampled points, world units"));
            else
                EditorGUILayout.PropertyField(m_ParamStepProperty, new GUIContent("Param Step", "Curve parameter step per segment"));

            serializedObject.ApplyModifiedProperties();

            EditorGUI.BeginChangeCheck();

            m_DrawHandles = EditorGUILayout.Toggle(new GUIContent("Draw Handles", "Show position handles for the control points in the scene view"), m_DrawHandles);

            if (EditorGUI.EndChangeCheck())
            {
                EditorPrefs.SetBool(DRAW_HANDLES_PREF_KEY, m_DrawHandles);
                SceneView.RepaintAll();
            }
        }

        void OnSceneGUI()
        {
            if (!m_DrawHandles)
                return;

            if (IsTransformsSource)
                DrawTransformHandles();
            else
                DrawPointHandles();
        }

        #endregion

        #region Service properties

        bool IsTransformsSource => m_PointsSourceProperty.enumValueIndex == (int) Spline3D.PointsSource.TRANSFORMS;

        #endregion

        #region Service methods

        void DrawPointHandles()
        {
            Spline3D spline = (Spline3D)target;

            serializedObject.Update();

            bool changed = false;

            for (int i = 0; i < m_PointsProperty.arraySize; i++)
            {
                SerializedProperty pointProperty = m_PointsProperty.GetArrayElementAtIndex(i);

                EditorGUI.BeginChangeCheck();

                Vector3 newPosition = Handles.PositionHandle(pointProperty.vector3Value, Quaternion.identity);

                if (EditorGUI.EndChangeCheck())
                {
                    pointProperty.vector3Value = newPosition;
                    changed                    = true;
                }
            }

            if (changed)
            {
                serializedObject.ApplyModifiedProperties();
                spline.SetDirty();
            }
        }

        void DrawTransformHandles()
        {
            Spline3D spline = (Spline3D)target;

            IReadOnlyList<Transform> sourceTransforms = spline.GetSourceTransforms();

            foreach (Transform pointTransform in sourceTransforms)
            {
                if (pointTransform == null)
                    continue;

                EditorGUI.BeginChangeCheck();

                Vector3 newWorldPosition = Handles.PositionHandle(pointTransform.position, Quaternion.identity);

                if (EditorGUI.EndChangeCheck())
                {
                    Undo.RecordObject(pointTransform, "Move Spline3D Point");
                    pointTransform.position = newWorldPosition;
                    spline.SetDirty();
                }
            }
        }

        #endregion
    }
}
