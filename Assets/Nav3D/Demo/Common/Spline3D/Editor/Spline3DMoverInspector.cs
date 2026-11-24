using UnityEngine;
using UnityEditor;

namespace Nav3D.Demo.Editor
{
    [CustomEditor(typeof(Spline3DMover))]
    [CanEditMultipleObjects]
    public class Spline3DMoverInspector : UnityEditor.Editor
    {
        #region Attributes

        SerializedProperty m_SplineProperty;
        SerializedProperty m_MotionModeProperty;
        SerializedProperty m_EndBehaviorProperty;
        SerializedProperty m_PlayOnStartProperty;
        SerializedProperty m_StartDistanceProperty;

        SerializedProperty m_AlignToPathProperty;
        SerializedProperty m_RotationSpeedProperty;

        SerializedProperty m_SpeedProperty;

        SerializedProperty m_DurationProperty;
        SerializedProperty m_ProgressCurveProperty;

        SerializedProperty m_AccelerationProperty;
        SerializedProperty m_DecelerationProperty;
        SerializedProperty m_MaxSpeedProperty;


        SerializedProperty m_FlightMaxSpeedProperty;
        SerializedProperty m_FlightAccelerationProperty;
        SerializedProperty m_LookaheadProperty;
        SerializedProperty m_MaxLateralAccelerationProperty;
        SerializedProperty m_MaxDeviationProperty;
        SerializedProperty m_HardDeviationLimitProperty;
        SerializedProperty m_BankingProperty;
        SerializedProperty m_MaxBankAngleProperty;
        SerializedProperty m_BankSmoothingProperty;
        SerializedProperty m_WanderProperty;
        SerializedProperty m_WanderFrequencyProperty;

        #endregion

        #region Unity events

        void OnEnable()
        {
            //a domain reload can re-enable an inspector whose target is destroyed: touching serializedObject then throws
            //SerializedObjectNotCreatableException, and the broken instance floods the log on every repaint until the editor restarts
            if (target == null)
                return;

            m_SplineProperty        = serializedObject.FindProperty("m_Spline");
            m_MotionModeProperty    = serializedObject.FindProperty("m_MotionMode");
            m_EndBehaviorProperty   = serializedObject.FindProperty("m_EndBehavior");
            m_PlayOnStartProperty   = serializedObject.FindProperty("m_PlayOnStart");
            m_StartDistanceProperty = serializedObject.FindProperty("m_StartDistance");

            m_AlignToPathProperty   = serializedObject.FindProperty("m_AlignToPath");
            m_RotationSpeedProperty = serializedObject.FindProperty("m_RotationSpeed");

            m_SpeedProperty = serializedObject.FindProperty("m_Speed");

            m_DurationProperty      = serializedObject.FindProperty("m_Duration");
            m_ProgressCurveProperty = serializedObject.FindProperty("m_ProgressCurve");

            m_AccelerationProperty = serializedObject.FindProperty("m_Acceleration");
            m_DecelerationProperty = serializedObject.FindProperty("m_Deceleration");
            m_MaxSpeedProperty     = serializedObject.FindProperty("m_MaxSpeed");


            m_FlightMaxSpeedProperty         = serializedObject.FindProperty("m_FlightMaxSpeed");
            m_FlightAccelerationProperty     = serializedObject.FindProperty("m_FlightAcceleration");
            m_LookaheadProperty              = serializedObject.FindProperty("m_Lookahead");
            m_MaxLateralAccelerationProperty = serializedObject.FindProperty("m_MaxLateralAcceleration");
            m_MaxDeviationProperty           = serializedObject.FindProperty("m_MaxDeviation");
            m_HardDeviationLimitProperty     = serializedObject.FindProperty("m_HardDeviationLimit");
            m_BankingProperty                = serializedObject.FindProperty("m_Banking");
            m_MaxBankAngleProperty           = serializedObject.FindProperty("m_MaxBankAngle");
            m_BankSmoothingProperty          = serializedObject.FindProperty("m_BankSmoothing");
            m_WanderProperty                 = serializedObject.FindProperty("m_Wander");
            m_WanderFrequencyProperty        = serializedObject.FindProperty("m_WanderFrequency");
        }

        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            EditorGUILayout.PropertyField(m_SplineProperty);
            EditorGUILayout.PropertyField(m_MotionModeProperty);
            EditorGUILayout.PropertyField(m_EndBehaviorProperty);
            EditorGUILayout.PropertyField(m_PlayOnStartProperty);

            Spline3DMover.MotionMode mode = (Spline3DMover.MotionMode) m_MotionModeProperty.enumValueIndex;

            if (mode != Spline3DMover.MotionMode.EASED)
                EditorGUILayout.PropertyField(m_StartDistanceProperty);

            EditorGUILayout.Space();

            switch (mode)
            {
                case Spline3DMover.MotionMode.CONSTANT:
                    EditorGUILayout.PropertyField(m_SpeedProperty);
                    break;

                case Spline3DMover.MotionMode.EASED:
                    EditorGUILayout.PropertyField(m_DurationProperty);
                    EditorGUILayout.PropertyField(m_ProgressCurveProperty);
                    break;

                case Spline3DMover.MotionMode.ACCELERATED:
                    EditorGUILayout.PropertyField(m_AccelerationProperty);
                    EditorGUILayout.PropertyField(m_DecelerationProperty);
                    EditorGUILayout.PropertyField(m_MaxSpeedProperty);
                    break;

                case Spline3DMover.MotionMode.FLIGHT:
                    EditorGUILayout.PropertyField(m_FlightMaxSpeedProperty, new GUIContent("Max Speed"));
                    EditorGUILayout.PropertyField(m_FlightAccelerationProperty, new GUIContent("Acceleration"));
                    EditorGUILayout.PropertyField(m_LookaheadProperty);
                    EditorGUILayout.PropertyField(
                            m_MaxLateralAccelerationProperty,
                            new GUIContent("Max Lateral Accel", "Turn-G limit: bounds the turn rate and enforces slowdown in sharp turns")
                        );

                    EditorGUILayout.PropertyField(
                            m_MaxDeviationProperty,
                            new GUIContent(
                                    "Max Deviation",
                                    "Allowed swing-out from the path in corners: relaxes the turn braking (0 = strict path following)"
                                )
                        );

                    EditorGUILayout.PropertyField(
                            m_HardDeviationLimitProperty,
                            new GUIContent(
                                    "Hard Deviation Limit",
                                    "Steer back toward the path when the actual distance from it exceeds Max Deviation"
                                )
                        );
                    EditorGUILayout.PropertyField(m_WanderProperty, new GUIContent("Wander", "Perlin-noise wandering of the pursuit target, world units (0 = off)"));

                    if (m_WanderProperty.floatValue > 0f)
                        EditorGUILayout.PropertyField(m_WanderFrequencyProperty);
                    break;
            }

            EditorGUILayout.Space();

            EditorGUILayout.PropertyField(m_AlignToPathProperty);

            if (m_AlignToPathProperty.boolValue)
            {
                EditorGUILayout.PropertyField(m_RotationSpeedProperty);

                if (mode == Spline3DMover.MotionMode.FLIGHT)
                {
                    EditorGUILayout.PropertyField(m_BankingProperty);

                    if (m_BankingProperty.boolValue)
                    {
                        EditorGUILayout.PropertyField(m_MaxBankAngleProperty);
                        EditorGUILayout.PropertyField(m_BankSmoothingProperty);
                    }
                }
            }

            serializedObject.ApplyModifiedProperties();

            if (Application.isPlaying && !serializedObject.isEditingMultipleObjects)
            {
                Spline3DMover mover = (Spline3DMover)target;

                EditorGUILayout.Space();
                EditorGUILayout.LabelField($"Progress: {mover.Progress:P0}   Speed: {mover.CurrentSpeed:F2}", EditorStyles.miniLabel);

                EditorGUILayout.BeginHorizontal();

                if (GUILayout.Button(mover.IsPlaying ? "Pause" : "Play"))
                {
                    if (mover.IsPlaying)
                        mover.Pause();
                    else
                        mover.Play();
                }

                if (GUILayout.Button("Restart"))
                    mover.Restart();

                EditorGUILayout.EndHorizontal();
            }
        }

        #endregion
    }
}
