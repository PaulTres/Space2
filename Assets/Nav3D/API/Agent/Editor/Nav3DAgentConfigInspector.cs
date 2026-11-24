using UnityEditor;
using UnityEngine;

namespace Nav3D.API.Editor
{
    [CustomEditor(typeof(Nav3DAgentConfig))]
    class Nav3DAgentConfigInspector : UnityEditor.Editor
    {

        #region Constants

        const string MAX_SPEED_OBTAIN_METHOD_VALUE      = "Absolute value";
        const string MAX_SPEED_OBTAIN_METHOD_MULTIPLIER = "Speed multiplier";

        const string CONTROL_MODE_TOOLTIP = "NAV3D: the agent moves itself. EXTERNAL: your own controller moves the body, acting on " +
                                            "Nav3DAgent.DesiredVelocity; Nav3D reads the body's position and velocity back every tick.";

        const string OPTIMALITY_TOOLTIP = "FAST: a fast search; the route may come out a little longer. "             +
                                          "QUALITY (the default): a quality route; the search costs more than FAST. " +
                                          "HIGH_QUALITY: a still shorter route; the search costs more still.";

        #endregion

        #region Attributes

        Nav3DAgentConfig m_Config;

        SerializedProperty m_LocalAvoidanceEnabled;
        SerializedProperty m_Radius;

        SerializedProperty m_Speed;
        SerializedProperty m_MaxSpeed;
        SerializedProperty m_MaxSpeedObtainMode;
        SerializedProperty m_SpeedToMaxSpeedMultiplier;

        SerializedProperty m_ControlMode;

        //V5 motion modes
        SerializedProperty m_MotionMode;
        SerializedProperty m_Deceleration;
        SerializedProperty m_EaseSectionAnchoring;
        SerializedProperty m_EaseStartSection;
        SerializedProperty m_EaseFinalSection;
        SerializedProperty m_EaseStartCurve;
        SerializedProperty m_EaseFinalCurve;
        SerializedProperty m_FlightMaxLateralAcceleration;
        SerializedProperty m_FlightMaxDeviation;
        SerializedProperty m_FlightBankAngleMax;
        SerializedProperty m_FlightBankResponsiveness;
        SerializedProperty m_AvoidanceLookahead;
        SerializedProperty m_AvoidStaticObstacles;
        SerializedProperty m_CertifiedClearanceGate;

        //ReSharper disable once InconsistentNaming
        SerializedProperty m_Acceleration;
        SerializedProperty m_AvoidanceAggressiveness;
        SerializedProperty m_AvoidancePriority;

        SerializedProperty m_PathfindingTimeout;
        SerializedProperty m_PathfindingTimeoutMode;
        SerializedProperty m_SmoothPath;
        SerializedProperty m_SmoothRatio;
        SerializedProperty m_AutoUpdatePath;
        SerializedProperty m_PathAutoUpdateCooldown;
        SerializedProperty m_TryRepositionStartIfOccupied;
        SerializedProperty m_TryRepositionTargetIfOccupied;

        // volumetric pathfinding
        SerializedProperty m_RadiusAwarePathfinding;
        SerializedProperty m_VolumetricPathfindingMode;
        SerializedProperty m_VolumetricOptimality;
        SerializedProperty m_PathfindingRadiusScale;

        SerializedProperty m_TargetReachDistance;
        SerializedProperty m_MaxAgentDegreesRotationPerSecond;
        SerializedProperty m_RotationVectorLerpFactor;

        SerializedProperty m_UseLog;
        SerializedProperty m_LogSize;

        #endregion

        #region Properties

        //foldout states outlive this inspector instance (see EditorFoldoutState): Unity builds a fresh one
        //for every asset you select, so plain fields would collapse every group on each selection change

        static readonly EditorFoldoutState s_RadiusFoldout                 = new EditorFoldoutState("AgentConfig.Radius");
        static readonly EditorFoldoutState s_MovementFoldout               = new EditorFoldoutState("AgentConfig.Movement");
        static readonly EditorFoldoutState s_LocalAvoidanceFoldout         = new EditorFoldoutState("AgentConfig.LocalAvoidance");
        static readonly EditorFoldoutState s_LocalAvoidanceAdvancedFoldout = new EditorFoldoutState("AgentConfig.LocalAvoidance.Advanced");
        static readonly EditorFoldoutState s_PathfindingFoldout            = new EditorFoldoutState("AgentConfig.Pathfinding");
        static readonly EditorFoldoutState s_PathfindingAdvancedFoldout    = new EditorFoldoutState("AgentConfig.Pathfinding.Advanced");
        static readonly EditorFoldoutState s_DebugFoldout                  = new EditorFoldoutState("AgentConfig.Debug");

        #endregion

        #region Unity methods

        void OnEnable()
        {
            m_Radius = serializedObject.FindProperty("m_Radius");

            m_Speed                     = serializedObject.FindProperty("m_Speed");
            m_MaxSpeed                  = serializedObject.FindProperty("m_MaxSpeed");
            m_MaxSpeedObtainMode        = serializedObject.FindProperty("m_MaxSpeedObtainMode");
            m_SpeedToMaxSpeedMultiplier = serializedObject.FindProperty("m_SpeedToMaxSpeedMultiplier");

            m_ControlMode                  = serializedObject.FindProperty("m_ControlMode");
            m_MotionMode                   = serializedObject.FindProperty("m_MotionMode");
            m_Deceleration                 = serializedObject.FindProperty("m_Deceleration");
            m_EaseSectionAnchoring         = serializedObject.FindProperty("m_EaseSectionAnchoring");
            m_EaseStartSection             = serializedObject.FindProperty("m_EaseStartSection");
            m_EaseFinalSection             = serializedObject.FindProperty("m_EaseFinalSection");
            m_EaseStartCurve               = serializedObject.FindProperty("m_EaseStartCurve");
            m_EaseFinalCurve               = serializedObject.FindProperty("m_EaseFinalCurve");
            m_FlightMaxLateralAcceleration = serializedObject.FindProperty("m_FlightMaxLateralAcceleration");
            m_FlightMaxDeviation           = serializedObject.FindProperty("m_FlightMaxDeviation");
            m_FlightBankAngleMax           = serializedObject.FindProperty("m_FlightBankAngleMax");
            m_FlightBankResponsiveness     = serializedObject.FindProperty("m_FlightBankResponsiveness");

            m_AvoidanceLookahead      = serializedObject.FindProperty("m_AvoidanceLookahead");
            m_AvoidStaticObstacles    = serializedObject.FindProperty("m_AvoidStaticObstacles");
            m_CertifiedClearanceGate  = serializedObject.FindProperty("m_CertifiedClearanceGate");
            m_Acceleration            = serializedObject.FindProperty("m_Acceleration");
            m_AvoidanceAggressiveness = serializedObject.FindProperty("m_AvoidanceAggressiveness");
            m_AvoidancePriority       = serializedObject.FindProperty("m_AvoidancePriority");

            m_LocalAvoidanceEnabled = serializedObject.FindProperty("m_LocalAvoidanceEnabled");

            m_PathfindingTimeout            = serializedObject.FindProperty("m_PathfindingTimeout");
            m_PathfindingTimeoutMode        = serializedObject.FindProperty("m_PathfindingTimeoutMode");
            m_SmoothPath                    = serializedObject.FindProperty("m_SmoothPath");
            m_SmoothRatio                   = serializedObject.FindProperty("m_SmoothRatio");
            m_AutoUpdatePath                = serializedObject.FindProperty("m_AutoUpdatePath");
            m_PathAutoUpdateCooldown        = serializedObject.FindProperty("m_PathAutoUpdateCooldown");
            m_TryRepositionStartIfOccupied  = serializedObject.FindProperty("m_TryRepositionStartIfOccupied");
            m_TryRepositionTargetIfOccupied = serializedObject.FindProperty("m_TryRepositionTargetIfOccupied");

            m_RadiusAwarePathfinding    = serializedObject.FindProperty("m_RadiusAwarePathfinding");
            m_VolumetricPathfindingMode = serializedObject.FindProperty("m_VolumetricPathfindingMode");
            m_VolumetricOptimality      = serializedObject.FindProperty("m_VolumetricOptimality");
            m_PathfindingRadiusScale    = serializedObject.FindProperty("m_PathfindingRadiusScale");

            m_TargetReachDistance              = serializedObject.FindProperty("m_TargetReachDistance");
            m_MaxAgentDegreesRotationPerSecond = serializedObject.FindProperty("m_MaxAgentDegreesRotationPerSecond");
            m_RotationVectorLerpFactor         = serializedObject.FindProperty("m_RotationVectorLerpFactor");

            m_UseLog  = serializedObject.FindProperty("m_UseLog");
            m_LogSize = serializedObject.FindProperty("m_LogSize");
        }

        public override void OnInspectorGUI()
        {
            m_Config = (Nav3DAgentConfig) target;

            serializedObject.Update();

            HeaderControls();

            //Radius setup
            RadiusFoldout();

            AgentRadiusWorldCheck.DrawHelpBox(m_Config);

            //Movement setup (mode + speed)
            MovementFoldout();

            //Local avoidance setup: the foldout carries its own master switch and is always present — when
            //the feature is off its settings are greyed rather than hidden, so nothing a user tuned
            //disappears from the inspector
            LocalAvoidanceFoldout();

            //Pathfinding setup
            PathfindingFoldout();

            DebugFoldout();

            serializedObject.ApplyModifiedProperties();
        }

        #endregion

        #region Service methods

        void HeaderControls()
        {
            EditorGUILayout.BeginHorizontal();

            if (GUILayout.Button("Expand All"))
            {
                s_RadiusFoldout.Value         = true;
                s_MovementFoldout.Value       = true;
                s_LocalAvoidanceFoldout.Value = true;
                s_PathfindingFoldout.Value    = true;
                s_DebugFoldout.Value          = true;
            }

            if (GUILayout.Button("Collapse All"))
            {
                s_RadiusFoldout.Value         = false;
                s_MovementFoldout.Value       = false;
                s_LocalAvoidanceFoldout.Value = false;
                s_PathfindingFoldout.Value    = false;
                s_DebugFoldout.Value          = false;
            }

            EditorGUILayout.EndHorizontal();

            if (GUILayout.Button("Reset to defaults") &&
                EditorUtility.DisplayDialog("Set default config parameters?", "Are you sure?", "Yes", "Cancel"))
                ((Nav3DAgentConfig) target).SetDefaultAttributes();

            if (GUILayout.Button("Save changes"))
                Save();
        }

        void Save()
        {
            EditorUtility.SetDirty(target);
            AssetDatabase.SaveAssets();
        }

        void RadiusFoldout()
        {
            //ReSharper disable once AssignmentInConditionalExpression
            if (s_RadiusFoldout.Value = EditorGUILayout.BeginFoldoutHeaderGroup(s_RadiusFoldout.Value, "Radius"))
            {
                EditorGUILayout.PropertyField(m_Radius);
            }

            EditorGUILayout.EndFoldoutHeaderGroup();
        }

        void MovementFoldout()
        {
            //ReSharper disable once AssignmentInConditionalExpression
            if (s_MovementFoldout.Value = EditorGUILayout.BeginFoldoutHeaderGroup(s_MovementFoldout.Value, "Movement"))
            {

                #if LITE_VERSION

                EditorGUILayout.HelpBox("External Control is available only in Nav3D Pro.", MessageType.Info);
                EditorGUI.BeginDisabledGroup(true);

                #endif

                EditorGUILayout.PropertyField(m_ControlMode, new GUIContent("Control mode", CONTROL_MODE_TOOLTIP));

                #if LITE_VERSION

                EditorGUI.EndDisabledGroup();

                #endif

                EditorGUILayout.Space();

                MotionModeSelector();

                EditorGUILayout.Space();

                EditorGUILayout.PropertyField(m_Speed, new GUIContent("Speed (units/second)"));

                if (m_MaxSpeedObtainMode.intValue == 0)
                {
                    m_MaxSpeed.floatValue = m_Speed.floatValue * m_SpeedToMaxSpeedMultiplier.floatValue;
                }
                else if (m_MaxSpeedObtainMode.intValue == 1)
                {
                    m_Speed.floatValue = Mathf.Clamp(m_Speed.floatValue, 0, Mathf.Max(m_Speed.floatValue, m_MaxSpeed.floatValue));
                }


                #if LITE_VERSION

                bool maxSpeedInteractive = false;

                EditorGUILayout.HelpBox("The settings below are available only in Nav3D Pro.", MessageType.Info);

                #else

                bool maxSpeedInteractive = m_LocalAvoidanceEnabled.boolValue;

                if (!maxSpeedInteractive)
                    EditorGUILayout.HelpBox(
                            "Max speed is the overspeed budget an avoidance maneuver may spend, so it applies only while "
                            + "Local avoidance is enabled. With avoidance off the agent moves at Speed.",
                            MessageType.Info
                        );

                #endif

                EditorGUI.BeginDisabledGroup(!maxSpeedInteractive);

                EditorGUILayout.LabelField("Select a deriving method for max speed:");

                m_MaxSpeedObtainMode.intValue = GUILayout.Toolbar(
                        m_MaxSpeedObtainMode.intValue,
                        new[] { MAX_SPEED_OBTAIN_METHOD_MULTIPLIER, MAX_SPEED_OBTAIN_METHOD_VALUE }
                    );

                if (m_MaxSpeedObtainMode.intValue == 0)
                {
                    EditorGUILayout.PropertyField(m_SpeedToMaxSpeedMultiplier);

                    EditorGUILayout.LabelField($"Max speed: {m_SpeedToMaxSpeedMultiplier.floatValue * m_Speed.floatValue}");
                }
                else
                {
                    EditorGUILayout.PropertyField(m_MaxSpeed, new GUIContent("Max speed (units/second)"));
                }

                EditorGUI.EndDisabledGroup();

                //former "Motion" foldout content — arrival + rotation are movement parameters, so they
                //live in the Movement foldout (the redesign's unified grouping)
                EditorGUILayout.Space();

                EditorGUILayout.BeginHorizontal();

                EditorGUILayout.PrefixLabel("Target reach distance");
                m_TargetReachDistance.floatValue = EditorGUILayout.FloatField(m_TargetReachDistance.floatValue);

                EditorGUILayout.EndHorizontal();

                EditorGUILayout.LabelField("Rotation", EditorStyles.boldLabel);

                //under External Control the game's controller turns the body

                #if !LITE_VERSION

                EditorGUI.BeginDisabledGroup(m_ControlMode.enumValueIndex == (int) AgentControlMode.EXTERNAL);

                #else

                EditorGUI.BeginDisabledGroup(false);

                #endif

                EditorGUI.indentLevel++;

                EditorGUILayout.BeginHorizontal();

                EditorGUILayout.PrefixLabel("Max rotation in degrees per second");
                m_MaxAgentDegreesRotationPerSecond.floatValue = Mathf.Max(0f, EditorGUILayout.FloatField(m_MaxAgentDegreesRotationPerSecond.floatValue));

                EditorGUILayout.EndHorizontal();

                EditorGUILayout.BeginHorizontal();

                EditorGUILayout.PrefixLabel("Rotation vector lerp factor");
                m_RotationVectorLerpFactor.floatValue = Mathf.Clamp(EditorGUILayout.FloatField(m_RotationVectorLerpFactor.floatValue), 0.001f, 1f);

                EditorGUILayout.EndHorizontal();

                EditorGUI.indentLevel--;

                EditorGUI.EndDisabledGroup();
            }

            EditorGUILayout.EndFoldoutHeaderGroup();
        }

        //the motion-mode toolbar + only the selected mode's fields (mirrors PathfindingModeSelector's shape)
        void MotionModeSelector()
        {

            #if LITE_VERSION

            EditorGUILayout.HelpBox("Motion modes other than Constant are available only in Nav3D Pro.", MessageType.Info);
            EditorGUI.BeginDisabledGroup(true);

            #endif

            EditorGUILayout.LabelField("Motion mode", EditorStyles.boldLabel);

            m_MotionMode.enumValueIndex = GUILayout.Toolbar(
                    m_MotionMode.enumValueIndex,
                    new[] { "Constant", "Eased", "Accelerated", "Flight" }
                );

            AgentMotionMode selectedMode = (AgentMotionMode) m_MotionMode.enumValueIndex;

            //no early return on CONSTANT: the Lite disabled-group wrapper below must always be closed
            if (selectedMode != AgentMotionMode.CONSTANT)
                DrawSelectedModeFields(selectedMode);

            #if LITE_VERSION

            EditorGUI.EndDisabledGroup();

            #endif

        }

        void DrawSelectedModeFields(AgentMotionMode _SelectedMode)
        {
            EditorGUI.indentLevel++;

            switch (_SelectedMode)
            {
                case AgentMotionMode.EASED:

                    EditorGUILayout.PropertyField(m_EaseSectionAnchoring, new GUIContent("Section anchoring"));

                    bool fractionAnchoring = m_EaseSectionAnchoring.enumValueIndex == (int) EaseSectionAnchoring.PATH_FRACTION;

                    string extentUnits = fractionAnchoring ? "path fraction" : "units";

                    m_EaseStartSection.floatValue = Mathf.Max(0f, EditorGUILayout.FloatField($"Start section ({extentUnits})", m_EaseStartSection.floatValue));
                    m_EaseFinalSection.floatValue = Mathf.Max(0f, EditorGUILayout.FloatField($"Final section ({extentUnits})", m_EaseFinalSection.floatValue));

                    if (fractionAnchoring && m_EaseStartSection.floatValue + m_EaseFinalSection.floatValue > 1f)
                        EditorGUILayout.HelpBox(
                                "Start + final fractions exceed 1 — both sections will be scaled down proportionally on every path.",
                                MessageType.Info
                            );

                    EditorGUILayout.PropertyField(
                            m_EaseStartCurve,
                            new GUIContent(
                                    "Start ease curve",
                                    "X: travel through the start section (0 = launch, 1 = cruise reached). " +
                                    "Y: speed ÷ cruise. Rising 0→1 = ramp-up."
                                )
                        );

                    EditorGUILayout.PropertyField(
                            m_EaseFinalCurve,
                            new GUIContent(
                                    "Final ease curve",
                                    "X: travel through the final section (0 = section entry, 1 = the target). " +
                                    "Y: speed ÷ cruise. Descending 1→0 = braking; must reach 0 to arrive."
                                )
                        );

                    break;

                case AgentMotionMode.ACCELERATED:

                    //0 is "auto" here — show the number it resolves to
                    float autoAcceleration = m_Speed.floatValue / Nav3DAgentConfig.AUTO_ACCELERATION_TIME_TO_CRUISE;

                    m_Acceleration.floatValue = Mathf.Max(
                            0f,
                            EditorGUILayout.FloatField(
                                    new GUIContent($"Acceleration (u/s², 0 = auto: {autoAcceleration:0.##})"),
                                    m_Acceleration.floatValue
                                )
                        );

                    m_Deceleration.floatValue = Mathf.Max(
                            0f,
                            EditorGUILayout.FloatField(
                                    new GUIContent("Deceleration (u/s², 0 = same as acceleration)"),
                                    m_Deceleration.floatValue
                                )
                        );

                    break;

                case AgentMotionMode.FLIGHT:

                    m_Acceleration.floatValue = Mathf.Max(0f, EditorGUILayout.FloatField(new GUIContent("Acceleration (u/s², 0 = instant)"), m_Acceleration.floatValue));

                    EditorGUILayout.PropertyField(m_FlightMaxLateralAcceleration, new GUIContent("Max lateral acceleration (u/s²)"));
                    EditorGUILayout.PropertyField(m_FlightMaxDeviation, new GUIContent("Max path deviation (units)"));
                    EditorGUILayout.PropertyField(m_FlightBankAngleMax, new GUIContent("Max bank angle (degrees)"));
                    EditorGUILayout.PropertyField(m_FlightBankResponsiveness, new GUIContent("Bank responsiveness (1/s)"));

                    break;
            }

            EditorGUI.indentLevel--;
        }

        void LocalAvoidanceFoldout()
        {
            //the master switch lives at the top of the block it governs, and the block's title reports its
            //state so a collapsed foldout still tells the truth

            #if LITE_VERSION

            bool   settingsInteractive = false;
            string title = "Local avoidance";

            #else

            bool   settingsInteractive = m_LocalAvoidanceEnabled.boolValue;
            string title               = settingsInteractive ? "Local avoidance" : "Local avoidance (off)";

            #endif

            //ReSharper disable once AssignmentInConditionalExpression
            if (s_LocalAvoidanceFoldout.Value = EditorGUILayout.BeginFoldoutHeaderGroup(s_LocalAvoidanceFoldout.Value, title))
            {

                #if LITE_VERSION

                EditorGUILayout.HelpBox("Local avoidance is available only in Nav3D Pro.", MessageType.Info);
                EditorGUI.BeginDisabledGroup(true);

                #endif

                EditorGUILayout.BeginHorizontal();

                EditorGUILayout.PrefixLabel("Local avoidance enabled");
                m_LocalAvoidanceEnabled.boolValue = EditorGUILayout.Toggle(m_LocalAvoidanceEnabled.boolValue);

                EditorGUILayout.EndHorizontal();

                #if LITE_VERSION

                EditorGUI.EndDisabledGroup();

                #endif

                EditorGUILayout.Space();

                //everything below the master switch follows it: GREYED, never hidden, so nothing a user
                //tuned disappears. The "Advanced" foldout header stays interactive either way — the values
                //inside must remain readable while avoidance is off

                //static-obstacle avoidance additionally needs a route that keeps a clearance for the body,
                //which only volumetric pathfinding produces — so it follows that switch as well
                bool staticInteractive = settingsInteractive && m_RadiusAwarePathfinding.boolValue;

                if (settingsInteractive && !staticInteractive)
                    EditorGUILayout.HelpBox("Avoiding static obstacles requires Volumetric pathfinding.", MessageType.Info);

                EditorGUI.BeginDisabledGroup(!staticInteractive);

                EditorGUILayout.BeginHorizontal();

                EditorGUILayout.PrefixLabel("Avoid static obstacles");
                m_AvoidStaticObstacles.boolValue = EditorGUILayout.Toggle(m_AvoidStaticObstacles.boolValue);

                EditorGUILayout.EndHorizontal();

                EditorGUI.EndDisabledGroup();

                EditorGUI.indentLevel++;

                //ReSharper disable once AssignmentInConditionalExpression
                if (s_LocalAvoidanceAdvancedFoldout.Value = EditorGUILayout.Foldout(s_LocalAvoidanceAdvancedFoldout.Value, "Advanced", true))
                {
                    EditorGUI.BeginDisabledGroup(!settingsInteractive);

                    m_AvoidanceLookahead.floatValue = Mathf.Max(
                            0f,
                            EditorGUILayout.FloatField(
                                    new GUIContent("Avoidance lookahead (body widths)"),
                                    m_AvoidanceLookahead.floatValue
                                )
                        );

                    EditorGUILayout.HelpBox(
                            m_AvoidanceLookahead.floatValue <= 0f
                                ? "At 0 agents only react once they are already touching. Avoidance can still " +
                                  "separate them, but nothing is anticipated."
                                : $"Agents begin negotiating about {m_AvoidanceLookahead.floatValue:F1}× their "   +
                                  "combined size before contact. The horizon in seconds is derived per encounter " +
                                  "from both bodies' radii and speeds — two agents of radius "                     +
                                  $"{m_Radius.floatValue:F2} and top speed {m_Config.MaxSpeed:F2} u/s negotiate "  +
                                  "over roughly "                                                                  +
                                  $"{EstimateHorizonSeconds(m_AvoidanceLookahead.floatValue, m_Radius.floatValue, m_Config.MaxSpeed):F2} s.",
                            MessageType.Info
                        );

                    //the gate reconciles the two layers, so it shows exactly when both of them are on
                    if (m_AvoidStaticObstacles.boolValue && m_RadiusAwarePathfinding.boolValue)
                    {
                        m_CertifiedClearanceGate.boolValue = EditorGUILayout.Toggle(
                                new GUIContent("Certified clearance gate"),
                                m_CertifiedClearanceGate.boolValue
                            );

                        EditorGUILayout.HelpBox(
                                m_CertifiedClearanceGate.boolValue
                                    ? "Radius-aware paths carry the clearance they were measured to keep, and " +
                                      "the static-obstacle constraints stand down while the agent stays on "    +
                                      "that certified line — it flies guaranteed narrow passages at full "      +
                                      "speed. The constraints return as the agent drifts off the path, and "    +
                                      "instantly if the world changes under it."
                                    : "Off: the static-obstacle constraints act at full force even on a path " +
                                      "already certified clear for this radius — the agent will brake and "    +
                                      "jitter near geometry the pathfinding accounted for. Tune "              +
                                      "PathfindingRadiusScale and Avoidance lookahead against each other by hand.",
                                MessageType.Info
                            );
                    }

                    m_Acceleration.floatValue = Mathf.Max(
                            0f,
                            EditorGUILayout.FloatField(
                                    new GUIContent("Acceleration (u/s², 0 = unlimited)"),
                                    m_Acceleration.floatValue
                                )
                        );

                    m_AvoidanceAggressiveness.floatValue = EditorGUILayout.Slider(
                            new GUIContent("Avoidance aggressiveness"),
                            m_AvoidanceAggressiveness.floatValue,
                            0f,
                            1f
                        );

                    if (m_AvoidanceAggressiveness.floatValue < 1f)
                        EditorGUILayout.HelpBox(
                                "Below 1.0 the avoidance result is blended toward the raw path velocity — softer " +
                                "maneuvers, but the collision-free guarantee no longer holds.",
                                MessageType.Warning
                            );

                    m_AvoidancePriority.floatValue = Mathf.Max(
                            0f,
                            EditorGUILayout.FloatField(new GUIContent("Avoidance priority"), m_AvoidancePriority.floatValue)
                        );

                    if (m_AvoidancePriority.floatValue > 1f)
                        EditorGUILayout.HelpBox(
                                "Above 1.0 this agent holds its line as if it were larger: when two agents meet they "  +
                                "share one maneuver, and each does a portion of it proportional to the OTHER's radius " +
                                "times priority.",
                                MessageType.Info
                            );
                    else if (m_AvoidancePriority.floatValue < 1f)
                        EditorGUILayout.HelpBox(
                                m_AvoidancePriority.floatValue > 0f
                                    ? "Below 1.0 this agent gives way more than its size alone would ask: when " +
                                      "two agents meet they share one maneuver, and each does a portion of it "  +
                                      "proportional to the OTHER's radius times priority."
                                    : "At 0 this agent takes the ENTIRE avoidance maneuver onto itself and no one " +
                                      "gives way to it. It can still be pushed against geometry it cannot pass, "   +
                                      "so leave a margin for anything that must stay clear.",
                                MessageType.Info
                            );

                    EditorGUI.EndDisabledGroup();
                }

                EditorGUI.indentLevel--;
            }

            EditorGUILayout.EndFoldoutHeaderGroup();
        }

        //what the derived horizon comes out as for a same-size, same-speed pair — the inspector's way of
        //showing that the setting has a concrete meaning in seconds without asking anyone to supply one
        static float EstimateHorizonSeconds(float _Lookahead, float _Radius, float _MaxSpeed)
        {
            return _MaxSpeed > 0f ? _Lookahead * _Radius / _MaxSpeed : 0f;
        }

        void PathfindingFoldout()
        {
            //ReSharper disable once AssignmentInConditionalExpression
            if (s_PathfindingFoldout.Value = EditorGUILayout.BeginFoldoutHeaderGroup(s_PathfindingFoldout.Value, "Pathfinding"))
            {
                PathfindingModeSelector();

                EditorGUILayout.Space();

                EditorGUILayout.BeginHorizontal();

                EditorGUILayout.PrefixLabel("Pathfinding timeout (ms)");
                m_PathfindingTimeout.intValue = EditorGUILayout.IntField(m_PathfindingTimeout.intValue);

                EditorGUILayout.EndHorizontal();

                EditorGUILayout.PropertyField(m_PathfindingTimeoutMode, new GUIContent("On timeout"));

                EditorGUILayout.BeginHorizontal();

                EditorGUILayout.PrefixLabel("Smooth the path");
                m_SmoothPath.boolValue = EditorGUILayout.Toggle(m_SmoothPath.boolValue);

                EditorGUILayout.EndHorizontal();

                if (m_SmoothPath.boolValue)
                {
                    EditorGUILayout.BeginHorizontal();

                    EditorGUILayout.PrefixLabel("Samples per min bucket volume");
                    m_SmoothRatio.intValue = Mathf.Max(1, EditorGUILayout.IntField(m_SmoothRatio.intValue));

                    EditorGUILayout.EndHorizontal();
                }

                EditorGUILayout.BeginHorizontal();

                EditorGUILayout.PrefixLabel("Auto-update path on stagnant behavior");
                m_AutoUpdatePath.boolValue = EditorGUILayout.Toggle(m_AutoUpdatePath.boolValue);

                EditorGUILayout.EndHorizontal();

                if (m_AutoUpdatePath.boolValue)
                {
                    EditorGUILayout.BeginHorizontal();

                    EditorGUILayout.PrefixLabel("Auto-update cooldown (ms)");
                    m_PathAutoUpdateCooldown.intValue = EditorGUILayout.IntField(m_PathAutoUpdateCooldown.intValue);

                    EditorGUILayout.EndHorizontal();
                }

                EditorGUI.indentLevel++;

                //ReSharper disable once AssignmentInConditionalExpression
                if (s_PathfindingAdvancedFoldout.Value = EditorGUILayout.Foldout(s_PathfindingAdvancedFoldout.Value, "Advanced", true))
                {
                    EditorGUILayout.BeginHorizontal();

                    EditorGUILayout.PrefixLabel("Try reposition start if occupied");
                    m_TryRepositionStartIfOccupied.boolValue = EditorGUILayout.Toggle(m_TryRepositionStartIfOccupied.boolValue);

                    EditorGUILayout.EndHorizontal();

                    EditorGUILayout.BeginHorizontal();

                    EditorGUILayout.PrefixLabel("Try reposition target if occupied");
                    m_TryRepositionTargetIfOccupied.boolValue = EditorGUILayout.Toggle(m_TryRepositionTargetIfOccupied.boolValue);

                    EditorGUILayout.EndHorizontal();
                }

                EditorGUI.indentLevel--;
            }

            EditorGUILayout.EndFoldoutHeaderGroup();
        }

        void PathfindingModeSelector()
        {

            #if LITE_VERSION

            EditorGUILayout.HelpBox("Volumetric pathfinding is available only in Nav3D Pro.", MessageType.Info);
            EditorGUI.BeginDisabledGroup(true);

            #endif

            EditorGUILayout.LabelField("Pathfinding mode", EditorStyles.boldLabel);

            int selectedMode = GUILayout.Toolbar(
                    m_RadiusAwarePathfinding.boolValue ? 1 : 0,
                    new[] { "Point", "Volumetric" }
                );

            m_RadiusAwarePathfinding.boolValue = selectedMode == 1;

            if (m_RadiusAwarePathfinding.boolValue)
            {
                EditorGUI.indentLevel++;

                EditorGUILayout.PropertyField(m_VolumetricPathfindingMode, new GUIContent("On too-narrow space"));
                UtilsEditor.OptimalityField(m_VolumetricOptimality, new GUIContent("Optimality", OPTIMALITY_TOOLTIP));

                EditorGUILayout.BeginHorizontal();

                EditorGUILayout.PrefixLabel("Pathfinding radius scale");
                m_PathfindingRadiusScale.floatValue = Mathf.Max(0.01f, EditorGUILayout.FloatField(m_PathfindingRadiusScale.floatValue));

                EditorGUILayout.EndHorizontal();

                EditorGUILayout.HelpBox(
                        $"The path keeps at least {m_Radius.floatValue * m_PathfindingRadiusScale.floatValue:F3} " +
                        "(Radius × scale) clear of obstacles and refuses passages narrower than twice that. "      +
                        "The effective radius is clamped to the world settings' Max agent radius at runtime.",
                        MessageType.Info
                    );

                EditorGUI.indentLevel--;
            }

            #if LITE_VERSION

            EditorGUI.EndDisabledGroup();

            #endif

        }

        void DebugFoldout()
        {
            //ReSharper disable once AssignmentInConditionalExpression
            if (s_DebugFoldout.Value = EditorGUILayout.BeginFoldoutHeaderGroup(s_DebugFoldout.Value, "Debug"))
            {
                EditorGUILayout.PropertyField(m_UseLog);

                if (m_UseLog.boolValue)
                {
                    EditorGUILayout.BeginHorizontal();

                    EditorGUILayout.PrefixLabel("Log records count:");
                    m_LogSize.intValue = EditorGUILayout.IntField(m_LogSize.intValue);

                    EditorGUILayout.EndHorizontal();

                    EditorGUILayout.Space();
                }
            }

            EditorGUILayout.EndFoldoutHeaderGroup();
        }

        #endregion

    }
}
