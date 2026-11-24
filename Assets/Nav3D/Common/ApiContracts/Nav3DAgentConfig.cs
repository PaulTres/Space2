using System;
using System.Text;
using UnityEngine;

namespace Nav3D.API
{
    [CreateAssetMenu(fileName = "AgentConfig", menuName = "Nav3D/Agent config", order = 150)]
    public partial class Nav3DAgentConfig : ScriptableObject
    {

        #region Constants : Debug

        const string SPEED_LOOKS_PER_TICK_WARNING =
            "Nav3DAgentConfig.Speed was set to {0} — since Nav3D 2.2 Speed is in units per SECOND "           +
            "(pre-2.2 it was units per FixedUpdate tick). A value this small usually means the calling code " +
            "still passes a per-tick value; multiply it by 1/Time.fixedDeltaTime (×50 at the default 0.02 timestep).";

        #endregion

        #region Constants : Defaults

        //config format version for the one-shot field migrations in MigrateLegacyFieldsIfNeeded;
        //0 = pre-movement-redesign (2.1.x and older)
        const int CONFIG_VERSION_CURRENT = 5;

        //migration sentinel: a fresh instance keeps -1, an old asset overwrites it with its per-tick value —
        //that is how the migration tells "old asset" from "new instance" without guessing from magnitudes
        const float LEGACY_UNSET = -1f;

        const float DEFAULT_RADIUS                        = 0.25f;
        const float DEFAULT_SPEED                         = 0.5f; //units/SECOND
        const float DEFAULT_SPEED_TO_MAX_SPEED_MULTIPLIER = 1.15f;
        const int   DEFAULT_MAX_SPEED_OBTAIN_MODE         = 0;

        const int   DEFAULT_PATHFINDING_TIMEOUT               = 2000; //ms
        const bool  DEFAULT_SMOOTH_PATH                       = false;
        const int   DEFAULT_SMOOTH_PATH_RATIO                 = 3;
        const bool  DEFAULT_LOCAL_AVOIDANCE_ENABLED           = true;
        const bool  DEFAULT_AUTOUPDATE_PATH                   = true;
        const int   DEFAULT_AUTOUPDATE_PATH_COOLDOWN          = 2000; //ms
        const float DEFAULT_MAX_DEGREES_ROTATION_PER_SECOND   = 250f; //degrees/SECOND (5 degrees at 50 Hz)
        const float DEFAULT_ROTATION_VECTOR_LERP_FACTOR       = 0.1f;
        const bool  DEFAULT_TRY_REPOSITION_START_IF_OCCUPIED  = true;
        const bool  DEFAULT_TRY_REPOSITION_TARGET_IF_OCCUPIED = true;

        const bool  DEFAULT_AVOID_STATIC_OBSTACLES   = true;
        const bool  DEFAULT_CERTIFIED_CLEARANCE_GATE = true;
        const float DEFAULT_ACCELERATION             = 0f; //u/s²; 0 = unlimited (no acceleration constraint)
        const float DEFAULT_AVOIDANCE_AGGRESSIVENESS = 1f; //1 = pure ORCA (the guarantee); <1 = soft blend toward VPref
        const float DEFAULT_AVOIDANCE_PRIORITY       = 1f; //1 = the agent's own radius decides its share of a manoeuvre

        //combined body widths, not seconds: the horizon in time is derived per encounter
        const float DEFAULT_AVOIDANCE_LOOKAHEAD = 2f;

        //clearance beyond the pathfinding radius the delivered trajectory is certified for, in body radii —
        //one radius of slack keeps the gate's static constraints down across ordinary tracking drift while
        //still re-arming them before the body leaves certified space
        const float CERTIFIED_CLEARANCE_MARGIN = 1f;

        const AgentControlMode     DEFAULT_CONTROL_MODE           = AgentControlMode.NAV3D;
        const AgentMotionMode      DEFAULT_MOTION_MODE            = AgentMotionMode.CONSTANT;
        const float                DEFAULT_DECELERATION           = 0f; //u/s²; 0 = follow Acceleration
        const EaseSectionAnchoring DEFAULT_EASE_SECTION_ANCHORING = EaseSectionAnchoring.ABSOLUTE_LENGTH;
        const float                DEFAULT_EASE_START_SECTION     = 1f; //units, or a fraction under PATH_FRACTION
        const float                DEFAULT_EASE_FINAL_SECTION     = 1.5f; //units, or a fraction under PATH_FRACTION

        //ACCELERATED with Acceleration 0: "instant" would only reproduce CONSTANT, so the ramp is derived from
        //Speed instead — the rate that reaches cruise in this many seconds (see AccelerationEffective)
        internal const float AUTO_ACCELERATION_TIME_TO_CRUISE = 0.5f;

        const float DEFAULT_FLIGHT_MAX_LATERAL_ACCELERATION = 6f; //u/s²; turn-G limit
        const float DEFAULT_FLIGHT_MAX_DEVIATION            = 1f; //units; allowed corner swing-out
        const float DEFAULT_FLIGHT_BANK_ANGLE_MAX           = 45f; //degrees
        const float DEFAULT_FLIGHT_BANK_RESPONSIVENESS      = 4f; //bank smoothing rate, 1/s

        const bool DEFAULT_USE_LOG  = false;
        const int  DEFAULT_LOG_SIZE = 50;

        // volumetric (agent-radius-aware) pathfinding
        const bool  DEFAULT_RADIUS_AWARE_PATHFINDING = false;
        const float DEFAULT_PATHFINDING_RADIUS_SCALE = 1f;

        #endregion

        #region Serialized fields

        [SerializeField] float m_Radius = DEFAULT_RADIUS;

        //units/SECOND; LEGACY_UNSET marks data the units migration still has to convert
        [SerializeField] float m_Speed                     = LEGACY_UNSET;
        [SerializeField] float m_MaxSpeed                  = LEGACY_UNSET;
        [SerializeField] float m_SpeedToMaxSpeedMultiplier = DEFAULT_SPEED_TO_MAX_SPEED_MULTIPLIER;
        [SerializeField] int   m_MaxSpeedObtainMode        = DEFAULT_MAX_SPEED_OBTAIN_MODE; //0-multiplier, 1-absolute

        [SerializeField] bool  m_AvoidStaticObstacles    = DEFAULT_AVOID_STATIC_OBSTACLES;
        [SerializeField] bool  m_CertifiedClearanceGate  = DEFAULT_CERTIFIED_CLEARANCE_GATE;
        [SerializeField] float m_AvoidanceLookahead      = DEFAULT_AVOIDANCE_LOOKAHEAD;
        [SerializeField] float m_Acceleration            = DEFAULT_ACCELERATION;
        [SerializeField] float m_AvoidanceAggressiveness = DEFAULT_AVOIDANCE_AGGRESSIVENESS;
        [SerializeField] float m_AvoidancePriority       = DEFAULT_AVOIDANCE_PRIORITY;

        [SerializeField] AgentControlMode     m_ControlMode          = DEFAULT_CONTROL_MODE;
        [SerializeField] AgentMotionMode      m_MotionMode           = DEFAULT_MOTION_MODE;
        [SerializeField] float                m_Deceleration         = DEFAULT_DECELERATION;
        [SerializeField] EaseSectionAnchoring m_EaseSectionAnchoring = DEFAULT_EASE_SECTION_ANCHORING;
        [SerializeField] float                m_EaseStartSection     = DEFAULT_EASE_START_SECTION;
        [SerializeField] float                m_EaseFinalSection     = DEFAULT_EASE_FINAL_SECTION;
        [SerializeField] AnimationCurve       m_EaseStartCurve       = AnimationCurve.EaseInOut(0f, 0f, 1f, 1f);

        //no initializer on purpose: null tells the v3 migration "fresh instance", so it can take the default
        //instead of reversing an old asset's keys
        [SerializeField] AnimationCurve m_EaseFinalCurve;

        [SerializeField] float m_FlightMaxLateralAcceleration = DEFAULT_FLIGHT_MAX_LATERAL_ACCELERATION;
        [SerializeField] float m_FlightMaxDeviation           = DEFAULT_FLIGHT_MAX_DEVIATION;
        [SerializeField] float m_FlightBankAngleMax           = DEFAULT_FLIGHT_BANK_ANGLE_MAX;
        [SerializeField] float m_FlightBankResponsiveness     = DEFAULT_FLIGHT_BANK_RESPONSIVENESS;

        [SerializeField] int                    m_PathfindingTimeout     = DEFAULT_PATHFINDING_TIMEOUT; //ms
        [SerializeField] PathfindingTimeoutMode m_PathfindingTimeoutMode = PathfindingTimeoutMode.DELIVER_FOUND_ROUTE;
        [SerializeField] bool                   m_SmoothPath             = DEFAULT_SMOOTH_PATH;
        [SerializeField] int                    m_SmoothRatio            = DEFAULT_SMOOTH_PATH_RATIO;

        [SerializeField] bool m_LocalAvoidanceEnabled = DEFAULT_LOCAL_AVOIDANCE_ENABLED;

        //the removed MotionNavigationType enum, kept as an int so an old asset's value (COMBINED=0, GLOBAL=1)
        //survives deserialization as the v0→v1 migration source. Read nowhere else.
        [SerializeField] int   m_MotionNavigationType;
        [SerializeField] int   m_ConfigVersion; //0 when the asset predates versioning
        [SerializeField] float m_TargetReachDistance              = DEFAULT_RADIUS;
        [SerializeField] bool  m_AutoUpdatePath                   = DEFAULT_AUTOUPDATE_PATH;
        [SerializeField] int   m_PathAutoUpdateCooldown           = DEFAULT_AUTOUPDATE_PATH_COOLDOWN; //ms
        [SerializeField] float m_MaxAgentDegreesRotationPerSecond = DEFAULT_MAX_DEGREES_ROTATION_PER_SECOND;

        #if UNITY_EDITOR

        //Read old text AND binary assets in the editor, then persist the new value before a build.
        //This compatibility source is absent from player code; no serialized-name remapping is used.
        [SerializeField, HideInInspector] float m_MaxAgentDegreesRotationPerTick = LEGACY_UNSET;

        //Serialized so a script/domain reload before the deferred save cannot erase pending migration.
        [SerializeField, HideInInspector] int  m_AssetMigrationSourceVersion = -1;
        internal                          int  AssetMigrationSourceVersion                  => m_AssetMigrationSourceVersion;
        internal                          void SetAssetMigrationSourceVersion(int _Version) => m_AssetMigrationSourceVersion = _Version;
        internal                          void MarkAssetMigrationPersisted()                => m_AssetMigrationSourceVersion = -1;

        #endif

        [SerializeField] float m_RotationVectorLerpFactor      = DEFAULT_ROTATION_VECTOR_LERP_FACTOR;
        [SerializeField] bool  m_TryRepositionStartIfOccupied  = DEFAULT_TRY_REPOSITION_START_IF_OCCUPIED;
        [SerializeField] bool  m_TryRepositionTargetIfOccupied = DEFAULT_TRY_REPOSITION_TARGET_IF_OCCUPIED;

        // volumetric (agent-radius-aware) pathfinding
        [SerializeField] bool                      m_RadiusAwarePathfinding    = DEFAULT_RADIUS_AWARE_PATHFINDING;
        [SerializeField] VolumetricPathfindingMode m_VolumetricPathfindingMode = VolumetricPathfindingMode.STRICT;
        [SerializeField] VolumetricOptimality      m_VolumetricOptimality      = VolumetricOptimality.QUALITY;
        [SerializeField] float                     m_PathfindingRadiusScale    = DEFAULT_PATHFINDING_RADIUS_SCALE;

        [SerializeField] bool m_UseLog  = DEFAULT_USE_LOG;
        [SerializeField] int  m_LogSize = DEFAULT_LOG_SIZE;

        #endregion

        #region Properties

        /// <summary>
        /// Agent radius used for local avoidance.
        /// </summary>
        public float Radius
        {
            get => m_Radius;
            set
            {
                if (value <= 0)
                    return;

                m_Radius = value;
            }
        }

        /// <summary>
        /// Desired speed in world units per second (config version ≥2; pre-2.2 configs held units-per-tick
        /// and are converted by the migration / the "Tools/Nav3D/Upgrade Agent Configs" tool).
        /// </summary>
        public float Speed
        {
            get => m_Speed;
            set
            {
                if (value <= 0)
                    return;

                //a value at the old per-tick scale almost certainly comes from caller code that was never
                //updated for the units change
                if (value <= 0.05f)
                    Debug.LogWarning(string.Format(SPEED_LOOKS_PER_TICK_WARNING, value));

                //no equality early-return: the MaxSpeed derivation below must run even when the assigned value
                //equals the current one, since a fresh instance may still hold no MaxSpeed
                if (m_MaxSpeedObtainMode == 0)
                {
                    m_Speed = value;

                    MaxSpeed = m_Speed * m_SpeedToMaxSpeedMultiplier;
                }
                else if (m_MaxSpeedObtainMode == 1)
                {
                    m_Speed = Mathf.Clamp(value, 0, m_MaxSpeed);
                }
                else
                {
                    throw new Exception($"Unknown m_MaxSpeedObtainMode type {value}");
                }
            }
        }

        /// <summary>
        /// The maximum speed allowed for performing local avoidance, world units per second (config version ≥2).
        /// </summary>
        public float MaxSpeed
        {
            get => m_MaxSpeed;
            set => m_MaxSpeed = Mathf.Max(value, m_Speed);
        }

        public float FactualMaxSpeed
        {
            get
            {
                //through the property, not the field: overspeed only exists while avoidance is effectively active
                if (!LocalAvoidanceEnabled)
                    return Speed;

                return MaxSpeed;
            }
        }

        //The config's public surface is units/SECOND while the movement internals work per FixedUpdate tick.
        //The accessors below are the only sanctioned conversion points. Main-thread only (Time API).

        internal float MaxSpeedPerTick => m_MaxSpeed * Time.fixedDeltaTime;

        internal float FactualMaxSpeedPerTick => FactualMaxSpeed * Time.fixedDeltaTime;

        //per-tick velocity-change budget (u/tick per tick): a·dt² ; 0 = unconstrained
        internal float AccelerationPerTick => m_Acceleration * Time.fixedDeltaTime * Time.fixedDeltaTime;

        public float SpeedToMaxSpeedMultiplier
        {
            get => m_SpeedToMaxSpeedMultiplier;
            set => m_SpeedToMaxSpeedMultiplier = Mathf.Max(1 + float.Epsilon, value);
        }

        internal int MaxSpeedObtainMode
        {
            get => m_MaxSpeedObtainMode;
            set => m_MaxSpeedObtainMode = value;
        }

        public int PathfindingTimeout
        {
            get => m_PathfindingTimeout;
            set
            {
                if (value <= 0 || m_PathfindingTimeout == value)
                    return;

                m_PathfindingTimeout = value;
            }
        }

        /// <summary>
        /// What a search does when PathfindingTimeout passes while it already holds a complete route: deliver it
        /// flagged TimedOut (DELIVER_FOUND_ROUTE, the default) or fail the request with TIMEOUT (FAIL).
        /// </summary>
        public PathfindingTimeoutMode TimeoutMode
        {
            get => m_PathfindingTimeoutMode;
            set => m_PathfindingTimeoutMode = value;
        }

        public bool SmoothPath
        {
            get => m_SmoothPath;
            set => m_SmoothPath = value;
        }

        public int SmoothRatio
        {
            get => m_SmoothRatio;
            set
            {
                if (value <= 0 || m_SmoothRatio == value)
                    return;

                m_SmoothRatio = value;
            }
        }

        public float TargetReachDistance
        {
            get => m_TargetReachDistance;
            set
            {
                if (value < 0)
                    return;

                m_TargetReachDistance = value;
            }
        }

        public bool AutoUpdatePath
        {
            get => m_AutoUpdatePath;
            set => m_AutoUpdatePath = value;
        }

        public int PathAutoUpdateCooldown
        {
            get => m_PathAutoUpdateCooldown;
            set => m_PathAutoUpdateCooldown = Mathf.Max(1, value);
        }

        /// <summary>Maximum angular speed in degrees per second; zero disables rotation.</summary>
        public float MaxAgentDegreesRotationPerSecond
        {
            get => m_MaxAgentDegreesRotationPerSecond;
            set => m_MaxAgentDegreesRotationPerSecond = Mathf.Max(0f, value);
        }

        //Source compatibility for existing callers; runtime rotation uses the per-second setting only.
        [Obsolete("Use MaxAgentDegreesRotationPerSecond (degrees per second).")]
        public float MaxAgentDegreesRotationPerTick
        {
            get => MaxAgentDegreesRotationPerSecond * Time.fixedDeltaTime;
            set => MaxAgentDegreesRotationPerSecond = value / (Time.fixedDeltaTime > 0f ? Time.fixedDeltaTime : 0.02f);
        }

        internal float RotationVectorLerpFactor
        {
            get => m_RotationVectorLerpFactor;
            set => m_RotationVectorLerpFactor = Mathf.Max(0f, value);
        }

        /// <summary>
        /// Whether an agent that finds itself inside an obstacle — one moved onto it, or a spawn point inside geometry —
        /// starts its path from the nearest free point instead of failing the order.
        /// </summary>
        public bool TryRepositionStartIfOccupied
        {
            get => m_TryRepositionStartIfOccupied;
            set => m_TryRepositionStartIfOccupied = value;
        }

        /// <summary>
        /// Whether a destination inside an obstacle is replaced by the nearest free point outside it, where the path then
        /// ends, instead of failing the order.
        /// </summary>
        public bool TryRepositionTargetIfOccupied
        {
            get => m_TryRepositionTargetIfOccupied;
            set => m_TryRepositionTargetIfOccupied = value;
        }

        /// <summary>
        /// Whether pathfinding keeps the found path at least the agent radius clear of obstacles and
        /// refuses passages narrower than the agent (volumetric pathfinding). Off = point pathfinding (the
        /// default, cheaper mode), where the body's clearance is nobody's job and
        /// <see cref="AvoidStaticObstacles"/> stays inert.
        /// </summary>
        public bool RadiusAwarePathfinding
        {
            get => m_RadiusAwarePathfinding;
            set => m_RadiusAwarePathfinding = value;
        }

        /// <summary>
        /// What to do when no corridor wide enough for the agent exists: fail honestly (STRICT) or
        /// deliver a point path flagged as degraded (FALLBACK_TO_DEFAULT).
        /// </summary>
        public VolumetricPathfindingMode VolumetricMode
        {
            get => m_VolumetricPathfindingMode;
            set => m_VolumetricPathfindingMode = value;
        }

        /// <summary>
        /// How the volumetric route trades quality for search time: FAST (a fast search, the route may come
        /// out a little longer), QUALITY (the default: a quality route, a costlier search than FAST) or HIGH_QUALITY (a
        /// still shorter route for a still costlier search).
        /// </summary>
        public VolumetricOptimality Optimality
        {
            get => m_VolumetricOptimality;
            set => m_VolumetricOptimality = value;
        }

        /// <summary>
        /// Multiplier on Radius for the pathfinding clearance (e.g. 1.2 gives fast agents a safety
        /// margin without touching the local-avoidance radius). Must be positive.
        /// </summary>
        public float PathfindingRadiusScale
        {
            get => m_PathfindingRadiusScale;
            set
            {
                if (value > 0)
                    m_PathfindingRadiusScale = value;
            }
        }

        /// <summary>
        /// Whether the agent's local avoidance treats nearby obstacle surfaces as hard constraints.
        /// Takes effect only together with volumetric pathfinding: a point path is routed for a
        /// point, so it carries no clearance the constraints could be reconciled against, and they
        /// end up fighting the very route the agent was given. Under point pathfinding the authored
        /// value is kept but the agent avoids other agents only; it applies again as soon as
        /// volumetric pathfinding is switched on.
        /// </summary>
        public bool AvoidStaticObstacles
        {
            //PathfindingRadius is the clearance the search keeps: positive under volumetric pathfinding, 0 otherwise
            get => m_AvoidStaticObstacles && PathfindingRadius > 0f;
            set => m_AvoidStaticObstacles = value;
        }

        /// <summary>
        /// The lookahead a movable with no config of its own (shells, evaders) contributes to a pair.
        /// </summary>
        public static float DefaultAvoidanceLookahead => DEFAULT_AVOIDANCE_LOOKAHEAD;

        /// <summary>
        /// How far ahead this agent is willing to start avoiding, measured in COMBINED body WIDTHS: 2 (the
        /// default) means a pair begins negotiating when the gap between their surfaces is about twice their
        /// combined size. The horizon in seconds is derived from it per encounter, from the two bodies' radii
        /// and speeds, because the time an encounter needs is a property of the pair rather than of one agent.
        /// A pair negotiates on the smaller of the two lookaheads. Raise it for earlier, smoother avoidance;
        /// lower it to make agents commit later and swerve harder.
        /// </summary>
        public float AvoidanceLookahead
        {
            get => m_AvoidanceLookahead;
            set => m_AvoidanceLookahead = Mathf.Max(0f, value);
        }

        /// <summary>
        /// Maximum velocity change, world units/second². 0 = unlimited. When set, the avoidance LP
        /// additionally constrains the new velocity to the acceleration ball around the current one
        /// (as relaxable constraints — safety wins over comfort in dense situations). In the ACCELERATED
        /// motion mode 0 means "auto" for the speed ramp — the rate that reaches Speed in
        /// <see cref="AUTO_ACCELERATION_TIME_TO_CRUISE"/> seconds (<see cref="AccelerationEffective"/>) — while
        /// the avoidance constraint stays off.
        /// </summary>
        public float Acceleration
        {
            get => m_Acceleration;
            set => m_Acceleration = Mathf.Max(0f, value);
        }

        /// <summary>
        /// 0..1 blend of the avoidance solution toward the raw path velocity: 1 (default) = the pure ORCA
        /// solution and its collision-free guarantee; lower values consciously trade the guarantee for
        /// softer deviation.
        /// </summary>
        public float AvoidanceAggressiveness
        {
            get => m_AvoidanceAggressiveness;
            set => m_AvoidanceAggressiveness = Mathf.Clamp01(value);
        }

        /// <summary>
        /// How strongly this agent's size counts when a pair splits the work of avoiding each other.
        /// Two agents that meet share one manoeuvre, and each takes a portion of it proportional to the
        /// other's weight, where weight = Radius * AvoidancePriority: the smaller (or lower-priority)
        /// body does more of the moving, which is why a large agent is no longer pushed aside by a crowd
        /// of small ones. Equal radii and equal priorities give the classic even split.
        /// 1 (default) = the agent's own radius alone decides its portion; raise it for a body that
        /// should hold its line as if it were larger, lower it toward 0 for one that always gives way
        /// (0 = takes the entire manoeuvre onto itself). Never negative.
        /// </summary>
        public float AvoidancePriority
        {
            get => m_AvoidancePriority;
            set => m_AvoidancePriority = Mathf.Max(0f, value);
        }

        /// <summary>
        /// Arrival-braking rate, world units/second². 0 (default) = use Acceleration.
        /// </summary>
        public float Deceleration
        {
            get => m_Deceleration;
            set => m_Deceleration = Mathf.Max(0f, value);
        }

        //the ramp rate the ACCELERATED controller uses: the authored Acceleration, or the auto-derived rate when
        //it is 0. Deliberately not what the avoidance constraints read (AccelerationPerTick, the raw field) —
        //an auto-derived ramp must not switch those on
        public float AccelerationEffective => m_Acceleration > 0f || MotionMode != AgentMotionMode.ACCELERATED
            ? m_Acceleration
            : Speed / AUTO_ACCELERATION_TIME_TO_CRUISE;

        //the braking rate the speed controllers actually use (Deceleration, falling back to AccelerationEffective)
        public float DecelerationEffective => m_Deceleration > 0f ? m_Deceleration : AccelerationEffective;

        /// <summary>
        /// EASED mode: whether the start/final section extents are absolute arc lengths (world units) or
        /// fractions of the current path length.
        /// </summary>
        public EaseSectionAnchoring EaseAnchoring
        {
            get => m_EaseSectionAnchoring;
            set => m_EaseSectionAnchoring = value;
        }

        /// <summary>
        /// EASED mode: extent of the start (ramp-up) path section — world units, or a 0..1 fraction of the
        /// path length under PATH_FRACTION anchoring.
        /// </summary>
        public float EaseStartSection
        {
            get => m_EaseStartSection;
            set => m_EaseStartSection = Mathf.Max(0f, value);
        }

        /// <summary>
        /// EASED mode: extent of the final (braking) path section — world units, or a 0..1 fraction of the
        /// path length under PATH_FRACTION anchoring.
        /// </summary>
        public float EaseFinalSection
        {
            get => m_EaseFinalSection;
            set => m_EaseFinalSection = Mathf.Max(0f, value);
        }

        /// <summary>
        /// EASED mode: normalized ease curve of the start section (0..1 ramp progress → 0..1 speed factor
        /// between the section's entry speed and the cruise speed).
        /// </summary>
        public AnimationCurve EaseStartCurve
        {
            get => m_EaseStartCurve;
            set => m_EaseStartCurve = value ?? m_EaseStartCurve;
        }

        /// <summary>
        /// EASED mode: normalized ease curve of the final section (0..1 remaining-fraction → 0..1 speed
        /// factor of the cruise speed; 0 at 0 remaining).
        /// </summary>
        public AnimationCurve EaseFinalCurve
        {
            get => m_EaseFinalCurve;
            set => m_EaseFinalCurve = value ?? m_EaseFinalCurve;
        }

        /// <summary>
        /// FLIGHT mode: turn-G limit, world units/second² — bounds the turn rate and enforces slowdown in
        /// sharp turns.
        /// </summary>
        public float FlightMaxLateralAcceleration
        {
            get => m_FlightMaxLateralAcceleration;
            set => m_FlightMaxLateralAcceleration = Mathf.Max(0.01f, value);
        }

        /// <summary>
        /// FLIGHT mode: allowed swing-out from the path in corners, world units — relaxes the anticipatory
        /// turn braking.
        /// </summary>
        public float FlightMaxDeviation
        {
            get => m_FlightMaxDeviation;
            set => m_FlightMaxDeviation = Mathf.Max(0f, value);
        }

        /// <summary>
        /// FLIGHT mode: maximum banking (roll) angle in turns, degrees.
        /// </summary>
        public float FlightBankAngleMax
        {
            get => m_FlightBankAngleMax;
            set => m_FlightBankAngleMax = Mathf.Clamp(value, 0f, 90f);
        }

        /// <summary>
        /// FLIGHT mode: how quickly the bank angle follows the turn, 1/second.
        /// </summary>
        public float FlightBankResponsiveness
        {
            get => m_FlightBankResponsiveness;
            set => m_FlightBankResponsiveness = Mathf.Max(0f, value);
        }

        //per-tick danger radius. Computed rather than cached, so it tracks fixedDeltaTime. Main-thread only.
        public float VelocityRadius => Radius + FactualMaxSpeedPerTick;

        public bool UseLog
        {
            get => m_UseLog;
            set => m_UseLog = value;
        }

        public int LogSize
        {
            get => m_LogSize;
            set
            {
                int finalValue = Mathf.Max(value, 5);

                m_LogSize = finalValue;
            }
        }

        public static Nav3DAgentConfig DefaultConfig => CreateInstance(typeof(Nav3DAgentConfig)) as Nav3DAgentConfig;

        //the current config format version (for the editor upgrade tool's on-disk version comparison)
        internal static int ConfigVersionCurrent => CONFIG_VERSION_CURRENT;

        #endregion

        #region Public methods

        /// <summary>
        /// Configures config by default.
        /// </summary>
        internal void SetDefaultAttributes()
        {
            m_Radius                           = DEFAULT_RADIUS;
            m_Speed                            = DEFAULT_SPEED;
            m_SpeedToMaxSpeedMultiplier        = DEFAULT_SPEED_TO_MAX_SPEED_MULTIPLIER;
            m_MaxSpeedObtainMode               = DEFAULT_MAX_SPEED_OBTAIN_MODE;
            m_TargetReachDistance              = m_Radius;
            m_PathfindingTimeout               = DEFAULT_PATHFINDING_TIMEOUT;
            m_PathfindingTimeoutMode           = PathfindingTimeoutMode.DELIVER_FOUND_ROUTE;
            m_SmoothPath                       = DEFAULT_SMOOTH_PATH;
            m_SmoothRatio                      = DEFAULT_SMOOTH_PATH_RATIO;
            m_LocalAvoidanceEnabled            = DEFAULT_LOCAL_AVOIDANCE_ENABLED;
            m_ConfigVersion                    = CONFIG_VERSION_CURRENT;
            m_AutoUpdatePath                   = DEFAULT_AUTOUPDATE_PATH;
            m_PathAutoUpdateCooldown           = DEFAULT_AUTOUPDATE_PATH_COOLDOWN;
            m_MaxAgentDegreesRotationPerSecond = DEFAULT_MAX_DEGREES_ROTATION_PER_SECOND;
            m_RotationVectorLerpFactor         = DEFAULT_ROTATION_VECTOR_LERP_FACTOR;
            m_TryRepositionStartIfOccupied     = DEFAULT_TRY_REPOSITION_START_IF_OCCUPIED;
            m_TryRepositionTargetIfOccupied    = DEFAULT_TRY_REPOSITION_TARGET_IF_OCCUPIED;
            m_RadiusAwarePathfinding           = DEFAULT_RADIUS_AWARE_PATHFINDING;
            m_VolumetricPathfindingMode        = VolumetricPathfindingMode.STRICT;
            m_VolumetricOptimality             = VolumetricOptimality.QUALITY;
            m_PathfindingRadiusScale           = DEFAULT_PATHFINDING_RADIUS_SCALE;
            m_AvoidStaticObstacles             = DEFAULT_AVOID_STATIC_OBSTACLES;
            m_CertifiedClearanceGate           = DEFAULT_CERTIFIED_CLEARANCE_GATE;
            m_AvoidanceLookahead               = DEFAULT_AVOIDANCE_LOOKAHEAD;
            m_Acceleration                     = DEFAULT_ACCELERATION;
            m_AvoidanceAggressiveness          = DEFAULT_AVOIDANCE_AGGRESSIVENESS;
            m_AvoidancePriority                = DEFAULT_AVOIDANCE_PRIORITY;
            m_ControlMode                      = DEFAULT_CONTROL_MODE;
            m_MotionMode                       = DEFAULT_MOTION_MODE;
            m_Deceleration                     = DEFAULT_DECELERATION;
            m_EaseSectionAnchoring             = DEFAULT_EASE_SECTION_ANCHORING;
            m_EaseStartSection                 = DEFAULT_EASE_START_SECTION;
            m_EaseFinalSection                 = DEFAULT_EASE_FINAL_SECTION;
            m_EaseStartCurve                   = AnimationCurve.EaseInOut(0f, 0f, 1f, 1f);
            m_EaseFinalCurve                   = DefaultEaseFinalCurve();
            m_FlightMaxLateralAcceleration     = DEFAULT_FLIGHT_MAX_LATERAL_ACCELERATION;
            m_FlightMaxDeviation               = DEFAULT_FLIGHT_MAX_DEVIATION;
            m_FlightBankAngleMax               = DEFAULT_FLIGHT_BANK_ANGLE_MAX;
            m_FlightBankResponsiveness         = DEFAULT_FLIGHT_BANK_RESPONSIVENESS;
            m_UseLog                           = DEFAULT_USE_LOG;
            m_LogSize                          = DEFAULT_LOG_SIZE;
        }

        /// <summary>
        /// One-shot in-memory migration of legacy serialized fields; idempotent (keyed on m_ConfigVersion).
        /// Runs on deserialization (OnEnable); asset persistence happens via the config upgrade tool.
        /// </summary>
        internal void MigrateLegacyFieldsIfNeeded()
        {
            if (m_ConfigVersion >= CONFIG_VERSION_CURRENT)
                return;

            #if UNITY_EDITOR

            m_AssetMigrationSourceVersion = m_ConfigVersion;

            #endif

            if (m_ConfigVersion < 1)
            {
                //v0 -> v1: MotionNavigationType (COMBINED=0 / GLOBAL=1) collapsed into the LocalAvoidanceEnabled flag
                m_LocalAvoidanceEnabled = m_MotionNavigationType == 0;
            }

            if (m_ConfigVersion < 2)
            {
                //v1 -> v2: speeds units-per-TICK -> units-per-SECOND. LEGACY_UNSET -> fresh instance, take the
                //new default; a positive value -> old asset, convert it literally.
                float fixedDelta = Time.fixedDeltaTime > 0f ? Time.fixedDeltaTime : 0.02f;

                m_Speed = m_Speed > 0f ? m_Speed / fixedDelta : DEFAULT_SPEED;

                //also repairs MaxSpeed = 0, which an old asset may carry
                m_MaxSpeed = m_MaxSpeed > 0f
                    ? m_MaxSpeed / fixedDelta
                    : m_Speed    * Mathf.Max(1f + float.Epsilon, m_SpeedToMaxSpeedMultiplier);
            }

            if (m_ConfigVersion < 3)
            {
                //v2 -> v3: the final ease curve became chronological (X = travel through the section, read
                //left-to-right like the start curve); v2 stored the remaining-distance convention. Reversing
                //an old asset's keys is behaviour-preserving because the evaluation flipped with it.
                m_EaseFinalCurve = m_EaseFinalCurve == null || m_EaseFinalCurve.keys.Length == 0
                    ? DefaultEaseFinalCurve()
                    : ReverseCurve(m_EaseFinalCurve);
            }

            if (m_ConfigVersion < 4)
            {
                //v3 -> v4: the avoidance horizon in SECONDS (m_ORCATau / m_ORCATauObstacle, both removed) gave
                //way to m_AvoidanceLookahead. Nothing to convert — the old values were unusable across mixed
                //sizes, which is why they went. Dead YAML keys disappear on the next re-save.
                m_AvoidanceLookahead = DEFAULT_AVOIDANCE_LOOKAHEAD;
            }

            #if UNITY_EDITOR

            if (m_ConfigVersion < 5 && m_MaxAgentDegreesRotationPerTick >= 0f)
            {
                //The editor reads the old field under its actual name and persists the conversion.
                float fixedDelta = Time.fixedDeltaTime > 0f ? Time.fixedDeltaTime : 0.02f;
                m_MaxAgentDegreesRotationPerSecond = m_MaxAgentDegreesRotationPerTick / fixedDelta;
            }

            m_MaxAgentDegreesRotationPerTick = LEGACY_UNSET;

            #endif

            m_ConfigVersion = CONFIG_VERSION_CURRENT;
        }

        public void FixInvalidParams()
        {
            if (m_AvoidanceLookahead < 0f)
                m_AvoidanceLookahead = DEFAULT_AVOIDANCE_LOOKAHEAD;
        }

        public Nav3DAgentConfig Copy()
        {
            Nav3DAgentConfig copy = CreateInstance(typeof(Nav3DAgentConfig)) as Nav3DAgentConfig;

            //ReSharper disable once PossibleNullReferenceException
            copy.m_Radius                           = m_Radius;
            copy.m_MaxSpeedObtainMode               = m_MaxSpeedObtainMode;
            copy.m_SpeedToMaxSpeedMultiplier        = m_SpeedToMaxSpeedMultiplier;
            copy.m_Speed                            = m_Speed;
            copy.m_MaxSpeed                         = m_MaxSpeed;
            copy.m_TargetReachDistance              = m_TargetReachDistance;
            copy.m_PathfindingTimeout               = m_PathfindingTimeout;
            copy.m_PathfindingTimeoutMode           = m_PathfindingTimeoutMode;
            copy.m_SmoothPath                       = m_SmoothPath;
            copy.m_SmoothRatio                      = m_SmoothRatio;
            copy.m_LocalAvoidanceEnabled            = m_LocalAvoidanceEnabled;
            copy.m_MotionNavigationType             = m_MotionNavigationType;
            copy.m_ConfigVersion                    = m_ConfigVersion;
            copy.m_AutoUpdatePath                   = m_AutoUpdatePath;
            copy.m_PathAutoUpdateCooldown           = m_PathAutoUpdateCooldown;
            copy.m_MaxAgentDegreesRotationPerSecond = m_MaxAgentDegreesRotationPerSecond;
            copy.m_RotationVectorLerpFactor         = m_RotationVectorLerpFactor;
            copy.m_TryRepositionStartIfOccupied     = m_TryRepositionStartIfOccupied;
            copy.m_TryRepositionTargetIfOccupied    = m_TryRepositionTargetIfOccupied;
            copy.m_RadiusAwarePathfinding           = m_RadiusAwarePathfinding;
            copy.m_VolumetricPathfindingMode        = m_VolumetricPathfindingMode;
            copy.m_VolumetricOptimality             = m_VolumetricOptimality;
            copy.m_PathfindingRadiusScale           = m_PathfindingRadiusScale;
            copy.m_AvoidStaticObstacles             = m_AvoidStaticObstacles;
            copy.m_CertifiedClearanceGate           = m_CertifiedClearanceGate;
            copy.m_AvoidanceLookahead               = m_AvoidanceLookahead;
            copy.m_Acceleration                     = m_Acceleration;
            copy.m_AvoidanceAggressiveness          = m_AvoidanceAggressiveness;
            copy.m_AvoidancePriority                = m_AvoidancePriority;
            copy.m_ControlMode                      = m_ControlMode;
            copy.m_MotionMode                       = m_MotionMode;
            copy.m_Deceleration                     = m_Deceleration;
            copy.m_EaseSectionAnchoring             = m_EaseSectionAnchoring;
            copy.m_EaseStartSection                 = m_EaseStartSection;
            copy.m_EaseFinalSection                 = m_EaseFinalSection;
            //AnimationCurve is a mutable reference type — deep-copy so per-agent config copies can't share keys
            copy.m_EaseStartCurve               = m_EaseStartCurve == null ? null : new AnimationCurve(m_EaseStartCurve.keys);
            copy.m_EaseFinalCurve               = m_EaseFinalCurve == null ? null : new AnimationCurve(m_EaseFinalCurve.keys);
            copy.m_FlightMaxLateralAcceleration = m_FlightMaxLateralAcceleration;
            copy.m_FlightMaxDeviation           = m_FlightMaxDeviation;
            copy.m_FlightBankAngleMax           = m_FlightBankAngleMax;
            copy.m_FlightBankResponsiveness     = m_FlightBankResponsiveness;
            copy.m_UseLog                       = m_UseLog;
            copy.m_LogSize                      = m_LogSize;

            return copy;
        }

        public override string ToString()
        {
            StringBuilder stringBuilder = new StringBuilder();

            stringBuilder.AppendLine($"{nameof(m_Radius)}: {m_Radius}");

            stringBuilder.AppendLine($"{nameof(m_Speed)}: {m_Speed}");
            stringBuilder.AppendLine($"{nameof(m_MaxSpeed)}: {m_MaxSpeed}");
            stringBuilder.AppendLine($"{nameof(m_SpeedToMaxSpeedMultiplier)}: {m_SpeedToMaxSpeedMultiplier}");
            stringBuilder.AppendLine($"{nameof(m_MaxSpeedObtainMode)}: {m_MaxSpeedObtainMode}");

            stringBuilder.AppendLine($"{nameof(m_PathfindingTimeout)}: {m_PathfindingTimeout}");
            stringBuilder.AppendLine($"{nameof(m_PathfindingTimeoutMode)}: {m_PathfindingTimeoutMode}");
            stringBuilder.AppendLine($"{nameof(m_SmoothPath)}: {m_SmoothPath}");
            stringBuilder.AppendLine($"{nameof(m_SmoothRatio)}: {m_SmoothRatio}");

            stringBuilder.AppendLine($"{nameof(m_LocalAvoidanceEnabled)}: {m_LocalAvoidanceEnabled}");

            stringBuilder.AppendLine($"{nameof(m_TargetReachDistance)}: {m_TargetReachDistance}");
            stringBuilder.AppendLine($"{nameof(m_AutoUpdatePath)}: {m_AutoUpdatePath}");
            stringBuilder.AppendLine($"{nameof(m_PathAutoUpdateCooldown)}: {m_PathAutoUpdateCooldown}");
            stringBuilder.AppendLine($"{nameof(m_MaxAgentDegreesRotationPerSecond)}: {m_MaxAgentDegreesRotationPerSecond}");
            stringBuilder.AppendLine($"{nameof(m_RotationVectorLerpFactor)}: {m_RotationVectorLerpFactor}");
            stringBuilder.AppendLine($"{nameof(m_TryRepositionStartIfOccupied)}: {m_TryRepositionStartIfOccupied}");
            stringBuilder.AppendLine($"{nameof(m_TryRepositionTargetIfOccupied)}: {m_TryRepositionTargetIfOccupied}");
            stringBuilder.AppendLine($"{nameof(m_RadiusAwarePathfinding)}: {m_RadiusAwarePathfinding}");
            stringBuilder.AppendLine($"{nameof(m_VolumetricPathfindingMode)}: {m_VolumetricPathfindingMode}");
            stringBuilder.AppendLine($"{nameof(m_VolumetricOptimality)}: {m_VolumetricOptimality}");
            stringBuilder.AppendLine($"{nameof(m_PathfindingRadiusScale)}: {m_PathfindingRadiusScale}");

            stringBuilder.AppendLine($"{nameof(m_AvoidStaticObstacles)}: {m_AvoidStaticObstacles}");
            stringBuilder.AppendLine($"{nameof(m_CertifiedClearanceGate)}: {m_CertifiedClearanceGate}");
            stringBuilder.AppendLine($"{nameof(m_AvoidanceLookahead)}: {m_AvoidanceLookahead}");
            stringBuilder.AppendLine($"{nameof(m_Acceleration)}: {m_Acceleration}");
            stringBuilder.AppendLine($"{nameof(m_AvoidanceAggressiveness)}: {m_AvoidanceAggressiveness}");
            stringBuilder.AppendLine($"{nameof(m_AvoidancePriority)}: {m_AvoidancePriority}");

            stringBuilder.AppendLine($"{nameof(m_ControlMode)}: {m_ControlMode}");
            stringBuilder.AppendLine($"{nameof(m_MotionMode)}: {m_MotionMode}");
            stringBuilder.AppendLine($"{nameof(m_Deceleration)}: {m_Deceleration}");
            stringBuilder.AppendLine($"{nameof(m_EaseSectionAnchoring)}: {m_EaseSectionAnchoring}");
            stringBuilder.AppendLine($"{nameof(m_EaseStartSection)}: {m_EaseStartSection}");
            stringBuilder.AppendLine($"{nameof(m_EaseFinalSection)}: {m_EaseFinalSection}");
            stringBuilder.AppendLine($"{nameof(m_FlightMaxLateralAcceleration)}: {m_FlightMaxLateralAcceleration}");
            stringBuilder.AppendLine($"{nameof(m_FlightMaxDeviation)}: {m_FlightMaxDeviation}");
            stringBuilder.AppendLine($"{nameof(m_FlightBankAngleMax)}: {m_FlightBankAngleMax}");
            stringBuilder.AppendLine($"{nameof(m_FlightBankResponsiveness)}: {m_FlightBankResponsiveness}");

            stringBuilder.AppendLine($"{nameof(m_UseLog)}: {m_UseLog}");
            stringBuilder.AppendLine($"{nameof(m_LogSize)}: {m_LogSize}");

            return stringBuilder.ToString();
        }

        #endregion

        #region Unity methods

        void OnEnable()
        {
            //covers every load path (asset deserialization, CreateInstance, Copy) before any consumer reads flags
            MigrateLegacyFieldsIfNeeded();
        }

        #endregion

        #region Service methods

        static AnimationCurve DefaultEaseFinalCurve()
        {
            return AnimationCurve.EaseInOut(0f, 1f, 1f, 0f);
        }

        //mirrors a curve around X = 0.5: key times complement, tangents/weights swap sides and negate.
        //Mutates in place so an open curve editor pointed at the instance stays valid across the swap
        static AnimationCurve ReverseCurve(AnimationCurve _Curve)
        {
            Keyframe[] source   = _Curve.keys;
            Keyframe[] reversed = new Keyframe[source.Length];

            for (int i = 0; i < source.Length; i++)
            {
                Keyframe key = source[source.Length - 1 - i];

                Keyframe reversedKey = new Keyframe(1f - key.time, key.value, -key.outTangent, -key.inTangent);

                reversedKey.inWeight  = key.outWeight;
                reversedKey.outWeight = key.inWeight;

                reversedKey.weightedMode = key.weightedMode == WeightedMode.In
                    ? WeightedMode.Out
                    : key.weightedMode == WeightedMode.Out
                        ? WeightedMode.In
                        : key.weightedMode;

                reversed[i] = reversedKey;
            }

            _Curve.keys = reversed;

            return _Curve;
        }

        #endregion

    }
}
