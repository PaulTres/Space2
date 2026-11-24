using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Nav3D.API;
using Nav3D.Common;
using Nav3D.LocalAvoidance;
using Nav3D.Obstacles;
using Nav3D.Pathfinding;
using UnityEngine;

namespace Nav3D.Agents
{
    /// <summary>
    /// The agent movement pipeline: one concrete class whose behavioural variation lives in composed stages
    /// rather than subclasses — path direction plus carrot (the "Path direction" region), scalar speed
    /// (<see cref="ISpeedController"/>) and the optional <see cref="IAvoidanceStage"/>, where null means local
    /// avoidance is disabled.
    /// </summary>
    public partial class AgentMotionPipeline : IMovable
    {

        #region Constants

        readonly string LOG_BEGIN_MOVE_TO_POINT              = $"{nameof(AgentMotionPipeline)}.{nameof(BeginMoveTo)}: {{0}}";
        readonly string LOG_BEGIN_MOVE_TO_POINTS             = $"{nameof(AgentMotionPipeline)}.{nameof(BeginMoveToPoints)}: {{0}}";
        readonly string LOG_BEGIN_MOVE_TO_WITH_EXISTING_PATH = $"{nameof(AgentMotionPipeline)}.{nameof(MoveToUsingExistingPath)}: {{0}}";
        readonly string LOG_RESTART_FOLLOWING                = $"{nameof(AgentMotionPipeline)}.{nameof(ReinitPathFollowingData)}";
        readonly string LOG_REINIT_PATH                      = $"{nameof(AgentMotionPipeline)}.{nameof(RestartFollowingForCurrentPath)}";
        readonly string LOG_PATH_UPDATE_SUCCESS              = $"{nameof(AgentMotionPipeline)}.{nameof(BeginPathFollowing)}";

        //How often the carrot is re-tested for having become blocked, and how far ahead the skip may look
        //(see SkipBlockedCarrot). The period is a hard per-agent interval: the test runs at most once per
        //period. The lookahead must span a door leaf plus its clearance on a smoothed path whose points sit
        //centimetres apart, yet stay far short of a path's length so a sealed route still fails to the
        //re-path layer instead of winding the cursor to the end.
        const int CARROT_CHECK_PERIOD_TICKS = 3;
        const int CARROT_SKIP_LOOKAHEAD     = 48;

        #endregion

        #region Attributes

        bool m_IsNeighborMovablesDirty  = true;
        bool m_IsNeighborObstaclesDirty = true;

        Vector3 m_CurrentPosition;
        Vector3 m_LastVelocity;
        Vector3 m_LastNonZeroVelocity;

        protected Nav3DAgentConfig m_Config;

        protected PathfindingRequest m_Request;

        protected Action<Vector3> m_OnTargetPassed;
        protected Action          m_OnLastTargetReached;

        protected readonly Log m_Log;

        protected Action<PathfindingResult> m_OnPathUpdated;

        //path direction state
        protected Nav3DPath      m_Path;
        protected PathFollowData m_CurrFollowData;

        protected Vector3 m_PathNextPoint;

        //blocked-carrot check state (see SkipBlockedCarrot); the delegate is cached because a method-group
        //conversion allocates a fresh Func per call
        int                 m_CarrotCheckCountdown;
        ObstacleManager     m_CarrotCheckManager;
        Func<Vector3, bool> m_CarrotBlockedTest;
        bool                m_CarrotProbeRefused;

        //direct-pursuit tick channel (TargetFollowingLogic's terminal homing): valid for exactly the tick it
        //was set for — consumed and cleared by the next GetVelocity, so a stale aim cannot outlive its order
        bool    m_DirectAimSet;
        Vector3 m_DirectAim;
        float   m_DirectAimSpeedCap;

        //the strike exclusion (see IsAvoiding): written in the apply phase, read in the solve phase by this
        //agent's stage and by the target's own. A single reference, so no copy is needed; null = avoid everyone
        IMovable m_StrikeTarget;

        protected Action<PathfindingResult> m_OnPathfindingFail;

        //deviation/auto-update state
        DateTime m_LastPathUpdateFireTime;

        bool m_DeviatedFromPath;
        bool m_DeviatedFromPathLastFrame;

        //the local-avoidance seam; null = local avoidance disabled
        protected IAvoidanceStage m_AvoidanceStage;

        //the scalar-speed seam (units/second out; see ISpeedController)
        protected ISpeedController m_SpeedController;

        #if !LITE_VERSION

        //typed alias of the speed controller in FLIGHT mode (null otherwise): the flight direction block
        //feeds it steering curvature outside the ISpeedController contract
        readonly FlightSpeedController m_FlightController;

        #endif

        //this tick's scalar speed budget in units/tick: the pipeline's single conversion point for the
        //speed path (the config's per-tick accessors are the conversion points for everything else)
        float m_TickSpeed;

        #if !LITE_VERSION

        //FLIGHT banking state (the actuator reads the banked up vector via TryGetFlightUp)
        float   m_CurrentBankDegrees;
        Vector3 m_FlightUp = Vector3.up;

        //FLIGHT: the nose, integrated tick by tick. While the body moves the applied velocity is the nose; this
        //state carries the turn toward the carrot across ticks in which the applied velocity is zero — the static
        //planes can stop a body dead against a face, and a nose read from the applied velocity would then re-aim
        //from the same frozen vector for ever
        Vector3 m_FlightHeading;

        #endif

        #endregion

        #region Events

        public event Action<IMovable, Vector3> OnPositionChanged;

        #endregion

        #region Properties

        //the minimal host seam: the API layer downcasts it to Nav3DAgent where it needs the user-facing
        //agent, while the pipeline itself only ever reads Operates
        public IAgentHost Host { get; }

        internal bool HasSingleTargetSource => m_CurrFollowData?.HasSingleTargetSource ?? false;

        //only movers that are executing a task need to be avoided — except an externally controlled body, which may still be
        //moving without an order
        public bool NeedToBeAvoided => ExternalControl || Host.Operates;

        //the strike exclusion, per pair: the body a FollowTarget order was told not to avoid — a kamikaze, a
        //missile, an interceptor that is supposed to reach what it chases. Published here so the excluded body
        //reads it too and takes the whole manoeuvre.
        //An externally controlled body without an order solves nothing, so it answers false and its neighbours take the whole
        //manoeuvre — the 1/0 branch of the share law.
        public bool IsAvoiding(IMovable _Other) => m_AvoidanceStage != null                 &&
                                                   !ReferenceEquals(_Other, m_StrikeTarget) &&
                                                   (!ExternalControl || Host.Operates);

        public bool IsNeighborMoversDirty    => m_IsNeighborMovablesDirty;
        public bool IsNeighborObstaclesDirty => m_IsNeighborObstaclesDirty;

        public Vector3 CurrentPosition => m_CurrentPosition;

        //the point this tick's preferred velocity aims at: the carrot on the path, or the live aim of a terminal homing
        internal Vector3 SteeringPoint => m_PathNextPoint;

        protected Vector3 LastVelocity
        {
            get => m_LastVelocity;
            set
            {
                m_LastVelocity = value;

                if (value != Vector3.zero)
                    m_LastNonZeroVelocity = value;
            }
        }

        public Vector3[] CurrentPath => m_Path?.Trajectory;

        //the live path handle, for the path drawer's read of its radius and flags
        internal Nav3DPath Path => m_Path;

        // The avoidance stage's view of the certified corridor: the clearance radius the current follow
        //polyline was certified with, plus the body's drift from that polyline. The drift is measured against the
        //cursor's local segment — O(1), and an overestimate when the cursor lags, which errs toward re-arming the
        //constraints. False when the trajectory carries no certificate.
        public bool TryGetCertifiedPathDeviation(Vector3 _Position, out float _CertifiedRadius, out float _Deviation)
        {
            PathFollowData followData = m_CurrFollowData;

            if (followData is not { IsValid: true } || followData.CertifiedRadius <= 0f)
            {
                _CertifiedRadius = 0f;
                _Deviation       = 0f;

                return false;
            }

            _CertifiedRadius = followData.CertifiedRadius;

            followData.GetDistToClosestOnPath(_Position, out _, out _Deviation);

            return true;
        }

        //the path-following preferred velocity: carrot direction × this tick's speed-stage budget
        protected Vector3 VPref => (m_PathNextPoint - m_CurrentPosition).normalized * m_TickSpeed;

        //narrow config reads for the task layer (TargetFollowingLogic's intercept prediction and arrive law)
        public float CruiseSpeed => m_Config.Speed;
        public float BodyRadius  => m_Config.Radius;

        float VelocityDangerDistance => m_Config.VelocityRadius * 2f;

        #endregion

        #region Constructors

        public AgentMotionPipeline(Vector3 _Position, Nav3DAgentConfig _Config, IAgentHost _Host, Log _Log)
        {
            Host              = _Host;
            m_CurrentPosition = _Position;

            #if !LITE_VERSION

            m_ExternalControl = _Config.ControlMode == AgentControlMode.EXTERNAL;

            #endif

            m_Log = _Log;

            ApplyConfig(_Config);

            m_SpeedController = CreateSpeedController();

            #if !LITE_VERSION

            m_FlightController = m_SpeedController as FlightSpeedController;

            #endif

            if (m_Config.LocalAvoidanceEnabled)
                m_AvoidanceStage = CreateAvoidanceStage();
        }

        #endregion

        #region Public methods : orders

        public void BeginMoveTo(
                PathfindingRequest        _Request,
                Nav3DPath                 _Path,
                Action<PathfindingResult> _OnPathUpdated,
                Action<PathfindingResult> _OnPathfindingFail = null
            )
        {
            m_Log?.WriteFormat(LOG_BEGIN_MOVE_TO_POINT, UtilsCommon.GetPointsString(_Request.Points));

            m_SpeedController.OnNewOrder(CurrentSpeedUnitsPerSecond, _Request.Loop);

            m_Request           = _Request;
            m_Path              = _Path;
            m_OnPathUpdated     = _OnPathUpdated;
            m_OnPathfindingFail = _OnPathfindingFail;

            m_Path.OnPathfindingSuccess += BeginPathFollowing;

            //so that failed re-plans (auto-update, deviation) reach the caller's fail handler too
            if (m_OnPathfindingFail != null)
                m_Path.OnPathfindingFail += m_OnPathfindingFail;

            BeginPathFollowing(_Path.PathfindingResult);
        }

        public void BeginMoveToPoints(
                PathfindingRequest        _Request,
                Nav3DPath                 _Path,
                Action<PathfindingResult> _OnPathUpdated,
                Action<Vector3>           _OnTargetPointPassed = null,
                Action                    _OnLastPointReached  = null,
                Action<PathfindingResult> _OnPathfindingFail   = null
            )
        {
            m_Log?.WriteFormat(LOG_BEGIN_MOVE_TO_POINTS, UtilsCommon.GetPointsString(_Request.Points));

            m_SpeedController.OnNewOrder(CurrentSpeedUnitsPerSecond, _Request.Loop);

            m_Request             = _Request;
            m_Path                = _Path;
            m_OnTargetPassed      = _OnTargetPointPassed;
            m_OnLastTargetReached = _OnLastPointReached;
            m_OnPathUpdated       = _OnPathUpdated;
            m_OnPathfindingFail   = _OnPathfindingFail;

            m_Path.OnPathfindingSuccess += BeginPathFollowing;

            if (m_OnPathfindingFail != null)
                m_Path.OnPathfindingFail += m_OnPathfindingFail;

            BeginPathFollowing(_Path.PathfindingResult);
        }

        public void MoveToUsingExistingPath(
                PathfindingRequest        _Request,
                Action<PathfindingResult> _OnPathUpdated,
                Action<PathfindingResult> _OnPathfindingFail = null
            )
        {
            m_Log?.WriteFormat(LOG_BEGIN_MOVE_TO_WITH_EXISTING_PATH, UtilsCommon.GetPointsString(_Request.Points));

            m_Path?.Find(_Request, _OnPathUpdated, _OnPathfindingFail, true);
        }

        public void ReinitPathFollowingData()
        {
            m_Log?.Write(LOG_RESTART_FOLLOWING);

            m_CurrFollowData.RestartFollowing(CurrentPosition, m_Config.TargetReachDistance);
        }

        public void RestartFollowingForCurrentPath()
        {
            m_Log?.Write(LOG_REINIT_PATH);

            RestartPathFollowing(_SkipCurrentPositionAddition: true);
        }

        public void DisposePath()
        {
            if (m_Path == null)
                return;

            m_Path.Dispose();
            m_Path = null;
        }

        #endregion

        #region Public methods

        public Vector3 GetFrameVelocity()
        {
            return GetVelocity();
        }

        /// <summary>
        /// Aims this tick's preferred velocity straight at a live point instead of the follow carrot (the
        /// terminal homing of a FollowTarget order). Valid for exactly one tick, so path following resumes the
        /// moment the caller stops re-asserting it. While set, the deviation auto-replan and the follow-data
        /// validity guard are bypassed — homing needs no path — and the avoidance stage still post-processes
        /// the result.
        /// </summary>
        /// <param name="_AimPoint">The live point to steer at.</param>
        /// <param name="_SpeedCapUnitsPerSecond">Arrive-law speed ceiling; negative = uncapped.</param>
        public void SetDirectAimForTick(Vector3 _AimPoint, float _SpeedCapUnitsPerSecond)
        {
            m_DirectAimSet      = true;
            m_DirectAim         = _AimPoint;
            m_DirectAimSpeedCap = _SpeedCapUnitsPerSecond;
        }

        /// <summary>
        /// Names the one body this agent's pursuit is meant to reach and therefore does not avoid. The exclusion
        /// is published through <see cref="IsAvoiding"/>, so the named body takes the whole avoidance manoeuvre of
        /// the pair; static obstacles and every other neighbour are avoided as before. Null clears it.
        /// </summary>
        public void SetStrikeTarget(IMovable _Target)
        {
            m_StrikeTarget = _Target;
        }

        public void Uninitialize()
        {
            //Doomed covers the scene cleanup pass
            if (!AgentManager.Doomed)
                AgentManager.Instance.UnregisterAgentMover(this);

            DisposePath();
        }

        public void SetCurrentPosition(Vector3 _Position)
        {
            m_CurrentPosition = _Position;

            OnPositionChanged?.Invoke(this, m_CurrentPosition);
        }

        public bool IsDeviatedFromPathLastFrame()
        {
            return m_DeviatedFromPathLastFrame;
        }

        /// <summary>
        /// FLIGHT mode's banked up vector for the actuator's rotation; false in every other mode (the
        /// default velocity-facing rotation policy applies).
        /// </summary>
        public bool TryGetFlightUp(out Vector3 _Up)
        {

            #if !LITE_VERSION

            _Up = m_FlightUp;

            return m_FlightController != null;

            #else

            _Up = Vector3.up;

            return false;

            #endif

        }

        public string GetInnerStatusString()
        {
            StringBuilder stringBuilder = new StringBuilder();

            stringBuilder.AppendLine("-------MOVER STATUS-------");

            stringBuilder.AppendLine(
                    $"Mover type: {GetType().Name}, " +
                    $"avoidance: {(m_AvoidanceStage == null ? "disabled" : m_AvoidanceStage.GetType().Name)}"
                );

            stringBuilder.AppendLine("-------FOLLOW DATA STATUS-------");

            if (m_CurrFollowData == null)
            {
                stringBuilder.AppendLine("m_CurrFollowData: null");
            }
            else
            {
                stringBuilder.AppendLine($"m_CurrFollowData: {m_CurrFollowData.GetHashCode()}, IsValid: {m_CurrFollowData.IsValid}");
                stringBuilder.AppendLine(m_CurrFollowData.GetNextTargetInfo());
            }

            stringBuilder.AppendLine("-------PATHFINDING STATUS-------");

            string pathString = m_Path == null
                ? "m_Path: null"
                : $"m_Path:\n\t{m_Path.GetHashCode()},\n\tIsValid: {m_Path.IsValid},\n\tIsPathfindingInProgress: {m_Path.IsPathfindingInProgress}";

            stringBuilder.AppendLine(pathString);

            return stringBuilder.ToString();
        }

        public string GetLastPathfindingStatsString()
        {
            StringBuilder stringBuilder = new StringBuilder();

            if (m_Path == null)
            {
                stringBuilder.AppendLine($"{nameof(m_Path)}: null");

                return stringBuilder.ToString();
            }

            PathfindingResult pathfindingResult = m_Path.PathfindingResult;

            if (pathfindingResult == null)
            {
                stringBuilder.AppendLine($"{nameof(m_Path.PathfindingResult)}: null");

                return stringBuilder.ToString();
            }

            stringBuilder.Append(m_Path.PathfindingResult.GetLastPathfindingStatsString());

            return stringBuilder.ToString();
        }

        #endregion

        #region IMovable

        public Vector3 GetPosition()
        {
            return m_CurrentPosition;
        }

        public float GetRadius()
        {
            return m_Config.Radius;
        }

        public float GetAvoidanceWeight()
        {
            return m_Config.Radius * m_Config.AvoidancePriority;
        }

        public float GetMaxSpeed()
        {
            return m_Config.FactualMaxSpeedPerTick;
        }

        public float GetAvoidanceLookahead()
        {
            return m_Config.AvoidanceLookahead;
        }

        public float GetStaticObstaclesDangerDistance()
        {
            return m_Config.VelocityRadius;
        }

        public void SetNeighborMovablesDirty(bool _Dirty)
        {
            m_IsNeighborMovablesDirty = _Dirty;
        }

        public void SetNeighborObstaclesDirty(bool _Dirty)
        {
            m_IsNeighborObstaclesDirty = _Dirty;
        }

        #endregion

        #region Service methods

        void ApplyConfig(Nav3DAgentConfig _Config)
        {
            m_Config = _Config;

            if (!AgentManager.Doomed)
                AgentManager.Instance.RegisterAgentMover(this);
        }

        //Null when nothing registered a stage — which is every Lite build (the avoidance files are deleted there,
        //so no registration exists) and any build that strips local avoidance out. The null case is the "no
        //avoidance" behaviour, so the join point needs no edition fence of its own.
        protected virtual IAvoidanceStage CreateAvoidanceStage()
        {
            return AvoidanceProviders.Current?.CreateAgentStage(this, m_Config);
        }

        //the motion-mode dispatch point
        protected virtual ISpeedController CreateSpeedController()
        {

            #if !LITE_VERSION

            switch (m_Config.MotionMode)
            {
                case AgentMotionMode.EASED:
                    return new SectionedEaseSpeedController(m_Config);

                case AgentMotionMode.ACCELERATED:
                    return new AcceleratedSpeedController(m_Config);

                case AgentMotionMode.FLIGHT:
                    return new FlightSpeedController(m_Config);

                default:
                    return new ConstantSpeedController(m_Config);
            }

            #else

            return new ConstantSpeedController(m_Config);

            #endif

        }

        //the per-tick velocity computation: validity → deviation auto-replan → carrot+speed assembly →
        //optional avoidance stage
        protected Vector3 GetVelocity()
        {
            m_DeviatedFromPathLastFrame = false;

            //direct pursuit outranks path following for the one tick it was asserted for: no deviation
            //auto-replan (the body is EXPECTED off the stale path while homing) and no follow-data
            //requirement, which also erases the initial-pathfinding gap when a follow order starts in range
            if (m_DirectAimSet)
            {
                m_DirectAimSet = false;

                Vector3 vPrefDirect = ComputeDirectAimVelocity();

                if (m_AvoidanceStage == null)
                    return LastVelocity = vPrefDirect;

                Vector3 directResult = m_AvoidanceStage.Apply(vPrefDirect);

                SetBlendedVelocityDebug(directResult);

                return LastVelocity = directResult;
            }

            if (m_Path is not { IsValid: true } || m_CurrFollowData is not { IsValid: true })
                return LastVelocity = Vector3.zero;

            //re-plan once the agent has drifted far enough from the path
            if (m_Config.AutoUpdatePath && (AgentManager.Now - m_LastPathUpdateFireTime).TotalMilliseconds >
                Mathf.Max(m_Config.PathAutoUpdateCooldown, m_Config.PathfindingTimeout))
            {
                m_CurrFollowData.GetDistToClosestOnPath(m_CurrentPosition, out _, out float magnitude);

                #if !LITE_VERSION

                //FLIGHT deliberately swings out of corners up to FlightMaxDeviation — widen the deviation
                //threshold accordingly so intended swing-out doesn't fire re-plans
                float deviationThreshold = m_FlightController == null
                    ? VelocityDangerDistance
                    : VelocityDangerDistance + m_Config.FlightMaxDeviation;

                #else

                float deviationThreshold = VelocityDangerDistance;

                #endif

                if (magnitude > deviationThreshold)
                {
                    RestartPathFollowing();

                    m_LastPathUpdateFireTime    = AgentManager.Now;
                    m_DeviatedFromPath          = true;
                    m_DeviatedFromPathLastFrame = true;

                    return LastVelocity = Vector3.zero;
                }
            }

            //the path-following preferred velocity (carrot + speed stage; flight adds pursuit steering)
            Vector3 vPref = ComputeFrameVelocity();

            if (m_AvoidanceStage == null)
                return LastVelocity = vPref;

            Vector3 resultVelocity = m_AvoidanceStage.Apply(vPref);

            SetBlendedVelocityDebug(resultVelocity);

            return LastVelocity = resultVelocity;
        }

        partial void SetBlendedVelocityDebug(Vector3 _Velocity);

        //An obstacle appearing around an agent — a door leaf enabled, a moving body overrunning it — puts the
        //carrot inside itself, and path following then commands motion into the obstacle for as long as it stays
        //there. Throttled unconditionally to CARROT_CHECK_PERIOD_TICKS because the test costs one octree point
        //query: the resulting delay of at most two ticks is covered by the avoidance stage's escape state.
        void SkipBlockedCarrot()
        {
            if (m_CurrFollowData == null)
                return;

            if (--m_CarrotCheckCountdown > 0)
                return;

            m_CarrotCheckCountdown = CARROT_CHECK_PERIOD_TICKS;

            ObstacleManager manager = ObstacleManager.Instance;

            if (manager == null)
                return;

            m_CarrotCheckManager = manager;

            //bound once: the probe reads m_CarrotCheckManager instead of closing over a manager instance
            m_CarrotBlockedTest ??= CarrotBlocked;

            m_CarrotProbeRefused = false;

            if (m_CurrFollowData.TryAdvancePastBlockedPoints(m_CarrotBlockedTest, CARROT_SKIP_LOOKAHEAD))
                m_PathNextPoint = m_CurrFollowData.GetMovePoint(0f, m_CurrentPosition);

            //the scan was abandoned because the octree was busy publishing — retry next tick, so a refusal
            //costs latency and not a missed check
            if (m_CarrotProbeRefused)
                m_CarrotCheckCountdown = 1;
        }

        //A non-BLOCKING octree read: this runs on the main thread for every agent, and a blocking one would stall
        //the whole FixedUpdate behind a publish. A refusal must answer "blocked", never "free" — otherwise the
        //carrot would advance onto a point nothing verified; blocked simply leaves it where it was. The flag
        //latches so one refusal abandons the scan instead of asking the tree once per lookahead point.
        bool CarrotBlocked(Vector3 _Point)
        {
            if (m_CarrotProbeRefused)
                return true;

            if (m_CarrotCheckManager.TryIsPointInsideObstacle(_Point, out bool blocked))
                return blocked;

            m_CarrotProbeRefused = true;

            return true;
        }

        //Assembles this tick's direct-PURSUIT velocity, with the aim point playing the carrot. Outside FLIGHT the
        //speed controllers are deliberately not advanced: their path context is meaningless against a live point
        //(EASED, for one, would read the aim distance as both total and remaining and crawl at its ramp floor for
        //ever), so homing flies at cruise speed under the caller's arrive-law cap. FLIGHT keeps its physical model
        //— thrust inertia and arrival braking read the aim distance as the remaining path, which is what it is.
        Vector3 ComputeDirectAimVelocity()
        {
            Vector3 toAim    = m_DirectAim - m_CurrentPosition;
            float   distance = toAim.magnitude;

            m_PathNextPoint = m_DirectAim;

            #if !LITE_VERSION

            if (m_FlightController != null)
            {
                Vector3 flightVelocity = ComputeFlightPursuit(m_DirectAim, distance, distance);

                return m_DirectAimSpeedCap >= 0f
                    ? Vector3.ClampMagnitude(flightVelocity, m_DirectAimSpeedCap * Time.fixedDeltaTime)
                    : flightVelocity;
            }

            #endif

            m_TickSpeed = m_Config.Speed * Time.fixedDeltaTime;

            if (m_DirectAimSpeedCap >= 0f)
                m_TickSpeed = Mathf.Min(m_TickSpeed, m_DirectAimSpeedCap * Time.fixedDeltaTime);

            if (distance < 1e-4f)
                return Vector3.zero;

            //never overshoot the aim itself within one tick
            m_TickSpeed = Mathf.Min(m_TickSpeed, distance);

            return toAim / distance * m_TickSpeed;
        }

        //assembles this tick's path-following velocity: carrot + speed stage (+ pursuit steering in FLIGHT)
        Vector3 ComputeFrameVelocity()
        {

            #if !LITE_VERSION

            if (m_FlightController != null)
                return ComputeFlightFrameVelocity();

            #endif

            m_TickSpeed = AdvanceTickSpeed();

            m_PathNextPoint = m_CurrFollowData.GetMovePoint(m_TickSpeed, m_CurrentPosition);

            SkipBlockedCarrot();

            return (m_PathNextPoint - m_CurrentPosition).normalized * m_TickSpeed;
        }

        #if !LITE_VERSION

        //the FLIGHT direction block (the Spline3D pursuit model ported to re-plannable paths): the carrot is
        //advanced with the previous tick's speed budget (curvature must be known before the speed stage runs —
        //a one-tick lag with no practical effect), heading turns toward it bounded by the lateral-G budget,
        //and the same geometry drives the banking the actuator reads via TryGetFlightUp
        Vector3 ComputeFlightFrameVelocity()
        {
            //pure pursuit cannot hit a point tighter than its turn circle (radius v²/a_lat) — it orbits it
            //forever instead (the re-order-at-cruise trap: a sharp first-target turn at full speed).
            //Flight therefore passes path points within its current turning radius: the widened
            //carrot-advance budget keeps the carrot ahead of the turn circle, while arrival braking
            //shrinks the radius toward the path end so the final target still gets the precise
            //reach-distance hit.
            float prevSpeedUnitsPerSecond = m_TickSpeed                                       / Time.fixedDeltaTime;
            float turnRadius              = prevSpeedUnitsPerSecond * prevSpeedUnitsPerSecond / m_Config.FlightMaxLateralAcceleration;

            m_PathNextPoint = m_CurrFollowData.GetMovePoint(m_TickSpeed + turnRadius, m_CurrentPosition);

            return ComputeFlightPursuit(m_PathNextPoint, -1f, -1f);
        }

        //the flight steering block proper, shared by path following (carrot from the follow data) and
        //direct pursuit (carrot = the live aim point, remaining/total = the live aim distance)
        Vector3 ComputeFlightPursuit(Vector3 _Carrot, float _RemainingOverride, float _TotalOverride)
        {
            Vector3 toCarrot    = _Carrot - m_CurrentPosition;
            float   carrotRange = toCarrot.magnitude;

            Vector3 fallbackDir = m_LastNonZeroVelocity == Vector3.zero ? Vector3.forward : m_LastNonZeroVelocity.normalized;
            Vector3 pursuitDir  = carrotRange           > 1e-4f ? toCarrot / carrotRange : fallbackDir;

            //the nose: the applied motion while there is one, the integrated heading while the avoidance stage
            //holds the body still, the pursuit direction itself before the first tick of flight
            Vector3 heading = m_LastVelocity != Vector3.zero ? m_LastVelocity.normalized
                : m_FlightHeading            != Vector3.zero ? m_FlightHeading
                                                               : pursuitDir;

            //pure-pursuit arc curvature toward the carrot; the speed budget it caps into is delegated to
            //FlightSpeedProfile (thrust inertia + lateral-G turn slowdown), steering itself stays here
            float turnAngle = Mathf.Acos(Mathf.Clamp(Vector3.Dot(heading, pursuitDir), -1f, 1f));
            float curvature = 2f * Mathf.Sin(turnAngle) / Mathf.Max(carrotRange, 0.01f);

            m_FlightController.SetTickCurvature(curvature);

            m_TickSpeed = AdvanceTickSpeed(_RemainingOverride, _TotalOverride);

            //turn rate is bounded by the same lateral acceleration budget: fast flight means wide turns
            float speedUnitsPerSecond = m_TickSpeed                                                                  / Time.fixedDeltaTime;
            float maxTurnRadians      = m_Config.FlightMaxLateralAcceleration / Mathf.Max(speedUnitsPerSecond, 0.5f) * Time.fixedDeltaTime;

            Vector3 newHeading = Vector3.RotateTowards(heading, pursuitDir, maxTurnRadians, 0f).normalized;

            m_FlightHeading = newHeading;

            UpdateFlightBank(heading, newHeading, speedUnitsPerSecond);

            return newHeading * m_TickSpeed;
        }

        //signed-yaw-rate banking (the Spline3D flight model's roll), smoothed by FlightBankResponsiveness
        void UpdateFlightBank(Vector3 _OldHeading, Vector3 _NewHeading, float _SpeedUnitsPerSecond)
        {
            float deltaTime  = Time.fixedDeltaTime;
            float bankTarget = 0f;

            if (deltaTime > 0f)
            {
                //signed yaw rate approximation: positive = right turn
                float yawRate      = Vector3.Dot(Vector3.Cross(_OldHeading, _NewHeading), Vector3.up) / deltaTime;
                float lateralAccel = yawRate                                                          * _SpeedUnitsPerSecond;

                bankTarget = Mathf.Clamp(
                        Mathf.Atan2(lateralAccel, 9.81f) * Mathf.Rad2Deg,
                        -m_Config.FlightBankAngleMax,
                        m_Config.FlightBankAngleMax
                    );
            }

            m_CurrentBankDegrees = Mathf.Lerp(m_CurrentBankDegrees, bankTarget, 1f - Mathf.Exp(-m_Config.FlightBankResponsiveness * deltaTime));

            m_FlightUp = Quaternion.AngleAxis(-m_CurrentBankDegrees, _NewHeading) * Vector3.up;
        }

        #endif

        //runs the speed stage for this tick and converts its units/second output to the tick budget;
        //non-negative overrides substitute the follow data's path context (direct pursuit runs without
        //valid follow data, so the live aim distance stands in for the remaining path)
        float AdvanceTickSpeed(float _RemainingOverride = -1f, float _TotalOverride = -1f)
        {
            float remainingDistance = 0f;
            float totalPathLength   = 0f;

            if (m_SpeedController.NeedsPathContext)
            {
                if (_RemainingOverride >= 0f)
                {
                    remainingDistance = _RemainingOverride;
                    totalPathLength   = Mathf.Max(_TotalOverride, _RemainingOverride);
                }
                else
                {
                    totalPathLength   = m_CurrFollowData.TotalLength;
                    remainingDistance = m_CurrFollowData.GetRemainingDistance(m_CurrentPosition);
                }
            }

            return m_SpeedController.Advance(Time.fixedDeltaTime, remainingDistance, totalPathLength) * Time.fixedDeltaTime;
        }

        #endregion

        #region Service methods : path direction

        void BeginPathFollowing(PathfindingResult _Result)
        {
            m_Log?.Write(LOG_PATH_UPDATE_SUCCESS);

            if (m_Path is { IsValid: true })
            {
                m_OnPathUpdated.Invoke(_Result);

                if (m_Path.TryGetFollowData(
                            OnTargetPointPassed,
                            m_OnLastTargetReached,
                            m_CurrentPosition,
                            m_Config.TargetReachDistance,
                            out PathFollowData followData
                        ))
                    m_CurrFollowData = followData;
            }
        }

        protected void RestartPathFollowing(Vector3? _StartPoint = null, bool _SkipCurrentPositionAddition = false)
        {
            //Keep only points present in the initial target list, each at most its initial multiplicity. The
            //follow data's targets come from the previous silent update, which also carried that moment's
            //position as a transient start, so without this filter transient points accumulate into the loop.
            //Source identity disambiguates equal authored coordinates after the tour rotates.
            List<Vector3> sourceTargets   = new List<Vector3>(m_Request.GetInitialPoints());
            List<int>     sourceIndices   = Enumerable.Range(0, sourceTargets.Count).ToList();
            Vector3[]     followTargets   = m_CurrFollowData.GetUnpassedTargets(m_Request.Loop);
            int[]         followSources   = m_CurrFollowData.GetUnpassedTargetSources(m_Request.Loop);
            List<Vector3> unpassedTargets = new List<Vector3>(followTargets.Length);
            List<int>     unpassedSources = new List<int>(followTargets.Length);

            for (int i = 0; i < followTargets.Length; i++)
            {
                int index = sourceTargets.IndexOf(followTargets[i]);

                if (followSources != null)
                {
                    int source = followSources[i];
                    index = source < 0 ? -1 : sourceIndices.FindIndex(_Index => m_Request.GetPointSource(_Index) == source);

                    if (index >= 0 && sourceTargets[index] != followTargets[i])
                        index = -1;
                }

                if (index > -1)
                {
                    unpassedTargets.Add(followTargets[i]);
                    unpassedSources.Add(m_Request.GetPointSource(sourceIndices[index]));
                    sourceTargets.RemoveAt(index);
                    sourceIndices.RemoveAt(index);
                }
            }

            if (_StartPoint.HasValue)
            {
                unpassedTargets.Insert(0, _StartPoint.Value);
                unpassedSources.Insert(0, -1);
            }

            bool currentPositionInserted = unpassedTargets.Count == 1 || !_SkipCurrentPositionAddition;

            if (currentPositionInserted)
            {
                unpassedTargets.Insert(0, m_CurrentPosition);
                unpassedSources.Insert(0, -1);
            }

            //A path needs two POINTS, and the composition above can produce fewer once no unpassed target is
            //left. That is a no-op rather than an error: the next tick, or the next re-aim for a moving target,
            //composes a real request.
            if (unpassedTargets.Count < 2)
                return;

            //the inserted current position is a transient tour point, not an authored destination: marking it
            //lets the volumetric lane escape-hop around it instead of refusing the whole request when the body
            //stands within its own radius of voxelized geometry. The request
            //carries no order-time extension: the tour is exactly the list composed above
            m_Path.Find(
                    m_Request.WithReplanTour(unpassedTargets.ToArray(), currentPositionInserted ? m_CurrentPosition : null, unpassedSources.ToArray()),
                    _UpdatePathSilently: true
                );
        }

        //The input point the target now being reported stands for; -1 for an inserted point or when unknown.
        internal int PassedTargetSource => m_CurrFollowData?.PassedTargetSource ?? -1;

        protected virtual void OnTargetPointPassed(Vector3 _Point)
        {
            m_OnTargetPassed?.Invoke(_Point);

            //re-anchor following after a deviation re-plan
            if (m_DeviatedFromPath)
            {
                m_DeviatedFromPath = false;
                RestartPathFollowing();
            }
        }

        #endregion

    }
}
