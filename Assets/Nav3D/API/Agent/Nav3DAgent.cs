using Nav3D.Common;
using Nav3D.LocalAvoidance;
using System;
using System.Text;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Nav3D.Agents;
using Nav3D.Common.AgentCore;
using Log = Nav3D.Common.Log;

namespace Nav3D.API
{
    public partial class Nav3DAgent : Nav3DAgentBase, IAgentHost
    {

        #region Constants

        readonly string LOG_CONFIG_SET           = $"{nameof(Nav3DAgent)}.{nameof(SetConfig)}";
        readonly string LOG_MOVE_TO              = $"{nameof(Nav3DAgent)}.{nameof(MoveTo)}: {{0}}";
        readonly string LOG_MOVE_TO_POINTS       = $"{nameof(Nav3DAgent)}.{nameof(MoveToPoints)}: {{0}}";
        readonly string LOG_MOVE_TO_POINTS_TRIMM = $"{nameof(Nav3DAgent)}.{nameof(MoveToPoints)}: Source points sequence was trimmed: {{0}} -> {{1}}";

        readonly string MOVE_TO_POINTS_COUNT_ERROR = $"{nameof(Nav3DAgent)}.{nameof(MoveToPoints)}: Array must contain at least one point.";

        readonly string LOG_FOLLOW_TARGET =
            $"{nameof(Nav3DAgent)}.{nameof(FollowTarget)}: Transfrom: {{0}}:{{1}}, ToleranceUpdate: {{2}}, Reach dist: {{3}}";

        readonly string LOG_STOP = $"{nameof(Nav3DAgent)}.{nameof(StopInternal)}";

        const string LOG_DISABLED_ERROR = "Logging is disabled for agent.";

        readonly string ON_INIT_UNSUBSCRIBE_ERROR = $"There is no need to unsubscribe from the {nameof(OnAgentInit)} event. " +
                                                    $"All subscriptions will be unsubscribed after {nameof(Nav3DAgent)} is initialized.";

        #endregion

        #region Serialized fields

        [SerializeField] Nav3DAgentConfig m_Config;

        #endregion

        #region Attributes

        readonly List<Action> m_OnInitSubscribers         = new List<Action>();
        readonly List<Action> m_OnInitInternalSubscribers = new List<Action>();

        event Action m_OnAgentInit;
        event Action m_OnAgentInitInternal;

        //ReSharper disable once MemberCanBePrivate.Global
        internal Nav3DAgentConfig    m_ConfigInternal;
        internal AgentMotionPipeline m_Mover;

        MovementLogic m_MovementLogic;

        AgentOrder m_ProcessingOrder;

        Log m_Log;

        Vector3 m_RotationVector;
        bool    m_InitializationPending;

        #endregion

        #region Events

        event Action OnAgentInitInternal
        {
            add
            {
                if (value == null)
                    return;

                if (Inited)
                {
                    value.Invoke();

                    return;
                }

                m_OnInitInternalSubscribers.Add(value);

                m_OnAgentInitInternal += value;
            }
            remove
            {
                m_OnInitInternalSubscribers.Remove(value);

                m_OnAgentInitInternal -= value;
            }
        }

        public event Action OnAgentInit
        {
            add
            {
                if (value == null)
                    return;

                if (Inited)
                {
                    ThreadDispatcher.BeginInvoke(value);

                    return;
                }

                Action subscriber = () => ThreadDispatcher.BeginInvoke(value);

                m_OnInitSubscribers.Add(subscriber);

                m_OnAgentInit += subscriber;
            }
            remove { Debug.LogError(ON_INIT_UNSUBSCRIBE_ERROR); }
        }

        public event Action<PathfindingResult> OnPathUpdated;

        #endregion

        #region Properties

        public bool Inited { get; private set; }

        internal override IMovable Movable => m_Mover;

        //Whether agent executing any task.
        public bool Operates => m_MovementLogic is { IsValid: true };

        /// <summary>
        /// Cached agent's transform
        /// </summary>
        public Transform Transform => m_CachedTransform;

        /// <summary>
        /// The agent's radius taken from its config
        /// </summary>
        public float Radius => m_ConfigInternal?.Radius ?? 0;

        /// <summary>
        /// Current path points.
        /// </summary>
        public Vector3[] CurrentPath => m_MovementLogic?.CurrentPath ?? new Vector3[] { };

        //the path handle behind CurrentPath: its radius and result flags, for Nav3DAgentPathDrawer
        internal Nav3DPath CurrentNav3DPath => m_MovementLogic?.Path;

        /// <summary>
        /// Agent config instance.
        /// </summary>
        public Nav3DAgentConfig Config => m_Config ?? m_ConfigInternal;

        //the fixed time of the agent's last step: what the External Control probe checks every step of a tick is finished before
        //the tick's first notification against
        internal float LastStepTime { get; private set; } = -1f;

        #endregion

        #region Public methods

        /// <summary>
        /// Replaces the agent's config. Ends the current order, including one whose path is still being searched,
        /// without invoking its callbacks; issue a new order after the call.
        /// </summary>
        /// <param name="_Config">The config to use from now on.</param>
        public void SetConfig(Nav3DAgentConfig _Config)
        {
            Nav3DManager.CheckInitedSoft();

            OnAgentInitInternal += () =>
            {
                //an order is bound to the pipeline that ApplyConfig is about to discard
                EndOrder();

                ApplyConfig(_Config);
            };
        }

        /// <summary>
        /// Starts following to the given point.
        /// </summary>
        /// <param name="_Point">The given point.</param>
        /// <param name="_ReachDistance">Distance to reach the point.</param>
        /// <param name="_OnReach">The action to invoke when point reached.</param>
        /// <param name="_OnPathReady">The callback notifying that path is successfully found.</param>
        /// <param name="_OnPathfindingFail">The action to do if pathfinding failed.</param>
        /// <param name="_InterruptCurrentOrderImmediate">Whether current order should be stopped immediate</param>
        public void MoveTo(
                Vector3                   _Point,
                Action                    _OnReach                        = null,
                Action                    _OnPathReady                    = null,
                Action<PathfindingResult> _OnPathfindingFail              = null,
                float?                    _ReachDistance                  = null,
                bool                      _InterruptCurrentOrderImmediate = false
            )
        {
            Nav3DManager.CheckInitedSoft();

            OnAgentInitInternal += () =>
            {
                m_Log?.WriteFormat(LOG_MOVE_TO, _Point.ToStringExt());

                m_ProcessingOrder?.Cancel();

                PathfindingRequest request = new PathfindingRequest(
                        new[] { BodyPosition, _Point },
                        false,
                        false,
                        m_ConfigInternal.TryRepositionStartIfOccupied,
                        m_ConfigInternal.TryRepositionTargetIfOccupied,
                        m_ConfigInternal.PathfindingTimeout,
                        m_ConfigInternal.SmoothPath,
                        m_ConfigInternal.SmoothRatio,
                        $"{name}({this.UnityIdText()})",
                        m_ConfigInternal.PathfindingRadius,
                        m_ConfigInternal.VolumetricMode,
                        m_ConfigInternal.PathfindingCertificationCap,
                        m_ConfigInternal.TimeoutMode,
                        m_ConfigInternal.Optimality
                    );

                m_ProcessingOrder = new AgentOrder(
                        request,
                        new PointsFollowingLogic(
                                m_CachedTransform,
                                m_Mover,
                                request,
                                //a distance, not its square: the logic squares it itself
                                _ReachDistance ?? m_ConfigInternal.TargetReachDistance,
                                null,
                                _OnReach,
                                InvokeOnPathUpdated,
                                _OnPathfindingFail
                            )
                    );

                m_ProcessingOrder.Init(
                        _Path =>
                        {
                            OnAgentOrderBecomeReady(_Path);
                            _OnPathReady?.Invoke();
                        },
                        _OnPathfindingFail,
                        m_Log
                    );

                if (_InterruptCurrentOrderImmediate)
                    DisposeCurrentMovementLogic();
            };
        }

        /// <summary>
        /// Start movement to the given points.
        /// </summary>
        /// <param name="_Points">The given points.</param>
        /// <param name="_Loop">Loop the movement when reach last point.</param>
        /// <param name="_StartFromClosest">Whether to start movement from the closest point to the given one.</param>
        /// <param name="_SkipUnpassableTargets">Exclude point if pathfinding failed, or invoke _OnPathfindingFail callback.</param>
        /// <param name="_ReachDistance">Distance to reach the point.</param>
        /// <param name="_OnTargetPassed">The action to do when the one of target points has passed.</param>
        /// <param name="_OnFinished">The action to do when the last one point reached.</param>
        /// <param name="_OnPathReady">The callback notifying that path is successfully found.</param>
        /// <param name="_OnPathfindingFail">The action to do if pathfinding failed.</param>
        /// <param name="_InterruptCurrentOrderImmediate">Whether current order should be stopped immediate</param>
        /// <param name="_OnTargetIndexPassed">
        /// The action to do when one of the given points has been passed, with that point's index in _Points. Reported once
        /// per pass; a looped route reports each index again on the next lap. A skipped point is never reported.
        /// </param>
        public void MoveToPoints(
                Vector3[]                 _Points,
                bool                      _Loop                           = false,
                bool                      _StartFromClosest               = false,
                bool                      _SkipUnpassableTargets          = false,
                float?                    _ReachDistance                  = null,
                Action<Vector3>           _OnTargetPassed                 = null,
                Action                    _OnFinished                     = null,
                Action                    _OnPathReady                    = null,
                Action<PathfindingResult> _OnPathfindingFail              = null,
                bool                      _InterruptCurrentOrderImmediate = false,
                Action<int>               _OnTargetIndexPassed            = null
            )
        {
            Nav3DManager.CheckInitedSoft();

            OnAgentInitInternal += () =>
            {
                m_Log?.WriteFormat(LOG_MOVE_TO_POINTS + $"\n{Environment.StackTrace}", UtilsCommon.GetPointsString(_Points));

                m_ProcessingOrder?.Cancel();

                if (!_Points.Any())
                    throw new Exception(MOVE_TO_POINTS_COUNT_ERROR);

                //the request's points keep the caller's indices, so a pass can be reported against _Points
                int[]     sources       = UtilsCommon.TrimEqualPointIndices(_Points);
                Vector3[] trimmedPoints = sources.Select(_Index => _Points[_Index]).ToArray();

                if (trimmedPoints.Length < _Points.Length)
                    m_Log?.WriteFormat(LOG_MOVE_TO_POINTS_TRIMM, UtilsCommon.GetPointsString(_Points), UtilsCommon.GetPointsString(trimmedPoints));

                Vector3 bodyPosition = BodyPosition;

                if (_StartFromClosest)
                {
                    int[] order = UtilsCommon.OrderStartingFromClosest(trimmedPoints, bodyPosition, _Loop);

                    trimmedPoints = order.Select(_Index => trimmedPoints[_Index]).ToArray();
                    sources       = order.Select(_Index => sources[_Index]).ToArray();
                }

                PathfindingRequest request = new AgentPathfindingRequest(
                        trimmedPoints,
                        bodyPosition,
                        _Loop,
                        _SkipUnpassableTargets,
                        m_ConfigInternal.TryRepositionStartIfOccupied,
                        m_ConfigInternal.TryRepositionTargetIfOccupied,
                        m_ConfigInternal.PathfindingTimeout,
                        m_ConfigInternal.SmoothPath,
                        m_ConfigInternal.SmoothRatio,
                        $"{name}({this.UnityIdText()})",
                        m_ConfigInternal.PathfindingRadius,
                        m_ConfigInternal.VolumetricMode,
                        m_ConfigInternal.PathfindingCertificationCap,
                        m_ConfigInternal.TimeoutMode,
                        m_ConfigInternal.Optimality
                    );

                request.PointSources = sources;

                m_ProcessingOrder = new AgentOrder(
                        request,
                        new PointsFollowingLogic(
                                m_CachedTransform,
                                m_Mover,
                                request,
                                //a distance, not its square: the logic squares it itself
                                _ReachDistance ?? m_ConfigInternal.TargetReachDistance,
                                _OnTargetPassed,
                                _OnFinished,
                                InvokeOnPathUpdated,
                                _OnPathfindingFail,
                                _OnPointIndexPassed: _OnTargetIndexPassed
                            )
                    );

                m_ProcessingOrder.Init(
                        _Path =>
                        {
                            OnAgentOrderBecomeReady(_Path);
                            _OnPathReady?.Invoke();
                        },
                        _OnPathfindingFail,
                        m_Log
                    );

                if (_InterruptCurrentOrderImmediate)
                    DisposeCurrentMovementLogic();
            };
        }

        /// <summary>
        /// Begins to follow a moving transform. Paths are planned toward a predicted intercept point
        /// (the target's velocity is estimated continuously), and near the target — with a clear line
        /// of sight — the agent homes on the live target directly, without re-planning.
        /// </summary>
        /// <param name="_Target">Target transform to follow</param>
        /// <param name="_FollowContinuously">
        /// Follows continuously with no reach callback invocation; the agent holds the reach distance, braking
        /// smoothly as it closes and speeding up again as the target pulls away.
        /// </param>
        /// <param name="_TargetOffsetUpdate">How far the predicted pursuit goal must drift from the planned one to trigger a path update.</param>
        /// <param name="_TargetReachDistance">The distance to the target when the agent should stop following (continuous follow: the hold distance).</param>
        /// <param name="_OnReach">On target reach callback.</param>
        /// <param name="_OnFail">Executes if the target transform reference is null or gameObject has disabled activeInHierarchy flag.</param>
        /// <param name="_OnPathReady">The callback notifying that path is successfully found.</param>
        /// <param name="_OnPathfindingFail">On pathfinding failed action (can be executed multiple times).</param>
        /// <param name="_InterruptCurrentOrderImmediate">Whether current order should be stopped immediately</param>
        /// <param name="_AvoidTarget">Whether the agent avoids the target's body. Pass false to let the agent reach it.</param>
        public void FollowTarget(
                Transform                 _Target,
                bool                      _FollowContinuously             = false,
                float                     _TargetOffsetUpdate             = 0.1f,
                float                     _TargetReachDistance            = 0,
                Action                    _OnReach                        = null,
                Action                    _OnFail                         = null,
                Action                    _OnPathReady                    = null,
                Action<PathfindingResult> _OnPathfindingFail              = null,
                bool                      _InterruptCurrentOrderImmediate = false,
                bool                      _AvoidTarget                    = true
            )
        {
            Nav3DManager.CheckInitedSoft();

            OnAgentInitInternal += () =>
            {
                if (m_ConfigInternal.UseLog)
                    m_Log?.WriteFormat(LOG_FOLLOW_TARGET, _Target.name, _Target.UnityIdText(), _TargetOffsetUpdate, _TargetReachDistance);

                m_ProcessingOrder?.Cancel();

                PathfindingRequest request = new PathfindingRequest(
                        new[] { BodyPosition, _Target.position },
                        false,
                        false,
                        m_ConfigInternal.TryRepositionStartIfOccupied,
                        m_ConfigInternal.TryRepositionTargetIfOccupied,
                        m_ConfigInternal.PathfindingTimeout,
                        m_ConfigInternal.SmoothPath,
                        m_ConfigInternal.SmoothRatio,
                        _RequesterID: $"{name}({this.UnityIdText()})",
                        _Radius: m_ConfigInternal.PathfindingRadius,
                        _VolumetricMode: m_ConfigInternal.VolumetricMode,
                        _MaxCertifiedRadius: m_ConfigInternal.PathfindingCertificationCap,
                        _TimeoutMode: m_ConfigInternal.TimeoutMode,
                        _Optimality: m_ConfigInternal.Optimality
                    );

                m_ProcessingOrder = new AgentOrder(
                        request,
                        new TargetFollowingLogic(
                                m_CachedTransform,
                                m_Mover,
                                request,
                                _Target,
                                _TargetOffsetUpdate,
                                _TargetReachDistance,
                                _FollowContinuously,
                                _AvoidTarget,
                                _OnReach,
                                InvokeOnPathUpdated,
                                _OnFail,
                                _OnPathfindingFail
                            )
                    );

                m_ProcessingOrder.Init(
                        _Path =>
                        {
                            OnAgentOrderBecomeReady(_Path);
                            _OnPathReady?.Invoke();
                        },
                        _OnPathfindingFail,
                        m_Log
                    );

                if (_InterruptCurrentOrderImmediate)
                    DisposeCurrentMovementLogic();
            };
        }

        /// <summary>
        /// Stops current order execution.
        /// </summary>
        public void Stop()
        {
            Nav3DManager.CheckInitedSoft();

            OnAgentInitInternal += StopInternal;
        }

        /// <summary>
        /// Returns the list of agents distance to is less than radius.
        /// </summary>
        /// <param name="_Radius">Radius.</param>
        /// <param name="_Predicate">Predicate for agents filtering.</param>
        public Nav3DAgent[] GetAgentsInRadius(float _Radius, Predicate<Nav3DAgent> _Predicate = null)
        {
            Nav3DManager.CheckInitedHard();

            Nav3DAgent[] agentsInSphere = Nav3DAgentManager.GetAgentsInSphere(BodyPosition, _Radius);

            return _Predicate != null ? agentsInSphere.Where(_Agent => _Predicate(_Agent)).ToArray() : agentsInSphere;
        }

        public string GetLogText()
        {
            return m_ConfigInternal.UseLog ? m_Log.GetText(out _) : LOG_DISABLED_ERROR;
        }

        public string GetInnerStatusString()
        {
            StringBuilder stringBuilder = new StringBuilder();

            stringBuilder.AppendLine($"{nameof(Nav3DAgent)} status:");
            stringBuilder.AppendLine($"{nameof(m_MovementLogic)}: {m_MovementLogic?.GetType()}");
            stringBuilder.AppendLine($"{nameof(m_MovementLogic)}: {nameof(m_MovementLogic.IsValid)}: {m_MovementLogic?.IsValid}");
            stringBuilder.Append($"{m_Mover.GetInnerStatusString()}");

            return stringBuilder.ToString();
        }

        public string GetLastPathfindingStatsString()
        {
            StringBuilder stringBuilder = new StringBuilder();

            stringBuilder.AppendLine("Last pathfinding stats:");
            stringBuilder.Append($"{m_Mover.GetLastPathfindingStatsString()}");

            return stringBuilder.ToString();
        }

        internal override void DoFixedUpdate()
        {
            if (!Inited)
                return;

            LastStepTime = Time.fixedTime;

            DoFrameLogic();
        }

        #endregion

        #region Service methods

        //the editor-only Draw path flag, implemented in Nav3DAgent.Editor.cs: while it is ticked the agent's route is drawn by Nav3DAgentPathDrawer
        partial void SyncPathDrawing(bool _Enabled);

        partial void ApplyStateSource();
        partial void ResetStateSample();
        partial void ResetOutputs();
        partial void PublishOutputs(Vector3 _FrameVelocity);

        void OnAgentOrderBecomeReady(Nav3DPath _Path)
        {
            //the order may have been canceled while this callback sat in the ThreadDispatcher queue
            if (m_ProcessingOrder == null || !_Path.IsValid || _Path.PathfindingResult == null)
                return;

            DisposeCurrentMovementLogic();

            m_MovementLogic = m_ProcessingOrder.MovementLogic;
            m_MovementLogic.Init(_Path);

            m_ProcessingOrder = null;
        }

        void DoAwake()
        {
            m_CachedTransform = transform;
        }

        void Initialize()
        {
            if (Inited || m_InitializationPending)
                return;

            m_InitializationPending = true;
            AgentManager.Instance.RegisterAgent(this);
            Nav3DManager.OnNav3DInitInternal += CompleteInitialization;
        }

        void CompleteInitialization()
        {
            if (this == null || !m_InitializationPending || !isActiveAndEnabled)
                return;

            m_InitializationPending = false;
            Nav3DAgentConfig config = m_Config?.Copy() ?? Nav3DAgentConfig.DefaultConfig;

            OnAgentInitInternal += () => ApplyConfig(config);
            MarkAsInited();
        }

        void InitLog()
        {
            m_Log = m_ConfigInternal.UseLog ? new Log($"{gameObject.name}: {gameObject.UnityIdText()}", m_ConfigInternal.LogSize) : null;
        }

        internal virtual void InitMover()
        {
            m_Mover?.Uninitialize();

            //one pipeline for every agent: local avoidance is a stage inside it, not a class choice
            m_Mover = new AgentMotionPipeline(m_CachedTransform.position, m_ConfigInternal, this, m_Log);
        }

        void Uninitialize()
        {
            Nav3DManager.OnNav3DInitInternal -= CompleteInitialization;
            m_InitializationPending          =  false;
            Inited                           =  false;

            ResetStateSample();

            if (!AgentManager.Doomed)
                AgentManager.Instance.UnregisterAgent(this);

            //the order is bound to the pipeline this disables; an enabled-again agent builds a new one
            EndOrder();

            m_Mover?.Uninitialize();
            m_Log?.Clear();
        }

        void MarkAsInited()
        {
            Inited = true;

            m_OnAgentInitInternal?.Invoke();
            m_OnAgentInit?.Invoke();

            UnsubscribeOnInitInternalSubscribers();
            UnsubscribeOnInitSubscribers();
        }

        void UnsubscribeOnInitInternalSubscribers()
        {
            foreach (Action subscriber in m_OnInitInternalSubscribers)
            {
                m_OnAgentInitInternal -= subscriber;
            }

            m_OnInitInternalSubscribers.Clear();
        }

        void UnsubscribeOnInitSubscribers()
        {
            foreach (Action subscriber in m_OnInitSubscribers)
            {
                m_OnAgentInit -= subscriber;
            }

            m_OnInitSubscribers.Clear();
        }

        void ApplyConfig(Nav3DAgentConfig _Config)
        {
            m_ConfigInternal = m_Config = _Config;

            //the config may carry invalid values when the project upgrade step was never run
            m_ConfigInternal.FixInvalidParams();

            Nav3DPath.ReportRadiusAboveWorldBound(m_ConfigInternal.PathfindingRadius, gameObject);

            InitLog();

            InitMover();

            ApplyStateSource();

            m_Log?.Write(LOG_CONFIG_SET);
        }

        void EndOrder()
        {
            m_ProcessingOrder?.Cancel();
            m_ProcessingOrder = null;

            DisposeCurrentMovementLogic();

            ResetOutputs();
        }

        void StopInternal()
        {
            m_ProcessingOrder?.Cancel();
            m_MovementLogic?.Dispose();

            ResetOutputs();

            if (m_ConfigInternal.UseLog)
                m_Log?.Write(LOG_STOP);
        }

        void DoFrameLogic()
        {
            MovementLogic logic = m_MovementLogic;

            if (!(logic is { IsValid: true }))
            {
                ResetOutputs();

                return;
            }

            logic.DoOnFrameStart();

            Vector3 frameVelocity = logic.GetFrameVelocity();

            //a target-passed callback inside the step may have replaced the order or the config (SetConfig): this step then belongs
            //to a discarded pipeline and must neither be published nor move the body
            if (!ReferenceEquals(m_MovementLogic, logic))
            {
                ResetOutputs();

                return;
            }

            PublishOutputs(frameVelocity);

            //under External Control the body belongs to the game's controller, which acts on DesiredVelocity
            if (!ExternalControl)
            {
                m_CachedTransform.position += frameVelocity;

                m_RotationVector = Vector3.Lerp(m_RotationVector, frameVelocity, m_ConfigInternal.RotationVectorLerpFactor);

                if (m_RotationVector != Vector3.zero)
                {
                    //FLIGHT banks around the heading (pipeline-computed up vector); other modes keep the
                    //default world-up look rotation (LookRotation(v) ≡ LookRotation(v, Vector3.up))
                    Quaternion targetRotation = m_Mover.TryGetFlightUp(out Vector3 flightUp)
                        ? Quaternion.LookRotation(m_RotationVector, flightUp)
                        : Quaternion.LookRotation(m_RotationVector);

                    m_CachedTransform.rotation = Quaternion.RotateTowards(
                            m_CachedTransform.rotation,
                            targetRotation,
                            m_ConfigInternal.MaxAgentDegreesRotationPerSecond * Time.fixedDeltaTime
                        );
                }
            }

            logic.DoOnFrameEnd();

            //an arrival callback inside DoOnFrameEnd may have ended the order, issued another or replaced the config
            if (!(m_MovementLogic is { IsValid: true }))
                ResetOutputs();
        }

        void DisposeCurrentMovementLogic()
        {
            m_MovementLogic?.Dispose();
            m_MovementLogic = null;
        }

        void InvokeOnPathUpdated(PathfindingResult _Result)
        {
            OnPathUpdated?.Invoke(_Result);
        }

        #endregion

        #region Unity methods

        void Awake()
        {
            DoAwake();
        }

        void OnEnable()
        {
            Initialize();

            SyncPathDrawing(true);
        }

        void OnDestroy()
        {
            Uninitialize();
        }

        void OnDisable()
        {
            SyncPathDrawing(false);

            Uninitialize();
        }

        #endregion

    }
}
