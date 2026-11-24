using System;
using System.Linq;
using Nav3D.Common;
using UnityEngine;
using Nav3D.Obstacles;
using Nav3D.Pathfinding;
using System.Threading.Tasks;

namespace Nav3D.API
{
    public partial class Nav3DPath : IDisposable
    {
        #region Constants

        const string TARGETS_LIST_IS_EMPTY_ERROR = "Error. Targets array is empty!";

        const string TARGET_UNREACHABLE_ERROR = "{0}: target {1} is inside or too close to an obstacle. " +
                                                "Move the target or enable TryRepositionTargetIfOccupied.";

        const string RADIUS_ABOVE_WORLD_BOUND_ERROR = "{0}: radius {1:0.##} is above Max agent radius {2:0.##} of the world settings, so " +
                                                      "its paths keep only {2:0.##} clear of obstacles. Raise Max agent radius or reduce the radius.";

        #endregion

        #region Attributes

        Path m_Path;

        PathfindingRequest m_Request;

        bool m_RadiusReported;

        readonly Log m_Log;

        Action<PathfindingResult> m_OnSuccess;
        Action<PathfindingResult> m_OnFail;

        #endregion

        #region Properties : internal

        //Off for an agent's order: the agent layer reports for it — it knows the GameObject, and whether the user passed a
        //fail handler (its movement logic always subscribes here)
        internal bool ReportsToConsole { get; set; } = true;

        #endregion

        #region Events

        public event Action<PathfindingResult> OnPathfindingSuccess;
        public event Action<PathfindingResult> OnPathfindingFail;
        public event Action<PathfindingResult> OnPathfindingComplete;

        #endregion

        #region Properties

        /// <summary>
        /// Whether the pathfinding process was completed successfully using the current pathfinding parameters.
        /// </summary>
        public bool IsValid { get; private set; }

        /// <summary>
        /// Whether the last pathfinding request still performing.
        /// </summary>
        public bool IsPathfindingInProgress { get; private set; }

        /// <summary>
        /// The last pathfinding resulting trajectory.
        /// </summary>
        public Vector3[] Trajectory => m_Path?.TrajectoryFinal;

        /// <summary>
        /// The bounds of the volume that embraces all path geometry.
        /// </summary>
        public Bounds? Bounds => m_Path?.Bounds;

        /// <summary>
        /// The result of the last successful pathfinding.
        /// </summary>
        public PathfindingResult PathfindingResult => m_Path?.LastPathfindingResult;

        /// <summary>
        /// True when the current trajectory was delivered by the FALLBACK_TO_DEFAULT volumetric mode
        /// dropping the clearance constraint (a point path despite a positive request radius).
        /// </summary>
        public bool IsDegraded => m_Path?.IsDegraded ?? false;

        /// <summary>
        /// True when the current trajectory was delivered after the request's timeout passed
        /// (PathfindingTimeoutMode.DELIVER_FOUND_ROUTE): a valid route whose refinement was cut short, so it may be
        /// longer than the optimum.
        /// </summary>
        public bool IsTimedOut => m_Path?.IsTimedOut ?? false;

        /// <summary>
        /// The clearance radius the current trajectory is certified with: a capsule of this radius
        /// along the trajectory was free of obstacles when the search answered. 0 = no promise (point search or
        /// degraded fallback).
        /// </summary>
        public float CertifiedRadius => m_Path?.CertifiedRadius ?? 0f;

        //the clearance radius the current request carries; 0 for a point path
        internal float Radius => m_Path?.Radius ?? 0f;

        #endregion

        #region Constructors

        public Nav3DPath(Log _Log = null)
        {
            m_Log = _Log;
        }

        #endregion

        #region Public methods

        /// <summary>
        /// Starts pathfinding from A to B.
        /// </summary>
        /// <param name="_Radius">
        /// Clearance radius the path must keep from obstacles. 0 = point pathfinding.
        /// </param>
        /// <param name="_VolumetricMode">
        /// What to do when no corridor wide enough for the radius exists (ignored when it is 0).
        /// </param>
        public Task<PathfindingResult> Find(
                Vector3                   _Start,
                Vector3                   _End,
                bool                      _Smooth         = false,
                int                       _SmoothRatio    = 3,
                Action<PathfindingResult> _OnSuccess      = null,
                Action<PathfindingResult> _OnFail         = null,
                float                     _Radius         = 0f,
                VolumetricPathfindingMode _VolumetricMode = VolumetricPathfindingMode.STRICT
            )
        {
            Nav3DRuntimeGate.ThrowIfNotInited();

            return RequestPathfinding(
                    new[] { _Start, _End },
                    false,
                    false,
                    _Smooth,
                    _SmoothRatio,
                    _OnSuccess,
                    _OnFail,
                    _Radius: _Radius,
                    _VolumetricMode: _VolumetricMode
                );
        }

        /// <summary>
        /// Starts pathfinding through all target points.
        /// </summary>
        /// <param name="_Radius">
        /// Clearance radius the path must keep from obstacles. 0 = point pathfinding.
        /// </param>
        /// <param name="_VolumetricMode">
        /// What to do when no corridor wide enough for the radius exists (ignored when it is 0).
        /// </param>
        public Task<PathfindingResult> Find(
                Vector3[]                 _Targets,
                bool                      _Loop,
                bool                      _SkipUnpassableTargets = false,
                bool                      _Smooth                = false,
                int                       _SmoothRatio           = 3,
                Action<PathfindingResult> _OnSuccess             = null,
                Action<PathfindingResult> _OnFail                = null,
                bool                      _UpdatePathSilently    = false,
                float                     _Radius                = 0f,
                VolumetricPathfindingMode _VolumetricMode        = VolumetricPathfindingMode.STRICT
            )
        {
            Nav3DRuntimeGate.ThrowIfNotInited();

            if (_Targets == null || !_Targets.Any())
                throw new ArgumentException(TARGETS_LIST_IS_EMPTY_ERROR);

            return RequestPathfinding(
                    _Targets,
                    _Loop,
                    _SkipUnpassableTargets,
                    _Smooth,
                    _SmoothRatio,
                    _OnSuccess,
                    _OnFail,
                    _UpdatePathSilently,
                    _Radius,
                    _VolumetricMode
                );
        }

        public Task<PathfindingResult> Find(
                PathfindingRequest        _Request,
                Action<PathfindingResult> _OnSuccess          = null,
                Action<PathfindingResult> _OnFail             = null,
                bool                      _UpdatePathSilently = false
            )
        {
            Nav3DRuntimeGate.ThrowIfNotInited();

            if (_Request.Points == null || !_Request.Points.Any())
                throw new ArgumentException(TARGETS_LIST_IS_EMPTY_ERROR);

            return RequestPathfinding(_Request, _OnSuccess, _OnFail, _UpdatePathSilently);
        }

        /// <summary>
        /// Perform pathfinding using recent conditions.
        /// </summary>
        public void Update()
        {
            m_Path?.Update();
        }

        public void Dispose()
        {
            IsValid = false;

            if (m_Path != null)
            {
                m_Path.OnPathfindingFinished -= OnPathfindingFinished;
                m_Path.Dispose();
                m_Path = null;
            }
        }

        internal bool TryGetFollowData(
                Action<Vector3>    _OnTargetPassed,
                Action             _OnLastTargetReached,
                Vector3            _FollowerPosition,
                float              _ReachDist,
                out PathFollowData _FollowData
            )
        {
            if (!IsValid)
            {
                _FollowData = null;
                return false;
            }

            return m_Path.GetFollowData(_OnTargetPassed, _OnLastTargetReached, _FollowerPosition, _ReachDist, out _FollowData);
        }

        #endregion

        #region Internal methods

        internal static void ReportUnreachableTarget(PathfindingRequest _Request, PathfindingResult _Result, GameObject _Requester = null)
        {
            if (_Request == null || _Request.TryRepositionTargetIfOccupied)
                return;

            if (_Result.Result != PathfindingResultCode.TARGET_POINT_INSIDE_OBSTACLE      &&
                _Result.Result != PathfindingResultCode.TARGET_POINT_INSIDE_SEALED_VOLUME &&
                _Result.Result != PathfindingResultCode.TARGET_POINT_TOO_NARROW)
                return;

            Vector3 target = _Result.TryGetFailedPoint(out Vector3 failedPoint) ? failedPoint : _Request.Points[^1];

            Debug.LogError(
                    string.Format(TARGET_UNREACHABLE_ERROR, _Requester != null ? _Requester.name : nameof(Nav3DPath), target.ToStringExt()),
                    _Requester
                );
        }

        internal static void ReportRadiusAboveWorldBound(float _Radius, GameObject _Requester)
        {
            #if !LITE_VERSION

            ObstacleManager manager = ObstacleManager.InstanceIfResolved;

            if (_Radius <= 0f || manager == null || ObstacleManager.Doomed || _Radius <= manager.MaxVolumetricRadius)
                return;

            Debug.LogError(
                    string.Format(RADIUS_ABOVE_WORLD_BOUND_ERROR, _Requester != null ? _Requester.name : nameof(Nav3DPath), _Radius, manager.MaxVolumetricRadius),
                    _Requester
                );

            #endif
        }

        #endregion

        #region Service methods

        void InvokeOnPathfindingSuccess(PathfindingResult _Result)
        {
            OnPathfindingSuccess?.Invoke(_Result);
        }

        void InvokeOnPathfindingFail(PathfindingResult _Result)
        {
            OnPathfindingFail?.Invoke(_Result);
        }

        void InvokeOnPathfindingComplete(PathfindingResult _Result)
        {
            OnPathfindingComplete?.Invoke(_Result);
        }

        Task<PathfindingResult> RequestPathfinding(
                Vector3[]                 _Targets,
                bool                      _Loop,
                bool                      _SkipUnpassableTargets,
                bool                      _Smooth         = false,
                int                       _SmoothRatio    = 3,
                Action<PathfindingResult> _OnSuccess      = null,
                Action<PathfindingResult> _OnFail         = null,
                bool                      _SilentUpdate   = false,
                float                     _Radius         = 0f,
                VolumetricPathfindingMode _VolumetricMode = VolumetricPathfindingMode.STRICT
            )
        {
            IsPathfindingInProgress = true;

            m_OnSuccess = _OnSuccess;
            m_OnFail    = _OnFail;

            if (m_Path == null)
            {
                m_Path = new Path(m_Log);

                m_Path.OnPathfindingFinished += OnPathfindingFinished;
            }

            m_Request = new PathfindingRequest(
                    _Targets,
                    _Loop,
                    _SkipUnpassableTargets,
                    _Smooth: _Smooth,
                    _SmoothRatio: _SmoothRatio,
                    _Radius: _Radius,
                    _VolumetricMode: _VolumetricMode
                );

            ReportRadiusOnce();

            return m_Path.Find(m_Request, _SilentUpdate);
        }

        Task<PathfindingResult> RequestPathfinding(
                PathfindingRequest        _Request,
                Action<PathfindingResult> _OnSuccess          = null,
                Action<PathfindingResult> _OnFail             = null,
                bool                      _UpdatePathSilently = false
            )
        {
            IsPathfindingInProgress = true;

            m_OnSuccess = _OnSuccess;
            m_OnFail    = _OnFail;

            if (m_Path == null)
            {
                m_Path = new Path(m_Log);

                m_Path.OnPathfindingFinished += OnPathfindingFinished;
            }

            m_Request = _Request;

            ReportRadiusOnce();

            return m_Path.Find(_Request, _UpdatePathSilently);
        }

        void ReportRadiusOnce()
        {
            if (!ReportsToConsole || m_RadiusReported)
                return;

            m_RadiusReported = true;

            ReportRadiusAboveWorldBound(m_Request.Radius, null);
        }

        void CleanCallbacks()
        {
            m_OnSuccess = null;
            m_OnFail    = null;
        }

        void OnPathfindingFinished(PathfindingResult _Result)
        {
            IsPathfindingInProgress = false;

            if (_Result.Failed)
            {
                IsValid = false;

                if (ReportsToConsole && m_OnFail == null && OnPathfindingFail == null)
                    ReportUnreachableTarget(m_Request, _Result);

                if (m_OnFail != null)
                {
                    Action<PathfindingResult> tmpFailCallback = m_OnFail;
                    ThreadDispatcher.BeginInvoke(tmpFailCallback, _Result);

                    CleanCallbacks();
                }

                if (OnPathfindingFail != null)
                    ThreadDispatcher.BeginInvoke(InvokeOnPathfindingFail, _Result);
            }
            else
            {
                IsValid = true;

                if (m_OnSuccess != null)
                {
                    Action<PathfindingResult> tmpSuccessCallback = m_OnSuccess;
                    ThreadDispatcher.BeginInvoke(tmpSuccessCallback, _Result);

                    CleanCallbacks();
                }

                if (OnPathfindingSuccess != null)
                    ThreadDispatcher.BeginInvoke(InvokeOnPathfindingSuccess, _Result);
            }

            if (OnPathfindingComplete != null)
                ThreadDispatcher.BeginInvoke(InvokeOnPathfindingComplete, _Result);
        }

        #endregion
    }
}
