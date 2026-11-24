using Nav3D.Common;
using Nav3D.API;
using System;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;
using PathfindingMethod =
    System.Func<
        Nav3D.API.PathfindingRequest,
        System.Threading.CancellationToken,
        System.Threading.CancellationToken,
        System.Action<string>,
        Nav3D.Common.Log,
        System.Threading.Tasks.Task<Nav3D.API.PathfindingResult>>;

namespace Nav3D.Pathfinding
{
    public partial class Path : IDisposable, ICurve
    {
        #region Constants : Log

        readonly string LOG_PATH_CTOR    = $"{nameof(Path)}.ctor";
        readonly string LOG_PATH_SUCCESS = $"{nameof(Path)}.{nameof(DoOnSuccess)}: Targets: {{0}}";
        readonly string LOG_PATH_FAIL    = $"{nameof(Path)}.{nameof(DoOnFail)}: Targets: {{0}}";
        readonly string LOG_PATH_DISPOSE = $"{nameof(Path)}.{nameof(Dispose)}";
        readonly string LOG_FIND_PATH    = $"{nameof(Path)}.{nameof(Find)}: Targets: {{0}}";
        readonly string LOG_PATH_UPDATE  = $"{nameof(Path)}.{nameof(Update)}";

        readonly string LOG_SILENT_FAIL_RETRY = $"{nameof(Path)}: silent re-path failed transiently ({{0}}), holding the current path and retrying ({{1}}/{{2}})";

        #endregion

        #region Constants : silent-update resilience

        //Silent re-paths (obstacle-change auto-updates, deviation restarts) fail transiently when a moving
        //obstacle overruns the agent, and killing the whole order for that would force user code into
        //teleport/re-queue churn. A silent failure with a spatial-blockage code therefore keeps the current
        //path alive and retries — obstacle-change triggers re-run Update anyway, and the delayed retry below
        //covers a world gone quiet. Only a persistent streak surfaces the honest failure. An explicit
        //(non-silent) Find keeps immediate-failure semantics.
        const int MAX_SILENT_FAIL_STREAK = 10;
        const int SILENT_RETRY_DELAY_MS  = 500;

        #endregion

        #region Attributes

        PathfindingRequest m_Request;

        int[] m_TargetIndices;
        int[] m_TargetSources;

        readonly Log m_Log;

        CancellationTokenSource m_UpdateTokenSource;

        //Coalescing state for Update(). A re-path triggered by an obstacle/region change must not cancel an in-flight
        //search and restart it — under repeated triggers that postpones finalization indefinitely and the path never
        //delivers a result. Instead Update() lets the running search complete (always firing its callback) and, if any
        //trigger arrived meanwhile, runs exactly one trailing pass. Guarded by m_RepathLock because the completion
        //continuation runs on a threadpool thread while Find()/Update() are usually called from the main thread.
        readonly object m_RepathLock = new object();
        bool            m_PathfindingInProgress;
        bool            m_UpdatePending;
        bool            m_Disposed;

        //Changes published while the current search runs. Cleared when a search starts: an earlier change is already in the
        //tree it reads.
        readonly List<Bounds> m_ChangedDuringSearch = new List<Bounds>();

        //silent-update resilience state: whether the current search (identified by m_UpdateTokenSource, set
        //together with it under m_RepathLock) is a silent one, and how many consecutive silent searches have
        //failed with a retryable code (reset by any success)
        bool m_CurrentSearchSilent;
        int  m_SilentFailStreak;

        PathFollowData m_LastGivenFollowData;

        #endregion

        #region Events

        public event Action<PathfindingResult> OnPathfindingFinished;

        #endregion

        #region Properties

        /// <summary>
        /// Final trajectory processed according to pathfinding parameters.
        /// </summary>
        //The internal logic is such that m_TrajectorySmoothed contains the actual trajectory in accordance with all requirements.
        public Vector3[] TrajectoryFinal { get; private set; }

        /// <summary>
        /// Initial trajectory obtained after searching for A* in octrees.
        /// </summary>
        public Vector3[] TrajectoryOriginal { get; private set; }

        /// <summary>
        /// Optimized initial trajectory.
        /// </summary>
        public Vector3[] TrajectoryOptimized { get; private set; }

        /// <summary>
        /// Smoothed optimized trajectory.
        /// </summary>
        public Vector3[] TrajectorySmoothed => TrajectoryFinal;

        /// <summary>
        /// Bounds enclosing all path segments.
        /// </summary>
        public Bounds Bounds { get; private set; }

        /// <summary>
        /// Is found path trajectory meets all pathfinding requirements.
        /// </summary>
        public bool IsValid { get; set; }

        /// <summary>
        /// Resulting path represented as Rays sequence.
        /// </summary>
        public Segment3[] Segments { get; private set; }

        public PathfindingResult LastPathfindingResult { get; private set; }

        // Volumetric pathfinding: the agent radius this path was requested with (0 = point path).
        //Single source of truth is the current request — a capsule path is a point path with Radius 0, there is no
        //second path kind anywhere (storage, re-path, follow all share this one type).
        public float Radius => m_Request?.Radius ?? 0f;

        // True when the last committed result came from the FallbackToDefault mode dropping the volumetric
        //constraint (the trajectory is a point path despite Radius > 0) — surfaced so callers/debug drawing can react.
        public bool IsDegraded { get; private set; }

        //True when the last committed result was delivered after its timeout passed (PathfindingTimeoutMode
        //DELIVER_FOUND_ROUTE): a valid route whose refinement was cut short, so possibly longer than the optimum.
        public bool IsTimedOut { get; private set; }

        // the clearance radius the current trajectory is certified with (see
        //PathfindingResult.CertifiedRadius); 0 = no promise (point search or degraded fallback)
        public float CertifiedRadius { get; private set; }

        #endregion

        #region Constructors

        public Path(Log _Log = null)
        {
            m_Log = _Log;

            m_Log?.Write(LOG_PATH_CTOR);
        }

        ~Path()
        {
            Dispose();
        }

        #endregion

        #region ICurve

        public float Thickness => Radius;

        public bool Intersects(Bounds _Bounds)
        {
            if (!IsValid)
                return false;

            foreach (Segment3 segment in Segments)
            {
                // volumetric: the path body is a capsule around the polyline, so a bounds within
                //Radius of a segment intersects it. UtilsCapsule is a Pro-only file, hence the fence.

                #if !LITE_VERSION

                if (UtilsCapsule.CapsuleIntersectsBounds(segment.Start, segment.End, Radius, _Bounds))
                    return true;

                #else

                if (_Bounds.IntersectSegment(segment))
                    return true;

                #endif
            }

            return false;
        }

        #endregion

        #region Public methods

        public Task<PathfindingResult> Find(PathfindingRequest _Request, bool _SilentUpdate = false)
        {
            m_Log?.WriteFormat(LOG_FIND_PATH, UtilsCommon.GetPointsString(_Request.Points));

            m_Request = _Request;

            if (!_SilentUpdate)
                IsValid = false;

            //A new query supersedes any in-flight search and any deferred recompute: Find cancels and restarts,
            //unlike Update, which coalesces. The token source is captured under the lock because it is the
            //search's identity — reading the field again outside it could grab a newer source installed by a
            //racing Find/Update, and two searches sharing one identity would both count as current.
            CancellationTokenSource requestTokenSource;

            lock (m_RepathLock)
            {
                m_UpdatePending         = false;
                m_PathfindingInProgress = true;
                m_CurrentSearchSilent   = _SilentUpdate;

                m_ChangedDuringSearch.Clear();

                m_UpdateTokenSource?.Cancel();
                m_UpdateTokenSource = requestTokenSource = new CancellationTokenSource();
            }

            return RequestPathfinding(requestTokenSource);
        }

        public void Update()
        {
            m_Log?.Write(LOG_PATH_UPDATE);

            //this search's identity — see the comment in Find
            CancellationTokenSource requestTokenSource;

            lock (m_RepathLock)
            {
                if (m_Disposed)
                    return;

                //A search is already running: do not cancel it. Remember that the world changed and run one more pass
                //once it finishes (drained in the completion continuation). This guarantees forward progress — the
                //running search delivers a result — with at most one trailing recompute after a burst of triggers.
                if (m_PathfindingInProgress)
                {
                    m_UpdatePending = true;
                    return;
                }

                m_PathfindingInProgress = true;
                m_CurrentSearchSilent   = true;

                m_ChangedDuringSearch.Clear();

                m_UpdateTokenSource?.Cancel();
                m_UpdateTokenSource = requestTokenSource = new CancellationTokenSource();
            }

            RequestPathfinding(requestTokenSource);
        }

        internal bool NoteWorldChanged(IReadOnlyList<Bounds> _ChangedVolumes)
        {
            lock (m_RepathLock)
            {
                if (m_Disposed)
                    return false;

                if (m_PathfindingInProgress)
                {
                    m_ChangedDuringSearch.AddRange(_ChangedVolumes);

                    return false;
                }

                return CrossesAny(_ChangedVolumes);
            }
        }

        public bool GetFollowData(
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

            //replace-and-invalidate atomically (main thread): the previous follow data stays valid right up
            //to this handover so the follower never sees an invalid-data gap between a silent re-path
            //landing (worker thread) and its adoption here — see the DoOnSuccess comment (E2)
            m_LastGivenFollowData?.Invalidate();

            _FollowData = m_LastGivenFollowData =
                new PathFollowData(
                        TrajectoryFinal,
                        m_TargetIndices,
                        _FollowerPosition,
                        _ReachDist,
                        _OnTargetPassed,
                        _OnLastTargetReached,
                        m_Log,
                        CertifiedRadius,
                        m_TargetSources
                    );
            return true;
        }

        public void Dispose()
        {
            //Mark disposed under the lock so an in-flight search's completion can't drain a trailing Update() that
            //would re-register and re-request a disposed path.
            lock (m_RepathLock)
            {
                m_Disposed      = true;
                m_UpdatePending = false;
            }

            m_LastGivenFollowData?.Invalidate();
            m_UpdateTokenSource?.Cancel();

            //Dispose also runs from the finalizer, which is always off the main thread — the manager may only be
            //reached through the resolved-instance gate, never resolved (see TryGetPathfindingManager). A gone world
            //has no storage left to unregister from, so false is simply "nothing to do".
            if (TryGetPathfindingManager(out PathfindingManager manager))
                manager.DisposePath(this);

            VolumetricDrawerUnregisterHook();

            m_Log?.Write(LOG_PATH_DISPOSE);
        }

        #endregion

        #region Service methods

        //the only way this class may reach the pathfinding manager. The completion continuations run on threadpool
        //threads and Dispose may run on the FINALIZER thread, while PathfindingManager.Instance resolves through a
        //scene search that throws off the main thread and would create a fresh manager during teardown. So: return
        //the already-resolved reference, never resolve, and treat "not resolved" as "the world is gone". Callers
        //must take their world-gone branch on false.
        static bool TryGetPathfindingManager(out PathfindingManager _Manager)
        {
            _Manager = PathfindingManager.InstanceIfResolved;

            return _Manager != null && !PathfindingManager.Doomed;
        }

        // volumetric drawer registration seams — implemented in Path.Volumetric.Editor.cs (editor-only
        //Pro extension file); zero-cost no-ops in player builds and in the Lite build (file stripped).
        partial void VolumetricDrawerRegisterHook();
        partial void VolumetricDrawerUnregisterHook();

        //Null on the production path -> the manager uses Pathfinder.FindPath. PathDebug (Dev) overrides this to route
        //through Pathfinder.FindPathDebug, which captures the A* expansion history + nav-node chain for visualization.
        protected virtual PathfindingMethod PathfindingMethodOverride => null;

        Task<PathfindingResult> RequestPathfinding(CancellationTokenSource _RequestTokenSource)
        {
            //Runs on threadpool threads too (Update via the trailing drain / the delayed silent retry), so the
            //manager comes through the resolved-instance gate only. A gone world quietly ends the request — there is
            //nobody left to search for, and resolving a fresh manager here is exactly the P6 teardown defect.
            if (m_Request == null || !TryGetPathfindingManager(out PathfindingManager manager))
            {
                lock (m_RepathLock)
                    m_PathfindingInProgress = false;

                return null;
            }

            manager.NoteSearchStarted(this);

            return manager.RequestPathfinding(
                              m_Request,
                              _RequestTokenSource.Token,
                              _Log: m_Log,
                              _PathfindingMethod: PathfindingMethodOverride
                          )
                          .ContinueWith(OnPathfindingFinishedLocal, TaskContinuationOptions.OnlyOnRanToCompletion);

            PathfindingResult OnPathfindingFinishedLocal(Task<PathfindingResult> _PathfindingTask)
            {
                //commit gate. A superseding Find cancels this search's token, but when the cancellation lands after
                //the result was already computed this continuation still runs, concurrently with the new search's.
                //Two commits racing would desynchronize the paths storage, and a success landing after Dispose would
                //re-register a disposed path for ever. So the result is committed only while this search is still
                //the current one and the path is alive; a stale result is discarded wholesale — no state write, no
                //storage write, no event, no flag mutation.
                bool searchWasSilent;

                lock (m_RepathLock)
                {
                    if (m_Disposed || !ReferenceEquals(_RequestTokenSource, m_UpdateTokenSource))
                        return _PathfindingTask.Result;

                    //still under the identity gate, so this is this search's flag, not a successor's
                    searchWasSilent = m_CurrentSearchSilent;
                }

                //Silent-update resilience: a transiently blocked silent re-path (see the constants' comment) is
                //swallowed — no DoOnFail (the path stays valid, followable, and registered in the storage so
                //obstacle-change triggers keep retrying it), no OnPathfindingFinished (no user-facing failure) —
                //and a delayed retry is scheduled below for the case where the world goes quiet.
                bool swallowedFail = false;
                bool succeeded     = false;

                if (_PathfindingTask.Result.Failed)
                {
                    if (searchWasSilent && IsRetryableSilentFail(_PathfindingTask.Result.Result) && m_SilentFailStreak < MAX_SILENT_FAIL_STREAK)
                    {
                        m_SilentFailStreak++;
                        swallowedFail = true;

                        m_Log?.WriteFormat(LOG_SILENT_FAIL_RETRY, _PathfindingTask.Result.Result, m_SilentFailStreak, MAX_SILENT_FAIL_STREAK);
                    }
                    else
                    {
                        DoOnFail(_PathfindingTask.Result);
                    }
                }
                else
                {
                    m_SilentFailStreak = 0;

                    DoOnSuccess(_PathfindingTask.Result);

                    succeeded = true;
                }

                if (!swallowedFail)
                    OnPathfindingFinished?.Invoke(_PathfindingTask.Result);

                //Release the in-flight flag and, if an Update arrived while this search ran, drain exactly one trailing
                //recompute. Runs on this continuation's threadpool thread — consistent with DoOnSuccess above, which
                //already touches the manager/storage off the main thread. A superseding Find clears m_UpdatePending,
                //so a fresh query never triggers a stale trailing pass. A change recorded during the search that crosses
                //the committed route is one more reason; sharing m_RepathLock with NoteWorldChanged means a change lands
                //either here or on the committed curve.
                bool runPending;
                lock (m_RepathLock)
                {
                    bool changedAcrossRoute = succeeded && CrossesAny(m_ChangedDuringSearch);

                    m_ChangedDuringSearch.Clear();

                    m_PathfindingInProgress = false;
                    runPending              = (m_UpdatePending || changedAcrossRoute) && !m_Disposed;
                    m_UpdatePending         = false;
                }

                //leaves the set only after the new curve is in the storage — see PathfindingManager.UpdateAllBoundsCrossingPaths
                if (TryGetPathfindingManager(out PathfindingManager searchManager))
                    searchManager.NoteSearchFinished(this);

                if (runPending)
                    Update();
                else if (swallowedFail)
                    ScheduleSilentRetry();

                return _PathfindingTask.Result;
            }
        }

        protected virtual void DoOnSuccess(PathfindingResult _Result)
        {
            m_Log?.WriteFormat(LOG_PATH_SUCCESS, UtilsCommon.GetPointsString(m_Request.Points));

            //Deliberately do not invalidate the previously given follow data here. This runs on a worker thread
            //while the follower adopts the new trajectory later on the main thread, so invalidating now opens a
            //window in which the follower emits a zero velocity — and under a re-path storm the repeated zeroing
            //re-anchors the acceleration comfort cuts faster than an agent can ramp out of them. The old follow
            //data is self-contained and stays valid until GetFollowData replaces it on the main thread.

            LastPathfindingResult = _Result;

            TrajectoryOriginal  = _Result.PathRaw;
            TrajectoryOptimized = _Result.PathOptimized;
            TrajectoryFinal     = _Result.PathFinal;
            m_TargetIndices     = _Result.TargetIndices;
            m_TargetSources     = _Result.TargetSources;
            IsDegraded          = _Result.Degraded;
            IsTimedOut          = _Result.TimedOut;
            CertifiedRadius     = _Result.CertifiedRadius;

            Bounds bounds = ExtensionBounds.PointBounds(TrajectoryFinal);
            // volumetric: the enclosing bounds cover the capsule body, not just the polyline
            bounds.Expand(2f * Radius);

            Bounds   = bounds;
            Segments = UtilsMath.PointSequenceToSegments(TrajectoryFinal);

            IsValid = true;

            //worker-thread continuation: the manager may only be reached through the resolved-instance gate. With
            //the world gone there is no paths storage left to update; the state writes above are pure C# and stand.
            if (TryGetPathfindingManager(out PathfindingManager manager))
                manager.UpdatePathInStorage(this);

            VolumetricDrawerRegisterHook();
        }

        protected void DoOnFail(PathfindingResult _Result)
        {
            m_Log?.WriteFormat(LOG_PATH_FAIL, UtilsCommon.GetPointsString(m_Request.Points));

            IsValid = false;

            //worker-thread continuation — same resolved-instance gate as DoOnSuccess
            if (TryGetPathfindingManager(out PathfindingManager manager))
                manager.RemovePathFromStorage(this);

            VolumetricDrawerUnregisterHook();
        }

        //Spatial-blockage codes a silent update may recover from once the world changes again (a moving
        //obstacle passing over the agent/target, a temporary seal). CANCELLED/TIMEOUT/UNKNOWN keep their
        //immediate semantics.
        static bool IsRetryableSilentFail(PathfindingResultCode _Code)
        {
            return _Code == PathfindingResultCode.START_POINT_INSIDE_OBSTACLE       ||
                   _Code == PathfindingResultCode.TARGET_POINT_INSIDE_OBSTACLE      ||
                   _Code == PathfindingResultCode.START_POINT_INSIDE_SEALED_VOLUME  ||
                   _Code == PathfindingResultCode.TARGET_POINT_INSIDE_SEALED_VOLUME ||
                   _Code == PathfindingResultCode.PATH_DOES_NOT_EXIST;
        }

        bool CrossesAny(IReadOnlyList<Bounds> _Volumes)
        {
            foreach (Bounds volume in _Volumes)
            {
                if (Intersects(volume))
                    return true;
            }

            return false;
        }

        //One delayed retry per swallowed silent failure: obstacle-change triggers are the primary retry
        //cadence, this covers a world that stops changing (obstacle parked on the agent). Update() itself
        //handles disposal/coalescing, so a late timer is harmless. The continuation is a threadpool thread:
        //with the world gone there is nothing to retry against, so it exits through the resolved-instance
        //gate instead of resolving the manager (P6 — see TryGetPathfindingManager).
        void ScheduleSilentRetry()
        {
            Task.Delay(SILENT_RETRY_DELAY_MS).ContinueWith(_Task =>
            {
                if (TryGetPathfindingManager(out PathfindingManager _))
                    Update();
            });
        }

        #endregion
    }
}
