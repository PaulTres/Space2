using Nav3D.API;
using Nav3D.Common;
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
    [DisallowMultipleComponent]
    public partial class PathfindingManager : MonoBehaviour
    {

        #region Constants : Log

        readonly string LOG_REQUEST_PATHFINDING = $"{nameof(PathfindingManager)}.{nameof(RequestPathfinding)}: OrderID: {{0}}, Points: {{1}}, Looped {{2}}";

        #endregion

        #region Constants : adaptive search concurrency

        //The signal is Nav3D's own main-thread time in the previous frame, never the whole frame — a frame spikes
        //for a GPU wait or someone else's script too, and that is not Nav3D's to react to. A frame is a spike when
        //that time is at least SPIKE_FLOOR_MS (below it a doubling is noise, not a stall) and exceeds both twice
        //its smoothed value and the smoothed value plus SPIKE_MARGIN_MS. A spike halves the live limit at once;
        //RECOVERY_FRAMES calm frames in a row grow it by one. Slices over SPIKE_IGNORE_MS are a scene load or a
        //pause rather than a load signal, and are ignored by both the average and the spike test.
        const float SPIKE_FLOOR_MS  = 6f;
        const float SPIKE_MARGIN_MS = 4f;
        const float SPIKE_IGNORE_MS = 1000f;
        const float COST_EMA_WEIGHT = 0.05f;
        const int   RECOVERY_FRAMES = 90;

        static readonly string CEILING_LOCKED_WARNING =
            //the two API names below are literals on purpose: this assembly sits under Nav3D.API, so it cannot
            //nameof() them
            "Nav3DPathfindingManager.MaxPathfindingTasks cannot be changed "                                             +
            "while adaptive search concurrency is enabled: the adaptive limit owns how many searches run at once, and "  +
            "moving its ceiling would discard whatever it has settled on. Set \"Max concurrent searches\" on the "       +
            $"{nameof(Nav3DWorldSettings)} asset before initialization, or turn \"Adaptive search concurrency\" off to " +
            "drive the ceiling yourself. The value stays at {0}.";

        #endregion

        #region Attributes

        //the ceiling stays ProcessorCount - 1 (lower it only while things are bad and restore it
        //the moment they are not — which is the adaptive limit's job, not the default's). The one measurement that
        //argued for a lower default (on a 10-core M4, nine concurrent Skyrift searches completed no more per minute
        //than three) is one machine and one scene; a user who knows theirs sets PathFindingTasksMaxCount.
        int m_PathFindingTasksMaxCount = Environment.ProcessorCount - 1;

        //adaptive search concurrency state (main thread only)
        float m_CostEmaMs;
        int   m_CalmFrames;
        int   m_MinSearchLimit = 1;
        bool  m_CeilingLockWarned;

        float m_StorageBucketSize;

        //Paths that were found in scene space.
        //Need to be stored to determine if any update to the obstacle will invalidate some path.
        //In this case, the invalid path needs to be updated.
        CurvesSpatialHashMap<Path> m_PathsStorage;

        //The storage cannot see a searching path: a first search has no curve yet, a re-path has only the one it replaces.
        readonly ConcurrentHashSet<Path> m_SearchingPaths = new ConcurrentHashSet<Path>();

        OrdersExecutor<IExecutable> m_PathfindingOrdersExecutor;

        #endregion

        #region Properties

        public static bool               Doomed   { get; private set; } = false;
        public static PathfindingManager Instance => Singleton<PathfindingManager>.Instance;

        //worker-SAFE counterpart of Instance: returns the already-resolved manager and never resolves a new one.
        //Instance goes through UtilsSceneObjects.FindAny, which throws outright on a worker thread — and on the main
        //thread would create a fresh manager GameObject during teardown. Anything running off the main thread
        //(Path's completion continuations, its delayed silent retry, the finalizer-driven Dispose) must use this and
        //treat null as "the world is gone". Same contract as ObstacleManager.InstanceIfResolved.
        public static PathfindingManager InstanceIfResolved => Singleton<PathfindingManager>.InstanceIfResolved;

        //the ceiling, and it is sealed while the adaptive limit is on (owner, 2026-09-08). Writing it there is not
        //merely ineffective, it is destructive: SetSearchCeiling recreates the orders executor, and a fresh executor
        //starts at the new ceiling — so the write silently throws away whatever the controller had backed off to and
        //restarts its recovery count from zero, which is the opposite of what a caller reaching for this property
        //wants. Refused with one warning and no change; the getter keeps answering the real ceiling, so a caller can
        //see that nothing moved. ConfigureSearchConcurrency is the one legitimate write and goes around this
        //deliberately (it is the configuration point, and it runs before a single search exists).
        public int PathFindingTasksMaxCount
        {
            get => m_PathFindingTasksMaxCount;
            set
            {
                if (AdaptiveSearchConcurrency)
                {
                    WarnCeilingLocked();

                    return;
                }

                SetSearchCeiling(value);
            }
        }

        public int PathFindingTasksOperatingCount => m_PathfindingOrdersExecutor?.CurrentAliveOrders ?? 0;

        //adaptive search concurrency. PathFindingTasksMaxCount is the ceiling; the number of searches actually
        //running at once is lowered while Nav3D's own main-thread time spikes and raised back while it stays
        //smooth, so the pool backs off exactly when its own work says it is losing the CPU. opt-IN and off by
        //default: a lowered limit means searches queue longer and an agent waiting for its route stands still,
        //which is the project's trade to make rather than the library's. Off = the ceiling is the limit.
        public bool AdaptiveSearchConcurrency { get; set; }

        //the floor under the adaptive limit (Nav3DWorldSettings "Min concurrent searches"): at least one, never
        //above the ceiling; a project that wants its agents moving through a spike raises it
        public int MinSearchLimit
        {
            get => m_MinSearchLimit;
            set => m_MinSearchLimit = Mathf.Max(1, value);
        }

        //how many searches may run right now (the ceiling when adaptation is off or the frame is smooth)
        public int ActiveSearchLimit => m_PathfindingOrdersExecutor?.ConcurrencyLimit ?? 0;

        #endregion

        #region Public methods

        public void Initialize(float _StorageBucketSize)
        {
            //also here, not only in Awake: edit mode runs no Awake, and the last play session's OnDestroy left it set
            Doomed = false;

            RecreateOrdersExecutor(m_PathFindingTasksMaxCount);

            m_StorageBucketSize = _StorageBucketSize;

            m_PathsStorage = new CurvesSpatialHashMap<Path>(m_StorageBucketSize);
        }

        //the Nav3DWorldSettings pathfinding block, applied at init: 0 searches = one per core less one
        public void ConfigureSearchConcurrency(int _MaxConcurrentSearches, bool _AdaptiveSearchConcurrency, int _MinConcurrentSearches)
        {
            //straight to the field's setter, not through the guarded property: this is the configuration point the
            //guard exempts, and it must not depend on the order these three lines happen to be written in
            SetSearchCeiling(_MaxConcurrentSearches > 0 ? _MaxConcurrentSearches : Environment.ProcessorCount - 1);

            AdaptiveSearchConcurrency = _AdaptiveSearchConcurrency;
            MinSearchLimit            = _MinConcurrentSearches;
        }

        public void Uninitialize(bool _NeedDestroy = true)
        {
            if (!_NeedDestroy)
                return;

            UtilsCommon.SmartDestroyImmediateWithHolder(this);
        }

        public Task<PathfindingResult> RequestPathfinding(
                PathfindingRequest _Request,
                CancellationToken  _CancellationToken,
                Log                _Log               = null,
                PathfindingMethod  _PathfindingMethod = null
            )
        {
            _Log?.WriteFormat(LOG_REQUEST_PATHFINDING, _Request.RequesterID, UtilsCommon.GetPointsString(_Request.Points), _Request.Loop);

            PathfindingOrder order = new PathfindingOrder(
                    _Request,
                    _CancellationToken,
                    _PathfindingMethod ?? Pathfinder.FindPath,
                    _Log: _Log
                );

            m_PathfindingOrdersExecutor.EnqueueOrder(order, _Log);

            return order.Task;
        }

        //Runs on the main thread (the octree publish dispatches OnRegionChanged there), so every millisecond spent
        //here is frame time. The two halves are instrumented separately because they fail differently: the query
        //walks a structure sized by the changed volume, the dispatch by the number of paths that actually cross it.
        public void UpdateAllBoundsCrossingPaths(Bounds _Bounds)
        {
            UpdateAllBoundsCrossingPaths(new[] { _Bounds });
        }

        //The form an octree operation actually notifies in: its footprint piece by piece, so an obstacle in
        //disjoint parts never claims the space between them changed (see GlobalOctree.NotifyRegionChanged). Taking
        //the whole list here rather than looping the single-volume overload outside is what keeps one path = one
        //Update(): the query answers for the union of the volumes in one pass.
        //Searching paths are walked before the storage is queried: a committing path enters the storage before it leaves
        //the searching set, so it is always caught by one of the two.
        public void UpdateAllBoundsCrossingPaths(IReadOnlyList<Bounds> _Volumes)
        {
            if (_Volumes == null || _Volumes.Count == 0)
                return;

            HashSet<Path> pathsToUpdate = null;

            foreach (Path searchingPath in m_SearchingPaths)
            {
                if (!searchingPath.NoteWorldChanged(_Volumes))
                    continue;

                pathsToUpdate ??= new HashSet<Path>();
                pathsToUpdate.Add(searchingPath);
            }

            if (m_PathsStorage.TryGetIntersectingCurves(_Volumes, out HashSet<Path> intersectingPaths))
            {
                if (pathsToUpdate == null)
                    pathsToUpdate = intersectingPaths;
                else
                    pathsToUpdate.UnionWith(intersectingPaths);
            }

            DispatchUpdates(pathsToUpdate != null, pathsToUpdate);
        }

        internal void NoteSearchStarted(Path _Path)
        {
            m_SearchingPaths.TryAdd(_Path);
        }

        internal void NoteSearchFinished(Path _Path)
        {
            m_SearchingPaths.TryRemove(_Path);
        }

        public void UpdatePathInStorage(Path _Path)
        {
            m_PathsStorage.Update(_Path);
        }

        public void RemovePathFromStorage(Path _Path)
        {
            m_PathsStorage.Unregister(_Path);
        }

        //A main-THREAD dispose does not wait for the storage. Dispose runs on the main thread for every agent that
        //leaves the scene, while the storage's lock is held by search continuations on the pool. Nothing here needs
        //the unregister to be synchronous: the path is already marked disposed, so a crossing-paths query that
        //still finds it calls an Update() that returns at once, and no search result can re-register it — the
        //commit gate checks the disposed flag first. Off the main thread the unregister stays inline.
        public void DisposePath(Path _Path)
        {
            m_SearchingPaths.TryRemove(_Path);

            if (Nav3DMainThreadCost.IsMainThread)
                Task.Run(() => m_PathsStorage.Unregister(_Path));
            else
                m_PathsStorage.Unregister(_Path);
        }

        #endregion

        #region Service methods

        void DispatchUpdates(bool _AnyCrossing, HashSet<Path> _IntersectingPaths)
        {
            if (!_AnyCrossing)
                return;

            _IntersectingPaths.ForEach(_Path => _Path.Update());
        }

        //Clamped before the no-op check, so a value that only differs from the current one before clamping (0 or a
        //negative against a ceiling of 1) no longer recreates the executor for nothing.
        void SetSearchCeiling(int _Value)
        {
            int ceiling = Mathf.Max(_Value, 1);

            if (ceiling == m_PathFindingTasksMaxCount)
                return;

            m_PathFindingTasksMaxCount = ceiling;

            RecreateOrdersExecutor(m_PathFindingTasksMaxCount);
        }

        void WarnCeilingLocked()
        {
            if (m_CeilingLockWarned)
                return;

            m_CeilingLockWarned = true;

            Debug.LogWarning(string.Format(CEILING_LOCKED_WARNING, m_PathFindingTasksMaxCount));
        }

        void RecreateOrdersExecutor(int _MaxAliveOrders)
        {
            m_PathfindingOrdersExecutor = new OrdersExecutor<IExecutable>(_MaxAliveOrders);

            m_CalmFrames = 0;
        }

        //the control loop: one read of Nav3D's main-thread cost in the frame that just ended, one decision
        void AdaptSearchConcurrency()
        {
            if (m_PathfindingOrdersExecutor == null)
                return;

            if (!AdaptiveSearchConcurrency)
            {
                m_PathfindingOrdersExecutor.SetConcurrencyLimit(m_PathFindingTasksMaxCount);

                return;
            }

            float costMs = (float) Nav3DMainThreadCost.PreviousFrameMs;

            if (costMs > SPIKE_IGNORE_MS)
                return;

            bool spike = costMs >= SPIKE_FLOOR_MS && costMs > Mathf.Max(2f * m_CostEmaMs, m_CostEmaMs + SPIKE_MARGIN_MS);

            //the average follows calm frames only, so a run of spikes cannot raise the bar they are judged against
            if (!spike)
                m_CostEmaMs = Mathf.Lerp(m_CostEmaMs, costMs, COST_EMA_WEIGHT);

            int limit = m_PathfindingOrdersExecutor.ConcurrencyLimit;
            int floor = Mathf.Min(m_MinSearchLimit, m_PathFindingTasksMaxCount);

            if (spike)
            {
                m_CalmFrames = 0;

                if (limit > floor)
                    m_PathfindingOrdersExecutor.SetConcurrencyLimit(Mathf.Max(floor, limit / 2));
            }
            else if (limit < m_PathFindingTasksMaxCount && ++m_CalmFrames >= RECOVERY_FRAMES)
            {
                m_CalmFrames = 0;

                m_PathfindingOrdersExecutor.SetConcurrencyLimit(limit + 1);
            }

        }

        #endregion

        #region Unity methods

        void Awake()
        {
            Doomed = false;
        }

        void Update()
        {
            AdaptSearchConcurrency();
        }

        void OnDestroy()
        {
            Doomed = true;

            Uninitialize(false);

            //Drop the static reference so m_PathsStorage / m_PathfindingOrdersExecutor are no longer GC-rooted
            //after this destroy — see Singleton<T>.ClearInstance.
            Singleton<PathfindingManager>.ClearInstance(this);
        }

        #endregion

    }
}
