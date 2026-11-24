using Nav3D.Common;
using Nav3D.LocalAvoidance;
using Nav3D.Obstacles;
using Nav3D.Pathfinding;
using UnityEngine;
using System;
using System.Collections.Generic;

namespace Nav3D.API
{
    public static partial class Nav3DManager
    {

        #region Constants

        //the factor used to compute path storage bucket size relative to min bucket size
        const int PATH_STORAGE_BUCKET_SIZE_FACTOR = 3;

        static readonly string NEED_INIT_ERROR = $"{nameof(Nav3DManager)} needs to be initialized before! " +
                                                 $"All your work with Nav3D entities should begin on the {nameof(Nav3DManager)}.{nameof(OnNav3DInit)} event execution.";

        static readonly string ON_INIT_UNSUBSCRIBE_ERROR = $"There is no need to unsubscribe from the {nameof(OnNav3DInit)} event. " +
                                                           $"All subscriptions will be invoked and unsubscribed after {nameof(Nav3DManager)} is initialized.";

        static readonly string ON_PRE_INIT_UNSUBSCRIBE_ERROR = $"There is no need to unsubscribe from the {nameof(OnNav3DPreInit)} event. " +
                                                               $"All subscriptions will be unsubscribed after {nameof(Nav3DManager)} is initialized.";

        static readonly string NOT_INITIALIZED_WARNING = $"{nameof(Nav3DManager)} is not initialized yet!";

        #endregion

        #region Nested types

        public class Nav3DManagerNotInitializedException : Exception
        {
            public Nav3DManagerNotInitializedException()
                : base(NEED_INIT_ERROR)
            {
            }
        }

        #endregion

        #region Attributes

        static readonly List<Action> s_OnInitSubscribers         = new List<Action>();
        static readonly List<Action> s_OnInitInternalSubscribers = new List<Action>();
        static readonly List<Action> s_OnPreInitSubscribers      = new List<Action>();

        static event Action s_OnInit;
        static event Action s_OnPreInit;
        static event Action s_OnInitInternal;

        #endregion

        #region Events

        /// <summary>
        /// Lets you know that Nav3DManager initialization has occurred.
        /// </summary>
        public static event Action OnNav3DInit
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

                s_OnInitSubscribers.Add(subscriber);

                s_OnInit += subscriber;
            }
            remove { Debug.LogError(ON_INIT_UNSUBSCRIBE_ERROR); }
        }

        internal static event Action OnNav3DInitInternal
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

                s_OnInitInternalSubscribers.Add(value);
                s_OnInitInternal += value;
            }
            remove
            {
                s_OnInitInternalSubscribers.Remove(value);
                s_OnInitInternal -= value;
            }
        }

        public static event Action OnNav3DPreInit
        {
            add
            {
                if (value == null)
                    return;

                if (PreInited)
                {
                    ThreadDispatcher.BeginInvoke(value);

                    return;
                }

                Action subscriber = () => ThreadDispatcher.BeginInvoke(value);

                s_OnPreInitSubscribers.Add(subscriber);

                s_OnPreInit += subscriber;
            }
            remove { Debug.LogError(ON_PRE_INIT_UNSUBSCRIBE_ERROR); }
        }

        #endregion

        #region Properties

        /// <summary>
        /// Shows if Nav3D is ready to work.
        /// </summary>
        public static bool Inited => Nav3DRuntimeGate.Inited;

        //Milliseconds of the previous frame Nav3D spent on the main thread: the agents' step, the dispatched
        //callbacks, any blocking wait on its locks. Worker-thread work (searches, octree construction) is not in
        //it. This is what the adaptive search concurrency reacts to.
        public static double LastFrameMainThreadMs => Nav3DMainThreadCost.PreviousFrameMs;

        internal static bool PreInited => Nav3DRuntimeGate.PreInited;

        #endregion

        #region Public methods

        /// <summary>
        /// Determines whether a point is inside space cell occupied by any obstacle.
        /// </summary>
        /// <param name="_Point"></param>
        public static bool IsPointInsideOccupiedVolume(Vector3 _Point)
        {
            if (!Inited)
                return false;

            return ObstacleManager.Instance.IsPointInsideObstacle(_Point);
        }

        //The three trailing parameters mirror the Nav3DWorldSettings pathfinding block. Their defaults reproduce
        //the behaviour of a caller that omits them: one search per core less one, adaptation off, floor 1.
        public static void InitNav3DRuntime(
                float _OctreeMinBucketSize,
                int   _OctreeLayersCount,
                float _MaxAgentRadius,
                int   _MaxConcurrentSearches     = Nav3DWorldSettings.DEFAULT_MAX_CONCURRENT_SEARCHES,
                bool  _AdaptiveSearchConcurrency = Nav3DWorldSettings.DEFAULT_ADAPTIVE_SEARCH_CONCURRENCY,
                int   _MinConcurrentSearches     = Nav3DWorldSettings.DEFAULT_MIN_CONCURRENT_SEARCHES
            )
        {
            if (_OctreeMinBucketSize <= 0)
                throw new ArgumentException($"[{nameof(Nav3DManager)}]: Init error. {nameof(_OctreeMinBucketSize)} value must be greater than zero.");

            bool handled = false;
            InitThreadDispatcherEditor(ref handled);

            if (!handled)
                ThreadDispatcher.Instance.Initialize();

            ObstacleManager.Instance.Initialize(_OctreeMinBucketSize, _OctreeLayersCount, _MaxAgentRadius);
            PathfindingManager.Instance.Initialize(_OctreeMinBucketSize * PATH_STORAGE_BUCKET_SIZE_FACTOR);
            PathfindingManager.Instance.ConfigureSearchConcurrency(_MaxConcurrentSearches, _AdaptiveSearchConcurrency, _MinConcurrentSearches);
            AgentManager.Instance.Initialize(new List<IMovable>(), _MaxAgentRadius);

            WireRegionChangedFanOut();

            #if !LITE_VERSION

            ResolutionRegionManager.Instance.Initialize();

            #endif

            MarksAsPreInited();

            #if !LITE_VERSION

            Nav3DObstacleLoader obstacleLoader = UtilsSceneObjects.FindAny<Nav3DObstacleLoader>();

            if (obstacleLoader != null && obstacleLoader.enabled)
            {
                obstacleLoader.OnLoadingFinished += MarksAsInited;
            }
            else
            {
                MarksAsInited();
            }

            #else

            Nav3DLiteObstacleCapture.Capture(
                    _MaxAgentRadius,
                    () =>
                    {
                        if (!PreInited || ObstacleManager.Doomed)
                            return;

                        MarksAsInited();
                    }
                );

            #endif

        }

        /// <summary>
        /// Nav3D initialization for edit mode purposes.
        /// </summary>
        /// <param name="_MinBucketSize">The size of the smallest pathfinding graph nodes.</param>
        public static void InitNav3DEditMode(float _OctreeMinBucketSize, int _OctreeLayersCount, float _MaxAgentRadius)
        {
            if (_OctreeMinBucketSize <= 0)
                throw new ArgumentException($"[{nameof(Nav3DManager)}]: Init error. {nameof(_OctreeMinBucketSize)} value must be greater than zero.");

            ThreadDispatcher.Instance.Initialize();
            ObstacleManager.Instance.Initialize(_OctreeMinBucketSize, _OctreeLayersCount, _MaxAgentRadius);
            PathfindingManager.Instance.Initialize(_OctreeMinBucketSize * PATH_STORAGE_BUCKET_SIZE_FACTOR);
            AgentManager.Instance.Initialize(new List<IMovable>(), _MaxAgentRadius);

            WireRegionChangedFanOut();

            #if !LITE_VERSION

            ResolutionRegionManager.Instance.Initialize();

            UtilsSceneObjects.FindAll<Nav3DResolutionRegion>().ForEach(_Region => _Region.InitializeEditMode());

            #endif

        }

        /// <summary>
        /// Clean all Objects used by Nav3D and stops all running async tasks.
        /// Call before loading a new scene.
        /// </summary>
        public static void Dispose3DNav()
        {
            Nav3DRuntimeGate.SetInited(false);
            Nav3DRuntimeGate.SetPreInited(false);

            UnsubscribeOnInitSubscribers();
            UnsubscribeOnInitInternalSubscribers();
            UnsubscribeOnPreInitSubscribers();

            s_OnInit         = null;
            s_OnInitInternal = null;
            s_OnPreInit      = null;

            if (!ThreadDispatcher.Doomed)
                ThreadDispatcher.Instance.Uninitialize();

            if (!AgentManager.Doomed)
                AgentManager.Instance.Uninitialize();

            if (!ObstacleManager.Doomed)
                ObstacleManager.Instance.Uninitialize();

            if (!PathfindingManager.Doomed)
                PathfindingManager.Instance.Uninitialize();

            #if !LITE_VERSION

            if (!ResolutionRegionManager.Doomed)
                ResolutionRegionManager.Instance.Uninitialize();

            #endif

        }

        internal static void Dispose3DNavEditMode()
        {
            ThreadDispatcher.Instance.Uninitialize();
            ObstacleManager.Instance.Uninitialize();
            PathfindingManager.Instance.Uninitialize();
            AgentManager.Instance.Uninitialize();

            #if !LITE_VERSION

            ResolutionRegionManager.Instance.Uninitialize();

            #endif

        }

        internal static void CheckInitedSoft()
        {
            if (!Inited)
                Debug.LogWarning(NOT_INITIALIZED_WARNING);
        }

        internal static void CheckInitedHard()
        {
            if (!Inited)
                throw new Nav3DManagerNotInitializedException();
        }

        #endregion

        #region Service methods

        //the changed-region fan-OUT LIVES here, not in ObstacleManager, because both of its consumers sit above the
        //octree: SpaceProcessing publishes the volumes and this composition root decides who reacts. Runs on the
        //main thread, dispatched by the octree after a publish. Two obstacle-side invalidations a dynamic obstacle
        //or resolution-region change must trigger, both Doomed-guarded so a change racing teardown never resurrects
        //a manager:
        //1) re-path every stored path crossing the region — stale trajectories through newly-blocked space, missed
        //    newly-opened shortcuts;
        //2) dirty the cached static-obstacle ORCA set of every agent whose bucket overlaps the region. Local
        //    avoidance refreshes that cache only when a mover CROSSES a hash bucket, so an agent that does not
        //    change bucket would otherwise keep a stale obstacle picture.
        //The two halves take the operation's volumes differently ON purpose. The path half gets the whole list in
        //one call, because a path crossing two volumes would otherwise be dispatched twice and each dispatch costs
        //real work. The agent half is called per volume: it only writes a flag on the movers of each bucket it
        //sweeps, so a second write costs nothing.
        static void WireRegionChangedFanOut()
        {
            ObstacleManager.Instance.OnRegionChanged += OnObstacleRegionChanged;
        }

        static void OnObstacleRegionChanged(IReadOnlyList<Bounds> _ChangedVolumes)
        {
            UnityEngine.Profiling.Profiler.BeginSample("Nav3D.RegionChanged");

            if (!PathfindingManager.Doomed)
                PathfindingManager.Instance.UpdateAllBoundsCrossingPaths(_ChangedVolumes);

            if (!AgentManager.Doomed)
            {
                foreach (Bounds changedVolume in _ChangedVolumes)
                    AgentManager.Instance.SetMovablesInBoundsObstacleDirty(changedVolume);
            }

            UnityEngine.Profiling.Profiler.EndSample();
        }

        //Hands the lowered guard (Nav3DRuntimeGate) the exception the layers under this one must throw. It runs
        //before any user code in a player and on every domain reload in the editor, so a guarded entry point can
        //never be reached before the factory is installed.

        #if UNITY_EDITOR

        [UnityEditor.InitializeOnLoadMethod]

        #endif

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        static void InstallNotInitializedExceptionFactory()
        {
            Nav3DRuntimeGate.SetNotInitializedExceptionFactory(() => new Nav3DManagerNotInitializedException());
        }

        static partial void InitThreadDispatcherEditor(ref bool _Handled);

        static void MarksAsInited()
        {
            Nav3DRuntimeGate.SetInited(true);

            s_OnInitInternal?.Invoke();
            s_OnInit?.Invoke();

            UnsubscribeOnInitInternalSubscribers();
            UnsubscribeOnInitSubscribers();
        }

        static void MarksAsPreInited()
        {
            Nav3DRuntimeGate.SetPreInited(true);

            s_OnPreInit?.Invoke();

            UnsubscribeOnPreInitSubscribers();
        }

        static void UnsubscribeOnInitSubscribers()
        {
            foreach (Action subscriber in s_OnInitSubscribers)
            {
                s_OnInit -= subscriber;
            }

            s_OnInitSubscribers.Clear();
        }

        static void UnsubscribeOnInitInternalSubscribers()
        {
            foreach (Action subscriber in s_OnInitInternalSubscribers)
            {
                s_OnInitInternal -= subscriber;
            }

            s_OnInitInternalSubscribers.Clear();
        }

        static void UnsubscribeOnPreInitSubscribers()
        {
            foreach (Action subscriber in s_OnPreInitSubscribers)
            {
                s_OnPreInit -= subscriber;
            }

            s_OnPreInitSubscribers.Clear();
        }

        #endregion

    }
}
