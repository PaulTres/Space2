using UnityEngine;
using System;

namespace Nav3D.API
{
    [DisallowMultipleComponent]
    [ExecuteInEditMode]
    public partial class Nav3DInitializer : MonoBehaviour
    {

        #region Constants

        readonly string INIT_ON_AWAKE_ERROR = $"{nameof(Nav3DWorldSettings)}.{nameof(Nav3DWorldSettings.Nav3DInitOnAwake)} flag is set to true, " +
                                              "so it is not possible to call initialization manually. The Nav3D initialization will happen on the Awake event.";

        const string MENU_PATH = "Tools/Nav3D/Create Initializer";

        readonly string INSTANCE_HAS_EXIST_ON_SCENE_ERROR = $"An instance of {nameof(Nav3DInitializer)} is already present in the scene, duplication is not possible.";

        #endregion

        #region Serialized fields

        [SerializeField] Nav3DWorldSettings m_Settings;

        [SerializeField, Tooltip("Create Nav3DOctreeDrawer when Nav3D initializes and switch it on: the octree is drawn in Play mode.")]
        bool m_InitObstacleDrawer;

        [SerializeField, Tooltip("Create Nav3DAgentPathDrawer when Nav3D initializes and switch it on: every agent's path is drawn in Play mode.")]
        bool m_InitPathsDrawer;

        #endregion

        #region Static attributes

        //The component that owns the Nav3D runtime, claimed first-come by Awake() or by an explicit
        //InitManually(). A scene may hold only one: InitNav3DRuntime re-Initialize()s every manager, so a
        //second initializer would silently re-configure the runtime with its own Nav3DWorldSettings.
        static Nav3DInitializer s_Owner;

        #endregion

        #region Attributes

        //set by the first of Awake / OnEnable to start the runtime; the instance is recreated when Play mode ends
        bool m_RuntimeStarted;

        #endregion

        #region Properties

        public float OctreeMinBucketSize   => m_Settings?.OctreeMinBucketSize   ?? Nav3DWorldSettings.DEFAULT_OCTREE_MIN_BUCKET_SIZE;
        public int   OctreeLayersCount     => m_Settings?.OctreeLayersCount     ?? Nav3DWorldSettings.DEFAULT_OCTREE_LAYERS_COUNT;
        public float MaxAgentRadius        => m_Settings?.MaxAgentRadius        ?? Nav3DWorldSettings.DEFAULT_MAX_AGENT_RADIUS;
        public bool  Nav3DInitOnAwake      => m_Settings?.Nav3DInitOnAwake      ?? Nav3DWorldSettings.DEFAULT_NAV3D_INIT_ON_AWAKE;
        public bool  Nav3DDisposeOnDestroy => m_Settings?.Nav3DDisposeOnDestroy ?? Nav3DWorldSettings.DEFAULT_NAV3D_DISPOSE_ON_DESTROY;

        public int  MaxConcurrentSearches     => m_Settings?.MaxConcurrentSearches     ?? Nav3DWorldSettings.DEFAULT_MAX_CONCURRENT_SEARCHES;
        public bool AdaptiveSearchConcurrency => m_Settings?.AdaptiveSearchConcurrency ?? Nav3DWorldSettings.DEFAULT_ADAPTIVE_SEARCH_CONCURRENCY;
        public int  MinConcurrentSearches     => m_Settings?.MinConcurrentSearches     ?? Nav3DWorldSettings.DEFAULT_MIN_CONCURRENT_SEARCHES;

        public bool IsSettingsValid => Nav3DWorldSettings.TryValidate(OctreeMinBucketSize, OctreeLayersCount, MaxAgentRadius, out _);

        public Nav3DWorldSettings Settings
        {
            get => m_Settings;
            set => m_Settings = value;
        }

        //read at initialization; the drawers compile into the editor, and into a player build only under NAV3D_DEBUG_VISUALIZATION
        public bool InitObstacleDrawer
        {
            get => m_InitObstacleDrawer;
            set => m_InitObstacleDrawer = value;
        }

        public bool InitPathsDrawer
        {
            get => m_InitPathsDrawer;
            set => m_InitPathsDrawer = value;
        }

        //Unity's overloaded null check is deliberate: an owner that has been destroyed reads as null, which
        //releases ownership to the next initializer that awakes.
        bool OwnedByAnother => s_Owner != null && !ReferenceEquals(s_Owner, this);

        #endregion

        #region Public methods

        /// <summary>
        /// Initializes Nav3D with a preset minimum bucket size equal to OctreeMinBucketSize.
        /// </summary>
        public void InitManually()
        {
            if (Nav3DInitOnAwake)
                throw new Exception(INIT_ON_AWAKE_ERROR);

            if (OwnedByAnother)
                throw new Exception(INSTANCE_HAS_EXIST_ON_SCENE_ERROR);

            Init();
        }

        /// <summary>
        /// Disposes Nav3D and all its entities;
        /// </summary>
        public void Utilize()
        {
            Nav3DManager.Dispose3DNav();
        }

        #endregion

        #region Service methods

        partial void CheckEditorAwakeGuard(ref bool _Skip);

        void StartRuntimeOnce()
        {
            if (m_RuntimeStarted)
                return;

            bool skip = false;
            CheckEditorAwakeGuard(ref skip);

            if (skip)
                return;

            //A redundant initializer stays inert rather than being removed: it is the user's component, and the
            //runtime it would clobber is already running. Reported once, here.
            if (OwnedByAnother)
            {
                Debug.LogError(INSTANCE_HAS_EXIST_ON_SCENE_ERROR, this);
                return;
            }

            m_RuntimeStarted = true;
            s_Owner          = this;

            if (!Nav3DDisposeOnDestroy)
                DontDestroyOnLoad(this);

            if (Nav3DInitOnAwake)
                Init();
        }

        void Init()
        {
            if (m_Settings == null)
                m_Settings = ScriptableObject.CreateInstance<Nav3DWorldSettings>();

            s_Owner = this;

            Nav3DManager.InitNav3DRuntime(
                    OctreeMinBucketSize,
                    OctreeLayersCount,
                    MaxAgentRadius,
                    MaxConcurrentSearches,
                    AdaptiveSearchConcurrency,
                    MinConcurrentSearches
                );

            InitDrawers();
        }

        void InitDrawers()
        {

            #if UNITY_EDITOR || NAV3D_DEBUG_VISUALIZATION

            if (!Application.isPlaying)
                return;

            if (m_InitObstacleDrawer)
                Nav3DOctreeDrawer.Instance.Active = true;

            if (m_InitPathsDrawer)
                Nav3DAgentPathDrawer.Instance.Active = true;

            #endif

        }

        #endregion

        #region Unity methods

        void Awake()
        {
            StartRuntimeOnce();
        }

        //Awake is not called on an [ExecuteInEditMode] component when Play mode starts with Scene Reload off (Enter Play
        //Mode Options, the Unity 6 project templates' default): the object already exists, so Unity re-enables it. OnEnable
        //is called either way, which is why both lead here and only the first one starts anything.
        void OnEnable()
        {
            StartRuntimeOnce();
        }

        void OnDestroy()
        {
            //an inert duplicate must not tear down the runtime the owner initialized
            if (OwnedByAnother)
                return;

            s_Owner = null;

            if (Nav3DDisposeOnDestroy)
                Nav3DManager.Dispose3DNav();
        }

        #endregion

    }
}