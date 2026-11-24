using Nav3D.Common;
using System.Collections.Generic;
using UnityEngine;

namespace Nav3D.API
{
    /// <summary>
    /// Runs one pathfinding request through the scene's target transforms and draws the answer, so a route can be
    /// judged in the Editor without code. The request is either the one an agent with the wired config would make
    /// (radius, mode, timeout, smoothing, repositioning read from the config) or the one the component's own fields
    /// describe; it re-runs whenever a target moves or a setting changes while Auto update is on.
    /// </summary>
    [DisallowMultipleComponent]
    public partial class Nav3DPathTester : MonoBehaviour
    {

        #region Constants

        const int   DEFAULT_TIMEOUT_MS   = 2000;
        const int   DEFAULT_SMOOTH_RATIO = 3;
        const float DEFAULT_RADIUS       = 0f;

        #endregion

        #region Serialized fields

        [SerializeField] List<Transform> m_Targets = new List<Transform>(2);

        [SerializeField] Nav3DAgentConfig m_Config;

        [SerializeField] bool m_Loop;
        [SerializeField] bool m_SkipUnpassableTargets;
        [SerializeField] bool m_TryRepositionStartIfOccupied  = true;
        [SerializeField] bool m_TryRepositionTargetIfOccupied = true;

        [SerializeField] int                    m_Timeout     = DEFAULT_TIMEOUT_MS;
        [SerializeField] PathfindingTimeoutMode m_TimeoutMode = PathfindingTimeoutMode.DELIVER_FOUND_ROUTE;
        [SerializeField] bool                   m_Smooth      = true;
        [SerializeField] int                    m_SmoothRatio = DEFAULT_SMOOTH_RATIO;

        [SerializeField] float                     m_Radius         = DEFAULT_RADIUS;
        [SerializeField] VolumetricPathfindingMode m_VolumetricMode = VolumetricPathfindingMode.STRICT;
        [SerializeField] VolumetricOptimality      m_Optimality     = VolumetricOptimality.QUALITY;

        [SerializeField] bool m_AutoUpdate = true;

        [SerializeField] bool m_Draw = true;

        #endregion

        #region Attributes

        readonly List<Vector3> m_CachedPositions = new List<Vector3>();
        readonly List<Vector3> m_Points          = new List<Vector3>();

        //the request the last search was built from, so a changed setting is detected without rebuilding one
        Nav3DAgentConfig          m_CachedConfig;
        bool                      m_CachedLoop;
        bool                      m_CachedSkipUnpassableTargets;
        bool                      m_CachedTryRepositionStartIfOccupied;
        bool                      m_CachedTryRepositionTargetIfOccupied;
        int                       m_CachedTimeout;
        PathfindingTimeoutMode    m_CachedTimeoutMode;
        bool                      m_CachedSmooth;
        int                       m_CachedSmoothRatio;
        float                     m_CachedRadius;
        VolumetricPathfindingMode m_CachedVolumetricMode;
        VolumetricOptimality      m_CachedOptimality;

        bool m_SearchPending;

        Nav3DPath m_Path;

        float m_SearchStartTime;

        #endregion

        #region Properties

        /// <summary>
        /// The request the last search was made with, or null before the first search.
        /// </summary>
        public PathfindingRequest LastRequest { get; private set; }

        /// <summary>
        /// The last search's result, or null while none has finished.
        /// </summary>
        public PathfindingResult LastResult { get; private set; }

        /// <summary>
        /// Length of the last found route in world units; 0 when the last search failed.
        /// </summary>
        public float LastPathLength { get; private set; }

        /// <summary>
        /// Wall-clock seconds the last search took, from the request to the delivered result.
        /// </summary>
        public float LastSearchSeconds { get; private set; }

        public bool IsSearching => m_Path != null && m_Path.IsPathfindingInProgress;

        /// <summary>
        /// The last found route, or null when there is none.
        /// </summary>
        public Vector3[] Trajectory => m_Path?.Trajectory;

        /// <summary>
        /// The clearance radius the next request carries: the wired config's pathfinding radius, else Radius.
        /// 0 is a point search.
        /// </summary>
        public float EffectiveRadius => m_Config != null ? m_Config.PathfindingRadius : Mathf.Max(0f, m_Radius);

        #endregion

        #region Public methods

        /// <summary>
        /// Runs the search now with the current targets and settings. With Auto update off this is the only way a
        /// search starts; it is what the inspector's Find path button calls.
        /// </summary>
        public void FindPath()
        {
            if (!Nav3DManager.Inited)
            {
                m_SearchPending = true;

                return;
            }

            m_SearchPending = false;

            CollectPoints();
            CacheSettings();

            if (m_Points.Count < 2)
            {
                ClearPath();

                return;
            }

            if (m_Path == null)
            {
                m_Path = new Nav3DPath();

                m_Path.OnPathfindingSuccess += OnPathfindingFinished;
                m_Path.OnPathfindingFail    += OnPathfindingFinished;
            }

            LastRequest = new PathfindingRequest(
                    m_Points.ToArray(),
                    m_Loop,
                    m_SkipUnpassableTargets,
                    m_Config != null ? m_Config.TryRepositionStartIfOccupied : m_TryRepositionStartIfOccupied,
                    m_Config != null ? m_Config.TryRepositionTargetIfOccupied : m_TryRepositionTargetIfOccupied,
                    m_Config != null ? m_Config.PathfindingTimeout : Mathf.Max(1, m_Timeout),
                    m_Config != null ? m_Config.SmoothPath : m_Smooth,
                    m_Config != null ? m_Config.SmoothRatio : Mathf.Max(1, m_SmoothRatio),
                    $"{gameObject.name} [{this.UnityIdText()}]",
                    EffectiveRadius,
                    m_Config               != null ? m_Config.VolumetricMode : m_VolumetricMode,
                    _TimeoutMode: m_Config != null ? m_Config.TimeoutMode : m_TimeoutMode,
                    _Optimality: m_Config  != null ? m_Config.Optimality : m_Optimality
                );

            m_SearchStartTime = Time.realtimeSinceStartup;

            m_Path.Find(LastRequest);
        }

        #endregion

        #region Unity methods

        void OnEnable()
        {
            //a fresh search on every enable: the cache of the previous life would otherwise hold the targets still
            m_SearchPending = true;
        }

        void Update()
        {
            if (!Nav3DManager.Inited)
                return;

            if (m_SearchPending || (m_AutoUpdate && (SettingsChanged() || TargetsMoved())))
                FindPath();
        }

        void OnDisable()
        {
            ClearPath();
        }

        #endregion

        #region Service methods

        //the targets' positions, empty slots skipped; a transform listed twice is two visits of one point
        void CollectPoints()
        {
            m_Points.Clear();
            m_CachedPositions.Clear();

            foreach (Transform target in m_Targets)
            {
                if (target == null)
                    continue;

                m_Points.Add(target.position);
                m_CachedPositions.Add(target.position);
            }
        }

        bool TargetsMoved()
        {
            int index = 0;

            foreach (Transform target in m_Targets)
            {
                if (target == null)
                    continue;

                if (index >= m_CachedPositions.Count || m_CachedPositions[index] != target.position)
                    return true;

                index++;
            }

            return index != m_CachedPositions.Count;
        }

        void CacheSettings()
        {
            m_CachedConfig                        = m_Config;
            m_CachedLoop                          = m_Loop;
            m_CachedSkipUnpassableTargets         = m_SkipUnpassableTargets;
            m_CachedTryRepositionStartIfOccupied  = m_TryRepositionStartIfOccupied;
            m_CachedTryRepositionTargetIfOccupied = m_TryRepositionTargetIfOccupied;
            m_CachedTimeout                       = m_Timeout;
            m_CachedTimeoutMode                   = m_TimeoutMode;
            m_CachedSmooth                        = m_Smooth;
            m_CachedSmoothRatio                   = m_SmoothRatio;
            m_CachedRadius                        = m_Radius;
            m_CachedVolumetricMode                = m_VolumetricMode;
            m_CachedOptimality                    = m_Optimality;
        }

        bool SettingsChanged()
        {
            return m_CachedConfig                        != m_Config                        ||
                   m_CachedLoop                          != m_Loop                          ||
                   m_CachedSkipUnpassableTargets         != m_SkipUnpassableTargets         ||
                   m_CachedTryRepositionStartIfOccupied  != m_TryRepositionStartIfOccupied  ||
                   m_CachedTryRepositionTargetIfOccupied != m_TryRepositionTargetIfOccupied ||
                   m_CachedTimeout                       != m_Timeout                       ||
                   m_CachedTimeoutMode                   != m_TimeoutMode                   ||
                   m_CachedSmooth                        != m_Smooth                        ||
                   m_CachedSmoothRatio                   != m_SmoothRatio                   ||
                   m_CachedRadius                        != m_Radius                        ||
                   m_CachedVolumetricMode                != m_VolumetricMode                ||
                   m_CachedOptimality                    != m_Optimality;
        }

        void ClearPath()
        {
            if (m_Path == null)
                return;

            m_Path.OnPathfindingSuccess -= OnPathfindingFinished;
            m_Path.OnPathfindingFail    -= OnPathfindingFinished;
            m_Path.Dispose();

            m_Path = null;
        }

        void OnPathfindingFinished(PathfindingResult _Result)
        {
            LastResult        = _Result;
            LastSearchSeconds = Time.realtimeSinceStartup - m_SearchStartTime;
            LastPathLength    = _Result.Failed ? 0f : MeasureLength(_Result.PathFinal);

            if (_Result.Failed)
                Debug.LogError($"{gameObject.name}: pathfinding failed: {_Result.Result}. {_Result.ErrorMessage}", this);
        }

        static float MeasureLength(Vector3[] _Points)
        {
            if (_Points == null)
                return 0f;

            float length = 0f;

            for (int i = 1; i < _Points.Length; i++)
                length += Vector3.Distance(_Points[i - 1], _Points[i]);

            return length;
        }

        #endregion

    }
}