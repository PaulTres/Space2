using System.Text;
using UnityEngine;

namespace Nav3D.API
{
    [CreateAssetMenu(fileName = "Nav3DWorldConfig", menuName = "Nav3D/World settings", order = 150)]
    public class Nav3DWorldSettings : ScriptableObject
    {
        #region Constants

        public const int   OCTREE_LAYERS_COUNT_MIN       = 2;
        const        float MAX_PROPOSED_OCTREE_ROOT_SIZE = 512f;

        #endregion

        #region Constants : Default values

        internal const float DEFAULT_OCTREE_MIN_BUCKET_SIZE = 0.5f;
        //The inspector exposes the root size, not the layer count: the count is derived from the root and the min
        //bucket, and a layer is a doubling. Six layers put the default root at 16 u.
        internal const int   DEFAULT_OCTREE_LAYERS_COUNT    = 6;

        internal const float DEFAULT_MAX_AGENT_RADIUS = 2f;

        internal const bool DEFAULT_NAV3D_INIT_ON_AWAKE      = true;
        internal const bool DEFAULT_NAV3D_DISPOSE_ON_DESTROY = true;

        //0 = one search per core less one. The adaptive limit is opt-in: an agent standing idle while its search
        //waits in the queue is not a trade every project wants to make for frame time.
        internal const int  DEFAULT_MAX_CONCURRENT_SEARCHES     = 0;
        internal const bool DEFAULT_ADAPTIVE_SEARCH_CONCURRENCY = false;
        internal const int  DEFAULT_MIN_CONCURRENT_SEARCHES     = 1;

        #endregion

        #region Serialized fields

        [SerializeField] float m_OctreeMinBucketSize = DEFAULT_OCTREE_MIN_BUCKET_SIZE;
        [SerializeField] int   m_OctreeLayersCount   = DEFAULT_OCTREE_LAYERS_COUNT;

        [SerializeField, Tooltip("The largest agent radius the navigation graph serves: an agent up to this radius can fly every free route, a larger one is planned as if it were this size.")]
        float m_MaxAgentRadius = DEFAULT_MAX_AGENT_RADIUS;

        [SerializeField] bool m_Nav3DInitOnAwake      = DEFAULT_NAV3D_INIT_ON_AWAKE;
        [SerializeField] bool m_Nav3DDisposeOnDestroy = DEFAULT_NAV3D_DISPOSE_ON_DESTROY;

        [SerializeField, Tooltip("How many path searches may run at once. 0 = automatic (one less than the number of processor cores).")]
        int m_MaxConcurrentSearches = DEFAULT_MAX_CONCURRENT_SEARCHES;

        [SerializeField, Tooltip(
                 "Lower the number of concurrent searches while Nav3D's own main-thread time spikes, and raise it "  +
                 "back while it stays smooth. Keeps the frame rate under CPU pressure at the price of searches "     +
                 "queueing longer — agents may wait for their route."
             )]
        bool m_AdaptiveSearchConcurrency = DEFAULT_ADAPTIVE_SEARCH_CONCURRENCY;

        [SerializeField, Tooltip(
                 "The number of concurrent searches the adaptive limit never goes below. Raise it to keep agents " +
                 "moving through a spike at the cost of less relief for the frame."
             )]
        int m_MinConcurrentSearches = DEFAULT_MIN_CONCURRENT_SEARCHES;

        #endregion

        #region Properties

        public float OctreeMinBucketSize   => m_OctreeMinBucketSize;
        public int   OctreeLayersCount     => m_OctreeLayersCount;
        public float MaxAgentRadius        => m_MaxAgentRadius;
        public bool  Nav3DInitOnAwake      => m_Nav3DInitOnAwake;
        public bool  Nav3DDisposeOnDestroy => m_Nav3DDisposeOnDestroy;

        public int  MaxConcurrentSearches     => m_MaxConcurrentSearches;
        public bool AdaptiveSearchConcurrency => m_AdaptiveSearchConcurrency;
        public int  MinConcurrentSearches     => m_MinConcurrentSearches;

        public bool IsValid => TryValidate(m_OctreeMinBucketSize, m_OctreeLayersCount, m_MaxAgentRadius, out _);

        #endregion

        #region Public methods

        public static int GetMaxLayersCount(float _MinBucketSize)
        {
            float currentBucketSize = _MinBucketSize;
            int   layer             = 0;

            while (currentBucketSize < MAX_PROPOSED_OCTREE_ROOT_SIZE)
            {
                currentBucketSize *= 2;
                layer++;
            }

            return Mathf.Max(OCTREE_LAYERS_COUNT_MIN + 1, layer + 1);
        }

        public static float GetMaxBucketSize(float _MinBucketSize, int _LayersCount)
        {
            return _MinBucketSize * Mathf.Pow(2, _LayersCount - 1);
        }

        //The nearest member of the ... 0.25, 0.5, 1, 2, 4 ... ladder, nearest in ratio rather than in difference
        //(0.3 goes to 0.25, 0.43 to 0.5). A min bucket on that ladder keeps every size derived from it — the root,
        //the resolution a region rounds to, the root grid's world alignment — a round number.
        internal static float GetNearestPowerOfTwo(float _Value)
        {
            if (_Value <= 0f)
                return _Value;

            return Mathf.Pow(2f, Mathf.Round(Mathf.Log(_Value, 2f)));
        }

        internal static bool IsPowerOfTwo(float _Value)
        {
            return _Value > 0f && Mathf.Approximately(GetNearestPowerOfTwo(_Value), _Value);
        }

        public static bool TryValidate(
                float      _OctreeMinBucketSize,
                int        _OctreeLayersCount,
                float      _MaxAgentRadius,
                out string _ErrorMessage
            )
        {
            StringBuilder errorStringBuilder = new StringBuilder();

            bool result = true;

            if (!TryValidateOctreeMinBucketSize(_OctreeMinBucketSize, out string octreeMinBucketSizeErrorMessage))
            {
                result = false;
                errorStringBuilder.AppendLine(octreeMinBucketSizeErrorMessage);
            }

            if (!TryValidateOctreeLayersCount(_OctreeMinBucketSize, _OctreeLayersCount, out string octreeLayersCountErrorMessage))
            {
                result = false;
                errorStringBuilder.AppendLine(octreeLayersCountErrorMessage);
            }

            //the agent-radius check below needs a valid octree
            if (!result)
            {
                _ErrorMessage = errorStringBuilder.ToString();
                return false;
            }

            float octreeMaxBucketSize = GetMaxBucketSize(_OctreeMinBucketSize, _OctreeLayersCount);

            if (!TryValidateMaxAgentRadius(_MaxAgentRadius, octreeMaxBucketSize, out string maxAgentRadiusErrorMessage))
            {
                result = false;
                errorStringBuilder.AppendLine(maxAgentRadiusErrorMessage);
            }

            if (result)
            {
                _ErrorMessage = string.Empty;
                return true;
            }

            _ErrorMessage = errorStringBuilder.ToString();
            return false;
        }

        internal static bool TryValidateOctreeMinBucketSize(float _OctreeMinBucketSize, out string _ErrorMessage)
        {
            if (_OctreeMinBucketSize > 0)
            {
                _ErrorMessage = string.Empty;
                return true;
            }

            _ErrorMessage = "Min bucket value must be greater than zero.";
            return false;
        }

        internal static bool TryValidateOctreeLayersCount(float _OctreeMinBucketSize, float _OctreeLayersCount, out string _ErrorMessage)
        {
            int maxLayersCount = GetMaxLayersCount(_OctreeMinBucketSize);

            if (_OctreeLayersCount > 1 && _OctreeLayersCount <= maxLayersCount)
            {
                _ErrorMessage = string.Empty;
                return true;
            }

            _ErrorMessage = $"Layers count for given min bucket size should be greater or equal then 1 and less or equal then {maxLayersCount}.";
            return false;
        }

        internal static bool TryValidateMaxAgentRadius(float _MaxAgentRadius, float _OctreeMaxBucketSize, out string _ErrorMessage)
        {
            if (_MaxAgentRadius > 0 && _MaxAgentRadius < _OctreeMaxBucketSize)
            {
                _ErrorMessage = string.Empty;
                return true;
            }

            _ErrorMessage = $"Max agent radius must be greater than zero and less than the root size ({_OctreeMaxBucketSize}).";
            return false;
        }

        #endregion
    }
}