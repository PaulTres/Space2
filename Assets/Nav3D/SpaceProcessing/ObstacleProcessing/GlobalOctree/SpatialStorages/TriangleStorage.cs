using Nav3D.Common;
using Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.OperationsProgress;
using System;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.SpatialStorages
{
    /// <summary>
    /// Triangle spatial storage backed by a Bounding Volume Hierarchy (BVH).
    /// Automatic SAH / Morton-balanced CPU construction with a shared flat array layout.
    /// SAH uses parallel subtrees, pre-computed centroids, adaptive stack-allocated bins;
    /// Morton uses spatial key ordering and bottom-up bounds. Immutable after construction —
    /// thread-safe for concurrent reads.
    /// </summary>
    partial class TriangleStorage : IOccupancyStorage
    {

        #region Constants

        const int   SAH_BIN_COUNT_LARGE    = 12;
        const int   SAH_BIN_COUNT_SMALL    = 6;
        const int   ADAPTIVE_BIN_THRESHOLD = 256;
        const int   MAX_LEAF_TRIANGLES     = 4;
        const float TRAVERSAL_COST         = 1.0f;
        const float INTERSECTION_COST      = 1.5f;
        const int   PARALLEL_THRESHOLD     = 2048;

        #endregion

        #region Nested types

        struct BvhNode
        {
            public Vector3 BoundsMin    { get; set; }
            public Vector3 BoundsMax    { get; set; }
            public int     StartOrLeft  { get; set; }
            public int     CountOrRight { get; set; }

            public bool IsLeaf        => CountOrRight > 0;
            public int  TriangleStart => StartOrLeft;
            public int  TriangleCount => CountOrRight;
            public int  LeftChild     => StartOrLeft;
            public int  RightChild    => -CountOrRight;
        }

        #endregion

        #region Attributes

        BvhNode[]  m_Nodes;
        Triangle[] m_Triangles;
        Vector3[]  m_Centroids;

        //the triangles' own boxes as flat arrays beside the centroids: the SAH build reads them in every pass of every
        //level, and reading them off the Triangle objects' Bounds (a centre/extents struct converted on each access)
        //cost the build several times what the arithmetic does
        Vector3[] m_Mins;
        Vector3[] m_Maxs;

        int m_NodeCount;

        int m_TriangleCount;

        #endregion

        #region Properties

        //The closed bodies a construction reading this storage classifies its free cells against; null when the operation
        //has none, which leaves every free cell FREE. Set by whoever builds the operation's geometry view.
        public SolidRegions Solids { get; set; }

        #endregion

        #region Constructors

        public TriangleStorage(Triangle[] _Triangles, GraphConstructionProgress _Progress = null)
        {
            if (_Triangles == null || _Triangles.Length == 0)
            {
                m_Triangles     = Array.Empty<Triangle>();
                m_Nodes         = Array.Empty<BvhNode>();
                m_NodeCount     = 0;
                m_TriangleCount = 0;
                return;
            }

            m_Triangles     = (Triangle[]) _Triangles.Clone();
            m_TriangleCount = m_Triangles.Length;

            BuildBvh();

            _Progress?.SetTrianglesStorageProgress(m_TriangleCount, m_TriangleCount);
        }

        #endregion

        #region Public methods

        public bool IsIntersect(Bounds _Bounds)
        {
            if (m_NodeCount == 0)
                return false;

            return IsIntersectTraversal(_Bounds);
        }

        public List<Triangle> GetIntersectedTriangles(Bounds _Bounds)
        {
            List<Triangle> result = new List<Triangle>();

            CollectIntersectedTriangles(_Bounds, result);

            return result;
        }

        //The allocation-free form: appends every triangle crossing _Bounds to a caller-owned collection.
        public void CollectIntersectedTriangles(Bounds _Bounds, ICollection<Triangle> _Result)
        {
            if (m_NodeCount == 0)
                return;

            GetIntersectedTraversal(_Bounds, _Result);
        }

        #endregion

        #region BVH Construction

        void BuildBvh()
        {
            BvhBuildStrategy strategy = SelectBuildStrategy();
            m_NodeCount = 0;

            if (strategy == BvhBuildStrategy.MORTON && !BuildMorton())
                strategy = BvhBuildStrategy.SAH;

            if (strategy == BvhBuildStrategy.SAH)
            {
                if (m_Centroids == null || m_Centroids.Length < m_TriangleCount)
                {
                    m_Centroids = new Vector3[m_Triangles.Length];
                    m_Mins      = new Vector3[m_Triangles.Length];
                    m_Maxs      = new Vector3[m_Triangles.Length];
                }

                if (m_Nodes == null || m_Nodes.Length < 2 * m_TriangleCount)
                    m_Nodes = new BvhNode[2 * m_Triangles.Length];

                for (int i = 0; i < m_TriangleCount; i++)
                {
                    Bounds bounds = m_Triangles[i].Bounds;

                    m_Mins[i]      = bounds.min;
                    m_Maxs[i]      = bounds.max;
                    m_Centroids[i] = bounds.center;
                }

                BuildRecursive(0, m_TriangleCount);
            }

            m_Centroids        = null;
            m_Mins             = null;
            m_Maxs             = null;
            m_MortonPrimitives = null;
            m_MortonReorder    = null;
        }

        int BuildRecursive(int _Start, int _End)
        {
            int nodeIndex = Interlocked.Increment(ref m_NodeCount) - 1;
            int count     = _End                                   - _Start;

            //the node's box and the centroids' box in one pass over the flat arrays
            Vector3 nodeMin     = m_Mins[_Start];
            Vector3 nodeMax     = m_Maxs[_Start];
            Vector3 centroidMin = m_Centroids[_Start];
            Vector3 centroidMax = centroidMin;

            for (int i = _Start + 1; i < _End; i++)
            {
                nodeMin     = Vector3.Min(nodeMin, m_Mins[i]);
                nodeMax     = Vector3.Max(nodeMax, m_Maxs[i]);
                centroidMin = Vector3.Min(centroidMin, m_Centroids[i]);
                centroidMax = Vector3.Max(centroidMax, m_Centroids[i]);
            }

            //Create leaf if few enough triangles
            if (count <= MAX_LEAF_TRIANGLES)
            {
                m_Nodes[nodeIndex] = new BvhNode
                {
                    BoundsMin    = nodeMin,
                    BoundsMax    = nodeMax,
                    StartOrLeft  = _Start,
                    CountOrRight = count
                };

                return nodeIndex;
            }

            //Find best split via SAH with stack-allocated bins
            int   bestAxis     = -1;
            int   bestSplitBin = -1;
            float bestCost     = float.MaxValue;
            float leafCost     = INTERSECTION_COST * count;

            Vector3 centroidExtent = centroidMax - centroidMin;

            //Adaptive bin count: fewer bins for small subtrees where extra granularity has no impact
            int sahBinCount = count >= ADAPTIVE_BIN_THRESHOLD ? SAH_BIN_COUNT_LARGE : SAH_BIN_COUNT_SMALL;

            //SAH bin evaluation per axis — the bins as min/max pairs on the stack, no Bounds conversions
            Span<Vector3> binMin   = stackalloc Vector3[SAH_BIN_COUNT_LARGE];
            Span<Vector3> binMax   = stackalloc Vector3[SAH_BIN_COUNT_LARGE];
            Span<int>     binCount = stackalloc int[SAH_BIN_COUNT_LARGE];
            Span<float>   leftArea = stackalloc float[SAH_BIN_COUNT_LARGE - 1];
            Span<int>     leftCnt  = stackalloc int[SAH_BIN_COUNT_LARGE   - 1];

            float nodeSa = NodeSurfaceArea(nodeMin, nodeMax);

            for (int axis = 0; axis < 3; axis++)
            {
                float extent = centroidExtent[axis];

                if (extent < 1e-6f)
                    continue;

                for (int i = 0; i < sahBinCount; i++)
                {
                    binCount[i] = 0;
                    binMin[i]   = new Vector3(float.MaxValue, float.MaxValue, float.MaxValue);
                    binMax[i]   = new Vector3(float.MinValue, float.MinValue, float.MinValue);
                }

                float invExtent = sahBinCount / extent;
                float axisMin   = centroidMin[axis];

                //Assign triangles to bins using pre-computed centroids
                for (int i = _Start; i < _End; i++)
                {
                    float centroid = m_Centroids[i][axis];
                    int   bin      = Math.Min((int) ((centroid - axisMin) * invExtent), sahBinCount - 1);

                    binMin[bin] = Vector3.Min(binMin[bin], m_Mins[i]);
                    binMax[bin] = Vector3.Max(binMax[bin], m_Maxs[i]);
                    binCount[bin]++;
                }

                //Left sweep: the area and count of everything left of each split
                Vector3 runningMin   = new Vector3(float.MaxValue, float.MaxValue, float.MaxValue);
                Vector3 runningMax   = new Vector3(float.MinValue, float.MinValue, float.MinValue);
                int     runningCount = 0;

                for (int i = 0; i < sahBinCount - 1; i++)
                {
                    if (binCount[i] > 0)
                    {
                        runningMin = Vector3.Min(runningMin, binMin[i]);
                        runningMax = Vector3.Max(runningMax, binMax[i]);
                    }

                    runningCount += binCount[i];
                    leftArea[i]  =  runningCount > 0 ? NodeSurfaceArea(runningMin, runningMax) : 0f;
                    leftCnt[i]   =  runningCount;
                }

                //Right sweep + cost evaluation
                Vector3 rightMin   = new Vector3(float.MaxValue, float.MaxValue, float.MaxValue);
                Vector3 rightMax   = new Vector3(float.MinValue, float.MinValue, float.MinValue);
                int     rightCount = 0;

                for (int i = sahBinCount - 1; i >= 1; i--)
                {
                    if (binCount[i] > 0)
                    {
                        rightMin = Vector3.Min(rightMin, binMin[i]);
                        rightMax = Vector3.Max(rightMax, binMax[i]);
                    }

                    rightCount += binCount[i];

                    int splitIndex = i - 1;
                    int lCount     = leftCnt[splitIndex];

                    if (lCount == 0 || rightCount == 0)
                        continue;

                    float cost = TRAVERSAL_COST +
                                 INTERSECTION_COST * (leftArea[splitIndex] * lCount + NodeSurfaceArea(rightMin, rightMax) * rightCount) / nodeSa;

                    if (cost < bestCost)
                    {
                        bestCost     = cost;
                        bestAxis     = axis;
                        bestSplitBin = i;
                    }
                }
            }

            //If no good split found or leaf is cheaper, make a leaf
            if (bestAxis == -1 || bestCost >= leafCost)
            {
                m_Nodes[nodeIndex] = new BvhNode
                {
                    BoundsMin    = nodeMin,
                    BoundsMax    = nodeMax,
                    StartOrLeft  = _Start,
                    CountOrRight = count
                };

                return nodeIndex;
            }

            //Partition triangles and centroids around the best split
            float splitAxisMin   = centroidMin[bestAxis];
            float splitInvExtent = sahBinCount / centroidExtent[bestAxis];

            int mid = Partition(_Start, _End, bestAxis, splitAxisMin, splitInvExtent, bestSplitBin, sahBinCount);

            if (mid == _Start || mid == _End)
                mid = _Start + count / 2;

            //Build children — parallelize if subtrees are large enough
            int leftChild, rightChild;

            if (count >= PARALLEL_THRESHOLD)
            {
                int rightChildCapture = 0;

                Task rightTask = Task.Run(() =>
                        {
                            rightChildCapture = BuildRecursive(mid, _End);
                        }
                    );

                leftChild = BuildRecursive(_Start, mid);
                rightTask.Wait();
                rightChild = rightChildCapture;
            }
            else
            {
                leftChild  = BuildRecursive(_Start, mid);
                rightChild = BuildRecursive(mid, _End);
            }

            m_Nodes[nodeIndex] = new BvhNode
            {
                BoundsMin    = nodeMin,
                BoundsMax    = nodeMax,
                StartOrLeft  = leftChild,
                CountOrRight = -rightChild
            };

            return nodeIndex;
        }

        int Partition(int _Start, int _End, int _Axis, float _AxisMin, float _InvExtent, int _SplitBin, int _BinCount)
        {
            int left  = _Start;
            int right = _End - 1;

            while (left <= right)
            {
                float centroid = m_Centroids[left][_Axis];
                int   bin      = Math.Min((int) ((centroid - _AxisMin) * _InvExtent), _BinCount - 1);

                if (bin < _SplitBin)
                {
                    left++;
                }
                else
                {
                    //Swap the triangle with its centroid and box
                    (m_Triangles[left], m_Triangles[right]) = (m_Triangles[right], m_Triangles[left]);
                    (m_Centroids[left], m_Centroids[right]) = (m_Centroids[right], m_Centroids[left]);
                    (m_Mins[left], m_Mins[right])           = (m_Mins[right], m_Mins[left]);
                    (m_Maxs[left], m_Maxs[right])           = (m_Maxs[right], m_Maxs[left]);

                    right--;
                }
            }

            return left;
        }

        static float NodeSurfaceArea(Vector3 _Min, Vector3 _Max)
        {
            float sx = _Max.x - _Min.x;
            float sy = _Max.y - _Min.y;
            float sz = _Max.z - _Min.z;
            return 2.0f * (sx * sy + sx * sz + sy * sz);
        }

        #endregion

        #region BVH Traversal

        bool IsIntersectTraversal(Bounds _Bounds)
        {
            Span<int> stack    = stackalloc int[64];
            int       stackPtr = 0;
            stack[stackPtr++] = 0;

            Vector3 queryMin = _Bounds.min;
            Vector3 queryMax = _Bounds.max;
            float   queryCx  = (queryMin.x + queryMax.x) * 0.5f;
            float   queryCy  = (queryMin.y + queryMax.y) * 0.5f;
            float   queryCz  = (queryMin.z + queryMax.z) * 0.5f;

            while (stackPtr > 0)
            {
                int         current = stack[--stackPtr];
                ref BvhNode node    = ref m_Nodes[current];

                if (queryMax.x < node.BoundsMin.x || queryMin.x > node.BoundsMax.x ||
                    queryMax.y < node.BoundsMin.y || queryMin.y > node.BoundsMax.y ||
                    queryMax.z < node.BoundsMin.z || queryMin.z > node.BoundsMax.z)
                    continue;

                if (node.IsLeaf)
                {
                    int end = node.TriangleStart + node.TriangleCount;

                    for (int i = node.TriangleStart; i < end; i++)
                    {
                        if (m_Triangles[i].Intersects(_Bounds))
                            return true;
                    }
                }
                else
                {
                    int left  = node.LeftChild;
                    int right = node.RightChild;

                    //Ordered traversal: push farther child first so closer child is popped first.
                    //Closer child is more likely to contain a hit, enabling earlier exit.
                    ref BvhNode leftNode  = ref m_Nodes[left];
                    ref BvhNode rightNode = ref m_Nodes[right];

                    float dx         = queryCx - (leftNode.BoundsMin.x + leftNode.BoundsMax.x) * 0.5f;
                    float dy         = queryCy - (leftNode.BoundsMin.y + leftNode.BoundsMax.y) * 0.5f;
                    float dz         = queryCz - (leftNode.BoundsMin.z + leftNode.BoundsMax.z) * 0.5f;
                    float leftDistSq = dx                                                      * dx + dy * dy + dz * dz;

                    dx = queryCx - (rightNode.BoundsMin.x + rightNode.BoundsMax.x) * 0.5f;
                    dy = queryCy - (rightNode.BoundsMin.y + rightNode.BoundsMax.y) * 0.5f;
                    dz = queryCz - (rightNode.BoundsMin.z + rightNode.BoundsMax.z) * 0.5f;
                    float rightDistSq = dx * dx + dy * dy + dz * dz;

                    if (leftDistSq <= rightDistSq)
                    {
                        stack[stackPtr++] = right;
                        stack[stackPtr++] = left;
                    }
                    else
                    {
                        stack[stackPtr++] = left;
                        stack[stackPtr++] = right;
                    }
                }
            }

            return false;
        }

        void GetIntersectedTraversal(Bounds _Bounds, ICollection<Triangle> _Result)

        {
            Span<int> stack    = stackalloc int[64];
            int       stackPtr = 0;
            stack[stackPtr++] = 0;

            Vector3 queryMin = _Bounds.min;
            Vector3 queryMax = _Bounds.max;

            while (stackPtr > 0)
            {
                int         current = stack[--stackPtr];
                ref BvhNode node    = ref m_Nodes[current];

                if (queryMax.x < node.BoundsMin.x || queryMin.x > node.BoundsMax.x ||
                    queryMax.y < node.BoundsMin.y || queryMin.y > node.BoundsMax.y ||
                    queryMax.z < node.BoundsMin.z || queryMin.z > node.BoundsMax.z)
                    continue;

                if (node.IsLeaf)
                {
                    int end = node.TriangleStart + node.TriangleCount;

                    for (int i = node.TriangleStart; i < end; i++)
                    {
                        if (m_Triangles[i].Intersects(_Bounds))
                            _Result.Add(m_Triangles[i]);
                    }
                }
                else
                {
                    stack[stackPtr++] = node.LeftChild;
                    stack[stackPtr++] = node.RightChild;
                }
            }
        }

        #endregion

    }
}