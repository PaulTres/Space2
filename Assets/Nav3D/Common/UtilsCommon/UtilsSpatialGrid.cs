using Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.Elements;
using System.Collections.Generic;
using UnityEngine;

namespace Nav3D.Common
{
    static class UtilsSpatialGrid
    {
        #region Attributes

        static Dictionary<Vector3Int, ForkChildOctIndex[]> s_ChildrenIndicesByDirection;
        static Vector3Int[]                                s_FacesDirections;

        #endregion

        #region Properties

        public static Dictionary<Vector3Int, ForkChildOctIndex[]> ChildrenIndicesByDirection =>
            s_ChildrenIndicesByDirection ??= new Dictionary<Vector3Int, ForkChildOctIndex[]>
            {
                { Vector3Int.forward, new[] { ForkChildOctIndex.I001, ForkChildOctIndex.I011, ForkChildOctIndex.I101, ForkChildOctIndex.I111 } },
                { Vector3Int.back, new[] { ForkChildOctIndex.I000, ForkChildOctIndex.I010, ForkChildOctIndex.I100, ForkChildOctIndex.I110 } },
                { Vector3Int.up, new[] { ForkChildOctIndex.I010, ForkChildOctIndex.I011, ForkChildOctIndex.I110, ForkChildOctIndex.I111 } },
                { Vector3Int.down, new[] { ForkChildOctIndex.I000, ForkChildOctIndex.I001, ForkChildOctIndex.I100, ForkChildOctIndex.I101 } },
                { Vector3Int.right, new[] { ForkChildOctIndex.I100, ForkChildOctIndex.I101, ForkChildOctIndex.I110, ForkChildOctIndex.I111 } },
                { Vector3Int.left, new[] { ForkChildOctIndex.I000, ForkChildOctIndex.I001, ForkChildOctIndex.I010, ForkChildOctIndex.I011 } }
            };

        public static Vector3Int[] FacesDirections =>
            s_FacesDirections ??= new[]
            {
                Vector3Int.right,
                Vector3Int.left,
                Vector3Int.up,
                Vector3Int.down,
                Vector3Int.forward,
                Vector3Int.back
            };

        #endregion

        #region Public methods

        public static Vector3Int GetBucketIndex(Vector3 _Point, float _BucketSize)
        {
            return new Vector3Int(
                    Mathf.FloorToInt(_Point.x / _BucketSize),
                    Mathf.FloorToInt(_Point.y / _BucketSize),
                    Mathf.FloorToInt(_Point.z / _BucketSize)
                );
        }

        //The crossed-cell walk as a range instead of a materialized list — for the read path, where the list was a
        //per-call allocation. Every octree read query (the point lane's segment test, the volumetric capsule test,
        //the clearance fills) opens by asking which root cells its query box crosses, tens to hundreds of thousands
        //of times per pathfinding request, and GetCrossedBucketsIndices below allocates a List and an array on each
        //of those calls. The caller loops the three indices itself and allocates nothing.
        public static void GetCrossedBucketsRange(Bounds _Bounds, float _BucketSize, out Vector3Int _MinIndex, out Vector3Int _MaxIndex)
        {
            _MinIndex = GetBucketIndex(_Bounds.min, _BucketSize);
            _MaxIndex = GetBucketIndex(_Bounds.max, _BucketSize);
        }

        public static Vector3Int[] GetCrossedBucketsIndices(Bounds _Bounds, float _BucketSize)
        {
            Vector3Int minIndex = GetBucketIndex(_Bounds.min, _BucketSize);
            Vector3Int maxIndex = GetBucketIndex(_Bounds.max, _BucketSize);

            List<Vector3Int> indices = new List<Vector3Int>();

            for (int x = minIndex.x; x <= maxIndex.x; x++)
                for (int y = minIndex.y; y <= maxIndex.y; y++)
                    for (int z = minIndex.z; z <= maxIndex.z; z++)
                    {
                        indices.Add(new Vector3Int(x, y, z));
                    }

            return indices.ToArray();
        }

        public static Bounds GetBucketBounds(Vector3 _InsidePoint, float _BucketSize)
        {
            return GetBucketBounds(GetBucketIndex(_InsidePoint, _BucketSize), _BucketSize);
        }

        public static Bounds GetBucketBounds(Vector3Int _Index, float _BucketSize)
        {
            return new Bounds(
                    new Vector3(
                            _Index.x * _BucketSize + 0.5f * _BucketSize,
                            _Index.y * _BucketSize + 0.5f * _BucketSize,
                            _Index.z * _BucketSize + 0.5f * _BucketSize
                        ),
                    new Vector3(_BucketSize, _BucketSize, _BucketSize)
                );
        }

        public static Vector3Int GetParentIndex(Vector3Int _Index)
        {
            return new Vector3Int(
                    Mathf.FloorToInt(_Index.x * 0.5f),
                    Mathf.FloorToInt(_Index.y * 0.5f),
                    Mathf.FloorToInt(_Index.z * 0.5f)
                );
        }

        //The eight children of a bucket, as (OctIndex, GridIndex) in ascending flag order — I000, I001, I010, I011,
        //I100, I101, I110, I111 — which is the order every consumer (construction, the serialization's child-ID
        //list, the cell descents) has always seen. allocation-FREE:
        //this used to return a fresh 8-tuple array, and it is asked once per fork constructed and once per level of
        //every cell descent an operation makes — a per-fork transient of the exact class Phase 4 removed from
        //ProduceChildren, left behind in the helper it calls. The struct below computes each entry on demand from
        //the parent index and a fixed offset table; `foreach`, indexing and Length all work as they did on the array.
        public static ChildrenIndices GetChildrenIndices(Vector3Int _BucketIndex)
        {
            return new ChildrenIndices(_BucketIndex);
        }

        static readonly ForkChildOctIndex[] s_ChildOctIndicesInOrder =
        {
            ForkChildOctIndex.I000, ForkChildOctIndex.I001, ForkChildOctIndex.I010, ForkChildOctIndex.I011,
            ForkChildOctIndex.I100, ForkChildOctIndex.I101, ForkChildOctIndex.I110, ForkChildOctIndex.I111,
        };

        static readonly Vector3Int[] s_ChildOffsetsInOrder =
        {
            new Vector3Int(0, 0, 0), new Vector3Int(0, 0, 1), new Vector3Int(0, 1, 0), new Vector3Int(0, 1, 1),
            new Vector3Int(1, 0, 0), new Vector3Int(1, 0, 1), new Vector3Int(1, 1, 0), new Vector3Int(1, 1, 1),
        };

        public readonly struct ChildrenIndices
        {
            public const int COUNT = 8;

            readonly Vector3Int m_ChildBaseIndex;

            public ChildrenIndices(Vector3Int _BucketIndex)
            {
                m_ChildBaseIndex = new Vector3Int(_BucketIndex.x * 2, _BucketIndex.y * 2, _BucketIndex.z * 2);
            }

            public int Length => COUNT;

            public (ForkChildOctIndex OctIndex, Vector3Int GridIndex) this[int _Index] =>
                (s_ChildOctIndicesInOrder[_Index], m_ChildBaseIndex + s_ChildOffsetsInOrder[_Index]);

            public Enumerator GetEnumerator()
            {
                return new Enumerator(this);
            }

            public struct Enumerator
            {
                readonly ChildrenIndices m_Children;

                int m_Index;

                public Enumerator(ChildrenIndices _Children)
                {
                    m_Children = _Children;
                    m_Index    = -1;
                }

                public (ForkChildOctIndex OctIndex, Vector3Int GridIndex) Current => m_Children[m_Index];

                public bool MoveNext()
                {
                    return ++m_Index < COUNT;
                }
            }
        }

        #endregion
    }
}
