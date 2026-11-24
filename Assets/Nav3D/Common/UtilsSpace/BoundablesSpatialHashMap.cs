using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Concurrent;
using System.Linq;
using UnityEngine;

namespace Nav3D.Common
{
    /// <summary>
    /// Spatial storage for objects described by a prismatic boundary (Bounds).
    /// The storage consists of a spatial bucket grid.Each stored object is located in the buckets that it intersects.
    /// There are connections: object -> intersecting buckets and bucket-> intersecting objects.
    /// The size of storage buckets is recalculated each time a new object is added and is taken equal to the largest size of the object's Bounds.
    /// </summary>
    /// <typeparam name="T">Object with Bounds</typeparam>
    partial class BoundablesSpatialHashMap<T> : IEnumerable<T> where T : IBoundable
    {
        #region Attributes

        float                                                           m_BucketSize = 0;
        readonly ConcurrentDictionary<T, List<Vector3Int>>              m_Boundables = new ConcurrentDictionary<T, List<Vector3Int>>();
        readonly ConcurrentDictionary<Vector3Int, ConcurrentHashSet<T>> m_Buckets    = new ConcurrentDictionary<Vector3Int, ConcurrentHashSet<T>>();

        #endregion

        #region Constructors

        public BoundablesSpatialHashMap()
        {
        }

        #endregion

        #region Public methods

        /// <summary>
        /// Returns the set of all boundables that cross the given bounds.
        /// </summary>
        /// <param name="_Bounds">The given bounds</param>
        public bool TryGetCrossingBoundables(Bounds _Bounds, out HashSet<T> _CrossingBoundables, Predicate<T> _Predicate = null)
        {
            HashSet<T> result = new HashSet<T>();

            CollectCrossingBoundables(_Bounds, result, _Predicate);

            _CrossingBoundables = result;

            return result.Count > 0;
        }

        //The allocation-free form: adds every boundable crossing _Bounds to a caller-owned set (not cleared here)
        //and reports whether any was found. The bucket range is the range between the min and max corner — the two
        //corners the box already exposes — where this used to build all eight corners into an array, project them
        //through LINQ into a list and take six LINQ extrema, plus a LINQ filter per bucket: ~6 KB of garbage per
        //call, and the octree asks it per footprint root per operation and per read query.
        //The bucket walk and the intersection test are unchanged.
        public bool CollectCrossingBoundables(Bounds _Bounds, HashSet<T> _Result, Predicate<T> _Predicate = null)
        {
            Vector3Int minBucket = UtilsSpatialGrid.GetBucketIndex(_Bounds.min, m_BucketSize);
            Vector3Int maxBucket = UtilsSpatialGrid.GetBucketIndex(_Bounds.max, m_BucketSize);

            bool any = false;

            for (int x = minBucket.x; x <= maxBucket.x; x++)
                for (int y = minBucket.y; y <= maxBucket.y; y++)
                    for (int z = minBucket.z; z <= maxBucket.z; z++)
                    {
                        if (!m_Buckets.TryGetValue(new Vector3Int(x, y, z), out ConcurrentHashSet<T> boundables))
                            continue;

                        foreach (T boundable in boundables)
                        {
                            if (!boundable.Bounds.Intersects(_Bounds))
                                continue;

                            if (_Predicate != null && !_Predicate(boundable))
                                continue;

                            _Result.Add(boundable);

                            any = true;

                        }
                    }

            return any;
        }

        
        public bool HasCrossingBoundables(Bounds _Bounds)
        {
            Vector3[]        corners       = _Bounds.GetCornerPoints();
            List<Vector3Int> cornerBuckets = corners.Select(_Corner => UtilsSpatialGrid.GetBucketIndex(_Corner, m_BucketSize)).ToList();

            int xMin = cornerBuckets.Min(_Bucket => _Bucket.x);
            int yMin = cornerBuckets.Min(_Bucket => _Bucket.y);
            int zMin = cornerBuckets.Min(_Bucket => _Bucket.z);
            int xMax = cornerBuckets.Max(_Bucket => _Bucket.x);
            int yMax = cornerBuckets.Max(_Bucket => _Bucket.y);
            int zMax = cornerBuckets.Max(_Bucket => _Bucket.z);

            for (int x = xMin; x <= xMax; x++)
                for (int y = yMin; y <= yMax; y++)
                    for (int z = zMin; z <= zMax; z++)
                    {
                        if (m_Buckets.TryGetValue(new Vector3Int(x, y, z), out ConcurrentHashSet<T> boundables) &&
                            boundables.Any(_Boundable => _Boundable.Bounds.Intersects(_Bounds)))
                            return true;
                    }

            return false;
        }

        public void Add(T _Boundable)
        {
            Bounds bounds        = _Boundable.Bounds;
            float  maxBoundsSize = bounds.GetMaxSize();

            if (maxBoundsSize > m_BucketSize)
            {
                List<T> boundables = new List<T>(m_Boundables.Keys);

                Initialize(maxBoundsSize, boundables);
            }

            Vector3[] corners = bounds.GetCornerPoints();

            for (int i = 0; i < corners.Length; i++)
            {
                AddToBucket(_Boundable, UtilsSpatialGrid.GetBucketIndex(corners[i], m_BucketSize));
            }
        }

        //Drops every entry. Not atomic across the two maps: for a map nobody reads any more (a dying epoch's)
        public void Clear()
        {
            m_Boundables.Clear();
            m_Buckets.Clear();
        }

        public void Remove(T _Boundable)
        {
            RemoveFromBuckets(_Boundable);

            CheckMaxBucketSize();
        }

        public IEnumerator<T> GetEnumerator()
        {
            return m_Boundables.Keys.GetEnumerator();
        }
        IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();
        
        #endregion

        #region Service methods

        void CheckMaxBucketSize()
        {
            float maxBoundsSize = 0;

            foreach (T boundable in m_Boundables.Keys)
            {
                float boundableMaxSize = boundable.Bounds.GetMaxSize();

                if (boundableMaxSize > maxBoundsSize)
                    maxBoundsSize = boundableMaxSize;
            }

            if (maxBoundsSize < m_BucketSize)
            {
                List<T> boundables = new List<T>(m_Boundables.Keys);

                Initialize(maxBoundsSize, boundables);
            }
        }

        void Initialize(float _BucketSize, IEnumerable<T> _Boundables = null)
        {
            m_Boundables.Clear();
            m_Buckets.Clear();

            m_BucketSize = _BucketSize;

            if (_Boundables.IsNullOrEmpty())
                return;

            foreach (T boundable in _Boundables)
                Add(boundable);
        }

        void AddToBucket(T _Boundable, Vector3Int _Bucket)
        {
            if (m_Buckets.TryGetValue(_Bucket, out ConcurrentHashSet<T> set))
            {
                set.TryAdd(_Boundable);
            }
            else
            {
                ConcurrentHashSet<T> newHashSet = new ConcurrentHashSet<T>();
                newHashSet.TryAdd(_Boundable);

                m_Buckets.TryAdd(_Bucket, newHashSet);
            }

            m_Boundables.GetOrAdd(_Boundable).Add(_Bucket);
        }

        void RemoveFromBuckets(T _Boundable)
        {
            if (!m_Boundables.TryGetValue(_Boundable, out List<Vector3Int> buckets))
                return;

            foreach (Vector3Int bucket in buckets)
            {
                if (m_Buckets.TryGetValue(bucket, out ConcurrentHashSet<T> bucketSet))
                {
                    bucketSet.TryRemove(_Boundable);

                    if (!bucketSet.Any())
                        m_Buckets.TryRemove(bucket, out _);
                }
            }

            m_Boundables.TryRemove(_Boundable, out List<Vector3Int> _);
        }

        #endregion
    }
}