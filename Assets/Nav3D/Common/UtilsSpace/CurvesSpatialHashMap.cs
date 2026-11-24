using System.Collections.Generic;
using System.Threading;
using UnityEngine;
using System.Linq;

namespace Nav3D.Common
{
    //Spatial hash map for curves (paths). One lock guards every operation because the invariant spans two containers
    //(curve → its buckets, bucket → its curves) and writers run on several threads; per-container concurrent
    //collections cannot keep a cross-container invariant — a raced double Register once leaked buckets, and
    //Unregister's cleanup dropped a bucket a concurrent Register had just filled, so an obstacle change there no
    //longer re-pathed that path. Registrations are rare, so a plain lock is the right tool.
    partial class CurvesSpatialHashMap<T> where T : ICurve
    {
        #region Constants


        #endregion

        #region Attributes

        float m_BucketSize;

        readonly object m_Lock = new object();

        readonly Dictionary<T, HashSet<Vector3Int>> m_Curves  = new Dictionary<T, HashSet<Vector3Int>>();
        readonly Dictionary<Vector3Int, HashSet<T>> m_Buckets = new Dictionary<Vector3Int, HashSet<T>>();

        //Scratch for TryGetIntersectingCurves: the curves it has already answered for during this query. Guarded by
        //m_Lock like everything else here, so only one query ever uses it at a time.
        readonly HashSet<T> m_Tested = new HashSet<T>();

        #endregion

        #region Constructors

        public CurvesSpatialHashMap(float _BucketSize)
        {
            m_BucketSize = _BucketSize;
        }

        #endregion

        #region Public methods

        //The bucket set is computed OFF the lock. Voxelizing every segment of a long path and inflating
        //the result by the path's thickness is the expensive half of a registration — measured as 18–20 ms holds on
        //Skyrift's 300 u glider tours — and it is a pure function of the curve, so nothing about it needs the lock.
        //What still runs under the lock is the swap: evict the old registration, record the new set, insert into the
        //buckets. The atomicity Update relies on (no reader sees a half-registered curve) is that swap, and it stands.
        //A path re-registered concurrently by two continuations computes two sets and swaps them in turn; the last
        //swap wins and both are correct for the trajectory they were computed from — the same as before, when the
        //two registrations queued on the lock.
        public void Register(T _Curve)
        {
            HashSet<Vector3Int> buckets = ComputeBuckets(_Curve);

            EnterLock();

            try
            {
                UnregisterUnderLock(_Curve);

                m_Curves[_Curve] = buckets;

                foreach (Vector3Int bucket in buckets)
                    m_Buckets.GetOrAdd(bucket).Add(_Curve);
            }
            finally
            {
                Monitor.Exit(m_Lock);
            }
        }

        public void Update(T _Curve)
        {
            //Register is idempotent (it first evicts the curve's previous registration), and the whole
            //unregister+re-register runs under one lock so no reader/writer can observe the intermediate state.
            Register(_Curve);
        }

        public void Unregister(T _Curve)
        {
            EnterLock();

            try
            {
                UnregisterUnderLock(_Curve);
            }
            finally
            {
                Monitor.Exit(m_Lock);
            }
        }

        //A curve spans as many buckets as its polyline crosses, so a query volume covering part of a curve finds that
        //same curve in every one of those buckets. Intersects is a pure function of (curve, bounds) — an O(segments)
        //capsule sweep over the whole polyline — so testing it once per bucket is pure repeat work: measured at
        //598 calls for 39 distinct curves (15.2x) per obstacle operation in the RuntimeObstacles showcase, which is
        //81 % of a handler that runs on the main thread every time an obstacle changes
        //. m_Tested collapses that to one call per distinct curve — the same
        //answer, by construction, since neither the curve nor the bounds changes between buckets.
        //Reused across calls (cleared per query) rather than allocated: this runs under the map's lock, so there is
        //exactly one query in flight, and the set otherwise churns a few hundred entries per obstacle operation.
        public bool TryGetIntersectingCurves(Bounds _Bounds, out HashSet<T> _IntersectingCurves)
        {
            HashSet<T> result = new HashSet<T>();

            EnterLock();

            try
            {
                CollectIntersectingUnderLock(_Bounds, result);

                m_Tested.Clear();
            }
            finally
            {
                Monitor.Exit(m_Lock);
            }

            _IntersectingCurves = result;

            return _IntersectingCurves.Any();
        }

        //Curves crossing any of the volumes, in one query. An octree operation notifies its footprint piece by piece
        //(a body in disjoint parts must not claim the space between them changed),
        //and the caller wants one answer for the whole operation: one lock acquisition, one result set, so a curve
        //crossing two volumes is returned once and its owner re-pathed once.
        //NOTE what is and is not shared across the volumes: the result is (a curve already known to cross needs no
        //further test — that is the cross-volume dedup), the m_Tested set is not (it means "already answered for this
        //volume", and a curve that missed volume A must still be tested against volume B — sharing it would silently
        //drop re-paths).
        public bool TryGetIntersectingCurves(IReadOnlyList<Bounds> _Volumes, out HashSet<T> _IntersectingCurves)
        {
            HashSet<T> result = new HashSet<T>();

            EnterLock();

            try
            {
                foreach (Bounds volume in _Volumes)
                    CollectIntersectingUnderLock(volume, result);

                m_Tested.Clear();
            }
            finally
            {
                Monitor.Exit(m_Lock);
            }

            _IntersectingCurves = result;

            return _IntersectingCurves.Any();
        }

        public void Draw()
        {
            lock (m_Lock)
            {
                foreach (Vector3Int bucket in m_Buckets.Keys)
                {
                    UtilsSpatialGrid.GetBucketBounds(bucket, m_BucketSize).Draw();
                }

                Gizmos.color = Color.red;

                foreach (KeyValuePair<T, HashSet<Vector3Int>> kvp in m_Curves)
                {
                    foreach (Segment3 segment in kvp.Key.Segments)
                        Gizmos.DrawLine(segment.Start, segment.End);
                }
            }
        }

        #endregion

        #region Service methods

        //The one lock, taken so that a CONTENDED acquisition is visible in the profiler: writers are the search
        //completion continuations on the pool (a path re-registers across every bucket its polyline crosses) and
        //readers include the main thread — the publish's crossing-paths query and every path's Dispose on an agent's
        //OnDisable — so a busy pool can hold the main thread here, and until this marker existed that wait was frame
        //time no marker named (Skyrift). The uncontended path costs one TryEnter.
        void EnterLock()
        {
            if (Monitor.TryEnter(m_Lock))
                return;

            UnityEngine.Profiling.Profiler.BeginSample("Nav3D.PathsStorageWait");

            bool onMainThread = Nav3DMainThreadCost.IsMainThread;
            long waitBegin    = onMainThread ? Nav3DMainThreadCost.Begin() : 0L;

            Monitor.Enter(m_Lock);

            if (onMainThread)
                Nav3DMainThreadCost.End(waitBegin);

            UnityEngine.Profiling.Profiler.EndSample();
        }

        //The bucket walk of one query volume, adding into a result set the caller owns. Must be called under m_Lock —
        //it reads both maps and uses the shared m_Tested scratch, which it resets for this volume (see the multi-volume
        //overload's note on why that reset is per volume and the result set is not).
        void CollectIntersectingUnderLock(Bounds _Bounds, HashSet<T> _Result)
        {
            m_Tested.Clear();

            Vector3Int minBucket = UtilsSpatialGrid.GetBucketIndex(_Bounds.min, m_BucketSize);
            Vector3Int maxBucket = UtilsSpatialGrid.GetBucketIndex(_Bounds.max, m_BucketSize);

            for (int x = minBucket.x; x <= maxBucket.x; x++)
                for (int y = minBucket.y; y <= maxBucket.y; y++)
                    for (int z = minBucket.z; z <= maxBucket.z; z++)
                    {
                        if (!m_Buckets.TryGetValue(new Vector3Int(x, y, z), out HashSet<T> curves))
                            continue;

                        foreach (T curve in curves)
                        {
                            //already crossing an earlier volume of this query — the answer cannot change
                            if (_Result.Contains(curve))
                                continue;

                            //already answered for this volume, either way — Add returns false on a repeat
                            if (!m_Tested.Add(curve))
                                continue;

                            if (curve.Intersects(_Bounds))
                                _Result.Add(curve);
                        }
                    }
        }

        //Idempotent: evicts the curve's previous registration first, so m_Curves' recorded bucket set and the curve's
        //actual m_Buckets memberships can never diverge (the old silent-TryAdd version left the curve in buckets its
        //record didn't list — an unremovable leak).
        //every bucket the curve's polyline crosses, inflated by its thickness — no shared state touched
        HashSet<Vector3Int> ComputeBuckets(T _Curve)
        {
            HashSet<Vector3Int> buckets = new HashSet<Vector3Int>();

            foreach (Segment3 segment in _Curve.Segments)
                buckets.AddRange(LineVoxelizer.GetVoxels(segment, m_BucketSize));

            // a thick (capsule) curve occupies buckets its body reaches, not just its polyline: expand the
            //polyline's bucket set by a Chebyshev ring covering the thickness, else a bounds sitting in an adjacent
            //bucket but within Thickness of the polyline never sees the curve in TryGetIntersectingCurves (a missed
            //re-path — the clearance promise silently breaks on the next obstacle change there). Over-inclusion is
            //harmless: the per-curve Intersects check decides exactly.
            float thickness = _Curve.Thickness;

            if (thickness > 0f)
            {
                int ring = Mathf.CeilToInt(thickness / m_BucketSize);

                HashSet<Vector3Int> inflated = new HashSet<Vector3Int>(buckets);

                foreach (Vector3Int bucket in buckets)
                {
                    for (int x = -ring; x <= ring; x++)
                        for (int y = -ring; y <= ring; y++)
                            for (int z = -ring; z <= ring; z++)
                                inflated.Add(new Vector3Int(bucket.x + x, bucket.y + y, bucket.z + z));
                }

                buckets = inflated;
            }

            return buckets;
        }

        void UnregisterUnderLock(T _Curve)
        {
            if (!m_Curves.TryGetValue(_Curve, out HashSet<Vector3Int> buckets))
                return;

            foreach (Vector3Int bucket in buckets)
            {
                if (m_Buckets.TryGetValue(bucket, out HashSet<T> bucketSet))
                {
                    bucketSet.Remove(_Curve);

                    if (bucketSet.Count == 0)
                        m_Buckets.Remove(bucket);
                }
            }

            m_Curves.Remove(_Curve);
        }

        #endregion
    }
}
