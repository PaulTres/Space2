using Nav3D.Common;
using Nav3D.Obstacles;
using Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.Elements;
using System;
using System.Collections.Generic;
using UnityEngine;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree
{
    //Where an endpoint inside an obstacle goes: the nearest point of free space, found by growing a box around it.
    partial class NavTree
    {
        #region Nested types

        //A free leaf's best offer for the query point, or false when the leaf has none (a radius-aware query whose body
        //fits nowhere in it).
        public delegate bool NavigableCandidate(Leaf _Leaf, Vector3 _Point, out Vector3 _Candidate);

        #endregion

        #region Constants

        //how many free leaves one query may weigh before it gives up: the answer for a point inside a body lies just
        //past its crust, so a query that has seen this many has left every plausible answer behind
        const int NEAREST_NAVIGABLE_MAX_LEAVES = 20000;

        //the root sizes the box may grow to — past the half-width of any body a click can land in
        const float NEAREST_NAVIGABLE_MAX_ROOTS = 8f;

        #endregion

        #region Public methods

        //The nearest point of FREE space to _Point — crossed and interior cells are not free — and the leaf holding it.
        //Every free leaf within ±reach offers its best point; once the best offer lies within reach, nothing outside the
        //box can beat it, otherwise the box doubles. _Candidate null offers the leaf's point nearest to _Point (a hair
        //inside, so the point resolves to that leaf); the radius-aware lane passes one that demands clearance. Caller
        //holds the read lock.
        public bool TryFindNearestNavigablePointInternal(Vector3 _Point, NavigableCandidate _Candidate, out Vector3 _Found, out Leaf _FoundLeaf)
        {
            return TryFindNearestNavigablePointInternal(_Point, m_Octree.RootSize * NEAREST_NAVIGABLE_MAX_ROOTS, _Candidate, out _Found, out _FoundLeaf);
        }

        //_Accept re-checks an offer a sealed volume pushed out of itself (the radius-aware lane: the body still fits)
        public bool TryFindNearestNavigablePointInternal(
                Vector3            _Point,
                float              _MaxDistance,
                NavigableCandidate _Candidate,
                out Vector3        _Found,
                out Leaf           _FoundLeaf,
                Predicate<Vector3> _Accept = null
            )
        {
            _Candidate ??= NearestPointInLeaf;

            SealedVolumeSnapshot sealedVolumes = SealedVolumeSnapshot.Current;

            _Found     = _Point;
            _FoundLeaf = null;

            float bestSqr = float.PositiveInfinity;
            float reach   = m_Octree.MinBucketSizeReal;

            HashSet<Leaf> offered = new HashSet<Leaf>();
            List<Leaf>    inBox   = new List<Leaf>();

            while (true)
            {
                inBox.Clear();
                CollectFreeLeaves(new Bounds(_Point, Vector3.one * (2f * reach)), inBox);

                //nearest leaves first, and none whose box already lies farther than the best offer: an offer lands in
                //its leaf or is pushed from it, so the box distance is where it starts
                inBox.Sort((_A, _B) => _A.Bounds.SqrDistance(_Point).CompareTo(_B.Bounds.SqrDistance(_Point)));

                foreach (Leaf leaf in inBox)
                {
                    if (leaf.Bounds.SqrDistance(_Point) >= bestSqr)
                        break;

                    if (!offered.Add(leaf) || !_Candidate(leaf, _Point, out Vector3 candidate))
                        continue;

                    //an authored sealed volume is no answer either: out of it by the shortest way, and still free
                    if (!sealedVolumes.IsEmpty && sealedVolumes.Contains(candidate))
                    {
                        candidate = sealedVolumes.PushOut(candidate, m_Octree.MinBucketSizeReal * 0.01f);

                        Leaf landing = GetClosestOrEmbracingLeaf(candidate);

                        if (sealedVolumes.Contains(candidate) || (landing != null && landing.Occupied) || (_Accept != null && !_Accept(candidate)))
                            continue;
                    }

                    float distanceSqr = (candidate - _Point).sqrMagnitude;

                    if (distanceSqr >= bestSqr)
                        continue;

                    bestSqr    = distanceSqr;
                    _Found     = candidate;
                    _FoundLeaf = leaf;
                }

                if (bestSqr <= reach * reach || reach >= _MaxDistance || offered.Count >= NEAREST_NAVIGABLE_MAX_LEAVES)
                    break;

                reach = Mathf.Min(reach * 2f, _MaxDistance);
            }

            return _FoundLeaf != null;
        }

        #endregion

        #region Service methods

        bool NearestPointInLeaf(Leaf _Leaf, Vector3 _Point, out Vector3 _Candidate)
        {
            _Candidate = ClampIntoLeaf(_Leaf, _Point);

            return true;
        }

        //_Point clamped into the leaf, kept a hair inside its faces so the answer resolves to this very leaf
        public Vector3 ClampIntoLeaf(Leaf _Leaf, Vector3 _Point)
        {
            Bounds bounds = _Leaf.Bounds;
            float  inset  = Mathf.Min(bounds.extents.x * 0.5f, m_Octree.MinBucketSizeReal * 0.01f);

            Vector3 min = bounds.min + Vector3.one * inset;
            Vector3 max = bounds.max - Vector3.one * inset;

            return Vector3.Min(Vector3.Max(_Point, min), max);
        }

        void CollectFreeLeaves(Bounds _Box, List<Leaf> _Result)
        {
            UtilsSpatialGrid.GetCrossedBucketsRange(_Box, m_Octree.RootSize, out Vector3Int minIndex, out Vector3Int maxIndex);

            for (int x = minIndex.x; x <= maxIndex.x; x++)
            for (int y = minIndex.y; y <= maxIndex.y; y++)
            for (int z = minIndex.z; z <= maxIndex.z; z++)
            {
                if (m_RootsGrid.TryGetNode(0, new Vector3Int(x, y, z), out Node root))
                    CollectFreeLeaves(root, _Box, _Result);
            }
        }

        static void CollectFreeLeaves(Node _Node, Bounds _Box, List<Leaf> _Result)
        {
            if (!_Node.Bounds.Intersects(_Box))
                return;

            if (_Node is Leaf leaf)
            {
                if (!leaf.Occupied)
                    _Result.Add(leaf);

                return;
            }

            foreach (Node child in ((Fork) _Node).ChildrenMap.Values)
                CollectFreeLeaves(child, _Box, _Result);
        }

        #endregion
    }
}
