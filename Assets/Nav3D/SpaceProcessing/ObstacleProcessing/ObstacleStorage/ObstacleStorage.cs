using Nav3D.API;
using Nav3D.Common;
using Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree;
using Nav3D.SpaceProcessing.ObstacleProcessing.UtilityTypes;
using System;
using System.Collections.Generic;
using System.Threading;
using UnityEngine;

namespace Nav3D.Obstacles
{
    //The sole, deliberate boundary between ObstacleManager (obstacle lifecycle/orchestration) and
    //GlobalOctree (spatial storage internals). ObstacleManager must never hold or call a GlobalOctree reference
    //directly -- every access goes through here, even where a member is a pure forward with no added logic today.
    partial class ObstacleStorage
    {

        #region Attributes

        readonly int   m_OctreeLayersCount;
        readonly float m_MaxAgentRadius;

        //the wrapped octree lives here, in the shipped file — dev tooling reaches it via the DevOctree
        //accessor in the strippable DevValidation partial, never the other way around
        readonly GlobalOctree m_Octree;

        #endregion

        #region Events

        //Forwards the octree's post-publish region-changed signal (main thread). The orchestrator (ObstacleManager)
        //subscribes to re-path crossing paths and mark agents obstacle-dirty. Carries the operation's volumes one by
        //one — an obstacle in disjoint pieces must not declare the space between them changed
        //.
        public event Action<IReadOnlyList<Bounds>> OnRegionChanged
        {
            add => m_Octree.OnRegionChanged += value;
            remove => m_Octree.OnRegionChanged -= value;
        }

        #endregion

        #region Constructors

        public ObstacleStorage(float _OctreeMinBucketSize, int _OctreeLayersCount, float _MaxAgentRadius)
        {
            m_MaxAgentRadius = _MaxAgentRadius;

            m_Octree = new GlobalOctree(_OctreeLayersCount, _OctreeMinBucketSize, m_MaxAgentRadius);
        }

        #endregion

        #region Public methods

        //the epoch is over: see GlobalOctree.ReleaseWorld
        public void ReleaseWorld()
        {
            m_Octree.ReleaseWorld();
        }

        public void AddObstacle(
                Func<ObstacleInfo>       _GetObstacleInfo,
                ObstacleAdditionProgress _AdditionProgress,
                CancellationToken        _CancellationToken,
                Action                   _OnFinish = null,
                bool                     _EditMode = false
            )
        {
            m_Octree.AddObstacle(_GetObstacleInfo, _AdditionProgress, _CancellationToken, _OnFinish, _EditMode);
        }

        public bool IsBoundsCrossAnyObstacleBody(Bounds _Bounds)
        {
            return m_Octree.IsBoundsCrossAnyObstacleBody(_Bounds);
        }

        public bool IsPointInsideOccupiedOctreeBucket(Vector3 _Point)
        {
            return m_Octree.IsPointInsideOccupiedLeaf(_Point);
        }

        internal bool TryIsPointInsideSolid(Vector3 _Point, out bool _Result) => m_Octree.TryIsPointInsideSolid(_Point, out _Result);

        public bool TryIsPointInsideOccupiedOctreeBucket(Vector3 _Point, out bool _Result)
        {
            return m_Octree.TryIsPointInsideOccupiedLeaf(_Point, out _Result);
        }

        //Leaf snapshot for debug visualization (read-locked inside the octree).
        public void SnapshotLeaves(List<NavLeafInfo> _Result, bool _IncludeFree = false, NavGraphSnapshot _Graph = null)
        {
            m_Octree.SnapshotLeaves(_Result, _IncludeFree, _Graph);
        }

        //Box-graph snapshot for debug visualization (read-locked inside the octree).
        public void SnapshotBoxes(NavBoxSnapshot _Result)
        {
            m_Octree.SnapshotBoxes(_Result);
        }

        public Triangle[] GetIntersectedObstaclesTriangles(Bounds _Bounds)
        {
            return m_Octree.GetIntersectedObstaclesTriangles(_Bounds);
        }

        public bool TryGetIntersectedObstaclesTriangles(Bounds _Bounds, out Triangle[] _Result)
        {
            return m_Octree.TryGetIntersectedObstaclesTriangles(_Bounds, out _Result);
        }

        public bool SegmentIntersectOccupiedLeaf(Segment3 _Segment)
        {
            return m_Octree.SegmentIntersectOccupiedLeaf(_Segment);
        }

        public bool TrySegmentIntersectOccupiedLeaf(Segment3 _Segment, out bool _Result)
        {
            return m_Octree.TrySegmentIntersectOccupiedLeaf(_Segment, out _Result);
        }

        public (List<Vector3> Path, PathfindingResultCode Code) FindPath(
                Vector3           _Start,
                Vector3           _Target,
                bool              _TryRepositionStartToFreeLeaf,
                bool              _TryRepositionTargetToFreeLeaf,
                CancellationToken _CancellationTokenExternal,
                CancellationToken _CancellationTokenTimeout
            )
        {
            return m_Octree.FindPath(
                    _Start,
                    _Target,
                    _TryRepositionStartToFreeLeaf,
                    _TryRepositionTargetToFreeLeaf,
                    _CancellationTokenExternal,
                    _CancellationTokenTimeout
                );
        }

        public string GetStorageContentDescription()
        {
            return m_Octree.GetStorageContentDescription();
        }

        #endregion

    }
}
