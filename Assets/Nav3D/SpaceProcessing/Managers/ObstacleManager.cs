using Nav3D.Common;
using Nav3D.API;
using Nav3D.LocalAvoidance;
using Nav3D.Pathfinding;
using Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree;
using Nav3D.SpaceProcessing.ObstacleProcessing.UtilityTypes;
using System;
using System.Collections.Generic;
using System.Threading;
using UnityEngine;

namespace Nav3D.Obstacles
{
    [DisallowMultipleComponent]
    partial class ObstacleManager : MonoBehaviour
    {

        #region Attributes

        ObstacleStorage m_ObstaclesStorage;

        CancellationTokenSource m_Cts;

        #endregion

        #region Events

        //Re-export of the octree's post-publish region-changed signal for external observers (currently the octree
        //octree drawer's snapshot-on-change rebuild). Main-thread dispatched, carries the operation's volumes one
        //by one — see ObstacleStorage.OnRegionChanged. Subscribe only after Nav3D init: the storage this forwards to
        //exists once Initialize has run.
        public event Action<IReadOnlyList<Bounds>> OnRegionChanged
        {
            add
            {
                if (m_ObstaclesStorage == null)
                {
                    Debug.LogError(
                            $"{nameof(ObstacleManager)}.{nameof(OnRegionChanged)}: subscribing before initialization; " +
                            //a literal, not a nameof: this assembly sits under Nav3D.API
                            "subscribe on Nav3DManager.OnNav3DInit instead."
                        );

                    return;
                }

                m_ObstaclesStorage.OnRegionChanged += value;
            }
            remove
            {
                if (m_ObstaclesStorage == null)
                    return;

                m_ObstaclesStorage.OnRegionChanged -= value;
            }
        }

        #endregion

        #region Properties

        public static ObstacleManager Instance => Singleton<ObstacleManager>.Instance;

        //worker-SAFE counterpart of Instance: returns the already-resolved manager and never resolves a new one.
        //Instance goes through UtilsSceneObjects.FindAny, which throws outright on a worker thread — and on the main
        //thread would create a fresh manager GameObject during teardown. Anything running off the main thread (the
        //whole Pathfinder core) must use this and treat null as "the world is gone".
        public static ObstacleManager InstanceIfResolved => Singleton<ObstacleManager>.InstanceIfResolved;

        public static CancellationToken CancellationToken { get; private set; }
        public static bool              Doomed            { get; private set; }
        public        bool              Inited            { get; private set; }
        public        float             MinBucketSize     { get; private set; }
        public        float             MinBucketSizeSqr  { get; private set; }

        #endregion

        #region Public methods

        //Leaf snapshot for debug visualization (read-locked inside the octree; safe to call from a worker thread on
        //a main-thread-captured instance — pure C# below this point). _IncludeFree adds the navigable leaves, _Graph the
        //stored navigation graph.
        public void SnapshotLeaves(List<NavLeafInfo> _Result, bool _IncludeFree = false, NavGraphSnapshot _Graph = null)
        {
            if (!Inited)
                return;

            m_ObstaclesStorage.SnapshotLeaves(_Result, _IncludeFree, _Graph);
        }

        //The box graph — the free boxes and the portals between them — copied out; same threading contract as
        //SnapshotLeaves.
        public void SnapshotBoxes(NavBoxSnapshot _Result)
        {
            if (!Inited)
                return;

            m_ObstaclesStorage.SnapshotBoxes(_Result);
        }

        public void Initialize(float _OctreeMinBucketSize, int _OctreeLayersCount, float _MaxAgentRadius)
        {
            //also here, not only in Awake: edit mode runs no Awake, and the last play session's OnDestroy left it set
            Doomed = false;

            MinBucketSize    = _OctreeMinBucketSize;
            MinBucketSizeSqr = _OctreeMinBucketSize * _OctreeMinBucketSize;

            m_Cts             = new CancellationTokenSource();
            CancellationToken = m_Cts.Token;

            m_ObstaclesStorage = new ObstacleStorage(_OctreeMinBucketSize, _OctreeLayersCount, _MaxAgentRadius);

            //The changed-region fan-out is not wired here. This subsystem publishes the volumes and nothing else —
            //who reacts (the path layer, local avoidance) is decided one composition root up, in
            //Nav3DManager.WireRegionChangedFanOut, because both of those layers sit above this one.
            Inited = true;
        }

        public void Uninitialize(bool _NeedDestroy = true)
        {
            //Cancel — but do not Dispose — the operation token source. Worker-thread obstacle ops may still be in flight
            //and holding this token; they observe the cancellation and exit. Disposing releases the token's WaitHandle,
            //and an op that then reads _CancellationToken.WaitHandle (ConstructRoots / BuildRootsReusingCache) throws
            //ObjectDisposedException — caught and logged as a bogus "Removal error" at play-exit / scene-unload
            //(Dispose3DNav) / OnDestroy. Cancel signals them to stop; the abandoned CTS's WaitHandle is reclaimed by GC.
            //This is a teardown-only path, so not eagerly disposing one CTS is harmless.
            m_Cts?.Cancel();
            m_Cts = null;

            //The world this epoch built is garbage from here on, and garbage this linked does not collect on its own:
            //one conservatively pinned node keeps the whole tree (measured: ~750 MB per scene switch in the editor, without
            //bound; up to one dead world in a Mono player). Idempotent, and it runs behind the operations just cancelled, off
            //this thread.
            m_ObstaclesStorage?.ReleaseWorld();

            if (!_NeedDestroy)
                return;

            UtilsCommon.SmartDestroyImmediateWithHolder(this);
        }

        public string GetStorageContentDescription()
        {
            return m_ObstaclesStorage.GetStorageContentDescription();
        }

        public ObstacleAdditionProgress AddObstacle(Func<ObstacleInfo> _GetObstacleInfo, CancellationToken _CancellationToken, Action _OnFinish = null, bool _EditMode = false)
        {
            if (!Inited)
                return null;

            ObstacleAdditionProgress additionProgress = ObstacleAdditionProgress.INITIAL;

            m_ObstaclesStorage.AddObstacle(_GetObstacleInfo, additionProgress, _CancellationToken, _OnFinish, _EditMode);

            return additionProgress;
        }

        public bool IsBoundsCrossAnyObstacleBody(Bounds _Bounds)
        {
            if (!Inited)
                return false;

            return m_ObstaclesStorage.IsBoundsCrossAnyObstacleBody(_Bounds);
        }

        public bool IsPointInsideObstacle(Vector3 _Point)
        {
            if (!Inited)
                return false;

            return m_ObstaclesStorage.IsPointInsideOccupiedOctreeBucket(_Point);
        }

        //the per-tick main-thread form (the agent's carrot probe, the avoidance stage's escape state): never waits for
        //the tree — false means a publish holds it or is queued for it, and the caller keeps its previous verdict
        internal bool TryIsPointInsideSolid(Vector3 _Point, out bool _Result)
        {
            _Result = false;
            return !Inited || m_ObstaclesStorage.TryIsPointInsideSolid(_Point, out _Result);
        }

        public bool TryIsPointInsideObstacle(Vector3 _Point, out bool _Result)
        {
            if (!Inited)
            {
                _Result = false;

                return true;
            }

            return m_ObstaclesStorage.TryIsPointInsideOccupiedOctreeBucket(_Point, out _Result);
        }

        public Triangle[] GetIntersectedObstaclesTriangles(Bounds _Bounds)
        {
            return m_ObstaclesStorage.GetIntersectedObstaclesTriangles(_Bounds);
        }

        //the local-avoidance refresh's form (main thread, every tick a mover is obstacle-dirty): never waits for the
        //tree — false means a publish is in flight, and the caller keeps its current triangles until the next tick
        public bool TryGetIntersectedObstaclesTriangles(Bounds _Bounds, out Triangle[] _Result)
        {
            if (!Inited)
            {
                _Result = Array.Empty<Triangle>();

                return true;
            }

            return m_ObstaclesStorage.TryGetIntersectedObstaclesTriangles(_Bounds, out _Result);
        }

        public bool SegmentIntersectOccupiedLeaf(Segment3 _Segment)
        {
            if (!Inited)
                return false;

            return m_ObstaclesStorage.SegmentIntersectOccupiedLeaf(_Segment);
        }

        //the per-tick main-thread form (the pursuit line-of-sight check), same contract as TryIsPointInsideObstacle
        public bool TrySegmentIntersectOccupiedLeaf(Segment3 _Segment, out bool _Result)
        {
            if (!Inited)
            {
                _Result = false;

                return true;
            }

            return m_ObstaclesStorage.TrySegmentIntersectOccupiedLeaf(_Segment, out _Result);
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
            if (!Inited)
                return (new List<Vector3> { _Start, _Target }, PathfindingResultCode.SUCCEEDED);

            return m_ObstaclesStorage.FindPath(
                    _Start,
                    _Target,
                    _TryRepositionStartToFreeLeaf,
                    _TryRepositionTargetToFreeLeaf,
                    _CancellationTokenExternal,
                    _CancellationTokenTimeout
                );
        }

        #endregion

        #region Service methods

        #endregion

        #region Unity methods

        void Awake()
        {
            m_Cts = new CancellationTokenSource();

            Doomed = false;
        }

        void OnDestroy()
        {
            m_Cts?.Cancel();

            Doomed = true;

            Uninitialize(false);

            //Drop the static reference so the whole octree/obstacle graph this instance owns (m_ObstaclesStorage →
            //GlobalOctree → the live tree + the baked-obstacle cache) is no longer GC-rooted after this destroy —
            //see Singleton<T>.ClearInstance.
            Singleton<ObstacleManager>.ClearInstance(this);
        }

        #endregion

    }
}
