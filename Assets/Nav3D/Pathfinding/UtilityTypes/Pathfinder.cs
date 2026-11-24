using System;
using System.Threading;
using System.Collections.Generic;
using System.Linq;
using Nav3D.API;
using Nav3D.Common;
using Nav3D.Obstacles;
using System.Threading.Tasks;
using UnityEngine;
using Debug = UnityEngine.Debug;

namespace Nav3D.Pathfinding
{
    static partial class Pathfinder
    {

        #region Constants

        const float SQRDISTANCE_EPSILON = 0.00001f;

        const string COMPOUND_FIND_PATH_FRAGMENT_RESULT    = "Fragment {0}/{1} stats: Start: {2}. Target: {3}, ResultCode: {4}, Duration(ms): {5}";
        const string COMPOUND_FIND_PATH_POSTPROCESS_RESULT = "Postprocess stats(ms): Optimizing duration: {0}, Smoothing duration: {1}";

        static readonly string FIND_PATH_INVOKE = $"{nameof(FindPath)}: for points {{0}} invoked";

        #endregion

        #region Public methods

        public static async Task<PathfindingResult> FindPath(
                PathfindingRequest _Request,
                CancellationToken  _CancellationTokenExternal,
                CancellationToken  _CancellationTokenTimeout,
                Action<string>     _StatusCallback,
                Log                _Log
            )
        {
            PathfindingResult result = await FindPathCore(_Request, _CancellationTokenExternal, _CancellationTokenTimeout, _StatusCallback, _Log);

            // clearance certification: a successful volumetric result carries the radius its delivered
            //trajectory is proven to keep — the search radius always, and, when the request caps higher, the largest
            //measured radius up to that cap (bisection over the capsule query). Measurement only: the route is
            //searched at Radius regardless, so a wide cap never trades a tight shortcut for a fat detour — which is
            //exactly the trap a "search wider first" ladder would step into. Consumed by the avoidance stage's
            //certified-corridor gate.
            if (!result.Failed && _Request.Radius > 0f)
                result.CertifiedRadius = CertifyTrajectoryClearance(result.PathFinal, _Request.Radius, _Request.MaxCertifiedRadius);

            // volumetric FALLBACK_TO_DEFAULT: when the radius-aware search finds no corridor, retry the
            //whole request as a point search (the default, cheaper pathfinding mode) and flag the delivered result
            //Degraded. Request-level (not per-fragment) so a multi-target path is never a mix of clearance-honest
            //and clearance-dropped pieces. Only corridor-class failures fall back — TIMEOUT/CANCELLED/UNKNOWN stay
            //what they are.
            if (result.Failed                                                            &&
                _Request.Radius         > 0f                                             &&
                _Request.VolumetricMode == VolumetricPathfindingMode.FALLBACK_TO_DEFAULT &&
                IsVolumetricCorridorFailure(result.Result))
            {
                _Log?.Write($"{nameof(Pathfinder)}: volumetric search failed ({result.Result}), falling back to point search (Degraded).");

                PathfindingResult fallback = await FindPathCore(_Request with { Radius = 0f }, _CancellationTokenExternal, _CancellationTokenTimeout, _StatusCallback, _Log);

                fallback.Degraded = true;

                return fallback;
            }

            return result;
        }

        //the only way this class may reach the obstacle manager. Every method below runs on a worker thread,
        //while ObstacleManager.Instance resolves through a scene search that throws off the main thread and would
        //create a manager during teardown — which is what a request still in flight when the world is disposed
        //used to do. Capturing the already-resolved reference is safe: the Unity object may be destroyed
        //afterwards, but the managed octree it holds stays alive and every call made into it is pure C#.
        static bool TryGetObstacleManager(out ObstacleManager _Manager)
        {
            _Manager = ObstacleManager.InstanceIfResolved;

            return _Manager != null && !ObstacleManager.Doomed;
        }

        //The largest radius in [_Radius, _Cap] whose capsule along the whole delivered polyline is free of
        //occupied volume. _Radius itself is free by construction (ShortenPath/SmoothPath validated every delivered
        //segment with it), so the answer is at least _Radius; a disagreeing lower bound is clamped rather than
        //trusted (paranoia against a stale octree read between postprocessing and this measurement).
        static float CertifyTrajectoryClearance(Vector3[] _Path, float _Radius, float _Cap)
        {
            const int BISECTION_STEPS = 5;

            #if !LITE_VERSION

            //the search and the capsule test are clamped to the world's bound, so nothing above it was measured
            if (TryGetObstacleManager(out ObstacleManager manager))
            {
                _Radius = Mathf.Min(_Radius, manager.MaxVolumetricRadius);
                _Cap    = Mathf.Min(_Cap, manager.MaxVolumetricRadius);
            }

            #endif

            if (_Path == null || _Path.Length < 2 || _Cap <= _Radius)
                return _Radius;

            if (!TrajectoryCapsuleBlocked(_Path, _Cap))
                return _Cap;

            float lo = _Radius;
            float hi = _Cap;

            for (int i = 0; i < BISECTION_STEPS; i++)
            {
                float mid = (lo + hi) * 0.5f;

                if (TrajectoryCapsuleBlocked(_Path, mid))
                    hi = mid;
                else
                    lo = mid;
            }

            return lo;
        }

        static bool TrajectoryCapsuleBlocked(Vector3[] _Path, float _Radius)
        {
            for (int i = 1; i < _Path.Length; i++)
            {
                if (RayIntersectOccupiedLeaf(_Path[i - 1], _Path[i], _Radius))
                    return true;
            }

            return false;
        }

        //a fragment after the first starts where the previous one ended — at a target — so its start's refusal is that
        //target's
        static PathfindingResultCode ToTargetCode(PathfindingResultCode _Code)
        {
            return _Code switch
            {
                PathfindingResultCode.START_POINT_INSIDE_OBSTACLE => PathfindingResultCode.TARGET_POINT_INSIDE_OBSTACLE,
                PathfindingResultCode.START_POINT_INSIDE_SEALED_VOLUME => PathfindingResultCode.TARGET_POINT_INSIDE_SEALED_VOLUME,
                _ => _Code
            };
        }

        static bool IsVolumetricCorridorFailure(PathfindingResultCode _Code)
        {
            return _Code == PathfindingResultCode.PATH_DOES_NOT_EXIST              ||
                   _Code == PathfindingResultCode.START_POINT_TOO_NARROW           ||
                   _Code == PathfindingResultCode.TARGET_POINT_TOO_NARROW          ||
                   _Code == PathfindingResultCode.START_POINT_INSIDE_OBSTACLE      ||
                   _Code == PathfindingResultCode.TARGET_POINT_INSIDE_OBSTACLE     ||
                   _Code == PathfindingResultCode.START_POINT_INSIDE_SEALED_VOLUME ||
                   _Code == PathfindingResultCode.TARGET_POINT_INSIDE_SEALED_VOLUME;
        }

        static async Task<PathfindingResult> FindPathCore(
                PathfindingRequest _Request,
                CancellationToken  _CancellationTokenExternal,
                CancellationToken  _CancellationTokenTimeout,
                Action<string>     _StatusCallback,
                Log                _Log
            )
        {
            const string STATUS_PATHFINDING = "Pathfinding fragment {0}/{1}";
            const string STATUS_FINISHED    = "Finished";

            try
            {
                _Log?.WriteFormat(FIND_PATH_INVOKE, UtilsCommon.GetPointsString(_Request.Points));

                Vector3[] inputPoints = _Request.GetExtendedPoints();

                bool      addedLoopStart = _Request.Loop && inputPoints.First() != inputPoints.Last();
                Vector3[] points         = addedLoopStart ? new[] { inputPoints.Last() }.Concat(inputPoints).ToArray() : inputPoints.Copy();

                //which input point each of points stands for; the prepended closing point is the ring's start, and the
                //same input point is reported where the ring ends
                int[] sources = _Request.GetExtendedPointSources();

                if (addedLoopStart)
                    sources = new[] { -1 }.Concat(sources).ToArray();

                List<int> targetSources = new List<int>(points.Length);

                int fragmentsCount = points.Length - 1;

                // fragments touching a transient tour point — the requester's own position rather
                //than an authored destination — get the reposition/escape-hop treatment on the volumetric lane.
                //The strict endpoint-fit refusal is meant for authored destinations; a body pressed within
                //Radius of voxelized geometry is exactly where re-plans legitimately start.
                List<Vector3> transientPoints = _Request.Radius > 0f ? _Request.GetTransientPoints().ToList() : null;

                List<List<Vector3>>                    pathFragments                 = new List<List<Vector3>>(fragmentsCount);
                List<(Vector3, PathfindingResultCode)> targetResultCodes             = new List<(Vector3, PathfindingResultCode)>(fragmentsCount);
                List<(Vector3, TimeSpan)>              fragmentsPathfindingDurations = new List<(Vector3, TimeSpan)>(fragmentsCount);

                TimeSpan pathfindingDuration = TimeSpan.Zero;
                int      startIndex          = 0;
                int      initialStartIndex   = 0;
                bool     skippedLoopStart    = false;

                for (int i = 1; i <= fragmentsCount; i++)
                {
                    Vector3 start  = points[startIndex];
                    Vector3 target = points[i];

                    //a loop repositions too: its ring starts and ends at the same point, and both ends resolve to the same
                    //nearest free point, so the ring stays closed
                    bool tryRepositionStartToFreeLeaf = (startIndex == initialStartIndex && _Request.TryRepositionStartIfOccupied) ||
                                                        (transientPoints?.Contains(start) ?? false);

                    bool tryRepositionTargetToFreeLeaf = (i == fragmentsCount && _Request.TryRepositionTargetIfOccupied) ||
                                                         (transientPoints?.Contains(target) ?? false);

                    _StatusCallback(string.Format(STATUS_PATHFINDING, i, fragmentsCount));

                    //dev: the outcome line of a refused search names its requester ( —
                    //a no-path storm is one requester asking again; the wrapper is shipped, the tag behind it is not)
                    Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.NavTree.DevSetRequesterHook(_Request.RequesterID);

                    List<Vector3> fragment = FindPath(
                            start,
                            target,
                            tryRepositionStartToFreeLeaf,
                            tryRepositionTargetToFreeLeaf,
                            _Request.Radius,
                            _Request.Optimality,
                            _CancellationTokenExternal,
                            _CancellationTokenTimeout,
                            out PathfindingResultCode code,
                            out TimeSpan fragmentPathfindingDuration
                        );

                    _Log?.WriteFormat(
                            COMPOUND_FIND_PATH_FRAGMENT_RESULT,
                            i,
                            fragmentsCount,
                            start.ToStringExt(),
                            target.ToStringExt(),
                            code,
                            fragmentPathfindingDuration.TotalMilliseconds
                        );

                    pathfindingDuration = pathfindingDuration.Add(fragmentPathfindingDuration);
                    targetResultCodes.Add((target, code));
                    fragmentsPathfindingDurations.Add((target, fragmentPathfindingDuration));

                    if (code != PathfindingResultCode.SUCCEEDED)
                    {
                        //The injected closing source is an authored target, not the request's real start. If it is
                        //unreachable, build the remaining tour first and reconnect its actual last reachable point.
                        if (_Request.SkipUnpassableTargets && addedLoopStart && i == 1)
                        {
                            startIndex       = initialStartIndex = 1;
                            skippedLoopStart = true;
                            continue;
                        }

                        if (startIndex == initialStartIndex &&
                            (code == PathfindingResultCode.START_POINT_INSIDE_OBSTACLE || code == PathfindingResultCode.START_POINT_INSIDE_SEALED_VOLUME))
                        {
                            return await Task.FromResult(
                                    new PathfindingResult(
                                            inputPoints,
                                            pathfindingDuration,
                                            _Request.Timeout,
                                            new List<(Vector3, PathfindingResultCode)> { (target, code) },
                                            new List<(Vector3, TimeSpan)> { (target, fragmentPathfindingDuration) },
                                            code
                                        )
                                );
                        }

                        if (!_Request.SkipUnpassableTargets)
                        {
                            return await Task.FromResult(
                                    new PathfindingResult(
                                            inputPoints,
                                            pathfindingDuration,
                                            _Request.Timeout,
                                            targetResultCodes,
                                            fragmentsPathfindingDurations,
                                            ToTargetCode(code)
                                        )
                                );
                        }

                        //a skipped target is never reported as passed, whatever the trajectory projects it onto
                        sources[i] = -1;

                        continue;
                    }

                    if (pathFragments.Count == 0)
                        targetSources.Add(sources[startIndex]);

                    pathFragments.Add(fragment);
                    targetSources.Add(sources[i]);
                    //A refused waypoint must never become the source of the next fragment.
                    startIndex = i;
                }

                //Every fragment was refused and SkipUnpassableTargets swallowed each refusal: there is no path at all,
                //and the empty trajectory must not go out as SUCCEEDED — the follower indexes its first point on
                //adoption (PathFollowData.GetClosestPoint) and threw out of the dispatcher. The verdict is the last
                //target's, with the start→target mapping the non-skipping branch applies; every target's own code
                //is in TargetResultCodes.
                if (pathFragments.Count == 0)
                {
                    PathfindingResultCode lastCode = targetResultCodes[targetResultCodes.Count - 1].Item2;

                    return await Task.FromResult(
                            new PathfindingResult(
                                    inputPoints,
                                    pathfindingDuration,
                                    _Request.Timeout,
                                    targetResultCodes,
                                    fragmentsPathfindingDurations,
                                    ToTargetCode(lastCode)
                                )
                        );
                }

                if (skippedLoopStart)
                {
                    List<Vector3> closingFragment = FindPath(
                            points[startIndex],
                            points[initialStartIndex],
                            false,
                            _Request.TryRepositionStartIfOccupied,
                            _Request.Radius,
                            _Request.Optimality,
                            _CancellationTokenExternal,
                            _CancellationTokenTimeout,
                            out PathfindingResultCode closingCode,
                            out TimeSpan closingDuration
                        );

                    pathfindingDuration = pathfindingDuration.Add(closingDuration);
                    targetResultCodes.Add((points[initialStartIndex], closingCode));
                    fragmentsPathfindingDurations.Add((points[initialStartIndex], closingDuration));

                    if (closingCode != PathfindingResultCode.SUCCEEDED)
                        return await Task.FromResult(
                                new PathfindingResult(
                                        inputPoints,
                                        pathfindingDuration,
                                        _Request.Timeout,
                                        targetResultCodes,
                                        fragmentsPathfindingDurations,
                                        ToTargetCode(closingCode)
                                    )
                            );

                    pathFragments.Insert(0, closingFragment);
                    targetSources.Insert(0, -1);
                    points[0] = points[startIndex];
                }

                _Log?.Write($"Pathfinding finished ({pathfindingDuration.TotalMilliseconds} ms.)");

                List<Vector3> entirePath = new List<Vector3>(pathFragments.Select(_Path => _Path.Count).Sum());

                foreach (List<Vector3> pathFragment in pathFragments)
                {
                    entirePath.AddRange(pathFragment);
                }

                //a loop whose start was moved out of an obstacle closes on the moved point, so the ring drops the origin it
                //left from — otherwise every lap would dip back into the obstacle
                if (_Request.Loop && entirePath.Count > 2 && entirePath[0] != entirePath[^1] && entirePath[1] == entirePath[^1])
                    entirePath.RemoveAt(0);

                if (_CancellationTokenExternal.IsCancellationRequested)
                {
                    return await Task.FromResult(
                            new PathfindingResult(
                                    inputPoints,
                                    pathfindingDuration,
                                    _Request.Timeout,
                                    targetResultCodes,
                                    fragmentsPathfindingDurations,
                                    PathfindingResultCode.CANCELLED
                                )
                        );
                }

                //the TIMEOUT contract. Reaching this line means every fragment delivered a route: the deadline passed
                //while a refinement was running (a radius-aware fragment's detour escalation ends with the centre
                //route it already held), not while the search was still looking. What happens next is the
                //request's choice — DELIVER_FOUND_ROUTE keeps the route and marks the result TimedOut, FAIL answers
                //TIMEOUT as every request did before 2026-09-23. A deadline met while a fragment was still searching
                //never gets here: that fragment's own code is TIMEOUT and the loop above has returned it.
                bool timedOut = false;

                if (_CancellationTokenTimeout.IsCancellationRequested)
                {
                    if (_Request.TimeoutMode == PathfindingTimeoutMode.FAIL)
                    {
                        return await Task.FromResult(
                                new PathfindingResult(
                                        inputPoints,
                                        pathfindingDuration,
                                        _Request.Timeout,
                                        targetResultCodes,
                                        fragmentsPathfindingDurations,
                                        PathfindingResultCode.TIMEOUT
                                    )
                            );
                    }

                    timedOut = true;

                    _Log?.Write($"{nameof(Pathfinder)}: the {_Request.Timeout} ms timeout passed after a route was found; delivering it as found (TimedOut).");
                }

                PostprocessPath(
                        pathFragments,
                        points.ToArray(),
                        _Request.Loop,
                        _Request.Smooth,
                        _Request.SmoothRatio,
                        _Request.Radius,
                        _StatusCallback,
                        _Log,
                        out Vector3[] pathOptimized,
                        out Vector3[] pathSmoothed,
                        out int[] targetIndices,
                        out TimeSpan optimizingDuration,
                        out TimeSpan smoothingDuration
                    );

                _Log?.WriteFormat(COMPOUND_FIND_PATH_POSTPROCESS_RESULT, optimizingDuration.TotalMilliseconds, smoothingDuration.TotalMilliseconds);
                _StatusCallback(STATUS_FINISHED);

                if (_CancellationTokenExternal.IsCancellationRequested)
                {
                    return await Task.FromResult(
                            new PathfindingResult(
                                    inputPoints,
                                    pathfindingDuration,
                                    _Request.Timeout,
                                    targetResultCodes,
                                    fragmentsPathfindingDurations,
                                    PathfindingResultCode.CANCELLED
                                )
                        );
                }

                if (_CancellationTokenTimeout.IsCancellationRequested)
                {
                    if (_Request.TimeoutMode == PathfindingTimeoutMode.FAIL)
                    {
                        return await Task.FromResult(
                                new PathfindingResult(
                                        inputPoints,
                                        pathfindingDuration,
                                        _Request.Timeout,
                                        targetResultCodes,
                                        fragmentsPathfindingDurations,
                                        PathfindingResultCode.TIMEOUT
                                    )
                            );
                    }

                    timedOut = true;
                }

                PathfindingResult result = new PathfindingResult(
                        entirePath.ToArray(),
                        pathOptimized,
                        pathSmoothed,
                        pathfindingDuration,
                        optimizingDuration,
                        smoothingDuration,
                        targetIndices,
                        targetResultCodes,
                        fragmentsPathfindingDurations
                    );

                result.TimedOut = timedOut;

                //the smoothed trajectory maps every point of the tour, refused ones included; the plain one maps the
                //start and each delivered fragment's target
                int[] resultSources = _Request.Smooth && pathOptimized.Length > 2 ? sources : targetSources.ToArray();

                //a degenerate smoothing answer maps its own targets; it then reports no input points rather than wrong ones
                result.TargetSources = targetIndices != null && resultSources.Length == targetIndices.Length ? resultSources : null;

                return await Task.FromResult(result);
            }
            catch (Exception exception)
            {
                Debug.LogException(exception);

                return await Task.FromResult(
                        new PathfindingResult(
                                _Request.Points,
                                TimeSpan.Zero,
                                _Request.Timeout,
                                new List<(Vector3, PathfindingResultCode)>(),
                                new List<(Vector3, TimeSpan)>(),
                                PathfindingResultCode.UNKNOWN
                            )
                    );
            }
        }

        #endregion

        #region Service methods

        static List<Vector3> FindPath(
                Vector3                   _Start,
                Vector3                   _Target,
                bool                      _TryRepositionStartToFreeLeaf,
                bool                      _TryRepositionTargetToFreeLeaf,
                float                     _Radius,
                VolumetricOptimality      _Optimality,
                CancellationToken         _CancellationTokenExternal,
                CancellationToken         _CancellationTokenTimeout,
                out PathfindingResultCode _Code,
                out TimeSpan              _PathfindingDuration,
                bool                      _SkipStart = false
            )
        {
            DateTime start = DateTime.Now;

            if ((_Start - _Target).sqrMagnitude < SQRDISTANCE_EPSILON)
            {
                _Code                = PathfindingResultCode.SUCCEEDED;
                _PathfindingDuration = DateTime.Now - start;
                return new List<Vector3> { _Start, _Target };
            }

            if (!TryGetObstacleManager(out ObstacleManager manager))
            {
                _Code                = PathfindingResultCode.SUCCEEDED;
                _PathfindingDuration = DateTime.Now - start;
                return new List<Vector3> { _Start, _Target };
            }

            // volumetric dispatch: a positive radius routes to the capsule search, 0 stays on the point
            //search (#if: the volumetric entry points live in Pro-only files — Lite compiles the point-search call
            //only and a request radius is silently ignored)

            #if !LITE_VERSION

            (List<Vector3> path, PathfindingResultCode code) = _Radius > 0f
                ? manager.FindPathVolumetric(
                        _Start,
                        _Target,
                        _Radius,
                        _TryRepositionStartToFreeLeaf,
                        _TryRepositionTargetToFreeLeaf,
                        _CancellationTokenExternal,
                        _CancellationTokenTimeout,
                        _Optimality
                    )
                : manager.FindPath(
                        _Start,
                        _Target,
                        _TryRepositionStartToFreeLeaf,
                        _TryRepositionTargetToFreeLeaf,
                        _CancellationTokenExternal,
                        _CancellationTokenTimeout
                    );

            #else

            (List<Vector3> path, PathfindingResultCode code) = manager.FindPath(
                    _Start,
                    _Target,
                    _TryRepositionStartToFreeLeaf,
                    _TryRepositionTargetToFreeLeaf,
                    _CancellationTokenExternal,
                    _CancellationTokenTimeout
                );

            #endif

            _Code                = code;
            _PathfindingDuration = DateTime.Now - start;

            if (code != PathfindingResultCode.SUCCEEDED)
                return null;

            //_SkipStart drops the leading start point when stitching consecutive fragments
            if (_SkipStart && path != null && path.Count > 0)
                path.RemoveAt(0);

            return path;
        }

        static void PostprocessPath(
                List<List<Vector3>> _PathFragments,
                Vector3[]           _Targets,
                bool                _Loop,
                bool                _Smooth,
                int                 _PerMinBucketSmoothSamples,
                float               _Radius,
                Action<string>      _StatusCallback,
                Log                 _Log,
                out Vector3[]       _PathOptimized,
                out Vector3[]       _PathSmoothed,
                out int[]           _TargetIndices,
                out TimeSpan        _OptimizingDuration,
                out TimeSpan        _SmoothingDuration
            )
        {
            const string STATUS_OPTIMIZING_DETAILING = "Postprocessing : Optimizing fragment {0}/{1} : Detailing";
            const string STATUS_OPTIMIZING_PRUNING   = "Postprocessing : Optimizing fragment {0}/{1} : Pruning";
            const string STATUS_SMOOTHING            = "Postprocessing : Smoothing";

            List<Vector3> pathOptimized = new List<Vector3>(_PathFragments.Sum(_Fragment => _Fragment.Count));
            Vector3[]     pathSmoothed;
            _TargetIndices = new int[_PathFragments.Count + 1];

            DateTime start = DateTime.Now;
            TimeSpan optimizingDuration;
            TimeSpan smoothingDuration;

            _Log?.Write($"Postprocessing fragments:");

            for (int i = 0; i < _PathFragments.Count; i++)
            {
                _Log?.Write($"{i + 1}/{_PathFragments.Count}");
                List<Vector3> fragment = _PathFragments[i];

                if (fragment.Count > 2)
                {
                    //increase points density to provide efficient shorten procedure
                    _StatusCallback(string.Format(STATUS_OPTIMIZING_DETAILING, i + 1, _PathFragments.Count));
                    fragment = DetailPath(fragment);
                    //prune redundant path pieces
                    _StatusCallback(string.Format(STATUS_OPTIMIZING_PRUNING, i + 1, _PathFragments.Count));
                    fragment = ShortenPath(fragment, _Radius);
                    fragment.Reverse();

                    //do the same in reverse
                    _StatusCallback(string.Format(STATUS_OPTIMIZING_DETAILING, i + 1, _PathFragments.Count));
                    fragment = DetailPath(fragment);
                    _StatusCallback(string.Format(STATUS_OPTIMIZING_PRUNING, i + 1, _PathFragments.Count));
                    fragment = ShortenPath(fragment, _Radius);
                    fragment.Reverse();
                }

                //Append the fragment, skipping every point that duplicates the current tail. Duplicates arise from
                //degenerate fragments (equal consecutive request points, see the SQRDISTANCE_EPSILON shortcut in
                //FindPath) and from fragments whose internal duplicate survived ShortenPath (rays around it blocked
                //by nearby geometry). The only verbatim append is a leading 2-point fragment: it cannot carry an
                //internal duplicate, except the degenerate {start, target} case, which a single-fragment request
                //must keep delivering as the 2-point path.
                bool leadDuplicatesTail = pathOptimized.Count                                    > 0 &&
                                          (fragment.First() - pathOptimized.Last()).sqrMagnitude < SQRDISTANCE_EPSILON;

                _TargetIndices[i] = leadDuplicatesTail ? pathOptimized.Count - 1 : pathOptimized.Count;

                if (i == 0 && fragment.Count <= 2)
                {
                    pathOptimized.AddRange(fragment);
                }
                else
                {
                    foreach (Vector3 point in fragment)
                    {
                        if (pathOptimized.Count > 0 && (point - pathOptimized.Last()).sqrMagnitude < SQRDISTANCE_EPSILON)
                            continue;

                        pathOptimized.Add(point);
                    }
                }

                if (i < _PathFragments.Count - 1)
                    pathOptimized.RemoveAt(pathOptimized.Count - 1);
            }


            _TargetIndices[_PathFragments.Count] = pathOptimized.Count - 1;

            //record path optimizing duration
            optimizingDuration = DateTime.Now - start;

            _Log?.Write($"Postprocessing finished({optimizingDuration.TotalMilliseconds} ms.).");

            start = DateTime.Now;

            if (_Smooth)
            {
                _Log?.Write($"Path smoothing started.");
                _StatusCallback(STATUS_SMOOTHING);

                pathSmoothed = pathOptimized.Count > 2
                    ? Smooth(pathOptimized.ToArray(), _Targets, _Loop, _PerMinBucketSmoothSamples, _Radius, out _TargetIndices)
                    : pathOptimized.ToArray();

                //record path smoothing duration
                smoothingDuration = DateTime.Now - start;

                _Log?.Write($"Path smoothing finished({smoothingDuration.TotalMilliseconds} ms.).");
            }
            else
            {
                pathSmoothed      = pathOptimized.ToArray();
                smoothingDuration = TimeSpan.Zero;
            }

            _PathOptimized      = pathOptimized.ToArray();
            _PathSmoothed       = pathSmoothed;
            _OptimizingDuration = optimizingDuration;
            _SmoothingDuration  = smoothingDuration;
        }

        static Vector3[] Smooth(Vector3[] _SourcePath, Vector3[] _Targets, bool _Loop, int _PerMinBucketSmoothSamples, float _Radius, out int[] _TargetIndices)
        {
            Vector3[] trimmedSourcePath = UtilsCommon.TrimEqualPoints(_SourcePath);


            return SmoothPath(trimmedSourcePath, _Targets, _Loop, _PerMinBucketSmoothSamples, _Radius, out _TargetIndices);
        }

        static List<Vector3> DetailPath(List<Vector3> _Path)
        {
            //the world can be disposed while this request is still in flight — then there is nothing to detail
            //against, and the path as found is the honest answer
            if (!TryGetObstacleManager(out ObstacleManager manager))
                return _Path;

            float minBucketSize    = manager.MinBucketSize;
            float minBucketSizeSqr = manager.MinBucketSizeSqr;

            int newArrayEstimatedLength = 1;

            for (int i = 0; i < _Path.Count - 1; i++)
            {
                float dist = Vector3.Distance(_Path[i], _Path[i + 1]);
                newArrayEstimatedLength += Mathf.CeilToInt(dist / minBucketSize);
            }

            List<Vector3> detailedPath = new List<Vector3>(newArrayEstimatedLength);

            Vector3 prevDirectionVectorNormalized = (_Path.Second() - _Path.First()).normalized;

            for (int i = 0; i < _Path.Count - 1; i++)
            {
                Vector3 curPoint  = _Path[i];
                Vector3 nextPoint = _Path[i + 1];

                detailedPath.Add(curPoint);

                Vector3 directionVector           = nextPoint - curPoint;
                Vector3 directionVectorNormalized = directionVector.normalized;

                //the prev and curr vectors enough co-directional (directions angle less than 5 deg.) so we have no need to detail straight path fragment
                if (Vector3.Dot(prevDirectionVectorNormalized, directionVectorNormalized) > 0.99619f)
                {
                    prevDirectionVectorNormalized = directionVectorNormalized;
                    continue;
                }

                Vector3 toNextVectorStep = directionVectorNormalized * minBucketSize;

                while ((nextPoint - curPoint).sqrMagnitude > minBucketSizeSqr)
                {
                    curPoint += toNextVectorStep;
                    detailedPath.Add(curPoint);
                }
            }

            detailedPath.Add(_Path.Last());

            return detailedPath;
        }

        static List<Vector3> ShortenPath(List<Vector3> _Path, float _Radius)
        {
            List<Vector3> result = new List<Vector3>(_Path);

            for (int i = 0; i < result.Count; i++)
            {
                Vector3 curPoint = result[i];

                for (int j = 2; j < result.Count - i; j++)
                {
                    Vector3 checkPoint = result[i + j];

                    if (!RayIntersectOccupiedLeaf(curPoint, checkPoint, _Radius))
                    {
                        result.RemoveAt(i + j - 1);
                        j--;
                    }
                    else
                    {
                        break;
                    }
                }
            }

            return result;
        }

        static Vector3[] SmoothPath(
                Vector3[] _Path,
                Vector3[] _Targets,
                bool      _Loop,
                int       _PerMinBucketSmoothSamples,
                float     _Radius,
                out int[] _TargetIndices
            )
        {
            try
            {
                if (_Path.Length == 2)
                {
                    _TargetIndices = new[] { 0, 1 };
                    return _Path.ToArray();
                }

                if (!TryGetObstacleManager(out ObstacleManager manager))
                {
                    _TargetIndices = null;
                    return null;
                }

                float minBucketSize = manager.MinBucketSize;
                float sampleLength  = minBucketSize / _PerMinBucketSmoothSamples;

                List<Vector3> sourceTrajectory = new List<Vector3>(_Path);

                int newArrayMinEstimatedLength = 1;

                for (int i = 0; i < _Path.Length - 1; i++)
                {
                    float dist = Vector3.Distance(_Path[i], _Path[i + 1]);
                    newArrayMinEstimatedLength += Mathf.CeilToInt(dist / minBucketSize);
                }

                List<Vector3> smoothedTrajectory = new List<Vector3>(newArrayMinEstimatedLength) { sourceTrajectory[0] };

                //accessory extreme points adding
                if (_Loop)
                {
                    sourceTrajectory.Insert(0, sourceTrajectory[sourceTrajectory.Count - 2]);
                    sourceTrajectory.Add(sourceTrajectory[2]);
                }
                else
                {
                    sourceTrajectory.Insert(0, sourceTrajectory[0] + sourceTrajectory[0] - sourceTrajectory[1]);
                    sourceTrajectory.Add(sourceTrajectory.Last()                         + (sourceTrajectory.Last() - sourceTrajectory[sourceTrajectory.Count - 2]));
                }

                int lastTimePointsAdded = 0;

                for (int i = 1; i <= sourceTrajectory.Count - 3; i++)
                {
                    smoothedTrajectory.Add(sourceTrajectory[i]);

                    int smoothRatio = (int) (Vector3.Distance(sourceTrajectory[i], sourceTrajectory[i + 1]) / sampleLength);

                    if (smoothRatio == 0)
                        continue;

                    float paramStep = 1f / smoothRatio;
                    float t         = 0;

                    Vector3 lastPoint = sourceTrajectory[i];

                    int addedPointsCounter = 0;

                    //centripetal, not uniform: the uniform form overshoots where a long straight run meets a
                    //tight corner — precisely what this loop is handed, since ShortenPath removed every point a
                    //straight segment could skip — and that overshoot is a visible loop in the flown path.
                    //Prepared once per span, since its knots depend only on the four control points; every branch
                    //below that INSERTS a point also breaks out, so the span cannot go stale.
                    UtilsMath.CentripetalSpan span = new UtilsMath.CentripetalSpan(
                            sourceTrajectory[i - 1],
                            sourceTrajectory[i],
                            sourceTrajectory[i + 1],
                            sourceTrajectory[i + 2]
                        );

                    for (int pass = 0; pass < smoothRatio - 1; pass++)
                    {
                        t += paramStep;

                        Vector3 newPoint = span.Evaluate(t);

                        //an occupied leaf on the way to the new point calls for an accessory point
                        if (lastPoint != newPoint && RayIntersectOccupiedLeaf(lastPoint, newPoint, _Radius))
                        {
                            if (i != sourceTrajectory.Count - 3)
                            {
                                sourceTrajectory.Insert(i + 2, sourceTrajectory[i + 1] + (sourceTrajectory[i + 2] - sourceTrajectory[i + 1]) * 0.5f);
                            }

                            sourceTrajectory.Insert(i + 1, sourceTrajectory[i] + (sourceTrajectory[i + 1] - sourceTrajectory[i]) * 0.5f);

                            //remove all new points added for current sample
                            for (int passCounter = 0; passCounter < pass; passCounter++)
                                smoothedTrajectory.RemoveLast();

                            if (i != 1)
                            {
                                sourceTrajectory.Insert(i, sourceTrajectory[i - 1] + (sourceTrajectory[i] - sourceTrajectory[i - 1]) * 0.5f);
                                i--;

                                //remove all new points added for previous sample
                                for (int passCounter = 0; passCounter < lastTimePointsAdded; passCounter++)
                                    smoothedTrajectory.RemoveLast();
                            }

                            addedPointsCounter = 0;

                            break;
                        }

                        smoothedTrajectory.Add(newPoint);
                        addedPointsCounter++;

                        lastPoint = newPoint;
                    }

                    lastTimePointsAdded = addedPointsCounter;
                }

                smoothedTrajectory.Add(_Path.Last());

                _TargetIndices = GetTargetIndices(smoothedTrajectory.ToArray(), _Targets);
                return smoothedTrajectory.ToArray();
            }
            catch (Exception exception)
            {
                Debug.LogException(exception);
                _TargetIndices = null;
                return null;
            }
        }

        //Maps every target onto the trajectory index that represents it. The result must be non-decreasing:
        //the follow layer advances a cursor through it, and its loop-completion certification requires the
        //trailing targets to resolve in order onto the final index. Two shapes defeat a plain exact-equality
        //scan — duplicate targets, which the engine produces routinely (a loop request prepends the closing
        //point, an agent request the agent's position), and targets the postprocessing DISPLACED so they no
        //longer appear verbatim. Both are handled by scanning forward from the previous match inclusively, so
        //duplicates collapse onto one index, and falling back to the nearest point at or after it.
        static int[] GetTargetIndices(Vector3[] _Path, Vector3[] _Targets)
        {
            int[] indices = new int[_Targets.Length];

            int searchStart = 0;

            for (int targetIndex = 0; targetIndex < _Targets.Length; targetIndex++)
            {
                Vector3 target  = _Targets[targetIndex];
                int     matched = -1;

                for (int i = searchStart; i < _Path.Length; i++)
                {
                    if (_Path[i] != target)
                        continue;

                    matched = i;

                    break;
                }

                if (matched < 0)
                    matched = GetNearestPathIndex(_Path, target, searchStart);

                indices[targetIndex] = matched;
                searchStart          = matched;
            }

            return indices;
        }

        //closest trajectory point at or after _From — keeps the indices array monotonic when a target
        //has no verbatim counterpart on the trajectory
        static int GetNearestPathIndex(Vector3[] _Path, Vector3 _Target, int _From)
        {
            int   best    = Mathf.Min(_From, _Path.Length - 1);
            float bestSqr = float.MaxValue;

            for (int i = best; i < _Path.Length; i++)
            {
                float sqr = (_Path[i] - _Target).sqrMagnitude;

                if (sqr >= bestSqr)
                    continue;

                bestSqr = sqr;
                best    = i;
            }

            return best;
        }

        static bool RayIntersectOccupiedLeaf(Vector3 _Start, Vector3 _End, float _Radius = 0f)
        {
            if (!TryGetObstacleManager(out ObstacleManager manager))
                return false;

            //perform lexicography sorting because due to inaccuracy during calculations the result can deffer for the same segment checked in different directions
            //so we have to check it in consistent direction independently to initial points order
            bool sorted = _Start.LexCompare(_End) >= 0;

            #if !LITE_VERSION

            // volumetric: the capsule counterpart of the segment check (0 = point search)
            if (_Radius > 0f)
            {
                return sorted
                    ? manager.CapsuleIntersectOccupiedLeaf(_End, _Start, _Radius)
                    : manager.CapsuleIntersectOccupiedLeaf(_Start, _End, _Radius);
            }

            #endif

            Segment3 segment = sorted ? new Segment3(_End, _Start) : new Segment3(_Start, _End);

            return manager.SegmentIntersectOccupiedLeaf(segment);
        }

        #endregion

    }
}
