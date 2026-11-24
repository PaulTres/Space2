using System;
using System.Collections.Generic;
using System.Text;
using UnityEngine;
using Nav3D.Common;

namespace Nav3D.API
{
    /// <summary>
    /// Pathfinding result container for internal usage.
    /// </summary>
    public class PathfindingResult
    {
        #region Constants

        const string FAIL_TIMEOUT                = "Pathfinding for points {0} failed;\nReason = TIMEOUT: {1} ms was taken, time limit is: {2} ms";
        const string FAIL_NO_PATH                = "Pathfinding for points {0} failed;\nReason = PATH DOES NOT EXIST: {1} ms was taken";
        const string FAIL_CANCELLED              = "Pathfinding for points {0} failed;\nReason = CANCELLED BY EXTERNAL CONTROLLER: {1} ms was taken";
        const string FAIL_START_INSIDE_OBSTACLE  = "Pathfinding for points {0} failed;\nReason = START POINT IS INSIDE THE OBSTACLE: {1} ms was taken";
        const string FAIL_TARGET_INSIDE_OBSTACLE = "Pathfinding for points {0} failed;\nReason = ONE OF THE POINTS IS INSIDE THE OBSTACLE; Failed point info {1}: {2} ms was taken";
        const string FAIL_START_INSIDE_SEALED    = "Pathfinding for points {0} failed;\nReason = START POINT IS INSIDE A SEALED VOLUME: {1} ms was taken";
        const string FAIL_TARGET_INSIDE_SEALED   = "Pathfinding for points {0} failed;\nReason = ONE OF THE POINTS IS INSIDE A SEALED VOLUME; Failed point info {1}: {2} ms was taken";
        const string FAIL_START_TOO_NARROW       = "Pathfinding for points {0} failed;\nReason = START POINT IS IN FREE SPACE, " +
                                                   "BUT IS TOO NARROW FOR THE REQUESTED AGENT RADIUS: {1} ms was taken";
        const string FAIL_TARGET_TOO_NARROW      = "Pathfinding for points {0} failed;\nReason = ONE OF THE POINTS IS IN FREE " +
                                                   "SPACE, BUT IS TOO NARROW FOR THE REQUESTED AGENT RADIUS; "                  +
                                                   "Failed point info {1}: {2} ms was taken";
        const string FAIL_UNKNOWN                = "Pathfinding for points {0} canceled;\nReason = UNKNOWN: {1} ms was taken";
        const string FAIL_GENERIC                = "Pathfinding for points {0} failed;\nReason = {1}: {2} ms was taken";
        
        #endregion
        
        #region Attributes

        readonly List<(Vector3 Point, PathfindingResultCode Code)> m_TargetResultCodes;
        readonly List<(Vector3 Point, TimeSpan Duration)>          m_FragmentPathfindingDurations;

        #endregion

        #region Properties

        public TimeSpan PathfindingDuration { get; private set; }
        public TimeSpan OptimizingDuration  { get; private set; }
        public TimeSpan SmoothingDuration   { get; private set; }

        /// <summary>
        /// Original path obtained after A* have performed. 
        /// </summary>
        public Vector3[] PathRaw { get; }
        /// <summary>
        /// The original path after optimization.
        /// </summary>
        public Vector3[] PathOptimized { get; }
        /// <summary>
        /// The optimized path after smoothing. If smoothing was disabled then the content is identical to PathOptimized. 
        /// </summary>
        public Vector3[] PathFinal { get; }
        /// <summary>
        /// The array that contains the indices of the target points in PathFinal array.
        /// </summary>
        public int[] TargetIndices { get; }

        //For each of TargetIndices, the index of the caller's input point that target stands for, or -1 for a point
        //the engine inserted (the requester's position, a loop's prepended closing point) or a refused one.
        internal int[] TargetSources { get; set; }

        /// <summary>
        /// Result code.
        /// </summary>
        public PathfindingResultCode Result { get; }

        // Volumetric pathfinding: true when this result was produced by the FALLBACK_TO_DEFAULT mode
        //dropping the volumetric constraint (the request had Radius > 0, the delivered trajectory is a point path).
        public bool Degraded { get; internal set; }

        /// <summary>
        /// True when the request's timeout passed after a complete route was found and the request's
        /// PathfindingTimeoutMode delivered that route as it stood: the refinement that was running was skipped, so
        /// the route is valid but may be longer than the optimum. Always false on a failed result.
        /// </summary>
        public bool TimedOut { get; internal set; }

        /// <summary>
        /// The clearance radius the delivered trajectory is certified with: a capsule of this radius
        /// along PathFinal intersects no occupied octree volume (as of the moment the search answered). Equals the
        /// request radius for a plain volumetric success, may exceed it up to the request's MaxCertifiedRadius when
        /// the route happens to run wider, and is 0 for point searches and degraded fallbacks (no promise).
        /// </summary>
        public float CertifiedRadius { get; internal set; }

        /// <summary>
        /// Whether pathfinding was failed.
        /// </summary>
        public bool Failed => Result != PathfindingResultCode.SUCCEEDED;
        
        public string ErrorMessage { get; private set; }

        #endregion

        #region Constructors

        public PathfindingResult(
                Vector3[]                              _PathRaw,
                Vector3[]                              _PathOptimized,
                Vector3[]                              _Smoothed,
                TimeSpan                               _PathfindingDuration,
                TimeSpan                               _OptimizingDuration,
                TimeSpan                               _SmoothingDuration,
                int[]                                  _TargetIndices,
                List<(Vector3, PathfindingResultCode)> _TargetResultCodes,
                List<(Vector3, TimeSpan)>              _FragmentPathfindingDurations
            )
        {
            PathRaw       = _PathRaw;
            PathFinal     = _Smoothed;
            PathOptimized = _PathOptimized;

            PathfindingDuration = _PathfindingDuration;
            OptimizingDuration  = _OptimizingDuration;
            SmoothingDuration   = _SmoothingDuration;

            TargetIndices                  = _TargetIndices;
            m_TargetResultCodes            = _TargetResultCodes;
            m_FragmentPathfindingDurations = _FragmentPathfindingDurations;

            Result = PathfindingResultCode.SUCCEEDED;
        }

        /// <summary>
        /// Error case constructor.
        /// </summary>
        public PathfindingResult(
                Vector3[]                              _Points,
                TimeSpan                               _PathfindingDuration,
                int                                    _Timeout,
                List<(Vector3, PathfindingResultCode)> _TargetResultCodes,
                List<(Vector3, TimeSpan)>              _FragmentPathfindingDurations,
                PathfindingResultCode                  _ResultCode
            )
        {
            m_TargetResultCodes            = _TargetResultCodes;
            m_FragmentPathfindingDurations = _FragmentPathfindingDurations;
            
            Result = _ResultCode;

            ErrorMessage = _ResultCode switch
            {
                PathfindingResultCode.PATH_DOES_NOT_EXIST => string.Format(
                        FAIL_NO_PATH,
                        UtilsCommon.GetPointsString(_Points),
                        _PathfindingDuration.TotalMilliseconds
                    ),
                PathfindingResultCode.TIMEOUT => string.Format(
                        FAIL_TIMEOUT,
                        UtilsCommon.GetPointsString(_Points),
                        _PathfindingDuration.TotalMilliseconds,
                        _Timeout
                    ),
                PathfindingResultCode.CANCELLED => string.Format(
                        FAIL_CANCELLED,
                        UtilsCommon.GetPointsString(_Points),
                        _PathfindingDuration.TotalMilliseconds
                    ),
                PathfindingResultCode.START_POINT_INSIDE_OBSTACLE => string.Format(
                        FAIL_START_INSIDE_OBSTACLE,
                        UtilsCommon.GetPointsString(_Points),
                        _PathfindingDuration.TotalMilliseconds
                    ),
                PathfindingResultCode.TARGET_POINT_INSIDE_OBSTACLE => string.Format(
                        FAIL_TARGET_INSIDE_OBSTACLE,
                        UtilsCommon.GetPointsString(_Points),
                        GetFailedPointInfo(),
                        _PathfindingDuration.TotalMilliseconds
                    ),
                PathfindingResultCode.START_POINT_TOO_NARROW => string.Format(
                        FAIL_START_TOO_NARROW,
                        UtilsCommon.GetPointsString(_Points),
                        _PathfindingDuration.TotalMilliseconds
                    ),
                PathfindingResultCode.TARGET_POINT_TOO_NARROW => string.Format(
                        FAIL_TARGET_TOO_NARROW,
                        UtilsCommon.GetPointsString(_Points),
                        GetFailedPointInfo(),
                        _PathfindingDuration.TotalMilliseconds
                    ),
                PathfindingResultCode.START_POINT_INSIDE_SEALED_VOLUME => string.Format(
                        FAIL_START_INSIDE_SEALED,
                        UtilsCommon.GetPointsString(_Points),
                        _PathfindingDuration.TotalMilliseconds
                    ),
                PathfindingResultCode.TARGET_POINT_INSIDE_SEALED_VOLUME => string.Format(
                        FAIL_TARGET_INSIDE_SEALED,
                        UtilsCommon.GetPointsString(_Points),
                        GetFailedPointInfo(),
                        _PathfindingDuration.TotalMilliseconds
                    ),
                PathfindingResultCode.UNKNOWN => string.Format(
                        FAIL_UNKNOWN,
                        UtilsCommon.GetPointsString(_Points),
                        _PathfindingDuration.TotalMilliseconds
                    ),
                _ => string.Format(
                        FAIL_GENERIC,
                        UtilsCommon.GetPointsString(_Points),
                        _ResultCode,
                        _PathfindingDuration.TotalMilliseconds
                    )
            };
        }

        #endregion

        #region Public methods

        /// <summary>
        /// Returns text info about what the point pathfinding process was failed on.
        /// Can be useful in case when targets count is greater than 2.
        /// </summary>
        public string GetFailedPointInfo()
        {
            int index = m_TargetResultCodes.FindIndex(_Pair => _Pair.Code != PathfindingResultCode.SUCCEEDED);

            if (index == -1)
                return string.Empty;

            return $"Target index: {index}, Position: {m_TargetResultCodes[index].Point.ToStringExt()}, Code: {m_TargetResultCodes[index].Code};";
        }

        internal bool TryGetFailedPoint(out Vector3 _Point)
        {
            int index = m_TargetResultCodes?.FindIndex(_Pair => _Pair.Code != PathfindingResultCode.SUCCEEDED) ?? -1;

            _Point = index == -1 ? default : m_TargetResultCodes[index].Point;

            return index != -1;
        }

        /// <summary>
        /// Returns detailed text info about pathfinding process. 
        /// </summary>
        public string GetLastPathfindingStatsString()
        {
            StringBuilder stringBuilder = new StringBuilder();

            stringBuilder.AppendLine($"{nameof(PathfindingResultCode)}: {Result}");

            if (Result != PathfindingResultCode.SUCCEEDED)
            {
                return stringBuilder.ToString();
            }

            stringBuilder.AppendLine("Pathfinding stages duration in ms:");
            stringBuilder.AppendLine($"{nameof(PathfindingDuration)}: {PathfindingDuration.Milliseconds}");
            stringBuilder.AppendLine($"{nameof(OptimizingDuration)}: {OptimizingDuration.Milliseconds}");
            stringBuilder.AppendLine($"{nameof(SmoothingDuration)}: {SmoothingDuration.Milliseconds}");

            stringBuilder.AppendLine("Targets stats:");

            List<(Vector3, PathfindingResultCode)> codesList          = m_TargetResultCodes.Copy();
            List<(Vector3, TimeSpan)>              fragmentsDurations = m_FragmentPathfindingDurations.Copy();

            for (int i = 0; i < TargetIndices.Length; i++)
            {
                int     targetIndex = TargetIndices[i];
                Vector3 target      = PathFinal[targetIndex];

                int index = codesList.FindIndex(_Data => _Data.Item1 == target);

                if (index < 0)
                    break;

                (Vector3, PathfindingResultCode) codeItem = codesList[index];
                codesList.RemoveAt(index);

                TimeSpan duration = fragmentsDurations[index].Item2;
                fragmentsDurations.RemoveAt(index);

                stringBuilder
                   .AppendLine($"[{i}]: [{target.ToStringExt()}], CODE: {codeItem.Item2}, DURATION (ms): {duration.TotalMilliseconds}");
            }

            return stringBuilder.ToString();
        }

        #endregion
    }
}
