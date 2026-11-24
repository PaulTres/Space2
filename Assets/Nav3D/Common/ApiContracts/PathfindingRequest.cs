using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Nav3D.API
{
    public record PathfindingRequest
    {
        #region Constants

        internal const int DEFAULT_SEARCH_TIMEOUT = 2000;
        internal const int DEFAULT_SMOOTH_RATIO   = 3;

        #endregion

        #region Properties

        public Vector3[] Points                        { get; set; }
        public bool      Loop                          { get; set; }
        public bool      SkipUnpassableTargets         { get; set; }
        public bool      TryRepositionStartIfOccupied  { get; set; }
        public bool      TryRepositionTargetIfOccupied { get; set; }
        public int       Timeout                       { get; set; }
        public bool      Smooth                        { get; set; }
        public int       SmoothRatio                   { get; set; }
        public string    RequesterID                   { get; set; }

        // A point in Points that is the requester's own transient position rather than an authored
        //destination — a re-plan inserts the agent's current position at the head of the tour. A volumetric search
        //refuses an authored endpoint that does not fit, but the requester's own position is by definition where the
        //body already legally is, so fragments touching a transient point get the reposition/escape-hop treatment
        //instead. Set by the re-plan composition; null everywhere else.
        internal Vector3? TransientPoint { get; set; }

        //For each of Points, the index of the caller's input point it stands for, or -1 for a point the engine inserted.
        //Null means Points is the caller's own array. Lets a waypoint tour report which input point it passed.
        internal int[] PointSources { get; set; }

        // Volumetric pathfinding: the agent radius the path must keep clear of obstacles.
        //0 (default) = point pathfinding.
        public float Radius { get; set; }

        // What to do when no corridor wide enough for Radius exists (ignored when Radius is 0).
        public VolumetricPathfindingMode VolumetricMode { get; set; }

        // How the volumetric route trades quality for search time (ignored when Radius is 0): FAST, QUALITY
        //(the default) or HIGH_QUALITY, each a better route for a costlier search.
        public VolumetricOptimality Optimality { get; set; }

        // Clearance-certification cap: on a volumetric success the delivered trajectory is post-measured
        //for the LARGEST radius in [Radius, MaxCertifiedRadius] whose capsule it actually keeps clear, and the
        //result's CertifiedRadius records it. Measurement only — the route is always searched at Radius, so this
        //never changes which path is found. 0 (default) = certify at Radius exactly.
        public float MaxCertifiedRadius { get; set; }

        //What to do when Timeout passes while the search already holds a complete route (see the enum).
        public PathfindingTimeoutMode TimeoutMode { get; set; }

        #endregion

        #region Constructors

        /// <param name="_Points">Target points through which the path should pass.</param>
        /// <param name="_Loop">Whether the path should be looped.</param>
        /// <param name="_SkipUnpassableTargets">
        /// Whether to skip points through which no path can be found. If false, an exception will be thrown.
        /// </param>
        /// <param name="_TryRepositionStartIfOccupied">
        /// If the starting point is inside an obstacle, start pathfinding from the nearest free point instead (for a
        /// radius-aware request, the nearest point the agent fits at). On by default.
        /// </param>
        /// <param name="_TryRepositionTargetIfOccupied">
        /// If a target point is inside an obstacle, path to the nearest free point instead and end there (for a
        /// radius-aware request, the nearest point the agent fits at). On by default.
        /// </param>
        /// <param name="_Timeout">
        /// The maximum allowed time for the pathfinding process, in milliseconds. Overrunning it cancels the
        /// request with the corresponding error code.
        /// </param>
        /// <param name="_Smooth">Whether the path should be smoothed.</param>
        /// <param name="_SmoothRatio">Smooth samples performed per one min bucket space volume.</param>
        /// <param name="_RequesterID">The unique ID of the requesting entity. Used for debugging only.</param>
        /// <param name="_Radius">
        /// The agent radius the path must keep clear of obstacles. 0 = point pathfinding.
        /// </param>
        /// <param name="_VolumetricMode">
        /// What to do when no corridor wide enough for the radius exists (ignored when it is 0).
        /// </param>
        /// <param name="_MaxCertifiedRadius">
        /// Post-measure the delivered trajectory's clearance up to this radius (see MaxCertifiedRadius).
        /// 0 = certify at the search radius.
        /// </param>
        /// <param name="_TimeoutMode">
        /// What to do when the timeout passes while a complete route is already found: deliver it flagged
        /// TimedOut (default), or fail the request with TIMEOUT.
        /// </param>
        /// <param name="_Optimality">
        /// How the volumetric route trades quality for search time (ignored when the radius is 0): FAST (a
        /// fast search, the route may come out a little longer), QUALITY (the default: a quality route, a costlier search
        /// than FAST) or HIGH_QUALITY (a still shorter route for a still costlier search).
        /// </param>
        public PathfindingRequest(
                Vector3[]                 _Points,
                bool                      _Loop                          = false,
                bool                      _SkipUnpassableTargets         = false,
                bool                      _TryRepositionStartIfOccupied  = true,
                bool                      _TryRepositionTargetIfOccupied = true,
                int                       _Timeout                       = DEFAULT_SEARCH_TIMEOUT,
                bool                      _Smooth                        = false,
                int                       _SmoothRatio                   = DEFAULT_SMOOTH_RATIO,
                string                    _RequesterID                   = null,
                float                     _Radius                        = 0f,
                VolumetricPathfindingMode _VolumetricMode                = VolumetricPathfindingMode.STRICT,
                float                     _MaxCertifiedRadius            = 0f,
                PathfindingTimeoutMode    _TimeoutMode                   = PathfindingTimeoutMode.DELIVER_FOUND_ROUTE,
                VolumetricOptimality      _Optimality                    = VolumetricOptimality.QUALITY
            )
        {
            Points                        = _Points;
            Loop                          = _Loop;
            SkipUnpassableTargets         = _SkipUnpassableTargets;
            TryRepositionStartIfOccupied  = _TryRepositionStartIfOccupied;
            TryRepositionTargetIfOccupied = _TryRepositionTargetIfOccupied;
            Timeout                       = _Timeout;
            Smooth                        = _Smooth;
            SmoothRatio                   = _SmoothRatio;
            RequesterID                   = _RequesterID ?? Guid.NewGuid().ToString();
            Radius                        = _Radius;
            VolumetricMode                = _VolumetricMode;
            MaxCertifiedRadius            = _MaxCertifiedRadius;
            TimeoutMode                   = _TimeoutMode;
            Optimality                    = _Optimality;
        }

        #endregion

        #region Public methods

        // Every point of the extended tour that is a transient position rather than an authored
        //destination — the volumetric lane relaxes its endpoint-fit refusal for fragments touching these.
        internal virtual IEnumerable<Vector3> GetTransientPoints()
        {
            if (TransientPoint.HasValue)
                yield return TransientPoint.Value;
        }

        internal Vector3[] GetInitialPoints()
        {
            return Points;
        }

        internal virtual Vector3[] GetExtendedPoints()
        {
            return Points;
        }

        internal int GetPointSource(int _PointIndex) => PointSources?[_PointIndex] ?? _PointIndex;

        //PointSources aligned with GetExtendedPoints.
        internal virtual int[] GetExtendedPointSources()
        {
            return PointSources ?? Enumerable.Range(0, Points.Length).ToArray();
        }

        //A re-plan's request: the same settings over a new tour, as a plain request. A `with` copy would keep the
        //derived type, and an agent request's extension (its order-TIME position prepended to the tour) would then
        //ride into every re-plan: a looped ring grew a detour through the spot the order was issued from, and a
        //one-shot tour re-searched from it. The tour already carries the requester's current position as its
        //transient head.
        internal PathfindingRequest WithReplanTour(Vector3[] _Points, Vector3? _TransientPoint, int[] _PointSources = null)
        {
            return new PathfindingRequest(this)
            {
                Points         = _Points,
                TransientPoint = _TransientPoint,
                PointSources   = _PointSources
            };
        }

        #endregion
    }
}
