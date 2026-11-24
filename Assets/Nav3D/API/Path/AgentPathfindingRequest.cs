using Nav3D.API;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Nav3D.Agents
{
    record AgentPathfindingRequest : PathfindingRequest
    {
        #region Properties

        Vector3 AgentPosition { get; }

        #endregion

        #region Constructors

        public AgentPathfindingRequest(
                Vector3[]                 _Points,
                Vector3                   _AgentPosition,
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
            ) : base(
                _Points,
                _Loop,
                _SkipUnpassableTargets,
                _TryRepositionStartIfOccupied,
                _TryRepositionTargetIfOccupied,
                _Timeout,
                _Smooth,
                _SmoothRatio,
                _RequesterID,
                _Radius,
                _VolumetricMode,
                _MaxCertifiedRadius,
                _TimeoutMode,
                _Optimality
            )
        {
            AgentPosition = _AgentPosition;
        }

        #endregion

        #region Public methods

        internal override Vector3[] GetExtendedPoints()
        {
            return new[] { AgentPosition }.Concat(Points).ToArray();
        }

        internal override int[] GetExtendedPointSources()
        {
            return new[] { -1 }.Concat(base.GetExtendedPointSources()).ToArray();
        }

        //the prepended agent position is the order-time transient — never an authored destination
        internal override IEnumerable<Vector3> GetTransientPoints()
        {
            foreach (Vector3 point in base.GetTransientPoints())
                yield return point;

            yield return AgentPosition;
        }

        #endregion
    }
}