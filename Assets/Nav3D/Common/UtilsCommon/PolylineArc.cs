using System.Collections.Generic;
using UnityEngine;

namespace Nav3D.Common
{
    /// <summary>
    /// Arc-length arithmetic on a polyline: its cumulative lengths, the position along it nearest to a point, and the piece of it
    /// between two arc-length positions — what a route drawer needs to split a path at the body that follows it.
    /// </summary>
    public static class PolylineArc
    {
        #region Public methods

        /// <summary>
        /// Fills <paramref name="_Cumulative"/> with the arc length at every point of the polyline; its last element is the total.
        /// </summary>
        public static void Measure(IReadOnlyList<Vector3> _Path, List<float> _Cumulative)
        {
            _Cumulative.Clear();

            if (_Path.Count == 0)
                return;

            _Cumulative.Add(0f);

            for (int i = 1; i < _Path.Count; i++)
                _Cumulative.Add(_Cumulative[i - 1] + Vector3.Distance(_Path[i - 1], _Path[i]));
        }

        /// <summary>
        /// The arc-length position of the point on the polyline nearest to <paramref name="_Position"/>.
        /// </summary>
        public static float ClosestArcLength(IReadOnlyList<Vector3> _Path, IReadOnlyList<float> _Cumulative, Vector3 _Position)
        {
            float bestSqrDistance = float.MaxValue;
            float bestArcLength   = 0f;

            for (int i = 1; i < _Path.Count; i++)
            {
                Vector3 segment       = _Path[i] - _Path[i - 1];
                float   segmentLength = segment.magnitude;

                if (segmentLength < Mathf.Epsilon)
                    continue;

                float   ratio      = Mathf.Clamp01(Vector3.Dot(_Position - _Path[i - 1], segment) / (segmentLength * segmentLength));
                Vector3 projection = _Path[i - 1] + segment * ratio;
                float   sqrOffset  = (projection - _Position).sqrMagnitude;

                if (sqrOffset >= bestSqrDistance)
                    continue;

                bestSqrDistance = sqrOffset;
                bestArcLength   = _Cumulative[i - 1] + segmentLength * ratio;
            }

            return bestArcLength;
        }

        /// <summary>
        /// The point at an arc-length position; the polyline's last point past its end.
        /// </summary>
        public static Vector3 PointAt(IReadOnlyList<Vector3> _Path, IReadOnlyList<float> _Cumulative, float _Distance)
        {
            for (int i = 1; i < _Path.Count; i++)
            {
                if (_Cumulative[i] < _Distance)
                    continue;

                float segmentLength = _Cumulative[i] - _Cumulative[i - 1];
                float ratio         = segmentLength < Mathf.Epsilon ? 0f : (_Distance - _Cumulative[i - 1]) / segmentLength;

                return Vector3.Lerp(_Path[i - 1], _Path[i], ratio);
            }

            return _Path[_Path.Count - 1];
        }

        /// <summary>
        /// Fills <paramref name="_Span"/> with the polyline between two arc-length positions, its ends interpolated; empty when the
        /// span is shorter than <paramref name="_MinLength"/>.
        /// </summary>
        public static void CollectSpan(IReadOnlyList<Vector3> _Path, IReadOnlyList<float> _Cumulative, float _From, float _To, List<Vector3> _Span, float _MinLength = 0.01f)
        {
            _Span.Clear();

            if (_Path.Count < 2 || _To - _From <= _MinLength)
                return;

            _Span.Add(PointAt(_Path, _Cumulative, _From));

            for (int i = 0; i < _Path.Count; i++)
            {
                if (_Cumulative[i] > _From && _Cumulative[i] < _To)
                    _Span.Add(_Path[i]);
            }

            _Span.Add(PointAt(_Path, _Cumulative, _To));
        }

        #endregion
    }
}
