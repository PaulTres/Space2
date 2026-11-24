using System.Collections.Generic;
using UnityEngine;

namespace Nav3D.Common
{
    /// <summary>
    /// Centripetal Catmull-Rom spline with arc-length parameterization.
    /// Each segment is converted to cubic polynomial form (Hermite tangents derived from centripetal knots),
    /// segment lengths are integrated with 5-point Gauss-Legendre quadrature, and arc length is inverted with
    /// Newton-Raphson, which allows placing points uniformly along the curve.
    /// Immutable after construction: rebuild the instance when control points change.
    /// </summary>
    public class CatmullRomSpline
    {
        #region Constants

        //centripetal parameterization exponent
        const float ALPHA = 0.5f;
        //lower bound for a knot interval; prevents division by zero on coincident control points
        const float KNOT_EPSILON = 1e-4f;

        const int   NEWTON_MAX_ITERATIONS = 10;
        //arc-length inversion tolerance, fraction of the segment length
        const float NEWTON_TOLERANCE = 1e-4f;

        //5-point Gauss-Legendre nodes and weights premapped from [-1, 1] to [0, 1]
        static readonly float[] GL_NODES =
        {
            0.04691007703f,
            0.23076534495f,
            0.5f,
            0.76923465505f,
            0.95308992297f
        };

        static readonly float[] GL_WEIGHTS =
        {
            0.11846344252f,
            0.23931433525f,
            0.28444444444f,
            0.23931433525f,
            0.11846344252f
        };

        #endregion

        #region Nested types

        //cubic segment C(u) = C0 + C1*u + C2*u^2 + C3*u^3, u in [0, 1]
        struct Segment
        {
            public Vector3 C0;
            public Vector3 C1;
            public Vector3 C2;
            public Vector3 C3;
            public float   Length;
            //total curve length from the spline start to the end of this segment
            public float   CumulativeLength;
        }

        #endregion

        #region Attributes

        readonly Vector3[] m_Points;
        readonly bool      m_Loop;
        readonly Segment[] m_Segments;
        readonly float     m_TotalLength;

        #endregion

        #region Properties

        public float TotalLength  => m_TotalLength;
        public int   SegmentCount => m_Segments.Length;
        public bool  Loop         => m_Loop;

        #endregion

        #region Constructors

        public CatmullRomSpline(IReadOnlyList<Vector3> _Points, bool _Loop)
        {
            m_Points = new Vector3[_Points.Count];
            for (int i = 0; i < _Points.Count; i++)
                m_Points[i] = _Points[i];

            //a loop needs at least 3 distinct vertices; degrade to an open spline otherwise
            m_Loop = _Loop && m_Points.Length > 2;

            m_Segments    = BuildSegments();
            m_TotalLength = m_Segments.Length == 0 ? 0f : m_Segments[m_Segments.Length - 1].CumulativeLength;
        }

        #endregion

        #region Public methods

        /// <summary>
        /// Returns points spaced uniformly along the curve. _MaxSpacing is the upper bound for the distance
        /// between consecutive points; the actual spacing is TotalLength / ceil(TotalLength / _MaxSpacing).
        /// The first point is the first control point; for a loop the last point closes back onto it.
        /// </summary>
        public Vector3[] GetUniformPoints(float _MaxSpacing)
        {
            if (m_Segments.Length == 0 || m_TotalLength <= 0f)
                return (Vector3[])m_Points.Clone();

            int   intervals = Mathf.Max(1, Mathf.CeilToInt(m_TotalLength / Mathf.Max(_MaxSpacing, Mathf.Epsilon)));
            float spacing   = m_TotalLength / intervals;

            Vector3[] points = new Vector3[intervals + 1];

            points[0] = m_Points[0];

            for (int i = 1; i < intervals; i++)
                points[i] = GetPointAtDistance(i * spacing);

            points[intervals] = m_Loop ? m_Points[0] : m_Points[m_Points.Length - 1];

            return points;
        }

        /// <summary>
        /// Returns points sampled with a fixed parameter step per segment (non-uniform world-space density).
        /// Every control point is included.
        /// </summary>
        public Vector3[] GetParameterSampledPoints(float _ParamStep)
        {
            if (m_Segments.Length == 0)
                return (Vector3[])m_Points.Clone();

            float step = Mathf.Clamp(_ParamStep, 0.001f, 1f);

            List<Vector3> points = new List<Vector3>(m_Segments.Length * (Mathf.CeilToInt(1f / step) + 1));

            for (int i = 0; i < m_Segments.Length; i++)
            {
                points.Add(Evaluate(i, 0f));

                for (float u = step; u < 1f; u += step)
                    points.Add(Evaluate(i, u));
            }

            points.Add(m_Loop ? m_Points[0] : m_Points[m_Points.Length - 1]);

            return points.ToArray();
        }

        /// <summary>
        /// Point on the curve at the given arc-length distance from the start.
        /// Wraps for loops, clamps to [0, TotalLength] otherwise.
        /// </summary>
        public Vector3 GetPointAtDistance(float _Distance)
        {
            if (m_Segments.Length == 0)
                return m_Points.Length > 0 ? m_Points[0] : Vector3.zero;

            FindSegmentParameter(_Distance, out int segmentIndex, out float u);

            return Evaluate(segmentIndex, u);
        }

        /// <summary>
        /// Normalized tangent at the given arc-length distance from the start.
        /// Wraps for loops, clamps to [0, TotalLength] otherwise.
        /// </summary>
        public Vector3 GetTangentAtDistance(float _Distance)
        {
            if (m_Segments.Length == 0)
                return Vector3.forward;

            FindSegmentParameter(_Distance, out int segmentIndex, out float u);

            Vector3 derivative = EvaluateDerivative(segmentIndex, u);

            return derivative.sqrMagnitude > Mathf.Epsilon ? derivative.normalized : Vector3.forward;
        }

        #endregion

        #region Service methods

        Segment[] BuildSegments()
        {
            int pointCount = m_Points.Length;

            if (pointCount < 2)
                return new Segment[0];

            //extend the control points with phantom ends: wrap for loops, mirror for open splines
            int       segmentCount = m_Loop ? pointCount : pointCount - 1;
            Vector3[] ext          = new Vector3[pointCount + 3];

            for (int i = 0; i < pointCount; i++)
                ext[i + 1] = m_Points[i];

            if (m_Loop)
            {
                ext[0]              = m_Points[pointCount - 1];
                ext[pointCount + 1] = m_Points[0];
                ext[pointCount + 2] = m_Points[1];
            }
            else
            {
                ext[0]              = m_Points[0] + (m_Points[0] - m_Points[1]);
                ext[pointCount + 1] = m_Points[pointCount - 1] + (m_Points[pointCount - 1] - m_Points[pointCount - 2]);
                //unused by open-spline segments, keep it harmless
                ext[pointCount + 2] = ext[pointCount + 1];
            }

            Segment[] segments = new Segment[segmentCount];

            float cumulativeLength = 0f;

            for (int i = 0; i < segmentCount; i++)
            {
                Segment segment = BuildSegment(ext[i], ext[i + 1], ext[i + 2], ext[i + 3]);

                cumulativeLength           += segment.Length;
                segment.CumulativeLength   =  cumulativeLength;
                segments[i]                =  segment;
            }

            return segments;
        }

        static Segment BuildSegment(Vector3 _P0, Vector3 _P1, Vector3 _P2, Vector3 _P3)
        {
            //centripetal knot intervals
            float dt0 = Mathf.Max(Mathf.Pow(Vector3.Distance(_P0, _P1), ALPHA), KNOT_EPSILON);
            float dt1 = Mathf.Max(Mathf.Pow(Vector3.Distance(_P1, _P2), ALPHA), KNOT_EPSILON);
            float dt2 = Mathf.Max(Mathf.Pow(Vector3.Distance(_P2, _P3), ALPHA), KNOT_EPSILON);

            //non-uniform Catmull-Rom tangents at the segment ends, rescaled to the unit parameter domain
            Vector3 m1 = ((_P1 - _P0) / dt0 - (_P2 - _P0) / (dt0 + dt1) + (_P2 - _P1) / dt1) * dt1;
            Vector3 m2 = ((_P2 - _P1) / dt1 - (_P3 - _P1) / (dt1 + dt2) + (_P3 - _P2) / dt2) * dt1;

            Segment segment = new Segment
            {
                C0 = _P1,
                C1 = m1,
                C2 = 3f * (_P2 - _P1) - 2f * m1 - m2,
                C3 = 2f * (_P1 - _P2) + m1 + m2
            };

            segment.Length = SegmentPartialLength(segment, 1f);

            return segment;
        }

        Vector3 Evaluate(int _SegmentIndex, float _U)
        {
            Segment s = m_Segments[_SegmentIndex];
            return s.C0 + _U * (s.C1 + _U * (s.C2 + _U * s.C3));
        }

        Vector3 EvaluateDerivative(int _SegmentIndex, float _U)
        {
            Segment s = m_Segments[_SegmentIndex];
            return s.C1 + _U * (2f * s.C2 + 3f * _U * s.C3);
        }

        static Vector3 EvaluateDerivative(in Segment _Segment, float _U)
        {
            return _Segment.C1 + _U * (2f * _Segment.C2 + 3f * _U * _Segment.C3);
        }

        //arc length of the segment from u = 0 to u = _U via Gauss-Legendre quadrature on [0, _U]
        static float SegmentPartialLength(in Segment _Segment, float _U)
        {
            float length = 0f;

            for (int i = 0; i < GL_NODES.Length; i++)
                length += GL_WEIGHTS[i] * EvaluateDerivative(_Segment, _U * GL_NODES[i]).magnitude;

            return length * _U;
        }

        //maps an arc-length distance to (segment index, curve parameter u)
        void FindSegmentParameter(float _Distance, out int _SegmentIndex, out float _U)
        {
            float distance = m_Loop ? Mathf.Repeat(_Distance, m_TotalLength) : Mathf.Clamp(_Distance, 0f, m_TotalLength);

            //binary search for the first segment whose cumulative length covers the distance
            int lo = 0;
            int hi = m_Segments.Length - 1;

            while (lo < hi)
            {
                int mid = (lo + hi) >> 1;

                if (m_Segments[mid].CumulativeLength < distance)
                    lo = mid + 1;
                else
                    hi = mid;
            }

            _SegmentIndex = lo;

            Segment segment       = m_Segments[lo];
            float   localDistance = distance - (segment.CumulativeLength - segment.Length);

            _U = InvertArcLength(segment, localDistance);
        }

        //Newton-Raphson inversion of s(u) = localDistance with a bisection fallback
        static float InvertArcLength(in Segment _Segment, float _LocalDistance)
        {
            if (_Segment.Length <= Mathf.Epsilon)
                return 0f;

            float target    = Mathf.Clamp(_LocalDistance, 0f, _Segment.Length);
            float tolerance = NEWTON_TOLERANCE * _Segment.Length;

            float u  = target / _Segment.Length;
            float lo = 0f;
            float hi = 1f;

            for (int i = 0; i < NEWTON_MAX_ITERATIONS; i++)
            {
                float error = SegmentPartialLength(_Segment, u) - target;

                if (Mathf.Abs(error) <= tolerance)
                    return u;

                //maintain the bracket for the bisection fallback
                if (error > 0f)
                    hi = u;
                else
                    lo = u;

                float derivative = EvaluateDerivative(_Segment, u).magnitude;

                float next = derivative > Mathf.Epsilon ? u - error / derivative : -1f;

                //fall back to bisection when the Newton step leaves the bracket
                u = next > lo && next < hi ? next : (lo + hi) * 0.5f;
            }

            return u;
        }

        #endregion
    }
}
