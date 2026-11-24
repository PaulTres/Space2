using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Nav3D.Common
{
    static class UtilsMath
    {
        #region Attributes

        static readonly System.Random s_Random = new System.Random();

        #endregion

        #region Properties

        public static readonly Vector3Int[] NeighborBuckets = new[]
        {
            new Vector3Int(-1, -1, -1),
            new Vector3Int(-1, -1, 0),
            new Vector3Int(-1, -1, 1),
            new Vector3Int(-1, 0, -1),
            new Vector3Int(-1, 0, 0),
            new Vector3Int(-1, 0, 1),
            new Vector3Int(-1, 1, -1),
            new Vector3Int(-1, 1, 0),
            new Vector3Int(-1, 1, 1),
            new Vector3Int(0, -1, -1),
            new Vector3Int(0, -1, 0),
            new Vector3Int(0, -1, 1),
            new Vector3Int(0, 0, -1),
            new Vector3Int(0, 0, 0),
            new Vector3Int(0, 0, 1),
            new Vector3Int(0, 1, -1),
            new Vector3Int(0, 1, 0),
            new Vector3Int(0, 1, 1),
            new Vector3Int(1, -1, -1),
            new Vector3Int(1, -1, 0),
            new Vector3Int(1, -1, 1),
            new Vector3Int(1, 0, -1),
            new Vector3Int(1, 0, 0),
            new Vector3Int(1, 0, 1),
            new Vector3Int(1, 1, -1),
            new Vector3Int(1, 1, 0),
            new Vector3Int(1, 1, 1)
        };

        #endregion

        #region Public methods

        //Centripetal Catmull-Rom, prepared once for a span and evaluated at any number of parameters along it: the
        //knot spacing and both tangents depend only on the four control points, and a smoother samples one span
        //dozens of times. Centripetal (knots spaced by the square root of the distance) is not optional: the uniform
        //form's tangent length is set by a point's neighbours rather than by the segment it traverses, so where a
        //long run meets a tight corner the curve overshoots into a loop; Yuksel/Schaefer/Keyser (2011) proved the
        //centripetal form produces no cusps and no self-intersections for any control polygon.
        public readonly struct CentripetalSpan
        {
            //knot spacing floor: a coincident control pair (this pipeline does produce them — duplicate waypoints
            //survive pruning when the rays around them are blocked) would otherwise divide by zero
            const float MIN_KNOT_SPAN = 1e-4f;

            readonly Vector3 m_P1;
            readonly Vector3 m_P2;
            readonly Vector3 m_M1;
            readonly Vector3 m_M2;

            public CentripetalSpan(Vector3 _P0, Vector3 _P1, Vector3 _P2, Vector3 _P3)
            {
                float span01 = KnotSpan(_P0, _P1);
                float span12 = KnotSpan(_P1, _P2);
                float span23 = KnotSpan(_P2, _P3);

                m_P1 = _P1;
                m_P2 = _P2;

                //non-uniform tangents, scaled into the [P1, P2] interval so Evaluate can take _T in [0, 1]
                m_M1 = span12 * ((_P1 - _P0) / span01 - (_P2 - _P0) / Mathf.Max(span01 + span12, MIN_KNOT_SPAN) + (_P2 - _P1) / span12);
                m_M2 = span12 * ((_P2 - _P1) / span12 - (_P3 - _P1) / Mathf.Max(span12 + span23, MIN_KNOT_SPAN) + (_P3 - _P2) / span23);
            }

            //the cubic Hermite basis; no roots, no divisions — every one of those was paid in the constructor
            public Vector3 Evaluate(float _T)
            {
                float squared = _T * _T;
                float cubed   = squared * _T;

                return (2f * cubed - 3f * squared + 1f) * m_P1 +
                       (cubed - 2f * squared + _T)      * m_M1 +
                       (-2f * cubed + 3f * squared)     * m_P2 +
                       (cubed - squared)                * m_M2;
            }

            //alpha = 0.5, i.e. the square root of the distance — reached as the fourth root of the squared distance,
            //which is two cheap roots instead of a magnitude plus a root plus a Pow
            static float KnotSpan(Vector3 _A, Vector3 _B)
            {
                return Mathf.Max(Mathf.Sqrt(Mathf.Sqrt((_B - _A).sqrMagnitude)), MIN_KNOT_SPAN);
            }
        }

        public static Vector3[] GetSphereSurfacePoints(Vector3 _Center, float _Radius, int _Count)
        {
            Vector3[] directions;

            directions = new Vector3[_Count];

            float goldenRatio    = (1 + Mathf.Sqrt(5)) / 2;
            float angleIncrement = Mathf.PI            * 2 * goldenRatio;

            for (int i = 0; i < _Count; i++)
            {
                float t           = (float) i / _Count;
                float inclination = Mathf.Acos(1 - 2 * t);
                float azimuth     = angleIncrement * i;

                float x = Mathf.Sin(inclination) * Mathf.Cos(azimuth);
                float y = Mathf.Sin(inclination) * Mathf.Sin(azimuth);
                float z = Mathf.Cos(inclination);
                directions[i] = new Vector3(x, y, z);
            }

            return directions.Select(_Direction => _Center + _Direction * _Radius).ToArray();
        }

        public static Vector3[] GetSpaceCirclePoints(Vector3 _Center, float _Radius, Vector3 _Normal, int _Count)
        {
            Quaternion rotation = Quaternion.FromToRotation(Vector3.up, _Normal.normalized);
            Vector3[]  result   = new Vector3[_Count];
            
            for (int i = 0; i < _Count; i++)
            {
                float   angle = (float) i / (_Count - 1)                           * 2 * Mathf.PI;
                Vector3 point = new Vector3(Mathf.Cos(angle), 0, Mathf.Sin(angle)) * _Radius;
                point     =  rotation * point; // Rotate to align with normal
                point     += _Center;
                result[i] =  point;
            }

            return result;
        }

        public static float TriangleHeronSquare(float _A, float _B, float _C)
        {
            float p = 0.5f * (_A + _B + _C);

            return Mathf.Sqrt(p * (p - _A) * (p - _B) * (p - _C));
        }

        public static Vector3 GetRandomVector(float _Magnitude = 1f)
        {
            return new Vector3(
                    SystemRandomRange(-_Magnitude, _Magnitude),
                    SystemRandomRange(-_Magnitude, _Magnitude),
                    SystemRandomRange(-_Magnitude, _Magnitude)
                );
        }

        //Gaussian distribution
        public static float GetRandomNormalValue(float _Min, float _Max)
        {
            float delta     = _Max - _Min;
            float halfDelta = delta * 0.5f;

            return Mathf.Clamp(_Min + halfDelta + (float) GenerateGaussianNoise().Z0 * halfDelta, _Min, _Max);
        }

        //Uniform distribution
        public static float GetRandomUniformValue(float _Min, float _Max)
        {
            return UnityEngine.Random.Range(_Min, _Max);
        }

        public static Segment3[] PointSequenceToSegments(Vector3[] _Sequence)
        {
            Segment3[] result = new Segment3[_Sequence.Length - 1];

            for (int i = 0; i < _Sequence.Length - 1; i++)
            {
                result[i] = new Segment3(_Sequence[i], _Sequence[i + 1]);
            }

            return result.ToArray();
        }

        public static Vector3 GetClosestArrayPoint(Vector3[] _Points, Vector3 _Point)
        {
            if (!_Points.Any())
                return _Point;

            return _Points.MinBy(_CurrPoint => (_CurrPoint - _Point).sqrMagnitude);
        }

        public static Vector3 GetClosestPointOnCurve(Vector3[] _Curve, Vector3 _Point)
        {
            if (!_Curve.Any())
                return _Point;

            List<Vector3> segmentsClosestPoints = new List<Vector3>();

            for (int i = 1; i < _Curve.Length; i++)
            {
                segmentsClosestPoints.Add(new Segment3(_Curve[i - 1], _Curve[i]).GetClosestPoint(_Point));
            }

            return segmentsClosestPoints.MinBy(_SegPoint => (_SegPoint - _Point).sqrMagnitude);
        }

        #endregion

        #region Service methods

        static float SystemRandomRange(float _Min, float _Max)
        {
            float delta        = _Max - _Min;
            float deltaMinZero = Mathf.Abs(_Min);

            float rnd = (float) s_Random.NextDouble() * delta - deltaMinZero;
            return rnd;
        }

        //Box–Muller transform
        static (double Z0, double Z1) GenerateGaussianNoise()
        {
            double x, y, s;

            do
            {
                x = UnityEngine.Random.Range(-1f, 1f);
                y = UnityEngine.Random.Range(-1f, 1f);

                s = x * x + y * y;
            } while (s > 1f || s == 0);

            double factor = Math.Sqrt(-2 * Math.Log(s) / s);

            double z0 = x * factor;
            double z1 = y * factor;

            return (z0, z1);
        }


        #endregion
    }
}
