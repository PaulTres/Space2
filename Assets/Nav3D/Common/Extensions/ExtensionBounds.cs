using UnityEngine;

namespace Nav3D.Common
{
    public static class ExtensionBounds
    {
        #region Constants


        #endregion

        #region Static methods

        public static void Draw(this Bounds _Bounds)
        {
            Gizmos.DrawWireCube(_Bounds.center, _Bounds.size);
        }

        public static float GetMaxSize(this Bounds _Bounds)
        {
            return Mathf.Max(_Bounds.size.x, _Bounds.size.y, _Bounds.size.z);
        }

        public static float GetMinSize(this Bounds _Bounds)
        {
            return Mathf.Min(_Bounds.size.x, _Bounds.size.y, _Bounds.size.z);
        }

        public static bool DoesNotIntersects(this Bounds _Bounds, Bounds _Other)
        {
            Vector3 boundsMin = _Bounds.min;
            Vector3 boundsMax = _Bounds.max;

            Vector3 otherMin = _Other.min;
            Vector3 otherMax = _Other.max;

            return boundsMin.x > otherMax.x ||
                   boundsMax.x < otherMin.x ||
                   boundsMin.y > otherMax.y ||
                   boundsMax.y < otherMin.y ||
                   boundsMin.z > otherMax.z ||
                   boundsMax.z < otherMin.z;
        }

        public static Bounds TrianglesBounds(Triangle[] _Triangles)
        {
            Bounds targetBounds = _Triangles[0].Bounds;

            if (_Triangles.Length == 1)
                return targetBounds;

            for (int i = 1; i < _Triangles.Length; i++)
            {
                targetBounds = targetBounds.Union(_Triangles[i].Bounds);
            }

            return targetBounds;
        }

        public static Bounds PointBounds(Vector3[] _Points)
        {
            float xMin = float.MaxValue;
            float yMin = float.MaxValue;
            float zMin = float.MaxValue;
            float xMax = float.MinValue;
            float yMax = float.MinValue;
            float zMax = float.MinValue;

            for (int i = 0; i < _Points.Length; i++)
            {
                Vector3 point = _Points[i];

                xMin = Mathf.Min(xMin, point.x);
                yMin = Mathf.Min(yMin, point.y);
                zMin = Mathf.Min(zMin, point.z);
                xMax = Mathf.Max(xMax, point.x);
                yMax = Mathf.Max(yMax, point.y);
                zMax = Mathf.Max(zMax, point.z);
            }

            float sizeX = xMax - xMin;
            float sizeY = yMax - yMin;
            float sizeZ = zMax - zMin;

            Vector3 center = new Vector3(xMin + sizeX * 0.5f, yMin + sizeY * 0.5f, zMin + sizeZ * 0.5f);

            Vector3 size = new Vector3(sizeX, sizeY, sizeZ);

            return new Bounds(center, size);
        }

        public static bool Embrace(this Bounds _Bounds, Bounds _OtherBounds)
        {
            Vector3 boundsMin = _Bounds.min;
            Vector3 boundsMax = _Bounds.max;
            Vector3 otherMin  = _OtherBounds.min;
            Vector3 otherMax  = _OtherBounds.max;

            return boundsMin.x <= otherMin.x && boundsMin.y <= otherMin.y && boundsMin.z <= otherMin.z &&
                   boundsMax.x >= otherMax.x && boundsMax.y >= otherMax.y && boundsMax.z >= otherMax.z;
        }

        public static Bounds MinMax(Vector3 _A, Vector3 _B)
        {
            float xMin = Mathf.Min(_A.x, _B.x);
            float yMin = Mathf.Min(_A.y, _B.y);
            float zMin = Mathf.Min(_A.z, _B.z);
            float xMax = Mathf.Max(_A.x, _B.x);
            float yMax = Mathf.Max(_A.y, _B.y);
            float zMax = Mathf.Max(_A.z, _B.z);

            float sizeX = xMax - xMin;
            float sizeY = yMax - yMin;
            float sizeZ = zMax - zMin;

            Vector3 center = new Vector3(xMin + sizeX * 0.5f, yMin + sizeY * 0.5f, zMin + sizeZ * 0.5f);

            Vector3 size = new Vector3(sizeX, sizeY, sizeZ);

            return new Bounds(center, size);
        }

        ///                Bounds A
        ///  _____________ /                         ___________________
        /// |             |                         |                   |
        /// |      _______|_____                    |                   |
        /// |     |       |     |-- Bounds B   ==>  |                   |
        /// |_____|_______|     |                   |                   |
        ///       |             |                   |                   |
        ///       |_____________|                   |___________________|
        public static Bounds Union(this Bounds _BoundsA, Bounds _BoundsB)
        {
            _BoundsA.Encapsulate(_BoundsB.min);
            _BoundsA.Encapsulate(_BoundsB.max);

            return _BoundsA;
        }

        public static Vector3[] GetCornerPoints(this Bounds _Bounds)
        {
            Vector3 min = _Bounds.min;
            Vector3 max = _Bounds.max;

            return new Vector3[8]
            {
                new Vector3(min.x, min.y, min.z),
                new Vector3(min.x, min.y, max.z),
                new Vector3(min.x, max.y, min.z),
                new Vector3(max.x, min.y, min.z),
                new Vector3(min.x, max.y, max.z),
                new Vector3(max.x, min.y, max.z),
                new Vector3(max.x, max.y, min.z),
                new Vector3(max.x, max.y, max.z),
            };
        }

        public static Bounds Enlarge(this Bounds _Bounds, float _Enlargement)
        {
            return new Bounds(_Bounds.center, _Bounds.size + new Vector3(_Enlargement, _Enlargement, _Enlargement));
        }

        /// <summary>
        /// Intersection cases:
        /// 1) The segment penetrates one or two faces of the bounds;
        /// 2) The segment is inside bounds;
        /// Implemented using the slab (Kay-Kajiya) method
        /// </summary>
        public static bool IntersectSegment(this Bounds _Bounds, Segment3 _Segment3)
        {
            Vector3 start     = _Segment3.Start;
            Vector3 direction = _Segment3.DirectionMagn;
            Vector3 boundsMin = _Bounds.min;
            Vector3 boundsMax = _Bounds.max;

            float tMin = 0f;
            float tMax = 1f;

            //X slab
            if (Mathf.Approximately(direction.x, 0f))
            {
                if (start.x < boundsMin.x || start.x > boundsMax.x)
                    return false;
            }
            else
            {
                float invD = 1f                      / direction.x;
                float t1   = (boundsMin.x - start.x) * invD;
                float t2   = (boundsMax.x - start.x) * invD;

                if (t1 > t2)
                {
                    (t1, t2) = (t2, t1);
                }

                tMin = Mathf.Max(tMin, t1);
                tMax = Mathf.Min(tMax, t2);

                if (tMin > tMax)
                    return false;
            }

            //Y slab
            if (Mathf.Approximately(direction.y, 0f))
            {
                if (start.y < boundsMin.y || start.y > boundsMax.y)
                    return false;
            }
            else
            {
                float invD = 1f                      / direction.y;
                float t1   = (boundsMin.y - start.y) * invD;
                float t2   = (boundsMax.y - start.y) * invD;

                if (t1 > t2)
                {
                    (t1, t2) = (t2, t1);
                }

                tMin = Mathf.Max(tMin, t1);
                tMax = Mathf.Min(tMax, t2);

                if (tMin > tMax)
                    return false;
            }

            //Z slab
            if (Mathf.Approximately(direction.z, 0f))
            {
                if (start.z < boundsMin.z || start.z > boundsMax.z)
                    return false;
            }
            else
            {
                float invD = 1f                      / direction.z;
                float t1   = (boundsMin.z - start.z) * invD;
                float t2   = (boundsMax.z - start.z) * invD;

                if (t1 > t2)
                {
                    (t1, t2) = (t2, t1);
                }

                tMin = Mathf.Max(tMin, t1);
                tMax = Mathf.Min(tMax, t2);

                if (tMin > tMax)
                    return false;
            }

            return true;
        }

        #endregion
    }
}