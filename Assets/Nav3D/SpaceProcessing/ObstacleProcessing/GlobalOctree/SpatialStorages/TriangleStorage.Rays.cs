using UnityEngine;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.SpatialStorages
{
    partial class TriangleStorage
    {
        #region Constants

        //how close to a triangle's edge, in barycentric units, or to its plane, in cosine, a hit is too close to call
        const double DEGENERATE_BARYCENTRIC = 1e-7;
        const double DEGENERATE_COSINE      = 1e-9;

        #endregion

        #region Public methods

        //The winding number of _Origin with respect to the stored triangles, counted along one ray: +1 where the ray leaves
        //through a face (the face's normal points along the ray), -1 where it enters. Off a surface without boundary the sum
        //is the same for every direction. Returns false when a hit lands too near an edge or grazes a face to be counted
        //reliably; the caller then tries another direction.
        public bool TryCountSignedCrossings(Vector3 _Origin, Vector3 _Direction, out int _Winding)
        {
            _Winding = 0;

            if (m_NodeCount == 0)
                return true;

            double ox = _Origin.x,    oy = _Origin.y,    oz = _Origin.z;
            double dx = _Direction.x, dy = _Direction.y, dz = _Direction.z;

            double invX = 1.0 / dx, invY = 1.0 / dy, invZ = 1.0 / dz;

            System.Span<int> stack    = stackalloc int[64];
            int              stackPtr = 0;

            stack[stackPtr++] = 0;

            while (stackPtr > 0)
            {
                ref BvhNode node = ref m_Nodes[stack[--stackPtr]];

                if (!RayHitsBox(ox, oy, oz, invX, invY, invZ, node.BoundsMin, node.BoundsMax))
                    continue;

                if (!node.IsLeaf)
                {
                    stack[stackPtr++] = node.LeftChild;
                    stack[stackPtr++] = node.RightChild;

                    continue;
                }

                int end = node.TriangleStart + node.TriangleCount;

                for (int i = node.TriangleStart; i < end; i++)
                {
                    switch (CrossTriangle(m_Triangles[i], ox, oy, oz, dx, dy, dz))
                    {
                        case CROSS_DEGENERATE:
                            return false;
                        case CROSS_LEAVING:
                            _Winding++;
                            break;
                        case CROSS_ENTERING:
                            _Winding--;
                            break;
                    }
                }
            }

            return true;
        }

        //The nearest triangle the ray from _Origin meets within _MaxDistance, skipping those _Ignore accepts; false when none.
        public bool TryRaycastFirst(Vector3 _Origin, Vector3 _Direction, float _MaxDistance, System.Predicate<Nav3D.Common.Triangle> _Ignore, out Nav3D.Common.Triangle _Hit, out float _Distance)
        {
            _Hit      = null;
            _Distance = _MaxDistance;

            if (m_NodeCount == 0)
                return false;

            double ox = _Origin.x,    oy = _Origin.y,    oz = _Origin.z;
            double dx = _Direction.x, dy = _Direction.y, dz = _Direction.z;

            double invX = 1.0 / dx, invY = 1.0 / dy, invZ = 1.0 / dz;

            System.Span<int> stack    = stackalloc int[64];
            int              stackPtr = 0;

            stack[stackPtr++] = 0;

            while (stackPtr > 0)
            {
                ref BvhNode node = ref m_Nodes[stack[--stackPtr]];

                if (!RayHitsBox(ox, oy, oz, invX, invY, invZ, node.BoundsMin, node.BoundsMax))
                    continue;

                if (!node.IsLeaf)
                {
                    stack[stackPtr++] = node.LeftChild;
                    stack[stackPtr++] = node.RightChild;

                    continue;
                }

                int end = node.TriangleStart + node.TriangleCount;

                for (int i = node.TriangleStart; i < end; i++)
                {
                    Nav3D.Common.Triangle triangle = m_Triangles[i];

                    if (_Ignore != null && _Ignore(triangle))
                        continue;

                    if (!TryHitDistance(triangle, ox, oy, oz, dx, dy, dz, out double t) || t >= _Distance)
                        continue;

                    _Distance = (float) t;
                    _Hit      = triangle;
                }
            }

            return _Hit != null;
        }

        #endregion

        #region Service methods

        //Möller–Trumbore distance, edges inclusive — the first-hit query wants the nearest surface, not a verdict on
        //grazing hits
        static bool TryHitDistance(Nav3D.Common.Triangle _Triangle, double _Ox, double _Oy, double _Oz, double _Dx, double _Dy, double _Dz, out double _T)
        {
            _T = 0;

            Vector3 v1 = _Triangle.V1, v2 = _Triangle.V2, v3 = _Triangle.V3;

            double e1x = v2.x - v1.x, e1y = v2.y - v1.y, e1z = v2.z - v1.z;
            double e2x = v3.x - v1.x, e2y = v3.y - v1.y, e2z = v3.z - v1.z;

            double px = _Dy * e2z - _Dz * e2y;
            double py = _Dz * e2x - _Dx * e2z;
            double pz = _Dx * e2y - _Dy * e2x;

            double det = e1x * px + e1y * py + e1z * pz;

            if (System.Math.Abs(det) < 1e-18)
                return false;

            double invDet = 1.0 / det;

            double tx = _Ox - v1.x, ty = _Oy - v1.y, tz = _Oz - v1.z;

            double u = (tx * px + ty * py + tz * pz) * invDet;

            if (u < -DEGENERATE_BARYCENTRIC || u > 1 + DEGENERATE_BARYCENTRIC)
                return false;

            double qx = ty * e1z - tz * e1y;
            double qy = tz * e1x - tx * e1z;
            double qz = tx * e1y - ty * e1x;

            double v = (_Dx * qx + _Dy * qy + _Dz * qz) * invDet;

            if (v < -DEGENERATE_BARYCENTRIC || u + v > 1 + DEGENERATE_BARYCENTRIC)
                return false;

            _T = (e2x * qx + e2y * qy + e2z * qz) * invDet;

            return _T > 0;
        }

        const int CROSS_NONE       = 0;
        const int CROSS_LEAVING    = 1;
        const int CROSS_ENTERING   = 2;
        const int CROSS_DEGENERATE = 3;

        static bool RayHitsBox(double _Ox, double _Oy, double _Oz, double _InvX, double _InvY, double _InvZ, Vector3 _Min, Vector3 _Max)
        {
            double t1 = (_Min.x - _Ox) * _InvX, t2 = (_Max.x - _Ox) * _InvX;
            double tMin = System.Math.Min(t1, t2), tMax = System.Math.Max(t1, t2);

            t1   = (_Min.y - _Oy) * _InvY;
            t2   = (_Max.y - _Oy) * _InvY;
            tMin = System.Math.Max(tMin, System.Math.Min(t1, t2));
            tMax = System.Math.Min(tMax, System.Math.Max(t1, t2));

            t1   = (_Min.z - _Oz) * _InvZ;
            t2   = (_Max.z - _Oz) * _InvZ;
            tMin = System.Math.Max(tMin, System.Math.Min(t1, t2));
            tMax = System.Math.Min(tMax, System.Math.Max(t1, t2));

            //a small slack keeps a hit exactly on a node's face from being culled by rounding
            return tMax >= System.Math.Max(tMin, 0.0) - 1e-6;
        }

        //Möller–Trumbore in double precision, t > 0 only.
        static int CrossTriangle(Nav3D.Common.Triangle _Triangle, double _Ox, double _Oy, double _Oz, double _Dx, double _Dy, double _Dz)
        {
            Vector3 v1 = _Triangle.V1, v2 = _Triangle.V2, v3 = _Triangle.V3;

            double e1x = v2.x - v1.x, e1y = v2.y - v1.y, e1z = v2.z - v1.z;
            double e2x = v3.x - v1.x, e2y = v3.y - v1.y, e2z = v3.z - v1.z;

            double px = _Dy * e2z - _Dz * e2y;
            double py = _Dz * e2x - _Dx * e2z;
            double pz = _Dx * e2y - _Dy * e2x;

            double det = e1x * px + e1y * py + e1z * pz;

            double e1Length = System.Math.Sqrt(e1x * e1x + e1y * e1y + e1z * e1z);
            double e2Length = System.Math.Sqrt(e2x * e2x + e2y * e2y + e2z * e2z);

            double tx = _Ox - v1.x, ty = _Oy - v1.y, tz = _Oz - v1.z;

            //parallel to the plane: no crossing, unless the ray runs inside the plane through the triangle
            if (System.Math.Abs(det) <= DEGENERATE_COSINE * e1Length * e2Length)
            {
                double nx = e1y * e2z - e1z * e2y, ny = e1z * e2x - e1x * e2z, nz = e1x * e2y - e1y * e2x;
                double nLength = System.Math.Sqrt(nx * nx + ny * ny + nz * nz);

                double planeDistance = nLength > 0 ? System.Math.Abs(tx * nx + ty * ny + tz * nz) / nLength : 0;

                return planeDistance <= 1e-6 * (e1Length + e2Length) ? CROSS_DEGENERATE : CROSS_NONE;
            }

            double invDet = 1.0 / det;

            double u = (tx * px + ty * py + tz * pz) * invDet;

            if (u < -DEGENERATE_BARYCENTRIC || u > 1 + DEGENERATE_BARYCENTRIC)
                return CROSS_NONE;

            double qx = ty * e1z - tz * e1y;
            double qy = tz * e1x - tx * e1z;
            double qz = tx * e1y - ty * e1x;

            double v = (_Dx * qx + _Dy * qy + _Dz * qz) * invDet;

            if (v < -DEGENERATE_BARYCENTRIC || u + v > 1 + DEGENERATE_BARYCENTRIC)
                return CROSS_NONE;

            double t = (e2x * qx + e2y * qy + e2z * qz) * invDet;

            if (t <= 0)
                return CROSS_NONE;

            if (u <= DEGENERATE_BARYCENTRIC || v <= DEGENERATE_BARYCENTRIC || u + v >= 1 - DEGENERATE_BARYCENTRIC)
                return CROSS_DEGENERATE;

            //the triangle's normal is e1 × e2 (Triangle.Normal: (V2 - V1) × (V3 - V2) is the same vector); it points
            //along the ray exactly when det < 0
            return det < 0 ? CROSS_LEAVING : CROSS_ENTERING;
        }

        #endregion
    }
}
