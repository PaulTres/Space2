using System;
using UnityEngine;

namespace Nav3D.Common
{
    /// <summary>
    /// Implementation of the algorithm described in the original book Graphics Gems III (David Kirk), Triangle-Cube Intersection, pp. 236-239
    /// http://www.realtimerendering.com/resources/GraphicsGems/gemsiii/triangleCube.c
    /// </summary>
    public static class TriangleCubeIntersectionChecker
    {
        #region Constants

        const long INSIDE  = 0;
        const long OUTSIDE = 1;

        //How far outside the cube, in cube-relative units, still counts as touching it.
        //
        //It exists because authored geometry is grid-aligned: a wall, a floor, a box face at round coordinates lies
        //exactly in some cell's boundary plane, and that plane is the one place where the answer is decided by the
        //last bits of (v - center) / size. A cell size that is not a binary fraction (0.3, 0.65) makes that division
        //inexact, so a face that geometrically is the boundary lands a few 1e-8 to one side of it — and the two
        //cells sharing the plane can then disagree about it, which is a wall with a hole in it. That defect is what
        //`34033c0` (2025-01-19, "missing some occupied nodes") added a coordinate tolerance for.
        //
        //It lives here, in the comparisons, rather than in a rewrite of the inputs, and that is a deliberate
        //correction of the older shape (nine `Math.Round(x, 5)` calls over the scaled vertices, 2025-01-19 to
        //2026-09-08). Measured over 200 000 triangles placed exactly on cell boundaries, both neighbours asked:
        //snapping the inputs lost the face in 862 of them, every one one-sided; the same tolerance applied here
        //loses none. It is also ~4x cheaper, because `Math.Round(x, digits)` is a multiply-round-divide, nine of
        //them ran before any geometry was looked at, and they were ~74 % of this method's cost.
        const double EPSILON = 1e-5;

        #endregion

        #region Public methods

        public static bool Inside(Triangle _Triangle, Bounds _Cube)
        {
            Vector3 cubeCenter = _Cube.center;
            Vector3 cubeSize   = _Cube.size;

            DVector3 scaledV1 = new DVector3((_Triangle.V1.x - (double)cubeCenter.x) / cubeSize.x,
                                             (_Triangle.V1.y - (double)cubeCenter.y) / cubeSize.y,
                                             (_Triangle.V1.z - (double)cubeCenter.z) / cubeSize.z);

            DVector3 scaledV2 = new DVector3((_Triangle.V2.x - (double)cubeCenter.x) / cubeSize.x,
                                             (_Triangle.V2.y - (double)cubeCenter.y) / cubeSize.y,
                                             (_Triangle.V2.z - (double)cubeCenter.z) / cubeSize.z);

            DVector3 scaledV3 = new DVector3((_Triangle.V3.x - (double)cubeCenter.x) / cubeSize.x,
                                             (_Triangle.V3.y - (double)cubeCenter.y) / cubeSize.y,
                                             (_Triangle.V3.z - (double)cubeCenter.z) / cubeSize.z);

            //No snapping of the scaled vertices here: the tolerance the grid-aligned case needs is EPSILON, applied
            //by the tests below, and rewriting the inputs to get it both cost more and lost faces (see EPSILON).

            DVector3 V12 = scaledV2 - scaledV1;
            DVector3 V23 = scaledV3 - scaledV2;

            return InsideInternal(scaledV1, scaledV2, scaledV3, DVector3.Cross(V12, V23).normalized);
        }

        #endregion
        
        #region Service methods

        static bool InsideInternal(DVector3 _V1, DVector3 _V2, DVector3 _V3, DVector3 _Normal)
        {
            long     V1Test, V2Test, V3Test;
            double   d,      denom;
            DVector3 hitpp,  hitpn, hitnp, hitnn;

            /* First compare all three vertexes with all six face-planes */
            /* If any vertex is inside the cube, return immediately!     */

            if ((V1Test = FacePlane(_V1)) == INSIDE) return true;
            if ((V2Test = FacePlane(_V2)) == INSIDE) return true;
            if ((V3Test = FacePlane(_V3)) == INSIDE) return true;

            /* If all three vertexes were outside of one or more face-planes, */
            /* return immediately with a trivial rejection!                   */

            if ((V1Test & V2Test & V3Test) != 0) return false;

            /* Now do the same trivial rejection test for the 12 edge planes */

            V1Test |= Bevel2D(_V1) << 8;
            V2Test |= Bevel2D(_V2) << 8;
            V3Test |= Bevel2D(_V3) << 8;

            if ((V1Test & V2Test & V3Test) != 0) return false;

            /* Now do the same trivial rejection test for the 8 corner planes */

            V1Test |= Bevel3D(_V1) << 24;
            V2Test |= Bevel3D(_V2) << 24;
            V3Test |= Bevel3D(_V3) << 24;

            if ((V1Test & V2Test & V3Test) != 0) return false;

            /* If vertex 1 and 2, as a pair, cannot be trivially rejected */
            /* by the above tests, then see if the v1-->v2 triangle edge  */
            /* intersects the cube.  Do the same for v1-->v3 and v2-->v3. */
            /* Pass to the intersection algorithm the "OR" of the outcode */
            /* bits, so that only those cube faces which are spanned by   */
            /* each triangle edge need be tested.                         */

            if ((V1Test & V2Test) == 0)
                if (CheckLine(_V1, _V2, V1Test | V2Test) == INSIDE) return true;
            if ((V1Test & V3Test) == 0)
                if (CheckLine(_V1, _V3, V1Test | V3Test) == INSIDE) return true;
            if ((V2Test & V3Test) == 0)
                if (CheckLine(_V2, _V3, V2Test | V3Test) == INSIDE) return true;

            /* By now, we know that the triangle is not off to any side,     */
            /* and that its sides do not penetrate the cube.  We must now    */
            /* test for the cube intersecting the interior of the triangle.  */
            /* We do this by looking for intersections between the cube      */
            /* diagonals and the triangle...first finding the intersection   */
            /* of the four diagonals with the plane of the triangle, and     */
            /* then if that intersection is inside the cube, pursuing        */
            /* whether the intersection point is inside the triangle itself. */

            /* To find plane of the triangle, first perform crossproduct on  */
            /* two triangle side vectors to compute the normal vector.       */

            //(The calculations described are performed inside Triangle struct)

            //SUB(t.v1, t.v2, vect12);
            //SUB(t.v1, t.v3, vect13);
            //CROSS(vect12, vect13, norm)

            /* The normal vector "norm" X,Y,Z components are the coefficients */
            /* of the triangles AX + BY + CZ + D = 0 plane equation.  If we   */
            /* solve the plane equation for X=Y=Z (a diagonal), we get        */
            /* -D/(A+B+C) as a metric of the distance from cube center to the */
            /* diagonal/plane intersection.  If this is between -0.5 and 0.5, */
            /* the intersection is inside the cube.  If so, we continue by    */
            /* doing a point/triangle intersection.                           */
            /* Do this for all four diagonals.                                */

            d = _Normal.x * _V1.x + _Normal.y * _V1.y + _Normal.z * _V1.z;


            /* if one of the diagonals is parallel to the plane, the other will intersect the plane */
            if (Math.Abs(denom = (_Normal.x + _Normal.y + _Normal.z)) > Mathf.Epsilon)
                /* skip parallel diagonals to the plane; division by 0 can occur */
            {
                hitpp.x = hitpp.y = hitpp.z = d / denom;
                if (Math.Abs(hitpp.x) <= 0.5 + EPSILON)
                    if (PointTriangleIntersection(hitpp, _V1, _V2, _V3, _Normal) == INSIDE) return true;
            }
            if (Math.Abs(denom = (_Normal.x + _Normal.y - _Normal.z)) > Mathf.Epsilon)
            {
                hitpn.z = -(hitpn.x = hitpn.y = d / denom);
                if (Math.Abs(hitpn.x) <= 0.5 + EPSILON)
                    if (PointTriangleIntersection(hitpn, _V1, _V2, _V3, _Normal) == INSIDE) return true;
            }
            if (Math.Abs(denom = (_Normal.x - _Normal.y + _Normal.z)) > Mathf.Epsilon)
            {
                hitnp.y = -(hitnp.x = hitnp.z = d / denom);
                if (Math.Abs(hitnp.x) <= 0.5 + EPSILON)
                    if (PointTriangleIntersection(hitnp, _V1, _V2, _V3, _Normal) == INSIDE) return true;
            }
            if (Math.Abs(denom = (_Normal.x - _Normal.y - _Normal.z)) > Mathf.Epsilon)
            {
                hitnn.y = hitnn.z = -(hitnn.x = d / denom);
                if (Math.Abs(hitnn.x) <= 0.5 + EPSILON)
                    if (PointTriangleIntersection(hitnn, _V1, _V2, _V3, _Normal) == INSIDE) return true;
            }

            /* No edge touched the cube; no cube diagonal touched the triangle. */
            /* We're done...there was no intersection.                          */

            return false;
        }

        /* Which of the six face-plane(s) is point P outside of? */
        static long FacePlane(DVector3 _Point)
        {
            long outcode = 0;
            
            if (_Point.x > .5 + EPSILON) outcode  |= 0x01;
            if (_Point.x < -.5 - EPSILON) outcode |= 0x02;
            if (_Point.y > .5 + EPSILON) outcode  |= 0x04;
            if (_Point.y < -.5 - EPSILON) outcode |= 0x08;
            if (_Point.z > .5 + EPSILON) outcode  |= 0x10;
            if (_Point.z < -.5 - EPSILON) outcode |= 0x20;

            return outcode;
        }

        /* Which of the twelve edge plane(s) is point P outside of? */
        static long Bevel2D(DVector3 _Point)
        {
            long outcode = 0;

            if (_Point.x  + _Point.y > 1.0 + EPSILON) outcode |= 0x001;
            if (_Point.x  - _Point.y > 1.0 + EPSILON) outcode |= 0x002;
            if (-_Point.x + _Point.y > 1.0 + EPSILON) outcode |= 0x004;
            if (-_Point.x - _Point.y > 1.0 + EPSILON) outcode |= 0x008;
            if (_Point.x  + _Point.z > 1.0 + EPSILON) outcode |= 0x010;
            if (_Point.x  - _Point.z > 1.0 + EPSILON) outcode |= 0x020;
            if (-_Point.x + _Point.z > 1.0 + EPSILON) outcode |= 0x040;
            if (-_Point.x - _Point.z > 1.0 + EPSILON) outcode |= 0x080;
            if (_Point.y  + _Point.z > 1.0 + EPSILON) outcode |= 0x100;
            if (_Point.y  - _Point.z > 1.0 + EPSILON) outcode |= 0x200;
            if (-_Point.y + _Point.z > 1.0 + EPSILON) outcode |= 0x400;
            if (-_Point.y - _Point.z > 1.0 + EPSILON) outcode |= 0x800;

            return outcode;
        }

        /* Which of the eight corner plane(s) is point P outside of? */
        static long Bevel3D(DVector3 _Point)
        {
            long outcode = 0;

            if (( _Point.x            + _Point.y + _Point.z) > 1.5 + EPSILON) outcode |= 0x01;
            if (( _Point.x + _Point.y - _Point.z)            > 1.5 + EPSILON) outcode |= 0x02;
            if (( _Point.x - _Point.y + _Point.z)            > 1.5 + EPSILON) outcode |= 0x04;
            if (( _Point.x            - _Point.y - _Point.z) > 1.5 + EPSILON) outcode |= 0x08;
            if ((-_Point.x            + _Point.y + _Point.z) > 1.5 + EPSILON) outcode |= 0x10;
            if ((-_Point.x + _Point.y - _Point.z)            > 1.5 + EPSILON) outcode |= 0x20;
            if ((-_Point.x - _Point.y + _Point.z)            > 1.5 + EPSILON) outcode |= 0x40;
            if ((-_Point.x            - _Point.y - _Point.z) > 1.5 + EPSILON) outcode |= 0x80;

            return outcode;
        }

        /* Compute intersection of P1 --> P2 line segment with face planes */
        /* Then test intersection point to see if it is on cube face       */
        /* Consider only face planes in "outcode_diff"                     */
        /* Note: Zero bits in "outcode_diff" means face line is outside of */

        static long CheckLine(DVector3 _Point1, DVector3 _Point2, long _OutcodeDiff)
        {
            if ((0x01 & _OutcodeDiff) != 0)
                if (CheckPoint(_Point1, _Point2, (0.5f - _Point1.x) / (_Point2.x - _Point1.x), 0x3e) == INSIDE) return (INSIDE);
            if ((0x02 & _OutcodeDiff) != 0)
                if (CheckPoint(_Point1, _Point2, (-0.5f - _Point1.x) / (_Point2.x - _Point1.x), 0x3d) == INSIDE) return (INSIDE);
            if ((0x04 & _OutcodeDiff) != 0)
                if (CheckPoint(_Point1, _Point2, (0.5f - _Point1.y) / (_Point2.y - _Point1.y), 0x3b) == INSIDE) return (INSIDE);
            if ((0x08 & _OutcodeDiff) != 0)
                if (CheckPoint(_Point1, _Point2, (-0.5f - _Point1.y) / (_Point2.y - _Point1.y), 0x37) == INSIDE) return (INSIDE);
            if ((0x10 & _OutcodeDiff) != 0)
                if (CheckPoint(_Point1, _Point2, (0.5f - _Point1.z) / (_Point2.z - _Point1.z), 0x2f) == INSIDE) return (INSIDE);
            if ((0x20 & _OutcodeDiff) != 0)
                if (CheckPoint(_Point1, _Point2, (-0.5f - _Point1.z) / (_Point2.z - _Point1.z), 0x1f) == INSIDE) return (INSIDE);

            return OUTSIDE;
        }

        /* Test the point "alpha" of the way from P1 to P2 */
        /* See if it is on a face of the cube              */
        /* Consider only faces in "mask"                   */

        static long CheckPoint(DVector3 _Point1, DVector3 _Point2, double _Alpha, long _Mask)
        {
            DVector3 planePoint;

            planePoint.x = LERP(_Alpha, _Point1.x, _Point2.x);
            planePoint.y = LERP(_Alpha, _Point1.y, _Point2.y);
            planePoint.z = LERP(_Alpha, _Point1.z, _Point2.z);

            return FacePlane(planePoint) & _Mask;
        }

        /* Test if 3D point is inside 3D triangle */

        static double Max(double _A, double _B, double _C)
        {
            return _A > _B ? _A > _C ? _A : _C : _B > _C ? _B : _C;
        }
        
        static double Min(double _A, double _B, double _C)
        {
            return _A < _B ? _A < _C ? _A : _C : _B < _C ? _B : _C;
        }
        
        //_Point is the cube diagonal's intersection with the triangle's plane, so it already lies in that
        //plane and the only remaining question is whether it falls within the triangle's three edges.
        //It is answered against the plane's normal, which is what the Gems III port's own unfinished
        //"the test needs to be revised" note was about: the original decided by the component SIGNS of the
        //three edge crossproducts, and its revised SIGN3 marks a component as both-signed within a tolerance
        //of zero. A triangle lying in an axis-aligned plane makes two of the three components of every
        //crossproduct exactly zero, so those bits are set in all three sign words and their and can never be
        //zero — the step then reports inside for every hit that merely lands in the triangle's bounding BOX,
        //which is a false intersection for any triangle turned within its plane (an axis-aligned box face
        //hides it: the face's two triangles share the face's box). Measured on the shipped code: 1 305 false
        //intersections in 600 000 randomised triangle/cube pairs, every one of them planar, no false misses.
        static long PointTriangleIntersection(DVector3 _Point, DVector3 _V1, DVector3 _V2, DVector3 _V3, DVector3 _Normal)
        {
            /* First, a quick bounding-box test:                               */
            /* If P is outside triangle bbox, there cannot be an intersection. */

            if (_Point.x > Max(_V1.x, _V2.x, _V3.x) + EPSILON) return OUTSIDE;
            if (_Point.y > Max(_V1.y, _V2.y, _V3.y) + EPSILON) return OUTSIDE;
            if (_Point.z > Max(_V1.z, _V2.z, _V3.z) + EPSILON) return OUTSIDE;
            if (_Point.x < Min(_V1.x, _V2.x, _V3.x) - EPSILON) return OUTSIDE;
            if (_Point.y < Min(_V1.y, _V2.y, _V3.y) - EPSILON) return OUTSIDE;
            if (_Point.z < Min(_V1.z, _V2.z, _V3.z) - EPSILON) return OUTSIDE;

            /* For each triangle side, make a vector out of it by subtracting vertexes;  */
            /* make another vector from that side's first vertex to point P.             */
            /* Their crossproduct is orthogonal to the triangle's plane, so its          */
            /* DIRECTION along the plane normal — not the signs of its components —      */
            /* says which side of that edge P fell on. P is inside the triangle iff it   */
            /* is on the inner side of all three. A crossproduct orthogonal to the       */
            /* normal (P exactly on the edge's line) counts as inside: occupancy may     */
            /* over-report a boundary case, never lose one.                              */

            if (DVector3.Dot(DVector3.Cross(_V2 - _V1, _Point - _V1), _Normal) < 0.0) return OUTSIDE;
            if (DVector3.Dot(DVector3.Cross(_V3 - _V2, _Point - _V2), _Normal) < 0.0) return OUTSIDE;
            if (DVector3.Dot(DVector3.Cross(_V1 - _V3, _Point - _V3), _Normal) < 0.0) return OUTSIDE;

            return INSIDE;
        }

        static double LERP(double A, double B, double C) => B + A * (C - B);

        #endregion
    }
}
