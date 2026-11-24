using System;
using UnityEngine;

namespace Nav3D.Common
{
    public partial class Triangle : IBoundable, IEquatable<Triangle>
    {
        #region Properties

        public Vector3 V1     { get; }
        public Vector3 V2     { get; }
        public Vector3 V3     { get; }
        public Vector3 Normal { get; }
        public Bounds  Bounds { get; }
        public Plane   Plane  { get; }
        public Vector3 Center => (V1 + V2 + V3) / 3f;
        
        Vector3 V12 { get; }
        Vector3 V23 { get; }
        
        #endregion

        #region Constructors

        public Triangle(Vector3 _V1, Vector3 _V2, Vector3 _V3)
        {
            V1 = _V1;
            V2 = _V2;
            V3 = _V3;

            V12 = V2 - V1;
            V23 = V3 - V2;

            Normal = Vector3.Cross(V12, V23).normalized;

            //Compute bounds
            float minX = float.PositiveInfinity;
            float maxX = float.NegativeInfinity;
            float minY = float.PositiveInfinity;
            float maxY = float.NegativeInfinity;
            float minZ = float.PositiveInfinity;
            float maxZ = float.NegativeInfinity;

            if (V1.x < minX) minX = V1.x;
            if (V1.x > maxX) maxX = V1.x;

            if (V2.x < minX) minX = V2.x;
            if (V2.x > maxX) maxX = V2.x;

            if (V3.x < minX) minX = V3.x;
            if (V3.x > maxX) maxX = V3.x;

            if (V1.y < minY) minY = V1.y;
            if (V1.y > maxY) maxY = V1.y;

            if (V2.y < minY) minY = V2.y;
            if (V2.y > maxY) maxY = V2.y;

            if (V3.y < minY) minY = V3.y;
            if (V3.y > maxY) maxY = V3.y;

            if (V1.z < minZ) minZ = V1.z;
            if (V1.z > maxZ) maxZ = V1.z;

            if (V2.z < minZ) minZ = V2.z;
            if (V2.z > maxZ) maxZ = V2.z;

            if (V3.z < minZ) minZ = V3.z;
            if (V3.z > maxZ) maxZ = V3.z;

            Bounds = ExtensionBounds.MinMax(new Vector3(minX, minY, minZ), new Vector3(maxX, maxY, maxZ));

            Plane = new Plane(V1, V2, V3);
        }

        #endregion

        #region Public methods
        
        //Returns the closest point on the triangle to a given point (using barycentric coordinates)
        public Vector3 ClosestPointOnTriangle(Vector3 _Point)
        {
            //Compute vectors
            Vector3 ab = V2     - V1;
            Vector3 ac = V3     - V1;
            Vector3 ap = _Point - V1;

            //Compute dot products
            float d1 = Vector3.Dot(ab, ap);
            float d2 = Vector3.Dot(ac, ap);

            //Check if P in vertex region outside A
            if (d1 <= 0f && d2 <= 0f)
                return V1;

            //Check if P in vertex region outside B
            Vector3 bp = _Point - V2;
            float   d3 = Vector3.Dot(ab, bp);
            float   d4 = Vector3.Dot(ac, bp);
            if (d3 >= 0f && d4 <= d3)
                return V2;

            //Check if P in edge region of AB
            float vc = d1 * d4 - d3 * d2;
            if (vc <= 0f && d1 >= 0f && d3 <= 0f)
            {
                float v = d1 / (d1 - d3);
                return V1 + v * ab;
            }

            //Check if P in vertex region outside C
            Vector3 cp = _Point - V3;
            float   d5 = Vector3.Dot(ab, cp);
            float   d6 = Vector3.Dot(ac, cp);
            if (d6 >= 0f && d5 <= d6)
                return V3;

            //Check if P in edge region of AC
            float vb = d5 * d2 - d1 * d6;
            if (vb <= 0f && d2 >= 0f && d6 <= 0f)
            {
                float w = d2 / (d2 - d6);
                return V1 + w * ac;
            }

            //Check if P in edge region of BC
            float va = d3 * d6 - d5 * d4;
            if (va <= 0f && (d4 - d3) >= 0f && (d5 - d6) >= 0f)
            {
                float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
                return V2 + w * (V3 - V2);
            }

            //P inside face region. Compute Q through barycentric coordinates
            float denom  = 1f / (va + vb + vc);
            float vFinal = vb * denom;
            float wFinal = vc * denom;
            return V1 + ab * vFinal + ac * wFinal;
        }
        
        public bool Intersects(Bounds _Bounds)
        {
            if (_Bounds.DoesNotIntersects(Bounds))
                return false;

            return TriangleCubeIntersectionChecker.Inside(this, _Bounds);
        }

        #endregion

        #region Equality

        public bool Equals(Triangle _Other)
        {
            if (_Other is null)
                return false;
            
            if (ReferenceEquals(this, _Other))
                return true;
            
            return V1 == _Other.V1 && V2 == _Other.V2 && V3 == _Other.V3;
        }

        public override bool Equals(object _Obj)
        {
            return Equals(_Obj as Triangle);
        }

        public override int GetHashCode()
        {
            unchecked
            {
                int hash = V1.GetHashCode();
                hash = hash * 397 ^ V2.GetHashCode();
                hash = hash * 397 ^ V3.GetHashCode();
                return hash;
            }
        }

        #endregion
    }
}