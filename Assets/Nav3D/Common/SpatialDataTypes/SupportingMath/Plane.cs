using UnityEngine;

//The namespace is LocalAvoidance's, the assembly is Common's. What is left of this module is two pure geometry
//types whose surviving production consumer is PathFollowData, several layers below local avoidance; the names
//stayed because they are public.
namespace Nav3D.LocalAvoidance.SupportingMath
{
    /// <summary>
    /// Represents plane in space.
    /// A(x-x0) + B(y-y0) + C(z-z0) = 0
    /// or
    /// Ax + By + Cz + D = 0
    /// </summary>
    public class Plane
    {
        #region Attributes

        readonly float   m_Distance;
        readonly Vector3 m_Normal;

        #endregion

        #region Construction

        public Plane(Vector3 _Normal, Vector3 _Point)
        {
            m_Normal   = _Normal.normalized;
            m_Distance = -Vector3.Dot(m_Normal, _Point);
        }

        #endregion

        #region Public methods

        public float DistanceToPoint(Vector3 _Point)
        {
            return Vector3.Dot(m_Normal, _Point) + m_Distance;
        }

        public (Vector3 Point, float Distance) GetSurfaceClosestPoint(Vector3 _Point)
        {
            float distance = DistanceToPoint(_Point);
            return (_Point - m_Normal * distance, distance);
        }

        public Vector3 GetClosestPoint(Vector3 _Point)
        {
            return GetSurfaceClosestPoint(_Point).Point;
        }

        #endregion
    }
}