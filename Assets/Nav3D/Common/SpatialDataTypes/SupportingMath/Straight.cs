using UnityEngine;

namespace Nav3D.LocalAvoidance.SupportingMath
{
    /// <summary>
    /// Represents straight in space via direction vector and point belonging to line.
    /// </summary>
    class Straight
    {
        #region Attributes

        readonly Vector3 m_Direction;
        readonly Vector3 m_Point;

        #endregion

        #region Construction

        public Straight(Vector3 _Direction, Vector3 _Point)
        {
            m_Direction = _Direction;
            m_Point     = _Point;

            if (_Direction == Vector3.zero)
                Debug.LogWarning("Direction vector must be nonzero");
        }

        #endregion

        #region Public methods

        public Vector3 ClosestPoint(Vector3 _Point)
        {
            return new Plane(m_Direction, _Point).GetClosestPoint(m_Point);
        }

        #endregion
    }
}