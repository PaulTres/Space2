#if UNITY_EDITOR

using UnityEngine;

namespace Nav3D.Demo
{
    public partial class Spline3D
    {
        #region Unity events

        void OnDrawGizmos()
        {
            if (PointCount < 2)
                return;

            Vector3[] points = GetSplinePoints();

            Gizmos.color = Color.yellow;

            for (int i = 0; i < points.Length - 1; i++)
                Gizmos.DrawLine(points[i], points[i + 1]);

            Gizmos.color = Color.cyan;

            foreach (Vector3 point in points)
                Gizmos.DrawSphere(point, 0.03f);

            Gizmos.color = Color.red;

            for (int i = 0; i < PointCount; i++)
                Gizmos.DrawWireSphere(GetPoint(i), 0.08f);
        }

        #endregion
    }
}

#endif
