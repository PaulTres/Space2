#if UNITY_EDITOR

using Nav3D.Common.Debug;
using UnityEditor;
using UnityEngine;

namespace Nav3D.API
{
    public partial class Nav3DPathTester
    {
        #region Constants

        const float FAILED_POINT_MARKER_RADIUS = 0.15f;

        #endregion

        #region Unity methods

        void OnDrawGizmos()
        {
            if (!Application.isPlaying || !isActiveAndEnabled || !m_Draw)
                return;

            m_Path?.Draw();

            DrawFailedPoint();
        }

        #endregion

        #region Service methods

        //the endpoint a failed search names (inside an obstacle, too narrow for the radius), so the fix is visible
        void DrawFailedPoint()
        {
            if (LastResult == null || !LastResult.Failed || !LastResult.TryGetFailedPoint(out Vector3 failedPoint))
                return;

            using (UtilsGizmos.ColorPermanence)
            {
                Gizmos.color = Color.red;

                Gizmos.DrawWireSphere(failedPoint, Mathf.Max(FAILED_POINT_MARKER_RADIUS, LastRequest?.Radius ?? 0f));

                Handles.Label(failedPoint, LastResult.Result.ToString(), new GUIStyle { normal = { textColor = Color.red } });
            }
        }

        #endregion
    }
}

#endif
