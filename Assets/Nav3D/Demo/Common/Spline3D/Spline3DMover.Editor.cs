#if UNITY_EDITOR

using UnityEngine;

namespace Nav3D.Demo
{
    public partial class Spline3DMover
    {
        #region Unity events

        //flight debug: pursuit target and current velocity
        void OnDrawGizmosSelected()
        {
            if (!Application.isPlaying || !m_IsPlaying || m_MotionMode != MotionMode.FLIGHT || m_Spline == null)
                return;

            Vector3 carrotPoint = m_Spline.GetPointAtDistance(m_CarrotDistance);

            Gizmos.color = Color.magenta;
            Gizmos.DrawWireSphere(carrotPoint, 0.1f);
            Gizmos.DrawLine(transform.position, carrotPoint);

            Gizmos.color = Color.green;
            Gizmos.DrawRay(transform.position, m_Velocity);
        }

        #endregion
    }
}

#endif
