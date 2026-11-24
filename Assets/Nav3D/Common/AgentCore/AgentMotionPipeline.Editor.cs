#if UNITY_EDITOR

using UnityEngine;

namespace Nav3D.Agents
{
    //editor-only debug drawing of the pipeline: merged former Nav3DAgentMover/MoverGlobal/MoverCombined draws
    public partial class AgentMotionPipeline
    {
        #region Attributes

        Vector3 m_BlendedVelocity;

        #endregion

        #region Public methods

        public void Draw(bool _DrawRadius, bool _DrawPath, bool _DrawVelocities)
        {
            //the route itself is Nav3DAgentPathDrawer's; the gizmo keeps only the point the agent is heading for
            if (_DrawPath)
            {
                Gizmos.color = Color.yellow;
                Gizmos.DrawLine(m_CurrentPosition, m_PathNextPoint);
                Gizmos.DrawSphere(m_PathNextPoint, 0.025f);
            }

            if (_DrawVelocities && m_AvoidanceStage != null)
            {
                Gizmos.color = Color.yellow;
                Gizmos.DrawLine(m_CurrentPosition, m_CurrentPosition + VPref.normalized);

                Gizmos.color = Color.green;
                Gizmos.DrawLine(m_CurrentPosition, m_CurrentPosition + m_AvoidanceStage.LastAvoidanceVelocity.normalized);

                Gizmos.color = Color.blue;
                Gizmos.DrawLine(m_CurrentPosition, m_CurrentPosition + m_BlendedVelocity.normalized);
            }

            if (_DrawRadius)
            {
                Gizmos.color = Color.white;
                Gizmos.DrawWireSphere(m_CurrentPosition, m_Config.Radius);
            }
        }

        public void DrawNearestMovers()
        {
            m_AvoidanceStage?.DrawNearestMovers();
        }

        public void DrawNearestTriangles()
        {
            m_AvoidanceStage?.DrawNearestTriangles();
        }

        public void DrawTrianglesUpdateInfo()
        {
            m_AvoidanceStage?.DrawTrianglesUpdateInfo();
        }

        #endregion

        #region Service methods

        partial void SetBlendedVelocityDebug(Vector3 _Velocity)
        {
            m_BlendedVelocity = _Velocity;
        }

        #endregion
    }
}

#endif
