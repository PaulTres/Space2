#if UNITY_EDITOR

using UnityEditor;
using UnityEngine;
using Nav3D.Common.Debug;

namespace Nav3D.API
{
    public partial class Nav3DAgent
    {
        #region Serialized fields

        [Space, Header("Draw agent operating properties")]
        [SerializeField] bool m_DrawRadius;

        [SerializeField] bool m_DrawVelocities;
        [SerializeField] bool m_DrawCurrentPath;

        [Space, Header("Show agent nearest avoidance targets")]
        [SerializeField] bool m_ShowAgents;

        [SerializeField] bool m_DrawNearestTriangles;
        [SerializeField] bool m_DrawTrianglesUpdateInfo;

        #endregion

        #region Public methods

        public void Draw(bool _DrawRadius, bool _DrawPath, bool _DrawVelocities)
        {
            using (UtilsGizmos.ColorPermanence)
            {
                m_Mover?.Draw(_DrawRadius, _DrawPath, _DrawVelocities);
            }
        }
        
        internal void DrawNearestTriangles()
        {
            m_Mover?.DrawNearestTriangles();
        }
        
        internal void DrawTrianglesUpdateInfo()
        {
            m_Mover?.DrawTrianglesUpdateInfo();
        }

        #endregion

        #region Service methods

        void DrawNearestMovers()
        {
            m_Mover?.DrawNearestMovers();
        }

        partial void SyncPathDrawing(bool _Enabled)
        {
            if (_Enabled && m_DrawCurrentPath)
            {
                Nav3DAgentPathDrawer.Instance.Include(this);

                return;
            }

            Nav3DAgentPathDrawer drawer = Nav3DAgentPathDrawer.InstanceIfResolved;

            if (drawer != null)
                drawer.Exclude(this);
        }

        #endregion

        #region Unity methods

        //the flag ticked in the inspector mid-play; deferred, because OnValidate may not create the drawer's object
        void OnValidate()
        {
            if (!Application.isPlaying)
                return;

            EditorApplication.delayCall += () =>
                {
                    if (this != null)
                        SyncPathDrawing(isActiveAndEnabled);
                };
        }

        void OnDrawGizmos()
        {
            if (!enabled)
                return;

            if (!Application.isPlaying)
            {
                Gizmos.color = Color.white;

                if (m_DrawRadius && m_Config != null)
                    Gizmos.DrawWireSphere(transform.position, m_Config.Radius);
            }
            else
            {
                using (UtilsGizmos.ColorPermanence)
                {
                    if (m_DrawRadius || m_DrawVelocities || m_DrawCurrentPath)
                        Draw(m_DrawRadius, m_DrawCurrentPath, m_DrawVelocities);

                    if (m_ShowAgents)
                        DrawNearestMovers();

                    if (m_DrawNearestTriangles)
                        DrawNearestTriangles();

                    if (m_DrawTrianglesUpdateInfo)
                        DrawTrianglesUpdateInfo();
                }
            }
        }

        #endregion
    }
}

#endif