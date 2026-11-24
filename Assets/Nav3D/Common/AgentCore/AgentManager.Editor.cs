#if UNITY_EDITOR

using UnityEngine;

namespace Nav3D.LocalAvoidance
{
    public partial class AgentManager
    {
        #region Serialized fields

        [SerializeField] bool m_DrawAgentStorage;
        
        #endregion
        
        #region Unity methods

        void OnDrawGizmos()
        {
            using (Common.Debug.UtilsGizmos.ColorPermanence)
            {
                if (m_DrawAgentStorage)
                    m_AgentsStorage.Draw();
            }
        }
        
        #endregion
    }
}

#endif