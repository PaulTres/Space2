using Nav3D.API;
using UnityEngine;

namespace Nav3D.Demo
{
    //The whole answer the search returned, drawn end to end — with smoothing off its corners are the
    //octree cells it was built from, with smoothing on it is the line the courier actually flies.
    public class PathFollowingRoute : MonoBehaviour
    {
        #region Serialized fields

        [SerializeField] LineRenderer m_Line;
        [SerializeField] Nav3DAgent   m_Agent;

        #endregion

        #region Unity events

        void Start()
        {
            m_Line.positionCount = 0;
        }

        #endregion

        #region Public methods

        public void Trace(bool _HasOrder)
        {
            Vector3[] path = _HasOrder ? m_Agent.CurrentPath : null;

            if (path == null || path.Length < 2)
            {
                m_Line.positionCount = 0;

                return;
            }

            m_Line.positionCount = path.Length;
            m_Line.SetPositions(path);
        }

        #endregion
    }
}
