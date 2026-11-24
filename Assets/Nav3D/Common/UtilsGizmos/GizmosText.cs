#if UNITY_EDITOR

using UnityEditor;
using UnityEngine;

namespace Nav3D.Common.Debug
{
    class GizmosLabel : IDrawable
    {
        #region Attributes

        GUIStyle m_Style;
        Vector3  m_Position;
        string   m_Text;

        #endregion

        #region Properties

        #endregion

        #region Constructors

        public GizmosLabel(Vector3 _Position, string _Text, GUIStyle _Style)
        {
            m_Position = _Position;
            m_Style    = _Style;
            m_Text     = _Text;
        }

        #endregion

        #region IDrawable

        public void Draw()
        {
            Handles.Label(m_Position, m_Text, m_Style);
        }

        #endregion
    }
}

#endif