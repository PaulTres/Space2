using UnityEditor;

namespace Nav3D.API.Editor
{
    /// <summary>
    /// A foldout's expanded/collapsed state, kept in EditorPrefs so it survives selection changes and domain
    /// reloads — a state held in an inspector field does not, because Unity builds a fresh editor instance per
    /// selected object. Declare one static readonly instance per foldout and drive the foldout through Value.
    /// </summary>
    public sealed class EditorFoldoutState
    {
        #region Constants

        const string KEY_PREFIX = "Nav3D.Foldout.";

        #endregion

        #region Attributes

        readonly string m_Key;
        readonly bool   m_DefaultValue;

        #endregion

        #region Properties

        public bool Value
        {
            get => EditorPrefs.GetBool(m_Key, m_DefaultValue);
            set
            {
                //the getter runs on every repaint and the foldout assigns its result back every time, so
                //write only on an actual change — EditorPrefs is a disk-backed store
                if (EditorPrefs.GetBool(m_Key, m_DefaultValue) == value)
                    return;

                EditorPrefs.SetBool(m_Key, value);
            }
        }

        #endregion

        #region Constructors

        /// <param name="_Key">
        /// Identifies the foldout project-wide. By convention "Inspector.Foldout", e.g. "AgentConfig.Pathfinding".
        /// </param>
        /// <param name="_DefaultValue">What a user who has never touched this foldout sees.</param>
        public EditorFoldoutState(string _Key, bool _DefaultValue = false)
        {
            m_Key          = KEY_PREFIX + _Key;
            m_DefaultValue = _DefaultValue;
        }

        #endregion
    }
}
