#if UNITY_EDITOR

namespace Nav3D.API
{
    public partial class Nav3DPath
    {
        public void Draw()
        {
            m_Path?.Draw();
        }
    }
}
#endif
