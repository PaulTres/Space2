#if UNITY_EDITOR

namespace Nav3D.API
{
    public static partial class Nav3DManager
    {
        static partial void InitThreadDispatcherEditor(ref bool _Handled)
        {
            Common.ThreadDispatcher.Instance.Initialize(false);
            _Handled = true;
        }
    }
}
#endif
