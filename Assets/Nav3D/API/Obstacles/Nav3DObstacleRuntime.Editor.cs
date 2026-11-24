#if UNITY_EDITOR
using UnityEngine;

namespace Nav3D.API
{
    public partial class Nav3DObstacleRuntime
    {
        partial void CheckEditorDoEnableGuard(ref bool _Skip)
        {
            if (!Application.isPlaying || !enabled)
                _Skip = true;
        }

        partial void CheckEditorDoUpdateGuard(ref bool _Skip)
        {
            if (!Application.isPlaying || !enabled)
                _Skip = true;
        }
    }
}
#endif
