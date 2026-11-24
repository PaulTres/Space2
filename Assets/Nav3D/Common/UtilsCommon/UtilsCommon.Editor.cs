#if UNITY_EDITOR
using UnityEngine;
using Object = UnityEngine.Object;

namespace Nav3D.Common
{
    public static partial class UtilsCommon
    {
        static partial void SmartDestroyEditor(Object _Object, ref bool _Handled)
        {
            if (!Application.isPlaying)
            {
                GameObject gameObjectRef = null;

                if (_Object is Component component)
                    gameObjectRef = component.gameObject;

                Object.DestroyImmediate(_Object);

                if (gameObjectRef != null)
                    Object.DestroyImmediate(gameObjectRef);

                _Handled = true;
            }
        }
    }
}
#endif
