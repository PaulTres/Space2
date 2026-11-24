using UnityEngine;

namespace Nav3D.Common
{
    static class ExtensionObject
    {
        //The object's Unity identifier as text, for log lines and requester names. Unity 6.3 introduced EntityId and
        //later turned GetInstanceID() into a compile error, so the text is the only form the two editor lines share.
        internal static string UnityIdText(this Object _Object)
        {

            #if UNITY_6000_3_OR_NEWER

            return _Object.GetEntityId().ToString();

            #else

            return _Object.GetInstanceID().ToString();

            #endif

        }
    }
}