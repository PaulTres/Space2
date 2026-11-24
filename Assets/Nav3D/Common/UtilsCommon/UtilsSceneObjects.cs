using UnityEngine;

namespace Nav3D.Common
{
    //The scene lookups Nav3D makes, behind the split Unity 6.4 opened: FindFirstObjectByType and the FindObjectsSortMode
    //overloads are deprecated there as instance-id ordered, and their order-free replacements do not exist on 2022.3.
    //Each editor line keeps the call it had, so nothing changes under the authoritative one.
    static class UtilsSceneObjects
    {

        #region Public methods

        internal static T FindAny<T>() where T : Object
        {

            #if UNITY_6000_4_OR_NEWER

            return Object.FindAnyObjectByType<T>();

            #else

            return Object.FindFirstObjectByType<T>();

            #endif

        }

        //Unordered; the parameterless overload is a 6.5 addition (the 6.4 editor deprecates the sort mode and takes it still).
        internal static T[] FindAll<T>() where T : Object
        {

            #if UNITY_6000_5_OR_NEWER

            return Object.FindObjectsByType<T>();

            #else

            return Object.FindObjectsByType<T>(FindObjectsSortMode.None);

            #endif

        }

        #endregion

    }
}