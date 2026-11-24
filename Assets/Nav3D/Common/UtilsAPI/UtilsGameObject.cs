#if UNITY_EDITOR

using UnityEngine;

namespace Nav3D.Common
{
    static class UtilsGameObject
    {
        #region Public methods

        /// <summary>
        /// Creates a GameObject and attaches an instance of component T to it.
        /// </summary>
        public static void InstantiateGOWithComponent<T>(string _Name) where T : MonoBehaviour
        {
            GameObject gameObject = new GameObject(_Name);
            gameObject.AddComponent<T>();
        }

        #endregion
    }
}

#endif