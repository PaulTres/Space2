using UnityEngine;

namespace Nav3D.Common
{
    static class Singleton<T> where T : MonoBehaviour
    {

        #region Attributes

        static T s_Instance;

        //ReSharper disable once StaticMemberInGenericType
        static readonly object s_Lock = new object();

        #endregion

        #region Properties

        public static T Instance => GetInstance();

        //Returns the already-resolved instance without resolving a new one. Unlike Instance/GetInstance this
        //never runs a scene lookup or creates a GameObject, so it is safe to call from worker threads.
        //May return null (never resolved) or a destroyed Unity object — callers must Unity null-check (!= null).
        public static T InstanceIfResolved => s_Instance;

        #endregion

        #region Public methods

        //Drops the static reference when the object being destroyed is the currently-referenced instance. Every
        //Singleton<T> consumer's OnDestroy must call this with itself. Without it, Object.Destroy tears down the
        //Unity-side object but s_Instance keeps a strong C# reference to the (now fake-null) wrapper, which keeps
        //everything it transitively references (e.g. the whole obstacle octree + baked terrain cache) GC-rooted
        //until the next GetInstance() call happens to overwrite the field — which may not happen for the rest of
        //the Editor session after Play stops, reading as unreleased memory.
        //ReferenceEquals (not ==) is deliberate: this must be a pure C# identity check, independent of Unity's
        //overloaded null-comparison semantics for a (soon-to-be) destroyed object — and it guards against clearing
        //a NEWER instance that was already resolved into s_Instance before this (older, already-superseded) one's
        //OnDestroy ran.
        public static void ClearInstance(T _Instance)
        {
            lock (s_Lock)
            {
                if (ReferenceEquals(s_Instance, _Instance))
                    s_Instance = null;
            }
        }

        public static T GetInstance(string _Name = null)
        {
            if (s_Instance == null)
            {
                lock (s_Lock)
                {
                    if (s_Instance == null)
                    {
                        //try find already created singleton in scene
                        s_Instance = UtilsSceneObjects.FindAny<T>();

                        //try find singleton by name in edit mode because all this have flag DontSave
                        if (s_Instance == null && !Application.isPlaying)
                        {
                            GameObject go = GameObject.Find(_Name ?? typeof(T).FullName);

                            if (go != null)
                                s_Instance = go.GetComponent<T>();
                        }

                        //create new instance of singleton
                        if (s_Instance == null)
                        {
                            GameObject go = new GameObject(_Name ?? typeof(T).FullName);

                            //do not destroy this singletons
                            if (Application.isPlaying)
                                Object.DontDestroyOnLoad(go);
                            else // set this flag only for singletons in edit mode
                                go.hideFlags |= HideFlags.DontSave;

                            //add component and return it
                            s_Instance = go.AddComponent<T>();
                        }
                    }

                    return s_Instance;
                }
            }

            return s_Instance;
        }

        #endregion

    }
}