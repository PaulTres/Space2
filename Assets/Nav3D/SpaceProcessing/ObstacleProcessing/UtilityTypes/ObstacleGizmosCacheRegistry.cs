#if UNITY_EDITOR

using System.Collections.Generic;

namespace Nav3D.Obstacles
{
    //The obstacle id → gizmo cache map the octree's "your cached gizmo geometry is stale" call resolves through.
    //Obstacle ids are Nav3D's own, so no Unity lookup can reverse them; the component registers itself when it takes
    //its id and leaves when it is destroyed. Main thread only: registration happens in Init and the octree dispatches
    //its invalidation to the main thread before it reads the map.
    static class ObstacleGizmosCacheRegistry
    {

        #region Attributes

        static readonly Dictionary<int, IObstacleGizmosCache> s_Caches = new Dictionary<int, IObstacleGizmosCache>();

        #endregion

        #region Public methods

        internal static void Register(int _ID, IObstacleGizmosCache _Cache)
        {
            s_Caches[_ID] = _Cache;
        }

        //Only the registrant leaves: a component that shares another obstacle's id (the merged obstacle of the capture
        //path) must not take the owner's entry with it.
        internal static void Unregister(int _ID, IObstacleGizmosCache _Cache)
        {
            if (s_Caches.TryGetValue(_ID, out IObstacleGizmosCache registered) && ReferenceEquals(registered, _Cache))
                s_Caches.Remove(_ID);
        }

        internal static bool TryGet(int _ID, out IObstacleGizmosCache _Cache)
        {
            return s_Caches.TryGetValue(_ID, out _Cache);
        }

        #endregion

    }
}

#endif