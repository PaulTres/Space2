#if UNITY_EDITOR

namespace Nav3D.Obstacles
{
    /// <summary>
    /// An obstacle component whose gizmo drawing caches geometry the octree can invalidate.
    /// </summary>
    //The seam exists so the octree can say "your cached gizmo geometry is stale" without naming an API component:
    //it resolves the instance IDs it holds to this interface instead of to Nav3DObstacle, which would point the
    //dependency at the layer above. Editor-only on both ends — the implementing partial is the obstacle's own
    //#if UNITY_EDITOR file, so a player build has neither the interface nor the base-list entry.
    interface IObstacleGizmosCache
    {
        void ClearGizmosCache();
    }
}

#endif
