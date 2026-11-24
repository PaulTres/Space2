using System;

namespace Nav3D.API
{
    //Which components of an obstacle's GameObjects give it geometry. When only one kind is present it is used whatever
    //the selection says, and when the selection names none of the present kinds the first present one by priority is
    //used: COLLIDER, then MESH, then TERRAIN.
    [Flags]
    public enum ObstacleGeometrySources
    {
        //No choice made: the first kind present, by priority.
        AUTO     = 0,
        //Box, sphere, capsule, mesh and terrain colliders that are enabled and not triggers.
        COLLIDER = 1,
        //MeshFilter meshes.
        MESH     = 2,
        //Enabled Terrain components.
        TERRAIN  = 4
    }
}
