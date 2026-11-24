namespace Nav3D.API
{
    public enum PathfindingResultCode
    {
        //Pathfinding finished successfully.
        SUCCEEDED,
        //There is no path between points.
        PATH_DOES_NOT_EXIST,
        //The pathfinding took longer than allowed and was aborted.
        TIMEOUT,
        //Pathfinding was canceled by the user or Nav3D internal logic.
        CANCELLED,
        //Pathfinding has been canceled because the start point of the path is inside an obstacle.
        START_POINT_INSIDE_OBSTACLE,
        //Pathfinding has been canceled because the target point of the path is inside an obstacle.
        TARGET_POINT_INSIDE_OBSTACLE,
        // Volumetric pathfinding: the start point is in free space, but an agent of the requested radius
        //does not fit there (occupied space closer than the radius).
        START_POINT_TOO_NARROW,
        // Volumetric pathfinding: the target point is in free space, but an agent of the requested radius
        //does not fit there (occupied space closer than the radius).
        TARGET_POINT_TOO_NARROW,
        // The start point of the path is inside a Nav3DSealedVolume.
        START_POINT_INSIDE_SEALED_VOLUME,
        // A target point of the path is inside a Nav3DSealedVolume.
        TARGET_POINT_INSIDE_SEALED_VOLUME,
        //Internal error.
        UNKNOWN
    }
}
