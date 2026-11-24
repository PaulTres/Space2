namespace Nav3D.API
{
    // What a volumetric (radius-aware) pathfinding request does when no corridor wide enough for the
    //agent exists. Ignored when the request radius is 0.
    public enum VolumetricPathfindingMode
    {
        //The request fails honestly (PATH_DOES_NOT_EXIST / *_TOO_NARROW): the path never violates the clearance.
        STRICT,
        //On a volumetric failure the whole request is retried as a point search (the default, cheaper pathfinding
        //mode); the delivered result carries Degraded = true (and Path.IsDegraded) so the caller can see the
        //clearance promise was dropped.
        FALLBACK_TO_DEFAULT
    }
}
