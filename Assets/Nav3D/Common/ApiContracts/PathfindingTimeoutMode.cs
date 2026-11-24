namespace Nav3D.API
{
    //What a pathfinding request does when its Timeout passes: the search may already hold a complete route and be
    //refining it (the radius-aware detour escalation is such a refinement), or it may still be looking.
    public enum PathfindingTimeoutMode
    {
        //A complete route found before the deadline is delivered as it stands, with PathfindingResult.TimedOut = true
        //(and Path.IsTimedOut): the refinement that was running is skipped, so the route may be longer than the
        //optimum. Only a search that holds no complete route answers TIMEOUT.
        DELIVER_FOUND_ROUTE,
        //The request answers TIMEOUT whatever the search holds.
        FAIL
    }
}
