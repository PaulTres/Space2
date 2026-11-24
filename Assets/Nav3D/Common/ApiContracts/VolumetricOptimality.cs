namespace Nav3D.API
{
    // How a volumetric (radius-aware) pathfinding request trades the route's quality for search time. Ignored when
    //the request radius is 0. Declared in order of quality and cost; QUALITY keeps the value 0, so it is the default.
    public enum VolumetricOptimality
    {
        //A fast search; the route may come out a little longer. The corridor's own route, pulled taut against obstacles at
        //the agent's radius; a route that comes out noticeably longer is found the QUALITY way instead.
        FAST = 1,
        //The default: a quality route, a costlier search than FAST. Found in the corridor the free-space search picks, it
        //follows the octree's leaves inside it.
        QUALITY = 0,
        //A still better, shorter route for a still costlier search: the QUALITY route, then pulled taut against obstacles at
        //the agent's radius.
        HIGH_QUALITY = 2
    }
}
