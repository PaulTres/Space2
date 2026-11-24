namespace Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.Elements
{
    //What a node's cell is to navigation: crossed by geometry, enclosed by a body's closed surface, or free. The value is
    //also the serialized occupancy byte, whose 0 and 1 predate INTERIOR, so the numbering must not change.
    enum LeafState : byte
    {
        FREE     = 0,
        CROSSED  = 1,
        INTERIOR = 2
    }
}
