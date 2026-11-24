//The namespace is the octree's, the assembly is Common's, and that is deliberate: this is a dependency-free
//octant index, and UtilsSpatialGrid -- which four Common containers use -- is written in terms of it. Moving the
//name would have been a breaking change; moving the file costs nothing.
namespace Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.Elements
{
    enum ForkChildOctIndex : byte
    {
        //I-XYZ
        I000 = 1,
        I001 = 1 << 1,
        I010 = 1 << 2,
        I011 = 1 << 3,
        I100 = 1 << 4,
        I101 = 1 << 5,
        I110 = 1 << 6,
        I111 = 1 << 7,
    }
}