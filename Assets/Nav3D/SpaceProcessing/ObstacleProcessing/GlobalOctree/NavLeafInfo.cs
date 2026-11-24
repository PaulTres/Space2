using UnityEngine;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree
{
    //A plain copied-out description of one octree leaf — occupied or free — produced by the leaf snapshot reader
    //(NavTree.SnapshotLeaves) for debug visualization. Deliberately carries no tree references: the consumer (the
    //octree drawer) holds these structs across frames while operations keep rebuilding the tree, so
    //everything here must stay valid without the read lock.
    //
    //Free leaves are in the model because the navigable side of the graph is what a route actually travels, and a
    //picture of it is what explains a refusal: an aperture narrower than a bucket holds no free leaf at all, and the
    //same aperture under a finer resolution region holds a column of them. Occupied cells can only show that as an
    //absence.
    readonly struct NavLeafInfo
    {
        #region Constants

        //all six face bits set: nothing occupied across any face
        public const byte ALL_FACES_OPEN = 0x3F;

        #endregion

        #region Properties

        public Bounds Bounds { get; }

        //Whether this leaf holds obstacle geometry. The two kinds are snapshotted by the same walk and are told
        //apart here rather than by living in separate lists — a consumer usually wants both, differently coloured.
        public bool Occupied { get; }

        //An occupied leaf enclosed by a body rather than crossed by its surface (LeafState.INTERIOR), so a consumer can
        //show the solid inside apart from the crust.
        public bool Interior { get; }

        //Bit d (0..5 = +x,-x,+y,-y,+z,-z, matching UtilsSpatialGrid.FacesDirections) is set iff the cell across that
        //face holds no occupied leaf — a free leaf or unconstructed space. The definition is the same for both leaf
        //kinds, so it reads two ways: an OCCUPIED leaf with a non-zero mask belongs to the occupied volume's boundary
        //crust (the surface navigation sees, interior leaves carry mask 0), and a free leaf with a mask short of
        //ALL_FACES_OPEN is the navigable skin hugging that surface.
        public byte OpenFacesMask { get; }

        //an occupied leaf on the crust
        public bool IsBoundary => Occupied && OpenFacesMask != 0;

        //a leaf of either kind with an occupied neighbour across at least one face
        public bool TouchesOccupied => OpenFacesMask != ALL_FACES_OPEN;

        #endregion

        #region Constructors

        public NavLeafInfo(Bounds _Bounds, bool _Occupied, bool _Interior, byte _OpenFacesMask)
        {
            Bounds        = _Bounds;
            Occupied      = _Occupied;
            Interior      = _Interior;
            OpenFacesMask = _OpenFacesMask;
        }

        #endregion
    }
}
