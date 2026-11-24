namespace Nav3D.API
{
    /// <summary>
    /// How the EASED motion mode's start/final section extents are interpreted: as absolute arc lengths in
    /// world units (identical physical ramps regardless of path length) or as fractions of the current path
    /// length. Paths shorter than start+final scale both sections down proportionally either way.
    /// </summary>
    public enum EaseSectionAnchoring
    {
        //section extents are world units of path arc length
        ABSOLUTE_LENGTH,
        //section extents are fractions (0..1) of the current path length
        PATH_FRACTION
    }
}
