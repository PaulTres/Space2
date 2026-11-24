using UnityEngine;

namespace Nav3D.Common
{
    interface ICurve
    {
        #region Properties

        public Segment3[] Segments { get; }

        // Volumetric pathfinding: the curve's body radius around its polyline (a capsule-segmented curve).
        //0 = a plain zero-width segmented curve (the default, cheaper behavior everywhere). Spatial registration and intersection
        //queries must account for it: a change within Thickness of the polyline does affect the curve.
        public float Thickness { get; }

        #endregion

        #region Public methods

        bool Intersects(Bounds _Bounds);

        #endregion
    }
}
