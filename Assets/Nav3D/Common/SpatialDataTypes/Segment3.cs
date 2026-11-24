using UnityEngine;

namespace Nav3D.Common
{
    /// <summary>
    /// Represents a segment in space.
    /// </summary>
    public struct Segment3
    {
        #region Properties

        public Vector3 Start           { get; }
        public Vector3 End             { get; }
        public Vector3 DirectionMagn   { get; }

        #endregion

        #region Constructors

        public Segment3(Vector3 _Start, Vector3 _End)
        {
            Start           = _Start;
            End             = _End;
            DirectionMagn   = _End - _Start;
        }

        #endregion

        #region Public methods

        public Vector3 GetClosestPoint(Vector3 _Point)
        {
            Vector3 dir        = End    - Start;
            Vector3 toPointDir = _Point - Start;

            float magnitudeSqr = dir.sqrMagnitude;
            if (magnitudeSqr == 0f)
                return Start; // Segment is a point

            float t = Vector3.Dot(toPointDir, dir) / magnitudeSqr;
            t = Mathf.Clamp01(t); // Clamp to [0, 1] so it stays on the segment

            return Start + t * dir;
        }
        
        #endregion
    }
}