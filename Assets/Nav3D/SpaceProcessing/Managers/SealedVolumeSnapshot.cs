using UnityEngine;

namespace Nav3D.Obstacles
{
    //The authored sealed volumes as one immutable set, published by reference: the main thread swaps in a new snapshot
    //whenever a volume appears, moves or goes, and construction and searches on any thread read Current with one atomic
    //read. Empty wherever no volume is authored.
    sealed class SealedVolumeSnapshot
    {
        #region Attributes

        //Empty is declared first: static initializers run in text order, and s_Current reads it
        static readonly SealedVolumeSnapshot s_Empty = new SealedVolumeSnapshot(new Bounds[0]);

        static volatile SealedVolumeSnapshot s_Current = s_Empty;

        readonly Bounds[] m_Volumes;

        #endregion

        #region Properties

        public static SealedVolumeSnapshot Empty => s_Empty;

        public static SealedVolumeSnapshot Current => s_Current;

        public bool IsEmpty => m_Volumes.Length == 0;

        #endregion

        #region Constructors

        public SealedVolumeSnapshot(Bounds[] _Volumes)
        {
            m_Volumes = _Volumes;
        }

        #endregion

        #region Public methods

        //main thread only
        public static void Publish(SealedVolumeSnapshot _Snapshot)
        {
            s_Current = _Snapshot ?? Empty;
        }

        public bool Contains(Vector3 _Point)
        {
            foreach (Bounds volume in m_Volumes)
                if (volume.Contains(_Point))
                    return true;

            return false;
        }

        //a cell counts as sealed only when a volume holds it whole: over-sealing navigable space is worse than leaving a
        //sliver of a volume's edge navigable, so an author gives a volume a margin
        public bool Embraces(Bounds _Bounds)
        {
            foreach (Bounds volume in m_Volumes)
                if (volume.Contains(_Bounds.min) && volume.Contains(_Bounds.max))
                    return true;

            return false;
        }

        //whether a body of _Radius moving from _From to _To enters a volume — a straight shot that must not be taken
        public bool SegmentEnters(Vector3 _From, Vector3 _To, float _Radius = 0f)
        {
            Vector3 delta  = _To - _From;
            float   length = delta.magnitude;

            foreach (Bounds volume in m_Volumes)
            {
                Bounds reach = volume;
                reach.Expand(2f * _Radius);

                if (reach.Contains(_From) || reach.Contains(_To))
                    return true;

                if (length > 0f && reach.IntersectRay(new Ray(_From, delta), out float distance) && distance <= length)
                    return true;
            }

            return false;
        }

        //the point moved just outside every volume that holds it, along the shortest way out of each
        public Vector3 PushOut(Vector3 _Point, float _Margin)
        {
            foreach (Bounds volume in m_Volumes)
            {
                if (!volume.Contains(_Point))
                    continue;

                Vector3 min = volume.min, max = volume.max;

                float best = float.PositiveInfinity;
                Vector3 exit = _Point;

                for (int axis = 0; axis < 3; axis++)
                {
                    float down = _Point[axis] - min[axis];
                    float up   = max[axis] - _Point[axis];

                    if (down < best)
                    {
                        best       = down;
                        exit       = _Point;
                        exit[axis] = min[axis] - _Margin;
                    }

                    if (up < best)
                    {
                        best       = up;
                        exit       = _Point;
                        exit[axis] = max[axis] + _Margin;
                    }
                }

                _Point = exit;
            }

            return _Point;
        }

        #endregion
    }
}
