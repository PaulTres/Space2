using Nav3D.API;
using UnityEngine;

namespace Nav3D.Demo
{
    //The part of the helicopter's current path still to fly, and the camera's look-ahead point along it
    public class SkylineRouteLine : MonoBehaviour
    {
        #region Serialized fields

        [SerializeField] LineRenderer  m_Line;
        [SerializeField] Nav3DAgent    m_Agent;
        [SerializeField] DemoCameraRig m_CameraRig;

        //a distance, not an angle: on a straight leg the point is dead ahead, in a corner it is already round it;
        //two seconds of cruise at the authored speed
        [SerializeField] float m_LookAheadDistance = 30f;

        #endregion

        #region Attributes

        //the line's points, re-allocated only when the count changes (a waypoint passed)
        Vector3[] m_Buffer = System.Array.Empty<Vector3>();

        #endregion

        #region Unity events

        void Start()
        {
            m_Line.positionCount = 0;
        }

        #endregion

        #region Public methods

        //the world line and the camera's look-ahead both start at the helicopter's projection onto CurrentPath: the agent does not
        //say which point it heads for, so the nearest segment is where the part still to fly begins. No order: the line empties
        //and the camera falls back to the helicopter's own forward
        public void Trace(bool _HasOrder)
        {
            Vector3[] path = _HasOrder ? m_Agent.CurrentPath : null;

            if (path == null || path.Length < 2)
            {
                m_Line.positionCount = 0;

                return;
            }

            Vector3 here    = m_Agent.transform.position;
            int     nearest = 0;
            float   best    = float.MaxValue;

            for (int i = 0; i < path.Length - 1; i++)
            {
                Vector3 a  = path[i];
                Vector3 ab = path[i + 1] - a;
                float   t  = ab.sqrMagnitude > 1e-6f ? Mathf.Clamp01(Vector3.Dot(here - a, ab) / ab.sqrMagnitude) : 0f;
                float   d  = (a + ab * t - here).sqrMagnitude;

                if (d >= best)
                    continue;

                best    = d;
                nearest = i;
            }

            int count = path.Length - nearest;

            if (m_Buffer.Length != count)
                m_Buffer = new Vector3[count];

            m_Buffer[0] = here;

            for (int i = 1; i < count; i++)
                m_Buffer[i] = path[nearest + i];

            m_Line.positionCount = count;
            m_Line.SetPositions(m_Buffer);

            m_CameraRig.LookAhead(LookAheadPoint(path, nearest, here));
        }

        #endregion

        #region Service methods

        //walked leg by leg from the helicopter; a route with less than that left ahead hands back its end
        Vector3 LookAheadPoint(Vector3[] _Path, int _Nearest, Vector3 _Here)
        {
            Vector3 point     = _Here;
            float   remaining = m_LookAheadDistance;

            for (int i = _Nearest + 1; i < _Path.Length; i++)
            {
                Vector3 leg    = _Path[i] - point;
                float   length = leg.magnitude;

                if (length >= remaining)
                    return point + leg * (remaining / length);

                remaining -= length;
                point     =  _Path[i];
            }

            return point;
        }

        #endregion
    }
}
