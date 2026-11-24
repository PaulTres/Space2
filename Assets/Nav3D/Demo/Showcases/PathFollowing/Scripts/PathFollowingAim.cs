using UnityEngine;

namespace Nav3D.Demo
{
    //Where a click would send the courier. The ray under the mouse hits the stage — the floor, a column,
    //the top of a stump — and the order goes a standoff off that surface along the hit normal, which is free air
    //whatever the ray landed on.
    public class PathFollowingAim : MonoBehaviour
    {

        #region Serialized fields

        [SerializeField] DemoCursor3D m_Cursor;

        //past the far edge of the stage from any camera pose the fly rig reaches
        [SerializeField] float m_RayDistance = 300f;

        //clear of the surface by more than the agent's body and more than a boundary cell of the octree
        [SerializeField] float m_SurfaceStandoff = 2.5f;

        [Tooltip("How near the cursor has to be to a waypoint's marker, in pixels, to be on it")] [SerializeField]
        float m_MarkerPickPixels = 30f;

        #endregion

        #region Public methods

        //what a click this frame would order, or null while the aim is off or the ray sees only sky
        public Vector3? Aim(bool _Live)
        {
            if (!_Live)
                return null;

            return Physics.Raycast(m_Cursor.ScreenRay, out RaycastHit hit, m_RayDistance)
                ? hit.point + hit.normal * m_SurfaceStandoff
                : (Vector3?) null;
        }

        //whether the cursor is on a point of the world as the screen shows it: a waypoint hangs in free air, where no ray lands
        public bool IsOn(Vector3 _WorldPoint)
        {
            Camera camera = Camera.main;

            if (camera == null)
                return false;

            Vector3 onScreen = camera.WorldToScreenPoint(_WorldPoint);

            //behind the camera the projection mirrors, and a marker there is not on screen at all
            if (onScreen.z <= 0f)
                return false;

            return Vector2.Distance(onScreen, DemoInput.MousePosition) <= m_MarkerPickPixels;
        }

        #endregion

    }
}