using UnityEngine;

namespace Nav3D.Demo
{
    //The shared aim ring (DemoAimRing) with Skyline's meaning — the ring is born on the camera's forward, steers the chase
    //camera, and where it lands in the world is the order a click would give: a pad's hover point, the point on a surface, or open air
    //at the cursor's depth. The destination marker shows that order while the hand aims
    public class SkylineSight : MonoBehaviour
    {

        #region Nested types

        //what a click would order
        public readonly struct Designation
        {
            public Designation(Vector3 _Point, SkylineLandingPad _Pad)
            {
                Point = _Point;
                Pad   = _Pad;
            }

            public Vector3 Point { get; }

            //null = open air
            public SkylineLandingPad Pad { get; }
        }

        #endregion

        #region Serialized fields

        [SerializeField] Camera        m_Camera;
        [SerializeField] DemoCameraRig m_CameraRig;
        [SerializeField] DemoCursor3D  m_Cursor;
        [SerializeField] DemoHUD       m_HUD;
        [SerializeField] DemoSceneMenu m_SceneMenu;
        [SerializeField] DemoDirector  m_Director;

        [SerializeField] Transform m_DestinationMarker;

        //how far a designation ray reaches: past the far edge of the district from any camera pose
        [SerializeField] float m_RayDistance = 600f;

        //the ring's mouse speed, screen inset and re-projection distance (Skyrift's values)
        [SerializeField] float   m_RingPixelsPerMouseUnit = 30f;
        [SerializeField] float   m_RingScreenInset        = 24f;
        [SerializeField] float   m_RingReferenceDistance  = 200f;
        [SerializeField] KeyCode m_FreeCursorKey          = KeyCode.LeftAlt;

        #endregion

        #region Attributes

        readonly DemoAimRing m_Ring = new DemoAimRing();

        #endregion

        #region Properties

        //the ring and the destination sphere are a sight, not scenery: up while the hand steers and for the rig's realign delay,
        //down once follow-align has the camera back behind the helicopter; a detached camera keeps it up
        public bool Aiming => m_CameraRig.SteerHeld;

        //where the ring sits in the world this frame: the surface under it, or open air at the cursor's depth
        public Vector3? RingWorldPoint { get; private set; }

        #endregion

        #region Unity events

        void Start()
        {
            m_DestinationMarker.gameObject.SetActive(false);
        }

        void OnDestroy()
        {
            DemoAimRing.ApplyCursorLock(false);
        }

        #endregion

        #region Public methods

        //what a click this frame would order, null while the ring is not steering; resolved every frame because the marker shows it every frame
        public Designation? Aim(bool _WorldReady)
        {
            Designation? designation = SteerRing();

            ShowDestinationMarker(_WorldReady && Aiming ? designation?.Point : null);

            return designation;
        }

        //the ring is re-seated on the sight so its next steer request does not pull the view back off
        public void ResetView()
        {
            m_Ring.Reseat();

            m_CameraRig.ResetView();
        }

        #endregion

        #region Service methods

        Designation? SteerRing()
        {
            bool cursorFree = DemoInput.GetKey(m_FreeCursorKey) || m_HUD.HelpOpen || m_HUD.SplashUp || m_SceneMenu.IsOpen || m_Director.IntroPlaying ||
                              Mathf.Approximately(Time.timeScale, 0f);

            DemoAimRing.ApplyCursorLock(!cursorFree);

            if (cursorFree || DemoInput.GetMouseButton(1))
            {
                m_Ring.ReleaseScreen();

                RingWorldPoint = null;

                return null;
            }

            Vector3 cameraPosition = m_Camera.transform.position;
            Vector2 held           = m_Ring.Steer(m_Camera, m_Camera.transform.forward, m_RingReferenceDistance, m_RingPixelsPerMouseUnit, m_RingScreenInset);

            Ray ray = new Ray(cameraPosition, m_Ring.Direction.Value);

            m_CameraRig.SteerToward(held, m_Ring.HandMoved);

            if (!Physics.Raycast(ray, out RaycastHit hit, m_RayDistance))
            {
                RingWorldPoint = ray.GetPoint(m_Cursor.FallbackDepth);

                return new Designation(RingWorldPoint.Value, null);
            }

            SkylineLandingPad pad = hit.collider.GetComponentInParent<SkylineLandingPad>();

            RingWorldPoint = hit.point;

            //a click on a tower's face is the order itself: TryRepositionTargetIfOccupied moves it to the nearest free point outside
            return new Designation(pad != null ? pad.HoverPoint : hit.point, pad);
        }

        void ShowDestinationMarker(Vector3? _Point)
        {
            bool shown = _Point.HasValue;

            if (m_DestinationMarker.gameObject.activeSelf != shown)
                m_DestinationMarker.gameObject.SetActive(shown);

            if (shown)
                m_DestinationMarker.position = _Point.Value;
        }

        #endregion

    }
}