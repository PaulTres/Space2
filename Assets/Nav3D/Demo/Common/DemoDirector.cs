using UnityEngine;
using UnityEngine.SceneManagement;

namespace Nav3D.Demo
{
    //DEMO kit — the shared control keys (R reload, Space pause = timeScale toggle, F2 routes, F3 octree view) and, in Playables,
    //the intro beat: the camera rides a Spline3DMover dolly, then the DemoCameraRig takes over; any click or key skips it.
    //Starts from Start unless m_IntroOnDemand (a scene behind a start-up splash calls PlayIntro).
    public class DemoDirector : MonoBehaviour
    {

        #region Serialized fields

        [SerializeField] Spline3DMover m_IntroDolly;

        [SerializeField] DemoCameraRig m_CameraRig;

        //what the camera looks at while riding the dolly: the dolly only drives position (aligned to the travel direction, the
        //camera looks away from the subject on an orbiting course). A mover on this transform makes a moving focus
        [SerializeField] Transform m_IntroFocus;

        //the help auto-open is delayed to the handoff: any click skips the intro, and the window would hide the shot
        [SerializeField] DemoHUD m_HUD;

        //opt-in: agent time freezes (timeScale 0, what Space does) while the help window is up, so a piloted scene does not drift
        //into an island behind the text; Space is the window's while it is open
        [SerializeField] bool m_PauseWhileHelpOpen;

        //opt-in: the intro does not start from Start; the scene calls PlayIntro when its world is ready (a splash held over a long
        //build has nothing to show an intro over)
        [SerializeField] bool m_IntroOnDemand;

        #endregion

        #region Attributes

        bool  m_IntroPlaying;
        bool  m_IntroPlayed;
        float m_TimeScaleBeforeHelp = 1f;

        #endregion

        #region Properties

        //camera-relative input (Skyrift's mouse aim) holds until the handoff, or the dolly sweep would steer the vehicle after itself
        public bool IntroPlaying => m_IntroPlaying;

        #endregion

        #region Public methods

        //plays at most once per scene entry
        public void PlayIntro()
        {
            if (m_IntroPlayed || m_IntroDolly == null || m_CameraRig == null)
                return;

            m_IntroPlayed       = true;
            m_IntroPlaying      = true;
            m_CameraRig.enabled = false;

            m_IntroDolly.OnFinished += EndIntro;

            m_IntroDolly.Play();
        }

        #endregion

        #region Unity events

        void Awake()
        {
            //Awake: the suppression must land before DemoHUD.Start resolves its auto-open
            if (m_IntroDolly != null && m_CameraRig != null && m_HUD != null)
                m_HUD.SuppressHelpAutoOpen();

            //same reason: a non-intro scene's help auto-opens from DemoHUD.Start
            if (m_PauseWhileHelpOpen && m_HUD != null)
                m_HUD.OnHelpVisibilityChanged += OnHelpVisibilityChanged;
        }

        void Start()
        {
            //an on-demand intro waits for the scene's PlayIntro call
            if (m_IntroOnDemand)
                return;

            PlayIntro();
        }

        void Update()
        {
            if (DemoInput.GetKeyDown(KeyCode.R))
            {
                Time.timeScale = 1f;

                SceneManager.LoadScene(SceneManager.GetActiveScene().buildIndex);

                return;
            }

            //while the help window owns the pause, Space is not a toggle — closing the window resumes
            if (DemoInput.GetKeyDown(KeyCode.Space) && !HelpOwnsPause)
                Time.timeScale = Mathf.Approximately(Time.timeScale, 0f) ? 1f : 0f;

            #if UNITY_EDITOR || NAV3D_DEBUG_VISUALIZATION

            //F2: every live agent's route, dormant until asked
            if (DemoInput.GetKeyDown(KeyCode.F2) && Nav3D.API.Nav3DManager.Inited)
            {
                Nav3D.API.Nav3DAgentPathDrawer pathDrawer = Nav3D.API.Nav3DAgentPathDrawer.Instance;

                pathDrawer.Active = !pathDrawer.Active;
            }

            //F3: the octree occupancy view, dormant until asked (its snapshot/instance/draw pipeline is priced per octree operation and per frame)
            if (DemoInput.GetKeyDown(KeyCode.F3) && Nav3D.API.Nav3DManager.Inited)
            {
                Nav3D.API.Nav3DOctreeDrawer drawer = Nav3D.API.Nav3DOctreeDrawer.Instance;

                drawer.Active = !drawer.Active;
            }

            #endif

            if (m_IntroPlaying && (DemoInput.AnyKeyDown || DemoInput.GetMouseButtonDown(0)))
                EndIntro();
        }

        void LateUpdate()
        {
            if (!m_IntroPlaying)
                return;

            Vector3 dollyPosition = m_IntroDolly.transform.position;

            Quaternion rotation = m_IntroFocus != null
                ? Quaternion.LookRotation(m_IntroFocus.position - dollyPosition, Vector3.up)
                : m_IntroDolly.transform.rotation;

            m_CameraRig.transform.SetPositionAndRotation(dollyPosition, rotation);
        }

        void OnDestroy()
        {
            if (m_IntroDolly != null)
                m_IntroDolly.OnFinished -= EndIntro;

            if (m_HUD != null)
                m_HUD.OnHelpVisibilityChanged -= OnHelpVisibilityChanged;

            //a pause (Space, or the help window's) outlives the scene otherwise: the editor keeps timeScale across
            //Play sessions, so the next scene would start frozen with nothing on screen to say why
            Time.timeScale = 1f;
        }

        #endregion

        #region Service methods

        bool HelpOwnsPause => m_PauseWhileHelpOpen && m_HUD != null && m_HUD.HelpOpen;

        void EndIntro()
        {
            if (!m_IntroPlaying)
                return;

            m_IntroPlaying = false;

            if (m_IntroDolly != null)
                m_IntroDolly.Stop();

            m_CameraRig.enabled = true;

            //the handoff is where the delayed help auto-open lands (skip included — a skip is a handoff)
            if (m_HUD != null)
                m_HUD.ShowHelpIfUnseen();
        }

        //the window borrows the pause: whatever time scale the scene ran at comes back on close. batch skips it: the window
        //auto-opens at the handoff and nobody closes it, so the pause would freeze every headless rig at second five
        void OnHelpVisibilityChanged(bool _Open)
        {
            if (Application.isBatchMode)
                return;

            if (_Open)
            {
                m_TimeScaleBeforeHelp = Time.timeScale;
                Time.timeScale        = 0f;

                return;
            }

            Time.timeScale = m_TimeScaleBeforeHelp;
        }

        #endregion

    }
}