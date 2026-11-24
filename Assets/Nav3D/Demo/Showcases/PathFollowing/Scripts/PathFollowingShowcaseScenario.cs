using Nav3D.API;
using UnityEngine;

namespace Nav3D.Demo
{
    //Path following — one agent, one static world, and the route it is given. Click
    //anywhere to send the courier there, Shift + click to add waypoints and to close them into a loop; left alone it
    //shuttles between the two rings. Tab switches smoothing.
    public class PathFollowingShowcaseScenario : MonoBehaviour
    {

        #region Constants

        const string MARKER_DESTINATION = "destination";
        const string MARKER_WAYPOINT    = "waypoint";

        static readonly Color DESTINATION_TINT = new Color(1f, 0.55f, 0.2f);
        static readonly Color WAYPOINT_TINT    = new Color(1f, 0.82f, 0.35f);

        #endregion

        #region Serialized fields

        [SerializeField] DemoHUD              m_HUD;
        [SerializeField] DemoSceneMenu        m_SceneMenu;
        [SerializeField] PathFollowingCourier m_Courier;
        [SerializeField] PathFollowingAim     m_Aim;
        [SerializeField] PathFollowingRoute   m_Route;

        //every obstacle of the stage: the first order waits for all of them, because an authored obstacle reaches the
        //octree asynchronously and a route asked for too early flies straight through the curtain
        [SerializeField] Nav3DObstacle[] m_StageObstacles;

        #endregion

        #region Attributes

        bool m_Inited;
        bool m_WorldRegistered;

        #endregion

        #region Properties

        //Nav3D initialized and the stage published: in a scene whose world registers after Awake, the init comes first
        public bool WorldReady => m_Inited && m_WorldRegistered;

        #endregion

        #region Unity events

        void Start()
        {
            ConfigureHUD();

            m_Courier.OrderFailed += OnOrderFailed;

            Nav3DManager.OnNav3DInit += OnNav3DInit;
        }

        //no OnNav3DInit unsubscription: Nav3DManager clears all subscriptions after init and logs an error on a manual one

        void Update()
        {
            if (!m_WorldRegistered)
                PollStage();

            //one after another, in this order: the courier moves on this frame's state, the aim sees this frame's
            //camera, and the line is drawn from the path the courier is flying now
            m_Courier.Tick();

            bool live = WorldReady && !m_HUD.HelpOpen && !m_SceneMenu.IsOpen;

            Vector3? aimed = m_Aim.Aim(live);

            if (live && DemoInput.GetMouseButtonDown(0))
                GiveOrder(aimed);

            if (WorldReady && DemoInput.GetKeyDown(KeyCode.Tab))
                ToggleSmoothing();

            m_Route.Trace(m_Courier.HasOrder);
        }

        #endregion

        #region Public methods

        //authors the HUD widgets into the scene; idempotent, ConfigureHUD reuses it at runtime
        public void ComposeHUD()
        {
            if (m_HUD == null)
                return;

            m_HUD.ComposeWorldMarker(MARKER_DESTINATION, "◎", DESTINATION_TINT, 34);

            //one numbered marker per waypoint a route can hold
            for (int i = 0; i < m_Courier.MaxWaypoints; i++)
                m_HUD.ComposeWorldMarker(MARKER_WAYPOINT + i, (i + 1).ToString(), WAYPOINT_TINT, 30);
        }

        #endregion

        #region Service methods

        void OnNav3DInit()
        {
            m_Inited = true;
        }

        void PollStage()
        {
            foreach (Nav3DObstacle obstacle in m_StageObstacles)
            {
                if (obstacle.AdditionProgress == null || !obstacle.AdditionProgress.Finished)
                    return;
            }

            if (!m_Inited)
                return;

            m_WorldRegistered = true;
            m_Courier.Armed   = true;
        }

        //A plain click is a new destination. With Shift held the click adds a waypoint to the standing route — and on the route's
        //first waypoint it closes the route into a loop instead, once there are two points to loop through. The first waypoint is
        //asked for before the stage is: it hangs in free air, where the aim's ray may find nothing at all
        void GiveOrder(Vector3? _Aimed)
        {
            bool shift = DemoInput.GetKey(KeyCode.LeftShift) || DemoInput.GetKey(KeyCode.RightShift);

            if (shift && m_Courier.CanCloseLoop && m_Aim.IsOn(m_Courier.Route[0]))
            {
                m_Courier.CloseLoopByHand();

                m_HUD.Toast($"Route closed — MoveToPoints with Loop ON, {m_Courier.Route.Count} waypoints round and round");

                return;
            }

            if (!_Aimed.HasValue)
                return;

            if (!shift)
                m_Courier.OrderByHand(_Aimed.Value);
            else if (!m_Courier.AppendByHand(_Aimed.Value))
                m_HUD.Toast($"The route is full — {m_Courier.MaxWaypoints} waypoints");
        }

        void ToggleSmoothing()
        {
            m_Courier.SmoothPath = !m_Courier.SmoothPath;

            m_Courier.Requery();

            m_HUD.Toast(
                    m_Courier.SmoothPath
                        ? "Smooth Path ON — the route is the line the courier flies"
                        : "Smooth Path OFF — the route turns at every octree cell it crosses"
                );
        }

        void OnOrderFailed(PathfindingResult _Result)
        {
            m_HUD.Toast($"No route: {_Result.Result}");
        }

        void ConfigureHUD()
        {
            //idempotent: re-authoring the widgets here also covers play mode right after a code change
            ComposeHUD();

            //a single destination wears the target glyph; a route of several wears its numbers
            m_HUD.BindWorldMarker(MARKER_DESTINATION, "◎", DESTINATION_TINT, 34, () => m_Courier.Route.Count == 1 ? m_Courier.Route[0] : (Vector3?) null);

            for (int i = 0; i < m_Courier.MaxWaypoints; i++)
            {
                int index = i;

                m_HUD.BindWorldMarker(
                        MARKER_WAYPOINT + index,
                        (index + 1).ToString(),
                        WAYPOINT_TINT,
                        30,
                        () => m_Courier.Route.Count > 1 && index < m_Courier.Route.Count ? m_Courier.Route[index] : (Vector3?) null
                    );
            }

            //the one stat: the setting Tab flips (owner, 2026-09-24 — the rest was numbers out of the mechanism)
            m_HUD.AddStat("Smooth Path", () => m_Courier.SmoothPath ? "ON" : "OFF");
        }

        #endregion

    }
}
