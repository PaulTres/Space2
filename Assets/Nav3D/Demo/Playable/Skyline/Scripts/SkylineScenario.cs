using Nav3D.API;
using System.Collections.Generic;
using UnityEngine;

namespace Nav3D.Demo
{
    //The Skyline playable — a city read into the navigation space once, at start-up, and flown across from then on.
    //A screen ring points at the city and a click orders the helicopter there: to the wall it lands on, or to a landing on a pad. The
    //order itself is the pilot's (SkylinePilot); this scenario reads the click, keeps the frame order and speaks
    public class SkylineScenario : MonoBehaviour
    {

        #region Constants

        const string WIDGET_SPEED  = "speed";
        const string WIDGET_CAMERA = "camera";
        const string WIDGET_LOADER = "navspace";

        const KeyCode RESET_CAMERA_KEY = KeyCode.C;
        const string  MARKER_TARGET    = "target";
        const string  MARKER_RING      = "ring";

        //hint-line states declared in SkylineLocalization.json; the file's first entry is what the scene opens with
        const string HINTS_TAKEOFF = "takeoff";
        const string HINTS_FLYING  = "flying";
        const string HINTS_LANDING = "landing";

        const string SPLASH_TITLE   = "Skyline";
        const string SPLASH_CAPTION = "The city is being read into the navigation space — once, at start. The scene opens when it is complete.";

        static readonly Color LOADER_TINT = new Color(0.55f, 0.85f, 1f);
        static readonly Color TARGET_TINT = new Color(1f, 0.85f, 0.35f);
        static readonly Color RING_TINT   = new Color(1f, 1f, 1f, 0.8f);

        #endregion

        #region Serialized fields

        [SerializeField] DemoHUD      m_HUD;
        [SerializeField] DemoDirector m_Director;

        [SerializeField] SkylineWorldLoader m_Loader;
        [SerializeField] SkylineSight       m_Sight;
        [SerializeField] SkylineRouteLine   m_Route;

        //the player's helicopter and its order taker; the speed slider is the only runtime mutation, OnDestroy restores it
        [SerializeField] SkylinePilot     m_Pilot;
        [SerializeField] Nav3DAgentConfig m_PlayerConfig;

        //the speed slider's range
        [SerializeField] float m_SpeedMin = 4f;
        [SerializeField] float m_SpeedMax = 30f;

        [SerializeField] SkylineLandingPad[] m_Pads;

        #endregion

        #region Attributes

        bool  m_Inited; //Nav3DManager.OnNav3DInit has fired
        bool  m_WorldLoaded; //every authored obstacle reports Finished
        float m_LoadSeconds;

        float m_AuthoredSpeed;

        #endregion

        #region Properties

        //Nav3D initialized and every authored obstacle published: in a scene whose obstacles register one by one, OnNav3DInit arrives first
        public bool WorldReady => m_Inited && m_WorldLoaded;

        public IReadOnlyList<SkylineLandingPad> Pads => m_Pads;

        public SkylineHelicopter Player => m_Pilot.Helicopter;

        public SkylineLandingPad TargetPad => m_Pilot.TargetPad;

        public Vector3? Target => m_Pilot.Target;

        #endregion

        #region Unity events

        void Start()
        {
            m_AuthoredSpeed = m_PlayerConfig.Speed;

            m_Pilot.Airborne    += OnAirborne;
            m_Pilot.Landing     += OnLanding;
            m_Pilot.Landed      += OnLanded;
            m_Pilot.OrderFailed += OnOrderFailed;

            ConfigureHUD();

            Nav3DManager.OnNav3DInit += OnNav3DInit;
        }

        void OnDestroy()
        {
            //no OnNav3DInit unsubscription: Nav3DManager auto-clears all subscriptions after init

            //hand the asset back exactly as authored (the slider changes it during play)
            if (m_PlayerConfig != null)
                m_PlayerConfig.Speed = m_AuthoredSpeed;
        }

        void Update()
        {
            //one call after another, in this order: the sight sees this frame's load state and the click this frame's sight
            if (!m_WorldLoaded)
                PollLoader();

            SkylineSight.Designation? designation = m_Sight.Aim(WorldReady);

            m_Route.Trace(m_Pilot.Target.HasValue);

            if (DemoInput.GetKeyDown(RESET_CAMERA_KEY))
                m_Sight.ResetView();

            if (!WorldReady || designation == null)
                return;

            if (DemoInput.GetMouseButtonDown(0))
                Designate(designation.Value.Point, designation.Value.Pad);
        }

        #endregion

        #region Public methods

        //authors the widgets into the scene asset; idempotent, ConfigureHUD reuses it
        public void ComposeHUD()
        {
            if (m_HUD == null)
                return;

            m_HUD.ComposeSectionHeader("Navigation space");
            m_HUD.ComposeProgressBar(WIDGET_LOADER, LOADER_TINT);

            //named after the real config setting the slider drives: Nav3DAgentConfig.Speed
            m_HUD.ComposeSectionHeader("Helicopter");
            m_HUD.ComposeSlider(WIDGET_SPEED);

            m_HUD.ComposeSectionHeader("Camera");
            m_HUD.ComposeButton(WIDGET_CAMERA, "Behind the helicopter  (C)");

            m_HUD.ComposeWorldMarker(MARKER_TARGET, "◎", TARGET_TINT, 34);
            m_HUD.ComposeWorldMarker(MARKER_RING, "○", RING_TINT, 40);

            //the splash goes over everything, the help window included: composed last
            m_HUD.ComposeSplash(SPLASH_TITLE, SPLASH_CAPTION, LOADER_TINT);
        }

        //what a click resolves to, and what the play probe drives the scene with; the pilot answers and the HUD says what happened
        public void Designate(Vector3 _Point, SkylineLandingPad _Pad)
        {
            if (!WorldReady)
                return;

            switch (m_Pilot.Designate(_Point, _Pad))
            {
                case SkylinePilot.Verdict.REFUSED_LANDING:
                    m_HUD.Toast("Landing — orders resume once on the deck");

                    return;

                case SkylinePilot.Verdict.REFUSED_TAKING_OFF:
                    m_HUD.Toast("Lifting off — your order runs once airborne");

                    return;

                case SkylinePilot.Verdict.LIFTING_OFF:
                    m_HUD.ShowHints(HINTS_TAKEOFF);

                    return;

                case SkylinePilot.Verdict.FLYING_TO_PAD:
                    m_HUD.Toast($"Landing on {_Pad.name}");

                    return;
            }
        }

        #endregion

        #region Service methods

        void OnNav3DInit()
        {
            m_Inited = true;
        }

        void PollLoader()
        {
            if (!m_Loader.AllFinished || !m_Inited)
                return;

            m_WorldLoaded = true;
            m_LoadSeconds = m_Loader.ElapsedSeconds;

            m_HUD.ShowHints(HINTS_FLYING);
            m_HUD.Toast($"Navigation space ready in {m_LoadSeconds:F1} s — click anywhere to fly there");

            //the card lifts on its own (the HUD watches WorldReady); the intro rides out from under it
            m_Director.PlayIntro();
        }

        float LoaderFraction()
        {
            return m_WorldLoaded ? 1f : m_Loader.Fraction;
        }

        void OnAirborne()
        {
            m_HUD.ShowHints(HINTS_FLYING);
        }

        void OnLanding(SkylineLandingPad _Pad)
        {
            m_HUD.ShowHints(HINTS_LANDING);
        }

        void OnLanded()
        {
            m_HUD.ShowHints(HINTS_FLYING);
            m_HUD.Toast("Landed — click anywhere to lift off and fly there");
        }

        //the code goes on screen verbatim
        void OnOrderFailed(PathfindingResult _Result)
        {
            m_HUD.Toast($"Pathfinding failed: {_Result.Result}");
        }

        void ConfigureHUD()
        {
            //compose is a no-op when the widgets are authored; binding is live
            ComposeHUD();

            m_HUD.BindProgressBar(
                    WIDGET_LOADER,
                    LOADER_TINT,
                    () => m_WorldLoaded ? $"ready in {m_LoadSeconds:F1} s" : $"{LoaderFraction() * 100f:F0} %   ·   {m_Loader.TotalTriangles:N0} triangles",
                    LoaderFraction
                );

            //up until the world is ready; no key or click dismisses it
            m_HUD.BindSplash(
                    SPLASH_TITLE,
                    SPLASH_CAPTION,
                    LOADER_TINT,
                    () => $"{m_Loader.TotalTriangles:N0} triangles   ·   {m_Loader.Status}   ·   {m_Loader.ElapsedSeconds:F0} s",
                    LoaderFraction,
                    () => WorldReady
                );

            m_HUD.BindSlider(
                    WIDGET_SPEED,
                    () => $"Speed  {m_PlayerConfig.Speed:F0} u/s",
                    m_SpeedMin,
                    m_SpeedMax,
                    () => m_PlayerConfig.Speed,
                    _Value => m_PlayerConfig.Speed = _Value
                );

            m_HUD.BindButton(WIDGET_CAMERA, m_Sight.ResetView);

            m_HUD.BindWorldMarker(MARKER_TARGET, "◎", TARGET_TINT, 34, () => m_Pilot.Target);
            m_HUD.BindWorldMarker(MARKER_RING, "○", RING_TINT, 40, () => m_Sight.Aiming ? m_Sight.RingWorldPoint : null);
        }

        #endregion

    }
}
