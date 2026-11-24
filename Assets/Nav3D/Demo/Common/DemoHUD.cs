using Nav3D.Common;
using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace Nav3D.Demo
{
    //DEMO kit — the uGUI overlay every scene shares: title, hints, stats, toast, control-panel widgets, help window, splash.
    //Compose* authors a widget into the scene asset (idempotent); Bind* wires behaviour at runtime and auto-composes
    //when the scene carries no authored widget. The words come from the scene's own .json, read in Awake by key.
    public class DemoHUD : MonoBehaviour
    {
        #region Constants

        const float  STATS_REFRESH_INTERVAL = 0.25f;
        const float  TOAST_DURATION         = 3f;
        const float  TOAST_FADE_TAIL        = 1f;
        const int    CANVAS_SORT_ORDER      = 10;
        const string BUILTIN_FONT           = "LegacyRuntime.ttf";
        const string HINT_SEPARATOR         = "   |   ";

        const float PANEL_WIDTH           = 300f;
        const float BUTTON_HEIGHT         = 40f;
        const float SLIDER_HEIGHT         = 26f;
        const float SECTION_HEADER_HEIGHT = 26f;
        const float PANEL_SPACING         = 8f;
        const float PROGRESS_BAR_HEIGHT   = 24f;

        const float HELP_WIDTH              = 780f;

        //the glyph points up unrotated
        const string THREAT_ARROW_GLYPH       = "▲";
        const float  THREAT_ARROW_EDGE_MARGIN = 70f;
        const float  THREAT_ARROW_HOVER       = 46f;
        //the help window is capped at the canvas height minus these; taller content scrolls inside it
        const float HELP_SCREEN_MARGIN      = 48f;
        const float HELP_DEFAULT_HEIGHT     = 900f;
        const float HELP_PADDING_X          = 28f;
        const float HELP_PADDING_Y          = 24f;
        const float HELP_SCROLL_SENSITIVITY = 40f;
        const float HELP_BUTTON_SIZE        = 48f;
        const float HELP_ANIMATION_DURATION = 0.35f;
        const float HELP_SECTION_SPACING    = 14f;

        const float SPLASH_BAR_WIDTH     = 620f;
        const float SPLASH_BAR_HEIGHT    = 16f;
        const float SPLASH_FADE_DURATION = 0.8f;

        static readonly Color SPLASH_BACKGROUND = new Color(0.03f, 0.04f, 0.06f, 0.97f);
        static readonly Color SPLASH_BAR_TRACK  = new Color(1f, 1f, 1f, 0.10f);

        static readonly Color HELP_BACKGROUND   = new Color(0.04f, 0.05f, 0.07f, 0.94f);
        static readonly Color HELP_BLOCKER      = new Color(0f, 0f, 0f, 0.45f);
        static readonly Color HELP_HEADER_COLOR = new Color(0.45f, 0.75f, 1f);
        static readonly Color HELP_BODY_COLOR   = new Color(0.85f, 0.88f, 0.92f);

        //readouts stack beside the screen's centre, right-aligned so they grow away from the sight
        const float READOUT_OFFSET_X = -220f;
        const float READOUT_SPACING  = 42f;
        const float READOUT_WIDTH    = 360f;

        //the graph panel sits bottom-right, clear of the stats block, the control panel and the hint line
        const float GRAPH_WIDTH        = 560f;
        const float GRAPH_HEIGHT       = 240f;
        const float GRAPH_MARGIN_X     = -24f;
        const float GRAPH_MARGIN_Y     = 64f;
        const float GRAPH_PADDING      = 14f;
        const float GRAPH_HEADER       = 26f;
        const float GRAPH_FOOTER       = 24f;
        const float GRAPH_LINE_WIDTH   = 2.5f;

        static readonly Color GRAPH_BACKGROUND = new Color(0f, 0f, 0f, 0.55f);
        static readonly Color GRAPH_AXIS       = new Color(1f, 1f, 1f, 0.18f);
        static readonly Color GRAPH_REFERENCE  = new Color(1f, 1f, 1f, 0.35f);
        static readonly Color GRAPH_CAPTION    = new Color(0.78f, 0.82f, 0.88f);

        static readonly Color PANEL_BACKGROUND    = new Color(0f, 0f, 0f, 0.55f);
        static readonly Color BUTTON_IDLE         = new Color(1f, 1f, 1f, 0.10f);
        static readonly Color BUTTON_SELECTED     = new Color(0.25f, 0.65f, 1f, 0.45f);
        static readonly Color SLIDER_BACKGROUND   = new Color(1f, 1f, 1f, 0.12f);
        static readonly Color SLIDER_FILL         = new Color(0.25f, 0.65f, 1f, 0.8f);
        static readonly Color SLIDER_HANDLE       = new Color(1f, 1f, 1f, 0.9f);

        #endregion

        #region Nested types

        readonly struct StatEntry
        {
            public StatEntry(string _Label, Func<string> _Getter)
            {
                Label  = _Label;
                Getter = _Getter;
            }

            public string       Label  { get; }
            public Func<string> Getter { get; }
        }

        class ButtonGroup
        {
            public ButtonGroup(Image[] _Backgrounds, Func<int> _GetSelected)
            {
                Backgrounds = _Backgrounds;
                GetSelected = _GetSelected;
            }

            public Image[]   Backgrounds { get; }
            public Func<int> GetSelected { get; }
        }

        class SliderEntry
        {
            public SliderEntry(Text _Label, Func<string> _LabelGetter, Slider _Slider, Func<float> _ValueGetter)
            {
                Label       = _Label;
                LabelGetter = _LabelGetter;
                Slider      = _Slider;
                ValueGetter = _ValueGetter;
            }

            public Text         Label       { get; }
            public Func<string> LabelGetter { get; }
            public Slider       Slider      { get; }
            public Func<float>  ValueGetter { get; }
        }

        class StepperEntry
        {
            public StepperEntry(Text _Label, Func<string> _LabelGetter)
            {
                Label       = _Label;
                LabelGetter = _LabelGetter;
            }

            public Text         Label       { get; }
            public Func<string> LabelGetter { get; }
        }

        //the fill's anchorMax.x tracks the fraction
        class BarEntry
        {
            public BarEntry(Text _Label, RectTransform _Fill, Func<string> _LabelGetter, Func<float> _FractionGetter)
            {
                Label          = _Label;
                Fill           = _Fill;
                LabelGetter    = _LabelGetter;
                FractionGetter = _FractionGetter;
            }

            public Text          Label          { get; }
            public RectTransform Fill           { get; }
            public Func<string>  LabelGetter    { get; }
            public Func<float>   FractionGetter { get; }
        }

        //a pool of glyphs: above an on-screen position pointing down at it, on the screen's edge pointing the way to turn otherwise
        class ThreatArrowsEntry
        {
            public ThreatArrowsEntry(RectTransform[] _Arrows, Func<IReadOnlyList<Vector3>> _WorldGetter)
            {
                Arrows      = _Arrows;
                WorldGetter = _WorldGetter;
            }

            public RectTransform[]              Arrows      { get; }
            public Func<IReadOnlyList<Vector3>> WorldGetter { get; }
        }

        class ReadoutEntry
        {
            public ReadoutEntry(Text _Value, Func<string> _Getter)
            {
                Value  = _Value;
                Getter = _Getter;
            }

            public Text         Value  { get; }
            public Func<string> Getter { get; }
        }

        class MarkerEntry
        {
            public MarkerEntry(RectTransform _Rect, Func<Vector3?> _WorldGetter)
            {
                Rect        = _Rect;
                WorldGetter = _WorldGetter;
            }

            public RectTransform  Rect        { get; }
            public Func<Vector3?> WorldGetter { get; }
        }

        readonly struct SplashEntry
        {
            public SplashEntry(CanvasGroup _Group, Text _Status, Text _Percent, RectTransform _Fill,
                               Func<string> _StatusGetter, Func<float> _FractionGetter, Func<bool> _DoneGetter)
            {
                Group          = _Group;
                Status         = _Status;
                Percent        = _Percent;
                Fill           = _Fill;
                StatusGetter   = _StatusGetter;
                FractionGetter = _FractionGetter;
                DoneGetter     = _DoneGetter;
            }

            public CanvasGroup   Group          { get; }
            public Text          Status         { get; }
            public Text          Percent        { get; }
            public RectTransform Fill           { get; }
            public Func<string>  StatusGetter   { get; }
            public Func<float>   FractionGetter { get; }
            public Func<bool>    DoneGetter     { get; }
        }

        #endregion

        #region Serialized fields

        //authored into the scene asset
        [SerializeField] Text m_TitleText;
        [SerializeField] Text m_CaptionText;
        [SerializeField] Text m_HintsText;
        [SerializeField] Text m_StatsText;
        [SerializeField] Text m_ToastText;

        //created by the first Compose* call during a bake; a scene without panel widgets never carries one
        [SerializeField] RectTransform m_ControlPanel;

        [Tooltip("The scene's <Scene>Localization.json — its title card, hint lines and help blocks. Read when the scene starts, so editing the file and " +
                 "restarting the scene is the whole edit loop.")]
        [SerializeField] TextAsset m_TextAsset;

        //closing shrinks the window into the "?" button: the motion teaches where help reopens
        [SerializeField] RectTransform m_HelpOverlay;
        [SerializeField] RectTransform m_HelpBlocker;
        [SerializeField] RectTransform m_HelpContent;
        [SerializeField] RectTransform m_HelpButton;

        [SerializeField] RectTransform m_Splash;

        #endregion

        #region Attributes

        readonly List<StatEntry>    m_Stats        = new List<StatEntry>();
        readonly List<ButtonGroup>  m_ButtonGroups = new List<ButtonGroup>();
        readonly List<SliderEntry>  m_Sliders      = new List<SliderEntry>();
        readonly List<StepperEntry> m_Steppers     = new List<StepperEntry>();
        readonly List<BarEntry>     m_Bars         = new List<BarEntry>();
        readonly List<MarkerEntry>  m_Markers      = new List<MarkerEntry>();
        readonly List<ReadoutEntry> m_Readouts     = new List<ReadoutEntry>();
        readonly List<ThreatArrowsEntry> m_ThreatArrows = new List<ThreatArrowsEntry>();

        //null after a missing or malformed file (the error is logged)
        Dictionary<string, object> m_Text;

        float m_NextStatsRefreshTime;
        float m_ToastShownTime = float.NegativeInfinity;

        SplashEntry? m_SplashEntry;
        float        m_SplashLiftTime = float.NegativeInfinity;

        bool  m_HelpAnimating;
        bool  m_HelpClosing;
        bool  m_HelpAutoOpenSuppressed;
        float m_HelpAnimationStartTime;

        //a static survives scene reloads (an R-reset must not re-open the help); cleared per play session because batch rigs run with
        //DisableDomainReload, so it would leak across sessions otherwise
        static readonly HashSet<string> s_HelpShownScenes = new HashSet<string>();

        #endregion

        #region Events

        //true on open, false on close
        public event Action<bool> OnHelpVisibilityChanged;

        #endregion

        #region Properties

        //the closing shrink counts as closed: the world may run again the moment the player dismisses it
        public bool HelpOpen => m_HelpOverlay != null && m_HelpOverlay.gameObject.activeSelf && !m_HelpClosing;

        //the lifting fade counts as down: the scene may take input again the moment its build is complete
        public bool SplashUp => m_Splash != null && m_Splash.gameObject.activeSelf && float.IsNegativeInfinity(m_SplashLiftTime);

        #endregion

        #region Unity events

        void Awake()
        {
            //Awake, not Start: the help window must exist before Start resolves its auto-open, and a director's Awake may suppress that first
            ApplyText();

            //the authored chrome of a scene built before 2026-09-24 still carries the raycast flag; nothing in it is clickable
            foreach (Text chrome in new[] { m_TitleText, m_CaptionText, m_HintsText, m_StatsText, m_ToastText })
            {
                if (chrome != null)
                    chrome.raycastTarget = false;
            }
        }

        void Start()
        {
            if (m_HelpOverlay != null)
                InitHelp();
        }

        void Update()
        {
            if (Time.unscaledTime >= m_NextStatsRefreshTime)
            {
                m_NextStatsRefreshTime = Time.unscaledTime + STATS_REFRESH_INTERVAL;

                RefreshStats();
                RefreshControls();
            }

            UpdateToast();
            UpdateHelpAnimation();
            UpdateSplash();
            RefreshThreatArrows();
            RefreshReadouts();

            if (HelpOpen)
                FitHelpWindow(_Rebuild: false);
            RefreshMarkers();
        }

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
        static void ResetHelpSessionMemory()
        {
            s_HelpShownScenes.Clear();
        }

        #endregion

        #region Public methods

        public void ApplyText()
        {
            if (m_TextAsset == null)
            {
                Debug.LogError($"[{nameof(DemoHUD)}] {gameObject.scene.name}: no HUD text asset is assigned — drag the scene's " +
                               $"<Scene>Localization.json into the Text Asset field of the {nameof(DemoHUD)} component");

                return;
            }

            try
            {
                m_Text = DemoJson.Parse(m_TextAsset.text);

                m_TitleText.text   = DemoJson.String(m_Text, "TITLE");
                m_CaptionText.text = DemoJson.String(m_Text, "CAPTION");

                //the file's order picks the opening hint state
                Dictionary<string, object> openingState = DemoJson.Object(DemoJson.Array(m_Text, "HINTS")[0]);

                ShowHints(DemoJson.String(openingState, "ID"));
                ComposeHelpOverlay();
            }
            catch (FormatException exception)
            {
                m_Text = null;

                Debug.LogError($"[{nameof(DemoHUD)}] {gameObject.scene.name}: {m_TextAsset.name}.json — {exception.Message}");
            }
        }

        //not called by scenes whose controls never change: the file's first state shows on its own
        public void ShowHints(string _StateId)
        {
            if (m_Text == null)
                return;

            foreach (object entry in DemoJson.Array(m_Text, "HINTS"))
            {
                Dictionary<string, object> hint = DemoJson.Object(entry);

                if (DemoJson.String(hint, "ID") != _StateId)
                    continue;

                m_HintsText.text = string.Join(HINT_SEPARATOR, DemoJson.Strings(hint, "PARTS"));

                return;
            }

            Debug.LogError($"[{nameof(DemoHUD)}] {gameObject.scene.name}: no \"{_StateId}\" entry in \"HINTS\"");
        }

        public void AddStat(string _Label, Func<string> _Getter)
        {
            m_Stats.Add(new StatEntry(_Label, _Getter));
        }

        public void ClearStats()
        {
            m_Stats.Clear();

            m_StatsText.text = string.Empty;
        }

        public void Toast(string _Message)
        {
            m_ToastText.text = _Message;
            m_ToastShownTime = Time.unscaledTime;
        }

        //from DemoDirector.Awake: the auto-open waits for the intro handoff, which calls ShowHelp
        public void SuppressHelpAutoOpen()
        {
            m_HelpAutoOpenSuppressed = true;
        }

        //first entry this play session only: an R-reset's replayed intro must not re-open a window the player closed
        public void ShowHelpIfUnseen()
        {
            if (!s_HelpShownScenes.Contains(gameObject.scene.name))
                ShowHelp();
        }

        public void ShowHelp()
        {
            if (m_HelpOverlay == null || (m_HelpOverlay.gameObject.activeSelf && !m_HelpClosing))
                return;

            s_HelpShownScenes.Add(gameObject.scene.name);

            m_HelpOverlay.gameObject.SetActive(true);

            if (m_HelpBlocker != null)
                m_HelpBlocker.gameObject.SetActive(true);

            m_HelpClosing            = false;
            m_HelpAnimating          = true;
            m_HelpAnimationStartTime = Time.unscaledTime;

            FitHelpWindow(_Rebuild: true);

            //the frame-0 pose at once, or the window flashes full-size before growing
            UpdateHelpAnimation();

            OnHelpVisibilityChanged?.Invoke(true);
        }

        public void CloseHelp()
        {
            if (m_HelpOverlay == null || !m_HelpOverlay.gameObject.activeSelf || m_HelpClosing)
                return;

            if (m_HelpBlocker != null)
                m_HelpBlocker.gameObject.SetActive(false);

            m_HelpClosing            = true;
            m_HelpAnimating          = true;
            m_HelpAnimationStartTime = Time.unscaledTime;

            OnHelpVisibilityChanged?.Invoke(false);
        }

        #region Widget composition (edit-time capable, idempotent)

        public void ComposeSectionHeader(string _Label)
        {
            EnsureControlPanel();

            if (FindPanelElement($"Header_{_Label}") != null)
                return;

            GameObject headerGO = CreatePanelElement($"Header_{_Label}", SECTION_HEADER_HEIGHT);

            Text text = CreateChildText(headerGO.transform, _Label.ToUpperInvariant(), 16, FontStyle.Bold, TextAnchor.LowerLeft);

            text.color = new Color(1f, 1f, 1f, 0.6f);

            text.rectTransform.anchorMin = Vector2.zero;
            text.rectTransform.anchorMax = Vector2.one;
            text.rectTransform.offsetMin = new Vector2(4f, 0f);
            text.rectTransform.offsetMax = Vector2.zero;
        }

        //the selected button stays highlighted
        //a group whose labels come from a list the user edits (a palette of prefabs) may have grown or shrunk since it was authored:
        //a missing button is created beside its group, a surplus one hidden, and every caption follows the list
        public void ComposeButtonGroup(string _Id, string[] _Labels)
        {
            EnsureControlPanel();

            for (int i = 0; i < _Labels.Length; i++)
            {
                Transform existing = FindPanelElement($"Group_{_Id}_{i}");

                if (existing != null)
                {
                    existing.gameObject.SetActive(true);

                    Text caption = existing.GetComponentInChildren<Text>(true);

                    if (caption != null)
                        caption.text = _Labels[i];

                    continue;
                }

                CreatePanelButton($"Group_{_Id}_{i}", _Labels[i]);

                Transform previous = i > 0 ? FindPanelElement($"Group_{_Id}_{i - 1}") : null;

                if (previous != null)
                    FindPanelElement($"Group_{_Id}_{i}").SetSiblingIndex(previous.GetSiblingIndex() + 1);
            }

            for (int i = _Labels.Length; FindPanelElement($"Group_{_Id}_{i}") != null; i++)
                FindPanelElement($"Group_{_Id}_{i}").gameObject.SetActive(false);
        }

        public void ComposeButton(string _Id, string _Label)
        {
            EnsureControlPanel();

            if (FindPanelElement($"Button_{_Id}") != null)
                return;

            CreatePanelButton($"Button_{_Id}", _Label);
        }

        public void ComposeStepper(string _Id)
        {
            EnsureControlPanel();

            if (FindPanelElement($"Stepper_{_Id}") != null)
                return;

            GameObject rowGO = CreatePanelElement($"Stepper_{_Id}", BUTTON_HEIGHT);

            CreateStepperButton(rowGO.transform, "Minus", "−", 0f);
            CreateStepperButton(rowGO.transform, "Plus", "+", 1f);

            GameObject labelGO = new GameObject("Label");

            labelGO.transform.SetParent(rowGO.transform, false);

            RectTransform labelRect = labelGO.AddComponent<RectTransform>();

            labelRect.anchorMin = new Vector2(0f, 0f);
            labelRect.anchorMax = new Vector2(1f, 1f);
            labelRect.offsetMin = new Vector2(BUTTON_HEIGHT + 4f, 0f);
            labelRect.offsetMax = new Vector2(-(BUTTON_HEIGHT + 4f), 0f);

            ConfigureText(labelGO, 19, FontStyle.Normal, TextAnchor.MiddleCenter);
        }

        public void ComposeSlider(string _Id)
        {
            EnsureControlPanel();

            if (FindPanelElement($"Slider_{_Id}") != null)
                return;

            GameObject labelGO = CreatePanelElement($"SliderLabel_{_Id}", SLIDER_HEIGHT);
            Text       label   = CreateChildText(labelGO.transform, string.Empty, 19, FontStyle.Normal, TextAnchor.MiddleLeft);

            label.rectTransform.anchorMin = Vector2.zero;
            label.rectTransform.anchorMax = Vector2.one;
            label.rectTransform.offsetMin = new Vector2(4f, 0f);
            label.rectTransform.offsetMax = Vector2.zero;

            GameObject sliderGO = CreatePanelElement($"Slider_{_Id}", SLIDER_HEIGHT);

            BuildSliderWidget(sliderGO, 0f, 1f, 0f);
        }

        //a bounded quantity under a live label; the tint is authored at compose time
        public void ComposeProgressBar(string _Id, Color _FillTint)
        {
            EnsureControlPanel();

            if (FindPanelElement($"Bar_{_Id}") != null)
                return;

            GameObject barGO = CreatePanelElement($"Bar_{_Id}", PROGRESS_BAR_HEIGHT);

            barGO.AddComponent<Image>().color = SLIDER_BACKGROUND;

            GameObject fillGO = new GameObject("Fill");

            fillGO.transform.SetParent(barGO.transform, false);

            RectTransform fillRect = fillGO.AddComponent<RectTransform>();

            fillRect.anchorMin = Vector2.zero;
            fillRect.anchorMax = Vector2.one;
            fillRect.offsetMin = Vector2.zero;
            fillRect.offsetMax = Vector2.zero;

            fillGO.AddComponent<Image>().color = _FillTint;

            Text label = CreateChildText(barGO.transform, string.Empty, 17, FontStyle.Bold, TextAnchor.MiddleCenter);

            label.rectTransform.anchorMin = Vector2.zero;
            label.rectTransform.anchorMax = Vector2.one;
            label.rectTransform.offsetMin = Vector2.zero;
            label.rectTransform.offsetMax = Vector2.zero;
        }

        //a glyph tracking a world point's projection every frame; a null point or one behind the camera hides it
        public void ComposeWorldMarker(string _Id, string _Glyph, Color _Tint, int _FontSize)
        {
            if (transform.Find($"Marker_{_Id}") != null)
                return;

            GameObject markerGO = new GameObject($"Marker_{_Id}");

            markerGO.transform.SetParent(transform, false);

            RectTransform rect = markerGO.AddComponent<RectTransform>();

            rect.anchorMin = new Vector2(0.5f, 0.5f);
            rect.anchorMax = new Vector2(0.5f, 0.5f);
            rect.pivot     = new Vector2(0.5f, 0.5f);
            rect.sizeDelta = new Vector2(64f, 64f);

            Text glyph = ConfigureText(markerGO, _FontSize, FontStyle.Bold, TextAnchor.MiddleCenter);

            glyph.text          = _Glyph;
            glyph.color         = _Tint;
            glyph.raycastTarget = false;
        }

        //one large value beside the screen's centre, refreshed every frame; composition order stacks them downward
        public void ComposeReadout(string _Id, Color _Tint, int _FontSize)
        {
            if (transform.Find($"Readout_{_Id}") != null)
                return;

            int index = 0;

            foreach (Transform child in transform)
            {
                if (child.name.StartsWith("Readout_", StringComparison.Ordinal))
                    index++;
            }

            GameObject readoutGO = new GameObject($"Readout_{_Id}");

            readoutGO.transform.SetParent(transform, false);

            RectTransform rect = readoutGO.AddComponent<RectTransform>();

            rect.anchorMin        = new Vector2(0.5f, 0.5f);
            rect.anchorMax        = new Vector2(0.5f, 0.5f);
            rect.pivot            = new Vector2(1f, 0.5f);
            rect.anchoredPosition = new Vector2(READOUT_OFFSET_X, -index * READOUT_SPACING);
            rect.sizeDelta        = new Vector2(READOUT_WIDTH, READOUT_SPACING);

            Text value = ConfigureText(readoutGO, _FontSize, FontStyle.Bold, TextAnchor.MiddleRight);

            value.color         = _Tint;
            value.raycastTarget = false;
        }

        //Splash card: no close affordance, it lifts when the bound DoneGetter says so. Compose it last,
        //after the help overlay — it must draw over everything, the help window included. Idempotent.
        public void ComposeSplash(string _Title, string _Caption, Color _Tint)
        {
            if (m_Splash != null)
            {
                m_Splash.SetAsLastSibling();

                return;
            }

            GameObject splashGO = new GameObject("Splash");

            splashGO.transform.SetParent(transform, false);

            m_Splash = splashGO.AddComponent<RectTransform>();

            m_Splash.anchorMin = Vector2.zero;
            m_Splash.anchorMax = Vector2.one;
            m_Splash.offsetMin = Vector2.zero;
            m_Splash.offsetMax = Vector2.zero;

            //the image swallows clicks: raycastTarget stays on
            splashGO.AddComponent<Image>().color = SPLASH_BACKGROUND;

            CanvasGroup group = splashGO.AddComponent<CanvasGroup>();

            group.alpha          = 1f;
            group.blocksRaycasts = true;

            Text title = CreateCentredText(m_Splash, "Title", _Title, 56, FontStyle.Bold, 96f, 900f, 70f);

            title.color = _Tint;

            Text caption = CreateCentredText(m_Splash, "Caption", _Caption, 21, FontStyle.Normal, 40f, 900f, 26f);

            caption.color = HELP_BODY_COLOR;

            GameObject trackGO = new GameObject("Bar");

            trackGO.transform.SetParent(m_Splash, false);

            RectTransform track = trackGO.AddComponent<RectTransform>();

            track.anchorMin        = new Vector2(0.5f, 0.5f);
            track.anchorMax        = new Vector2(0.5f, 0.5f);
            track.pivot            = new Vector2(0.5f, 0.5f);
            track.anchoredPosition = new Vector2(0f, -30f);
            track.sizeDelta        = new Vector2(SPLASH_BAR_WIDTH, SPLASH_BAR_HEIGHT);

            trackGO.AddComponent<Image>().color = SPLASH_BAR_TRACK;

            GameObject fillGO = new GameObject("Fill");

            fillGO.transform.SetParent(trackGO.transform, false);

            RectTransform fill = fillGO.AddComponent<RectTransform>();

            fill.anchorMin = Vector2.zero;
            fill.anchorMax = new Vector2(0f, 1f);
            fill.offsetMin = Vector2.zero;
            fill.offsetMax = Vector2.zero;

            fillGO.AddComponent<Image>().color = _Tint;

            Text percent = CreateCentredText(m_Splash, "Percent", string.Empty, 30, FontStyle.Bold, 44f, 400f, -70f);

            percent.color = _Tint;

            Text status = CreateCentredText(m_Splash, "Status", string.Empty, 18, FontStyle.Normal, 30f, 900f, -108f);

            status.color = HELP_BODY_COLOR;

            m_Splash.SetAsLastSibling();
        }

        //status and fraction are read every frame while up; DoneGetter true fades the card out and lets clicks through
        public void BindSplash(string _Title, string _Caption, Color _Tint, Func<string> _StatusGetter, Func<float> _FractionGetter, Func<bool> _DoneGetter)
        {
            ComposeSplash(_Title, _Caption, _Tint);

            m_Splash.gameObject.SetActive(true);

            m_SplashLiftTime = float.NegativeInfinity;

            m_SplashEntry = new SplashEntry(
                    m_Splash.GetComponent<CanvasGroup>(),
                    m_Splash.Find("Status").GetComponent<Text>(),
                    m_Splash.Find("Percent").GetComponent<Text>(),
                    (RectTransform) m_Splash.Find("Bar/Fill"),
                    _StatusGetter,
                    _FractionGetter,
                    _DoneGetter);
        }

        //. Rebuilt from the scene's .json at every start, so a window an older bake left behind is thrown away here
        void ComposeHelpOverlay()
        {
            if (m_HelpOverlay != null)
            {
                //a runtime Destroy lands at the end of the frame; the stale window must not draw once more
                m_HelpOverlay.gameObject.SetActive(false);

                UtilsCommon.SmartDestroy(m_HelpOverlay.gameObject);

                m_HelpOverlay = null;
                m_HelpContent = null;
            }

            ComposeHelpButton();
            ComposeHelpBlocker();

            GameObject overlayGO = new GameObject("HelpOverlay");

            overlayGO.transform.SetParent(transform, false);

            m_HelpOverlay = overlayGO.AddComponent<RectTransform>();

            m_HelpOverlay.anchorMin = new Vector2(0.5f, 0.5f);
            m_HelpOverlay.anchorMax = new Vector2(0.5f, 0.5f);
            m_HelpOverlay.pivot     = new Vector2(0.5f, 0.5f);
            m_HelpOverlay.sizeDelta = new Vector2(HELP_WIDTH, HELP_DEFAULT_HEIGHT);

            overlayGO.AddComponent<Image>().color = HELP_BACKGROUND;

            //the window itself is sized at runtime (FitHelpWindow)
            GameObject viewportGO = new GameObject("Viewport");

            viewportGO.transform.SetParent(overlayGO.transform, false);

            RectTransform viewport = viewportGO.AddComponent<RectTransform>();

            viewport.anchorMin = Vector2.zero;
            viewport.anchorMax = Vector2.one;
            viewport.offsetMin = new Vector2(HELP_PADDING_X, HELP_PADDING_Y);
            viewport.offsetMax = new Vector2(-HELP_PADDING_X, -HELP_PADDING_Y);

            viewportGO.AddComponent<RectMask2D>();

            GameObject contentGO = new GameObject("Content");

            contentGO.transform.SetParent(viewportGO.transform, false);

            m_HelpContent = contentGO.AddComponent<RectTransform>();

            m_HelpContent.anchorMin = new Vector2(0f, 1f);
            m_HelpContent.anchorMax = new Vector2(1f, 1f);
            m_HelpContent.pivot     = new Vector2(0.5f, 1f);
            m_HelpContent.offsetMin = Vector2.zero;
            m_HelpContent.offsetMax = Vector2.zero;

            VerticalLayoutGroup layout = contentGO.AddComponent<VerticalLayoutGroup>();

            layout.spacing                = HELP_SECTION_SPACING;
            layout.childControlWidth      = true;
            layout.childControlHeight     = true;
            layout.childForceExpandWidth  = true;
            layout.childForceExpandHeight = false;

            contentGO.AddComponent<ContentSizeFitter>().verticalFit = ContentSizeFitter.FitMode.PreferredSize;

            ScrollRect scroll = overlayGO.AddComponent<ScrollRect>();

            scroll.viewport          = viewport;
            scroll.content           = m_HelpContent;
            scroll.horizontal        = false;
            scroll.vertical          = true;
            scroll.movementType      = ScrollRect.MovementType.Clamped;
            scroll.scrollSensitivity = HELP_SCROLL_SENSITIVITY;

            Text title = CreateChildText(contentGO.transform, DemoJson.String(m_Text, "TITLE"), 27, FontStyle.Bold, TextAnchor.MiddleLeft);

            title.gameObject.name = "Title";

            foreach (object entry in DemoJson.Array(m_Text, "HELP"))
            {
                Dictionary<string, object> section = DemoJson.Object(entry);

                Text header = CreateChildText(contentGO.transform, DemoJson.String(section, "HEADER").ToUpperInvariant(), 18, FontStyle.Bold, TextAnchor.MiddleLeft);

                header.color           = HELP_HEADER_COLOR;
                header.gameObject.name = "Header";

                Text body = CreateChildText(contentGO.transform, DemoJson.String(section, "BODY"), 19, FontStyle.Normal, TextAnchor.UpperLeft);

                body.color           = HELP_BODY_COLOR;
                body.gameObject.name = "Body";
            }

            ComposeHelpCloseButton();
        }

        #endregion

        #region Widget binding (runtime)

        public void BindButtonGroup(string _Id, string[] _Labels, Func<int> _GetSelected, Action<int> _OnSelect)
        {
            ComposeButtonGroup(_Id, _Labels);

            Image[] backgrounds = new Image[_Labels.Length];

            for (int i = 0; i < _Labels.Length; i++)
            {
                int       captured = i;
                Transform element  = FindPanelElement($"Group_{_Id}_{i}");
                Button    button   = element.GetComponent<Button>();

                button.onClick.RemoveAllListeners();
                button.onClick.AddListener(() => _OnSelect(captured));

                backgrounds[i] = element.GetComponent<Image>();
            }

            m_ButtonGroups.Add(new ButtonGroup(backgrounds, _GetSelected));
        }

        public void BindButton(string _Id, Action _OnClick)
        {
            ComposeButton(_Id, _Id);

            Button button = FindPanelElement($"Button_{_Id}").GetComponent<Button>();

            button.onClick.RemoveAllListeners();
            button.onClick.AddListener(() => _OnClick());
        }

        public void BindStepper(string _Id, Func<string> _LabelGetter, Action _OnDecrement, Action _OnIncrement)
        {
            ComposeStepper(_Id);

            Transform row = FindPanelElement($"Stepper_{_Id}");

            BindStepperButton(row, "Minus", _OnDecrement);
            BindStepperButton(row, "Plus", _OnIncrement);

            m_Steppers.Add(new StepperEntry(row.Find("Label").GetComponent<Text>(), _LabelGetter));
        }

        public void BindSlider(string _Id, Func<string> _LabelGetter, float _Min, float _Max, Func<float> _ValueGetter, Action<float> _OnChanged)
        {
            ComposeSlider(_Id);

            Text   label  = FindPanelElement($"SliderLabel_{_Id}").GetComponentInChildren<Text>();
            Slider slider = FindPanelElement($"Slider_{_Id}").GetComponent<Slider>();

            slider.minValue = _Min;
            slider.maxValue = _Max;

            slider.SetValueWithoutNotify(_ValueGetter());

            slider.onValueChanged.RemoveAllListeners();
            slider.onValueChanged.AddListener(_Value => _OnChanged(_Value));

            label.text = _LabelGetter();

            m_Sliders.Add(new SliderEntry(label, _LabelGetter, slider, _ValueGetter));
        }

        public void BindProgressBar(string _Id, Color _FillTint, Func<string> _LabelGetter, Func<float> _FractionGetter)
        {
            ComposeProgressBar(_Id, _FillTint);

            Transform bar = FindPanelElement($"Bar_{_Id}");

            m_Bars.Add(new BarEntry(
                    bar.GetComponentInChildren<Text>(),
                    (RectTransform) bar.Find("Fill"),
                    _LabelGetter,
                    _FractionGetter));
        }

        //a bottom-right plot; background, title, captions, legend and tints are authored, the scenario pushes normalized points into the DemoGraph BindGraph returns
        public void ComposeGraph(string _Id, string _Title, string _AxisCaption, Color[] _SeriesTints, string[] _SeriesLabels)
        {
            if (FindGraphPanel(_Id) != null)
                return;

            GameObject panelGO = new GameObject($"Graph_{_Id}");

            panelGO.transform.SetParent(transform, false);

            RectTransform panel = panelGO.AddComponent<RectTransform>();

            panel.anchorMin        = new Vector2(1f, 0f);
            panel.anchorMax        = new Vector2(1f, 0f);
            panel.pivot            = new Vector2(1f, 0f);
            panel.anchoredPosition = new Vector2(GRAPH_MARGIN_X, GRAPH_MARGIN_Y);
            panel.sizeDelta        = new Vector2(GRAPH_WIDTH, GRAPH_HEIGHT);

            panelGO.AddComponent<Image>().color = GRAPH_BACKGROUND;

            Text title = CreateChildText(panel, _Title, 18, FontStyle.Bold, TextAnchor.UpperLeft);

            StretchInside(title.rectTransform, GRAPH_PADDING, GRAPH_PADDING, GRAPH_PADDING, GRAPH_PADDING);

            GameObject plotGO = new GameObject("Plot");

            plotGO.transform.SetParent(panel, false);

            RectTransform plot = plotGO.AddComponent<RectTransform>();

            plot.anchorMin = Vector2.zero;
            plot.anchorMax = Vector2.one;
            plot.offsetMin = new Vector2(GRAPH_PADDING, GRAPH_PADDING + GRAPH_FOOTER);
            plot.offsetMax = new Vector2(-GRAPH_PADDING, -GRAPH_PADDING - GRAPH_HEADER);

            CreateGraphAxis(plot, new Vector2(0f, 0f), new Vector2(1f, 0f), new Vector2(0f, 1f));
            CreateGraphAxis(plot, new Vector2(0f, 0f), new Vector2(0f, 1f), new Vector2(1f, 0f));

            DemoGraph graph = plotGO.AddComponent<DemoGraph>();

            graph.Configure(_SeriesTints, GRAPH_LINE_WIDTH, GRAPH_REFERENCE);
            graph.raycastTarget = false;

            Text caption = CreateChildText(panel, _AxisCaption, 14, FontStyle.Normal, TextAnchor.UpperRight);

            caption.color = GRAPH_CAPTION;

            StretchInside(caption.rectTransform, GRAPH_PADDING, GRAPH_PADDING, GRAPH_PADDING, GRAPH_PADDING);

            for (int i = 0; i < _SeriesLabels.Length; i++)
            {
                Text legend = CreateChildText(panel, _SeriesLabels[i], 15, FontStyle.Bold, TextAnchor.LowerCenter);

                legend.color = i < _SeriesTints.Length ? _SeriesTints[i] : Color.white;

                RectTransform rect = legend.rectTransform;

                rect.anchorMin        = new Vector2(i / (float) _SeriesLabels.Length, 0f);
                rect.anchorMax        = new Vector2((i + 1) / (float) _SeriesLabels.Length, 0f);
                rect.pivot            = new Vector2(0.5f, 0f);
                //anchors span the label's share of the width and the offsets give the height; anchoredPosition would fight them
                rect.offsetMin = new Vector2(0f, GRAPH_PADDING * 0.5f);
                rect.offsetMax = new Vector2(0f, GRAPH_PADDING * 0.5f + GRAPH_FOOTER);
            }
        }

        public DemoGraph BindGraph(string _Id, string _Title, string _AxisCaption, Color[] _SeriesTints, string[] _SeriesLabels)
        {
            ComposeGraph(_Id, _Title, _AxisCaption, _SeriesTints, _SeriesLabels);

            return FindGraphPanel(_Id).GetComponentInChildren<DemoGraph>(true);
        }

        public void SetGraphVisible(string _Id, bool _Visible)
        {
            Transform panel = FindGraphPanel(_Id);

            if (panel != null)
                panel.gameObject.SetActive(_Visible);
        }

        public void ComposeThreatArrows(string _Id, int _Count, Color _Tint, int _FontSize)
        {
            if (transform.Find($"Threats_{_Id}") != null)
                return;

            GameObject poolGO = new GameObject($"Threats_{_Id}");

            poolGO.transform.SetParent(transform, false);

            RectTransform poolRect = poolGO.AddComponent<RectTransform>();

            poolRect.anchorMin = Vector2.zero;
            poolRect.anchorMax = Vector2.one;
            poolRect.offsetMin = Vector2.zero;
            poolRect.offsetMax = Vector2.zero;

            for (int i = 0; i < _Count; i++)
            {
                GameObject arrowGO = new GameObject($"Arrow_{i}");

                arrowGO.transform.SetParent(poolGO.transform, false);

                RectTransform rect = arrowGO.AddComponent<RectTransform>();

                rect.anchorMin = new Vector2(0.5f, 0.5f);
                rect.anchorMax = new Vector2(0.5f, 0.5f);
                rect.pivot     = new Vector2(0.5f, 0.5f);
                rect.sizeDelta = new Vector2(64f, 64f);

                Text glyph = ConfigureText(arrowGO, _FontSize, FontStyle.Bold, TextAnchor.MiddleCenter);

                glyph.text          = THREAT_ARROW_GLYPH;
                glyph.color         = _Tint;
                glyph.raycastTarget = false;

                arrowGO.SetActive(false);
            }
        }

        //positions beyond _Count are dropped
        public void BindThreatArrows(string _Id, int _Count, Color _Tint, int _FontSize, Func<IReadOnlyList<Vector3>> _WorldGetter)
        {
            ComposeThreatArrows(_Id, _Count, _Tint, _FontSize);

            Transform pool = transform.Find($"Threats_{_Id}");

            RectTransform[] arrows = new RectTransform[pool.childCount];

            for (int i = 0; i < arrows.Length; i++)
                arrows[i] = (RectTransform) pool.GetChild(i);

            m_ThreatArrows.Add(new ThreatArrowsEntry(arrows, _WorldGetter));
        }

        public void BindWorldMarker(string _Id, string _Glyph, Color _Tint, int _FontSize, Func<Vector3?> _WorldGetter)
        {
            ComposeWorldMarker(_Id, _Glyph, _Tint, _FontSize);

            m_Markers.Add(new MarkerEntry((RectTransform) transform.Find($"Marker_{_Id}"), _WorldGetter));
        }

        public void BindReadout(string _Id, Color _Tint, int _FontSize, Func<string> _Getter)
        {
            ComposeReadout(_Id, _Tint, _FontSize);

            m_Readouts.Add(new ReadoutEntry(transform.Find($"Readout_{_Id}").GetComponent<Text>(), _Getter));
        }

        #endregion

        #endregion

        #region Service methods

        void RefreshStats()
        {
            if (m_Stats.Count == 0)
                return;

            System.Text.StringBuilder builder = new System.Text.StringBuilder();

            foreach (StatEntry stat in m_Stats)
            {
                string value;

                try
                {
                    value = stat.Getter();
                }
                catch (Exception exception)
                {
                    value = $"<{exception.GetType().Name}>";
                }

                builder.AppendLine($"{stat.Label}: {value}");
            }

            m_StatsText.text = builder.ToString();
        }

        void RefreshControls()
        {
            foreach (ButtonGroup group in m_ButtonGroups)
            {
                int selected = group.GetSelected();

                for (int i = 0; i < group.Backgrounds.Length; i++)
                    group.Backgrounds[i].color = i == selected ? BUTTON_SELECTED : BUTTON_IDLE;
            }

            foreach (SliderEntry entry in m_Sliders)
            {
                entry.Label.text = entry.LabelGetter();

                float value = entry.ValueGetter();

                if (!Mathf.Approximately(entry.Slider.value, value))
                    entry.Slider.SetValueWithoutNotify(value);
            }

            foreach (StepperEntry entry in m_Steppers)
                entry.Label.text = entry.LabelGetter();

            foreach (BarEntry entry in m_Bars)
            {
                entry.Label.text = entry.LabelGetter();

                entry.Fill.anchorMax = new Vector2(Mathf.Clamp01(entry.FractionGetter()), 1f);
            }
        }

        //every frame, unlike the quarter-second stats
        void RefreshReadouts()
        {
            foreach (ReadoutEntry entry in m_Readouts)
                entry.Value.text = entry.Getter();
        }

        //every frame, unlike the quarter-second stats
        void RefreshThreatArrows()
        {
            if (m_ThreatArrows.Count == 0)
                return;

            Camera        camera     = Camera.main;
            RectTransform canvasRect = (RectTransform) transform;
            Rect          canvas     = canvasRect.rect;

            float halfWidth  = canvas.width * 0.5f - THREAT_ARROW_EDGE_MARGIN;
            float halfHeight = canvas.height * 0.5f - THREAT_ARROW_EDGE_MARGIN;

            foreach (ThreatArrowsEntry entry in m_ThreatArrows)
            {
                IReadOnlyList<Vector3> threats = entry.WorldGetter();

                for (int i = 0; i < entry.Arrows.Length; i++)
                {
                    RectTransform arrow = entry.Arrows[i];

                    if (camera == null || threats == null || i >= threats.Count)
                    {
                        arrow.gameObject.SetActive(false);

                        continue;
                    }

                    Vector3 screen = camera.WorldToScreenPoint(threats[i]);

                    RectTransformUtility.ScreenPointToLocalPointInRectangle(canvasRect, screen, null, out Vector2 local);

                    //behind the camera the projection is mirrored
                    if (screen.z <= 0f)
                        local = -local;

                    bool onScreen = screen.z > 0f && Mathf.Abs(local.x) <= halfWidth && Mathf.Abs(local.y) <= halfHeight;

                    arrow.gameObject.SetActive(true);

                    if (onScreen)
                    {
                        arrow.anchoredPosition = local + Vector2.up * THREAT_ARROW_HOVER;
                        arrow.localRotation    = Quaternion.Euler(0f, 0f, 180f);

                        continue;
                    }

                    Vector2 direction = local.sqrMagnitude > 1e-4f ? local.normalized : Vector2.up;

                    float scale = Mathf.Min(
                            Mathf.Abs(direction.x) > 1e-4f ? halfWidth / Mathf.Abs(direction.x) : float.PositiveInfinity,
                            Mathf.Abs(direction.y) > 1e-4f ? halfHeight / Mathf.Abs(direction.y) : float.PositiveInfinity);

                    arrow.anchoredPosition = direction * scale;
                    arrow.localRotation    = Quaternion.Euler(0f, 0f, Mathf.Atan2(direction.y, direction.x) * Mathf.Rad2Deg - 90f);
                }
            }
        }

        void RefreshMarkers()
        {
            if (m_Markers.Count == 0)
                return;

            Camera        camera     = Camera.main;
            RectTransform canvasRect = (RectTransform) transform;

            foreach (MarkerEntry entry in m_Markers)
            {
                Vector3? world = entry.WorldGetter();

                if (camera == null || world == null)
                {
                    entry.Rect.gameObject.SetActive(false);

                    continue;
                }

                Vector3 screen = camera.WorldToScreenPoint(world.Value);

                if (screen.z <= 0f)
                {
                    entry.Rect.gameObject.SetActive(false);

                    continue;
                }

                RectTransformUtility.ScreenPointToLocalPointInRectangle(canvasRect, screen, null, out Vector2 local);

                entry.Rect.gameObject.SetActive(true);

                entry.Rect.anchoredPosition = local;
            }
        }

        Transform FindGraphPanel(string _Id)
        {
            return transform.Find($"Graph_{_Id}");
        }

        void CreateGraphAxis(RectTransform _Plot, Vector2 _AnchorMin, Vector2 _AnchorMax, Vector2 _Thickness)
        {
            GameObject axisGO = new GameObject("Axis");

            axisGO.transform.SetParent(_Plot, false);

            RectTransform rect = axisGO.AddComponent<RectTransform>();

            rect.anchorMin = _AnchorMin;
            rect.anchorMax = _AnchorMax;
            rect.pivot     = Vector2.zero;
            rect.offsetMin = Vector2.zero;
            rect.offsetMax = _Thickness;

            Image axis = axisGO.AddComponent<Image>();

            axis.color         = GRAPH_AXIS;
            axis.raycastTarget = false;
        }

        static void StretchInside(RectTransform _Rect, float _Left, float _Right, float _Top, float _Bottom)
        {
            _Rect.anchorMin = Vector2.zero;
            _Rect.anchorMax = Vector2.one;
            _Rect.offsetMin = new Vector2(_Left, _Bottom);
            _Rect.offsetMax = new Vector2(-_Right, -_Top);
        }

        Transform FindPanelElement(string _Name)
        {
            return m_ControlPanel != null ? m_ControlPanel.Find(_Name) : null;
        }

        void CreatePanelButton(string _Name, string _Label)
        {
            GameObject buttonGO = CreatePanelElement(_Name, BUTTON_HEIGHT);

            Image background = buttonGO.AddComponent<Image>();

            background.color = BUTTON_IDLE;

            Button button = buttonGO.AddComponent<Button>();

            button.targetGraphic = background;

            Text label = CreateChildText(buttonGO.transform, _Label, 20, FontStyle.Bold, TextAnchor.MiddleCenter);

            label.rectTransform.anchorMin = Vector2.zero;
            label.rectTransform.anchorMax = Vector2.one;
            label.rectTransform.offsetMin = Vector2.zero;
            label.rectTransform.offsetMax = Vector2.zero;
        }

        void CreateStepperButton(Transform _Row, string _Name, string _Symbol, float _AnchorX)
        {
            GameObject buttonGO = new GameObject(_Name);

            buttonGO.transform.SetParent(_Row, false);

            RectTransform rect = buttonGO.AddComponent<RectTransform>();

            rect.anchorMin        = new Vector2(_AnchorX, 0f);
            rect.anchorMax        = new Vector2(_AnchorX, 1f);
            rect.pivot            = new Vector2(_AnchorX, 0.5f);
            rect.anchoredPosition = Vector2.zero;
            rect.sizeDelta        = new Vector2(BUTTON_HEIGHT, 0f);

            Image background = buttonGO.AddComponent<Image>();

            background.color = BUTTON_IDLE;

            Button button = buttonGO.AddComponent<Button>();

            button.targetGraphic = background;

            Text symbol = CreateChildText(buttonGO.transform, _Symbol, 26, FontStyle.Bold, TextAnchor.MiddleCenter);

            symbol.rectTransform.anchorMin = Vector2.zero;
            symbol.rectTransform.anchorMax = Vector2.one;
            symbol.rectTransform.offsetMin = Vector2.zero;
            symbol.rectTransform.offsetMax = Vector2.zero;
        }

        static void BindStepperButton(Transform _Row, string _Name, Action _OnClick)
        {
            Button button = _Row.Find(_Name).GetComponent<Button>();

            button.onClick.RemoveAllListeners();
            button.onClick.AddListener(() => _OnClick());
        }

        void UpdateSplash()
        {
            if (m_SplashEntry == null || m_Splash == null || !m_Splash.gameObject.activeSelf)
                return;

            SplashEntry entry = m_SplashEntry.Value;

            if (float.IsNegativeInfinity(m_SplashLiftTime))
            {
                float fraction = Mathf.Clamp01(entry.FractionGetter());

                entry.Status.text    = entry.StatusGetter();
                entry.Percent.text   = $"{fraction * 100f:F0} %";
                entry.Fill.anchorMax = new Vector2(fraction, 1f);

                if (!entry.DoneGetter())
                    return;

                //the bar reads full for the whole fade; clicks go through from this frame on
                entry.Fill.anchorMax       = Vector2.one;
                entry.Percent.text         = "100 %";
                entry.Group.blocksRaycasts = false;
                m_SplashLiftTime           = Time.unscaledTime;
            }

            float progress = Mathf.Clamp01((Time.unscaledTime - m_SplashLiftTime) / SPLASH_FADE_DURATION);

            entry.Group.alpha = 1f - progress;

            if (progress < 1f)
                return;

            m_Splash.gameObject.SetActive(false);

            m_SplashEntry = null;
        }

        Text CreateCentredText(Transform _Parent, string _Name, string _Content, int _FontSize, FontStyle _Style, float _Height, float _Width, float _OffsetY)
        {
            GameObject textGO = new GameObject(_Name);

            textGO.transform.SetParent(_Parent, false);

            RectTransform rect = textGO.AddComponent<RectTransform>();

            rect.anchorMin        = new Vector2(0.5f, 0.5f);
            rect.anchorMax        = new Vector2(0.5f, 0.5f);
            rect.pivot            = new Vector2(0.5f, 0.5f);
            rect.anchoredPosition = new Vector2(0f, _OffsetY);
            rect.sizeDelta        = new Vector2(_Width, _Height);

            Text text = ConfigureText(textGO, _FontSize, _Style, TextAnchor.MiddleCenter);

            text.text          = _Content;
            text.raycastTarget = false;

            return text;
        }

        void UpdateToast()
        {
            //a live toast's clock is carried through a pause (the player is reading it behind the menu). Only a live one: a scene that
            //opens paused would walk a never-shown toast's stamp forward from zero. The help animation and the splash fade are not
            //held — the help window opens by pausing, and freezing what a pause starts would freeze it half-drawn
            if (Time.unscaledTime - m_ToastShownTime < TOAST_DURATION && Mathf.Approximately(Time.timeScale, 0f))
                m_ToastShownTime += Time.unscaledDeltaTime;

            float shownFor = Time.unscaledTime - m_ToastShownTime;

            if (shownFor >= TOAST_DURATION)
            {
                m_ToastText.enabled = false;

                return;
            }

            m_ToastText.enabled = true;

            float fadeStart = TOAST_DURATION - TOAST_FADE_TAIL;
            float alpha     = shownFor <= fadeStart ? 1f : 1f - (shownFor - fadeStart) / TOAST_FADE_TAIL;
            Color color     = m_ToastText.color;

            color.a           = alpha;
            m_ToastText.color = color;
        }

        void ComposeHelpButton()
        {
            if (m_HelpButton != null)
                return;

            GameObject buttonGO = new GameObject("HelpButton");

            buttonGO.transform.SetParent(transform, false);

            m_HelpButton = buttonGO.AddComponent<RectTransform>();

            m_HelpButton.anchorMin        = new Vector2(1f, 0f);
            m_HelpButton.anchorMax        = new Vector2(1f, 0f);
            m_HelpButton.pivot            = new Vector2(1f, 0f);
            m_HelpButton.anchoredPosition = new Vector2(-24f, 16f);
            m_HelpButton.sizeDelta        = new Vector2(HELP_BUTTON_SIZE, HELP_BUTTON_SIZE);

            Image background = buttonGO.AddComponent<Image>();

            background.color = PANEL_BACKGROUND;

            buttonGO.AddComponent<Button>().targetGraphic = background;

            Text label = CreateChildText(buttonGO.transform, "?", 30, FontStyle.Bold, TextAnchor.MiddleCenter);

            label.rectTransform.anchorMin = Vector2.zero;
            label.rectTransform.anchorMax = Vector2.one;
            label.rectTransform.offsetMin = Vector2.zero;
            label.rectTransform.offsetMax = Vector2.zero;
        }

        //keeps scene clicks from falling through; clicking it closes the window
        void ComposeHelpBlocker()
        {
            if (m_HelpBlocker != null)
                return;

            GameObject blockerGO = new GameObject("HelpBlocker");

            blockerGO.transform.SetParent(transform, false);

            m_HelpBlocker = blockerGO.AddComponent<RectTransform>();

            m_HelpBlocker.anchorMin = Vector2.zero;
            m_HelpBlocker.anchorMax = Vector2.one;
            m_HelpBlocker.offsetMin = Vector2.zero;
            m_HelpBlocker.offsetMax = Vector2.zero;

            blockerGO.AddComponent<Image>().color = HELP_BLOCKER;

            Button button = blockerGO.AddComponent<Button>();

            button.transition = Selectable.Transition.None;
        }

        void ComposeHelpCloseButton()
        {
            GameObject closeGO = new GameObject("Close");

            closeGO.transform.SetParent(m_HelpOverlay, false);

            RectTransform rect = closeGO.AddComponent<RectTransform>();

            rect.anchorMin        = new Vector2(1f, 1f);
            rect.anchorMax        = new Vector2(1f, 1f);
            rect.pivot            = new Vector2(1f, 1f);
            rect.anchoredPosition = new Vector2(-10f, -10f);
            rect.sizeDelta        = new Vector2(34f, 34f);

            closeGO.AddComponent<LayoutElement>().ignoreLayout = true;

            Image background = closeGO.AddComponent<Image>();

            background.color = BUTTON_IDLE;

            closeGO.AddComponent<Button>().targetGraphic = background;

            Text label = CreateChildText(closeGO.transform, "×", 24, FontStyle.Bold, TextAnchor.MiddleCenter);

            label.rectTransform.anchorMin = Vector2.zero;
            label.rectTransform.anchorMax = Vector2.one;
            label.rectTransform.offsetMin = Vector2.zero;
            label.rectTransform.offsetMax = Vector2.zero;
        }

        //from Start: after Awake built the window and after any director's Awake could suppress the open
        void InitHelp()
        {
            if (m_HelpButton != null)
            {
                Button reopen = m_HelpButton.GetComponent<Button>();

                reopen.onClick.RemoveAllListeners();
                reopen.onClick.AddListener(ShowHelp);
            }

            if (m_HelpBlocker != null)
            {
                Button blocker = m_HelpBlocker.GetComponent<Button>();

                blocker.onClick.RemoveAllListeners();
                blocker.onClick.AddListener(CloseHelp);
            }

            Transform close = m_HelpOverlay.Find("Close");

            if (close != null)
            {
                Button closeButton = close.GetComponent<Button>();

                closeButton.onClick.RemoveAllListeners();
                closeButton.onClick.AddListener(CloseHelp);
            }

            HideHelpImmediate();

            if (!m_HelpAutoOpenSuppressed && !s_HelpShownScenes.Contains(gameObject.scene.name))
                ShowHelp();
        }

        //a rebuild (on open) forces the layout so the preferred height is current
        void FitHelpWindow(bool _Rebuild)
        {
            if (m_HelpOverlay == null || m_HelpContent == null)
                return;

            if (_Rebuild)
            {
                LayoutRebuilder.ForceRebuildLayoutImmediate(m_HelpContent);

                ScrollRect scroll = m_HelpOverlay.GetComponent<ScrollRect>();

                if (scroll != null)
                    scroll.verticalNormalizedPosition = 1f;
            }

            float wanted    = LayoutUtility.GetPreferredHeight(m_HelpContent) + 2f * HELP_PADDING_Y;
            float available = ((RectTransform) transform).rect.height - 2f * HELP_SCREEN_MARGIN;
            float height    = Mathf.Min(wanted, Mathf.Max(available, HELP_PADDING_Y * 4f));

            if (!Mathf.Approximately(m_HelpOverlay.sizeDelta.y, height))
                m_HelpOverlay.sizeDelta = new Vector2(HELP_WIDTH, height);
        }

        void HideHelpImmediate()
        {
            m_HelpOverlay.gameObject.SetActive(false);

            if (m_HelpBlocker != null)
                m_HelpBlocker.gameObject.SetActive(false);
        }

        void UpdateHelpAnimation()
        {
            if (!m_HelpAnimating || m_HelpOverlay == null)
                return;

            float progress = Mathf.Clamp01((Time.unscaledTime - m_HelpAnimationStartTime) / HELP_ANIMATION_DURATION);

            //0 = full-size at screen centre, 1 = shrunk into the "?" button; closing accelerates into the button, opening decelerates out of it
            float shrink = m_HelpClosing ? progress * progress : (1f - progress) * (1f - progress);

            Vector3 buttonPosition = m_HelpButton != null ? m_HelpButton.localPosition : Vector3.zero;
            float   buttonScale    = HELP_BUTTON_SIZE / HELP_WIDTH;

            m_HelpOverlay.localPosition = Vector3.Lerp(Vector3.zero, buttonPosition, shrink);
            m_HelpOverlay.localScale    = Vector3.one * Mathf.Lerp(1f, buttonScale, shrink);

            if (progress < 1f)
                return;

            m_HelpAnimating = false;

            if (!m_HelpClosing)
                return;

            m_HelpOverlay.gameObject.SetActive(false);

            m_HelpOverlay.localPosition = Vector3.zero;
            m_HelpOverlay.localScale    = Vector3.one;
        }

        //authors the chrome into the scene; idempotent. Awake uses it only as a fallback for a scene without authored chrome
        public void ComposeChrome()
        {
            if (GetComponent<Canvas>() == null)
            {
                Canvas canvas = gameObject.AddComponent<Canvas>();

                canvas.renderMode   = RenderMode.ScreenSpaceOverlay;
                canvas.sortingOrder = CANVAS_SORT_ORDER;

                CanvasScaler scaler = gameObject.AddComponent<CanvasScaler>();

                scaler.uiScaleMode         = CanvasScaler.ScaleMode.ScaleWithScreenSize;
                scaler.referenceResolution = new Vector2(1920f, 1080f);

                gameObject.AddComponent<GraphicRaycaster>();
            }

            m_TitleText = CreateText(
                    "Title", TextAnchor.UpperLeft, new Vector2(0f, 1f), new Vector2(0f, 1f),
                    new Vector2(24f, -40f), new Vector2(900f, 46f), 34, FontStyle.Bold
                );

            m_CaptionText = CreateText(
                    "Caption", TextAnchor.UpperLeft, new Vector2(0f, 1f), new Vector2(0f, 1f),
                    new Vector2(24f, -88f), new Vector2(1100f, 34f), 20, FontStyle.Normal
                );

            m_HintsText = CreateText(
                    "Hints", TextAnchor.LowerCenter, new Vector2(0.5f, 0f), new Vector2(0.5f, 0f),
                    new Vector2(0f, 16f), new Vector2(1800f, 36f), 19, FontStyle.Normal
                );

            m_StatsText = CreateText(
                    "Stats", TextAnchor.UpperRight, new Vector2(1f, 1f), new Vector2(1f, 1f),
                    new Vector2(-24f, -40f), new Vector2(560f, 700f), 21, FontStyle.Normal
                );

            m_ToastText = CreateText(
                    "Toast", TextAnchor.MiddleCenter, new Vector2(0.5f, 0f), new Vector2(0.5f, 0f),
                    new Vector2(0f, 78f), new Vector2(1400f, 36f), 24, FontStyle.Bold
                );

            m_ToastText.enabled = false;
        }

        void EnsureControlPanel()
        {
            if (m_ControlPanel != null)
                return;

            GameObject panelGO = new GameObject("ControlPanel");

            panelGO.transform.SetParent(transform, false);

            m_ControlPanel = panelGO.AddComponent<RectTransform>();

            m_ControlPanel.anchorMin        = new Vector2(0f, 0.5f);
            m_ControlPanel.anchorMax        = new Vector2(0f, 0.5f);
            m_ControlPanel.pivot            = new Vector2(0f, 0.5f);
            m_ControlPanel.anchoredPosition = new Vector2(24f, 0f);
            m_ControlPanel.sizeDelta        = new Vector2(PANEL_WIDTH, 100f);

            Image background = panelGO.AddComponent<Image>();

            background.color = PANEL_BACKGROUND;

            VerticalLayoutGroup layout = panelGO.AddComponent<VerticalLayoutGroup>();

            layout.spacing              = PANEL_SPACING;
            layout.padding              = new RectOffset(10, 10, 10, 10);
            layout.childControlWidth    = true;
            layout.childControlHeight   = false;
            layout.childForceExpandWidth = true;

            ContentSizeFitter fitter = panelGO.AddComponent<ContentSizeFitter>();

            fitter.verticalFit = ContentSizeFitter.FitMode.PreferredSize;
        }

        GameObject CreatePanelElement(string _Name, float _Height)
        {
            GameObject elementGO = new GameObject(_Name);

            elementGO.transform.SetParent(m_ControlPanel, false);

            RectTransform rect = elementGO.AddComponent<RectTransform>();

            rect.sizeDelta = new Vector2(PANEL_WIDTH - 20f, _Height);

            LayoutElement layoutElement = elementGO.AddComponent<LayoutElement>();

            layoutElement.preferredHeight = _Height;
            layoutElement.minHeight       = _Height;

            return elementGO;
        }

        void BuildSliderWidget(GameObject _SliderGO, float _Min, float _Max, float _Value)
        {
            Slider slider = _SliderGO.AddComponent<Slider>();

            GameObject backgroundGO = new GameObject("Background");

            backgroundGO.transform.SetParent(_SliderGO.transform, false);

            RectTransform backgroundRect = backgroundGO.AddComponent<RectTransform>();

            backgroundRect.anchorMin = new Vector2(0f, 0.3f);
            backgroundRect.anchorMax = new Vector2(1f, 0.7f);
            backgroundRect.offsetMin = Vector2.zero;
            backgroundRect.offsetMax = Vector2.zero;

            backgroundGO.AddComponent<Image>().color = SLIDER_BACKGROUND;

            GameObject fillAreaGO = new GameObject("Fill Area");

            fillAreaGO.transform.SetParent(_SliderGO.transform, false);

            RectTransform fillAreaRect = fillAreaGO.AddComponent<RectTransform>();

            fillAreaRect.anchorMin = new Vector2(0f, 0.3f);
            fillAreaRect.anchorMax = new Vector2(1f, 0.7f);
            fillAreaRect.offsetMin = Vector2.zero;
            fillAreaRect.offsetMax = Vector2.zero;

            GameObject fillGO = new GameObject("Fill");

            fillGO.transform.SetParent(fillAreaGO.transform, false);

            RectTransform fillRect = fillGO.AddComponent<RectTransform>();

            fillRect.anchorMin = Vector2.zero;
            fillRect.anchorMax = Vector2.one;
            fillRect.offsetMin = Vector2.zero;
            fillRect.offsetMax = Vector2.zero;

            fillGO.AddComponent<Image>().color = SLIDER_FILL;

            GameObject handleAreaGO = new GameObject("Handle Slide Area");

            handleAreaGO.transform.SetParent(_SliderGO.transform, false);

            RectTransform handleAreaRect = handleAreaGO.AddComponent<RectTransform>();

            handleAreaRect.anchorMin = Vector2.zero;
            handleAreaRect.anchorMax = Vector2.one;
            handleAreaRect.offsetMin = new Vector2(8f, 0f);
            handleAreaRect.offsetMax = new Vector2(-8f, 0f);

            GameObject handleGO = new GameObject("Handle");

            handleGO.transform.SetParent(handleAreaGO.transform, false);

            RectTransform handleRect = handleGO.AddComponent<RectTransform>();

            handleRect.sizeDelta = new Vector2(16f, 0f);

            Image handleImage = handleGO.AddComponent<Image>();

            handleImage.color = SLIDER_HANDLE;

            slider.fillRect      = fillRect;
            slider.handleRect    = handleRect;
            slider.targetGraphic = handleImage;
            slider.minValue      = _Min;
            slider.maxValue      = _Max;

            slider.SetValueWithoutNotify(_Value);
        }

        Text CreateText(string _Name, TextAnchor _Anchor, Vector2 _AnchorMin, Vector2 _AnchorMax, Vector2 _AnchoredPosition, Vector2 _Size, int _FontSize, FontStyle _Style)
        {
            GameObject textGO = new GameObject(_Name);

            textGO.transform.SetParent(transform, false);

            RectTransform rect = textGO.AddComponent<RectTransform>();

            rect.anchorMin        = _AnchorMin;
            rect.anchorMax        = _AnchorMax;
            rect.pivot            = new Vector2(_AnchorMin.x, _AnchorMin.y);
            rect.anchoredPosition = _AnchoredPosition;
            rect.sizeDelta        = _Size;

            Text text = ConfigureText(textGO, _FontSize, _Style, _Anchor);

            //chrome is read, never clicked: a raycast target here swallows the world click under it (the 560 x 700 stats rect
            //took every press on the ResolutionRegion lens's upper right, 2026-09-24)
            text.raycastTarget = false;

            return text;
        }

        Text CreateChildText(Transform _Parent, string _Content, int _FontSize, FontStyle _Style, TextAnchor _Anchor)
        {
            GameObject textGO = new GameObject("Text");

            textGO.transform.SetParent(_Parent, false);

            textGO.AddComponent<RectTransform>();

            Text text = ConfigureText(textGO, _FontSize, _Style, _Anchor);

            text.text = _Content;

            return text;
        }

        static Text ConfigureText(GameObject _TextGO, int _FontSize, FontStyle _Style, TextAnchor _Anchor)
        {
            Text text = _TextGO.AddComponent<Text>();

            text.font      = Resources.GetBuiltinResource<Font>(BUILTIN_FONT);
            text.fontSize  = _FontSize;
            text.fontStyle = _Style;
            text.alignment = _Anchor;
            text.color     = Color.white;

            Outline outline = _TextGO.AddComponent<Outline>();

            outline.effectColor    = new Color(0f, 0f, 0f, 0.75f);
            outline.effectDistance = new Vector2(1f, -1f);

            return text;
        }

        #endregion
    }
}
