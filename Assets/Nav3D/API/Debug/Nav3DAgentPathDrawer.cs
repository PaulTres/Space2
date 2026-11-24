#if UNITY_EDITOR || NAV3D_DEBUG_VISUALIZATION

using Nav3D.Agents;
using Nav3D.Common;
using Nav3D.LocalAvoidance;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

namespace Nav3D.API
{
    /// <summary>
    /// Runtime (play-mode) renderer of the paths live <see cref="Nav3DAgent"/>s are following: a line per agent, split at
    /// the agent so the part already flown is dimmer, and for a radius-aware path also the capsule body the route was
    /// searched for, coloured by the result's state. <see cref="Active"/> draws every agent's path; an agent whose Draw path
    /// is ticked has its own drawn either way.
    /// Compiled in the editor, and in a player build only under the NAV3D_DEBUG_VISUALIZATION scripting define.
    /// </summary>
    [DisallowMultipleComponent]
    public class Nav3DAgentPathDrawer : MonoBehaviour
    {

        #region Nested types

        //what one agent owns in the drawer: its line, and the two halves of its body — the part flown and the part ahead
        sealed class AgentView
        {

            #region Properties

            public LineRenderer Line          { get; }
            public MeshFilter   TraveledBody  { get; }
            public MeshFilter   RemainingBody { get; }

            #endregion

            #region Constructors

            public AgentView(LineRenderer _Line, MeshFilter _TraveledBody, MeshFilter _RemainingBody)
            {
                Line          = _Line;
                TraveledBody  = _TraveledBody;
                RemainingBody = _RemainingBody;
            }

            #endregion

        }

        #endregion

        #region Constants

        //loaded from Resources so a player build ships them
        const string LINE_SHADER_RESOURCE = "Nav3DAgentPath";
        const string BODY_SHADER_RESOURCE = "Nav3DAgentPathBody";

        //shorter than this and a piece of a route is not worth a mesh
        const float MIN_DRAWN_LENGTH = 0.05f;

        #endregion

        #region Serialized fields

        [SerializeField, Tooltip(
                 "Draw the path of every agent. While inactive only the agents with Draw path ticked are drawn."
             )]
        bool m_Active;

        [SerializeField, Tooltip(
                 "Seconds between two reads of the agents' paths. A path is redrawn on this cadence, never per frame."
             )]
        float m_RefreshPeriod = 0.1f;

        [SerializeField] Color m_PathColor = new Color(0.45f, 0.8f, 1f, 0.85f);

        [SerializeField, Tooltip("The colour of a radius-aware route delivered as a point route because no corridor fit (Degraded).")]
        Color m_DegradedColor = new Color(1f, 0.6f, 0.2f, 0.85f);

        [SerializeField, Tooltip("The colour of a route delivered after its timeout passed, its refinement cut short (TimedOut).")]
        Color m_TimedOutColor = new Color(1f, 0.9f, 0.3f, 0.85f);

        [SerializeField, Tooltip("Line width in world units.")]
        float m_Width = 0.12f;

        [SerializeField, Range(0f, 0.5f), Tooltip(
                 "Share of the line's width, and of a body's visible radius, drawn as a rim in a contrasting tone of the path's colour, " +
                 "so a path stays readable on a bright sky and on dark geometry alike. 0 draws no rim."
             )]
        float m_Outline = 0.125f;

        [SerializeField, Tooltip("Alpha multiplier for the part of a path hidden behind geometry.")]
        float m_OccludedFade = 0.25f;

        [SerializeField, Range(0f, 1f), Tooltip("Alpha multiplier for the part of a route the agent has already flown.")]
        float m_TraveledFade = 0.35f;

        [SerializeField, Range(0f, 1f), Tooltip("Alpha of a route's capsule body relative to its line.")]
        float m_BodyAlpha = 0.45f;

        [SerializeField, Tooltip("How many bodies are rebuilt per refresh, nearest to the main camera first; the other radius-aware routes keep their line.")]
        int m_BodyBudget = 32;

        #endregion

        #region Attributes

        static readonly int s_ColorID        = Shader.PropertyToID("_Color");
        static readonly int s_OccludedFadeID = Shader.PropertyToID("_OccludedFade");
        static readonly int s_OutlineID      = Shader.PropertyToID("_Outline");

        //one view per agent, kept across refreshes and disabled (never destroyed) while an agent has no path
        readonly Dictionary<Nav3DAgent, AgentView> m_Views = new Dictionary<Nav3DAgent, AgentView>();

        //the agents whose Draw path is ticked: drawn whether or not the drawer is Active
        readonly HashSet<Nav3DAgent> m_Included = new HashSet<Nav3DAgent>();

        //scratch for the retirement pass: agents whose view must go because the agent itself is gone
        readonly List<Nav3DAgent> m_Dropped = new List<Nav3DAgent>();

        //scratch for one refresh: the radius-aware agents in camera order, and the arc-length arithmetic of one route
        readonly List<Nav3DAgent> m_BodyCandidates = new List<Nav3DAgent>();
        readonly List<float>      m_Cumulative     = new List<float>();
        readonly List<Vector3>    m_Span           = new List<Vector3>();

        readonly Gradient m_Gradient = new Gradient();

        MaterialPropertyBlock m_PropertyBlock;
        Material              m_LineMaterial;
        Material              m_BodyMaterial;
        Material              m_OccludedLineMaterial;
        Material              m_OccludedBodyMaterial;
        Material[]            m_LineMaterials;
        Material[]            m_BodyMaterials;
        bool                  m_ShaderMissingLogged;
        float                 m_NextRefreshTime;

        #endregion

        #region Properties

        public static Nav3DAgentPathDrawer Instance => Singleton<Nav3DAgentPathDrawer>.Instance;

        //the drawer if one exists, never creating it: an agent leaving during teardown must not spawn a new one
        internal static Nav3DAgentPathDrawer InstanceIfResolved => Singleton<Nav3DAgentPathDrawer>.InstanceIfResolved;

        //every agent's path, or with it off only the included agents'; a drawer with nothing to draw costs one comparison per frame
        public bool Active
        {
            get => m_Active;
            set
            {
                if (m_Active == value)
                    return;

                m_Active = value;

                m_NextRefreshTime = 0f;

                if (!m_Active && m_Included.Count == 0)
                    HideAll();
            }
        }

        public float RefreshPeriod
        {
            get => m_RefreshPeriod;
            set => m_RefreshPeriod = Mathf.Max(0f, value);
        }

        public Color PathColor
        {
            get => m_PathColor;
            set => m_PathColor = value;
        }

        public float Width
        {
            get => m_Width;
            set => m_Width = Mathf.Max(0.001f, value);
        }

        public float Outline
        {
            get => m_Outline;
            set => m_Outline = Mathf.Clamp(value, 0f, 0.5f);
        }

        //how many agents carry a drawn path right now
        public int DrawnPathsCount { get; private set; }

        //how many of them carry a drawn capsule body
        public int DrawnBodiesCount { get; private set; }

        #endregion

        #region Unity methods

        void LateUpdate()
        {
            if (!m_Active && m_Included.Count == 0)
                return;

            if (Time.unscaledTime < m_NextRefreshTime)
                return;

            m_NextRefreshTime = Time.unscaledTime + m_RefreshPeriod;

            Refresh();
        }

        void OnDestroy()
        {
            if (m_LineMaterial != null)
                Destroy(m_LineMaterial);

            if (m_BodyMaterial != null)
                Destroy(m_BodyMaterial);

            if (m_OccludedLineMaterial != null)
                Destroy(m_OccludedLineMaterial);

            if (m_OccludedBodyMaterial != null)
                Destroy(m_OccludedBodyMaterial);

            foreach (AgentView view in m_Views.Values)
            {
                DestroyMesh(view.TraveledBody);
                DestroyMesh(view.RemainingBody);
            }

            Singleton<Nav3DAgentPathDrawer>.ClearInstance(this);
        }

        #endregion

        #region Internal methods

        //an agent's Draw path: its route is drawn while the drawer is inactive too
        internal void Include(Nav3DAgent _Agent)
        {
            if (m_Included.Add(_Agent))
                m_NextRefreshTime = 0f;
        }

        internal void Exclude(Nav3DAgent _Agent)
        {
            if (!m_Included.Remove(_Agent) || m_Active)
                return;

            if (m_Views.TryGetValue(_Agent, out AgentView view))
                Hide(view);

            m_NextRefreshTime = 0f;
        }

        #endregion

        #region Service methods

        void Refresh()
        {
            DrawnPathsCount  = 0;
            DrawnBodiesCount = 0;

            //never resolve a manager here: a debug view must not create the agent manager, only read a live one
            AgentManager manager = Nav3DManager.Inited && !AgentManager.Doomed ? Singleton<AgentManager>.InstanceIfResolved : null;

            if (manager == null || !EnsureMaterials())
            {
                HideAll();

                return;
            }

            m_LineMaterial.SetColor(s_ColorID, Color.white);
            m_OccludedLineMaterial.SetFloat(s_OccludedFadeID, m_OccludedFade);
            m_LineMaterial.SetFloat(s_OutlineID, m_Outline);
            m_OccludedBodyMaterial.SetFloat(s_OccludedFadeID, m_OccludedFade);
            m_BodyMaterial.SetFloat(s_OutlineID, m_Outline);
            m_OccludedLineMaterial.SetFloat(s_OutlineID, m_Outline);
            m_OccludedBodyMaterial.SetFloat(s_OutlineID, m_Outline);

            RetireDeadViews();

            foreach (AgentView view in m_Views.Values)
                Hide(view);

            m_BodyCandidates.Clear();

            foreach (Nav3DAgentBase agentBase in manager.RegisteredAgents)
            {
                //evaders and shells are agent bodies without a route of their own — nothing to draw for them
                if (agentBase is not Nav3DAgent agent || !m_Active && !m_Included.Contains(agent))
                    continue;

                Nav3DPath path       = agent.CurrentNav3DPath;
                Vector3[] trajectory = path?.Trajectory;

                if (trajectory == null || trajectory.Length < 2)
                    continue;

                AgentView view  = EnsureView(agent);
                Color     color = StateColor(path);

                PolylineArc.Measure(trajectory, m_Cumulative);

                float traveled = PolylineArc.ClosestArcLength(trajectory, m_Cumulative, agent.transform.position);

                FillLine(view.Line, trajectory, traveled / Mathf.Max(m_Cumulative[m_Cumulative.Count - 1], Mathf.Epsilon), color);

                DrawnPathsCount++;

                if (path.Radius > 0f)
                    m_BodyCandidates.Add(agent);
            }

            DrawBodies();
        }

        //the bodies of the radius-aware routes, nearest to the main camera first, as many as the budget allows
        void DrawBodies()
        {
            if (m_BodyCandidates.Count == 0)
                return;

            Camera camera = Camera.main;

            if (camera != null && m_BodyCandidates.Count > m_BodyBudget)
            {
                Vector3 eye = camera.transform.position;

                m_BodyCandidates.Sort((_A, _B) => (_A.transform.position - eye).sqrMagnitude.CompareTo((_B.transform.position - eye).sqrMagnitude));
            }

            int count = Mathf.Min(m_BodyCandidates.Count, Mathf.Max(0, m_BodyBudget));

            for (int i = 0; i < count; i++)
            {
                Nav3DAgent agent      = m_BodyCandidates[i];
                Nav3DPath  path       = agent.CurrentNav3DPath;
                Vector3[]  trajectory = path?.Trajectory;

                if (trajectory == null || trajectory.Length < 2)
                    continue;

                AgentView view  = m_Views[agent];
                Color     color = StateColor(path);

                PolylineArc.Measure(trajectory, m_Cumulative);

                float total    = m_Cumulative[m_Cumulative.Count - 1];
                float traveled = PolylineArc.ClosestArcLength(trajectory, m_Cumulative, agent.transform.position);

                //the capsule is closed at the route's start and its end; where the two halves meet they share one open ring, so
                //the body reads as a single capsule with the agent sitting in the seam
                bool traveledDrawn = traveled > MIN_DRAWN_LENGTH;

                FillBody(view.TraveledBody, trajectory, 0f, traveled, path.Radius, true, false, color, m_TraveledFade);
                FillBody(view.RemainingBody, trajectory, traveled, total, path.Radius, !traveledDrawn, true, color, 1f);

                DrawnBodiesCount++;
            }
        }

        //the line along the whole route, its colour stepping from the dimmed flown part to the full part at the agent
        void FillLine(LineRenderer _Line, Vector3[] _Trajectory, float _TraveledRatio, Color _Color)
        {
            _Line.positionCount = _Trajectory.Length;
            _Line.SetPositions(_Trajectory);

            _Line.startWidth = m_Width;
            _Line.endWidth   = m_Width;

            Color dimmed = new Color(_Color.r, _Color.g, _Color.b, _Color.a * m_TraveledFade);
            float step   = Mathf.Clamp(_TraveledRatio, 0f, 0.999f);

            m_Gradient.SetKeys(
                    new[] { new GradientColorKey(dimmed, 0f), new GradientColorKey(dimmed, step), new GradientColorKey(_Color, step + 0.001f), new GradientColorKey(_Color, 1f) },
                    new[]
                    {
                        new GradientAlphaKey(
                                dimmed.a,
                                0f
                            ),
                        new GradientAlphaKey(
                                dimmed.a,
                                step
                            ),
                        new GradientAlphaKey(
                                _Color.a,
                                step + 0.001f
                            ),
                        new GradientAlphaKey(
                                _Color.a,
                                1f
                            )
                    }
                );

            _Line.colorGradient = m_Gradient;
            _Line.enabled       = true;
        }

        void FillBody(MeshFilter _Body, Vector3[] _Trajectory, float _From, float _To, float _Radius, bool _CapStart, bool _CapEnd, Color _Color, float _Fade)
        {
            PolylineArc.CollectSpan(_Trajectory, m_Cumulative, _From, _To, m_Span, MIN_DRAWN_LENGTH);

            if (m_Span.Count < 2)
                return;

            _Body.sharedMesh = PathCapsuleMesh.Build(m_Span, _Radius, _Body.sharedMesh, _CapStart: _CapStart, _CapEnd: _CapEnd, _Local: _Body.transform);

            MeshRenderer renderer = _Body.GetComponent<MeshRenderer>();

            m_PropertyBlock ??= new MaterialPropertyBlock();

            m_PropertyBlock.SetColor(s_ColorID, new Color(_Color.r, _Color.g, _Color.b, _Color.a * m_BodyAlpha * _Fade));

            renderer.SetPropertyBlock(m_PropertyBlock);

            renderer.enabled = true;
        }

        Color StateColor(Nav3DPath _Path)
        {
            if (_Path.IsDegraded)
                return m_DegradedColor;

            return _Path.IsTimedOut ? m_TimedOutColor : m_PathColor;
        }

        AgentView EnsureView(Nav3DAgent _Agent)
        {
            if (m_Views.TryGetValue(_Agent, out AgentView existing) && existing.Line != null)
                return existing;

            GameObject holder = new GameObject($"{nameof(Nav3DAgentPathDrawer)}_{_Agent.gameObject.name}")
            {
                hideFlags = HideFlags.HideAndDontSave
            };

            holder.transform.SetParent(transform, false);

            LineRenderer line = holder.AddComponent<LineRenderer>();

            line.useWorldSpace     = true;
            line.numCapVertices    = 2;
            line.numCornerVertices = 2;
            line.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            line.receiveShadows    = false;
            line.sharedMaterials   = m_LineMaterials;

            AgentView view = new AgentView(line, CreateBody(holder.transform, "Traveled"), CreateBody(holder.transform, "Remaining"));

            m_Views[_Agent] = view;

            return view;
        }

        //a body half: its own object under the drawer; the capsule is built in its local space, so the drawer may sit anywhere —
        //on a moving agent included
        MeshFilter CreateBody(Transform _Parent, string _Name)
        {
            GameObject bodyGo = new GameObject(_Name)
            {
                hideFlags = HideFlags.HideAndDontSave
            };

            bodyGo.transform.SetParent(_Parent, false);

            MeshFilter   filter   = bodyGo.AddComponent<MeshFilter>();
            MeshRenderer renderer = bodyGo.AddComponent<MeshRenderer>();

            renderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            renderer.receiveShadows    = false;
            renderer.sharedMaterials   = m_BodyMaterials;
            renderer.enabled           = false;

            filter.sharedMesh = new Mesh { name = $"{nameof(Nav3DAgentPathDrawer)}{_Name}", hideFlags = HideFlags.HideAndDontSave };

            return filter;
        }

        void RetireDeadViews()
        {
            foreach (KeyValuePair<Nav3DAgent, AgentView> pair in m_Views)
            {
                if (pair.Key == null)
                    m_Dropped.Add(pair.Key);
            }

            foreach (Nav3DAgent dropped in m_Dropped)
            {
                if (m_Views.TryGetValue(dropped, out AgentView view) && view.Line != null)
                {
                    DestroyMesh(view.TraveledBody);
                    DestroyMesh(view.RemainingBody);

                    Destroy(view.Line.gameObject);
                }

                m_Views.Remove(dropped);
            }

            m_Dropped.Clear();

            m_Included.RemoveWhere(_Agent => _Agent == null);
        }

        void HideAll()
        {
            DrawnPathsCount  = 0;
            DrawnBodiesCount = 0;

            foreach (AgentView view in m_Views.Values)
                Hide(view);
        }

        static void Hide(AgentView _View)
        {
            if (_View.Line == null)
                return;

            _View.Line.enabled = false;

            _View.TraveledBody.GetComponent<MeshRenderer>().enabled  = false;
            _View.RemainingBody.GetComponent<MeshRenderer>().enabled = false;
        }

        static void DestroyMesh(MeshFilter _Body)
        {
            if (_Body != null && _Body.sharedMesh != null)
                Destroy(_Body.sharedMesh);
        }

        bool EnsureMaterials()
        {
            if (m_LineMaterial != null && m_BodyMaterial != null)
                return true;

            Shader lineShader = Resources.Load<Shader>(LINE_SHADER_RESOURCE);
            Shader bodyShader = Resources.Load<Shader>(BODY_SHADER_RESOURCE);

            if (lineShader == null || bodyShader == null)
            {
                if (!m_ShaderMissingLogged)
                {
                    m_ShaderMissingLogged = true;

                    UnityEngine.Debug.LogError(
                            ($"{nameof(Nav3DAgentPathDrawer)}: shader resource '" +
                             $"{(lineShader == null ? LINE_SHADER_RESOURCE : BODY_SHADER_RESOURCE)}' is missing.")
                        );
                }

                return false;
            }

            m_LineMaterial         = new Material(lineShader) { hideFlags = HideFlags.HideAndDontSave };
            m_BodyMaterial         = new Material(bodyShader) { hideFlags = HideFlags.HideAndDontSave };
            m_OccludedLineMaterial = new Material(lineShader) { hideFlags = HideFlags.HideAndDontSave };
            m_OccludedBodyMaterial = new Material(bodyShader) { hideFlags = HideFlags.HideAndDontSave };
            m_OccludedLineMaterial.SetInt("_ZTest", (int) CompareFunction.Greater);
            m_OccludedBodyMaterial.SetInt("_ZTest", (int) CompareFunction.Greater);

            // Separate material draws work in SRPs, which select only one shader pass per material.
            m_LineMaterials = new[] { m_OccludedLineMaterial, m_LineMaterial };
            m_BodyMaterials = new[] { m_OccludedBodyMaterial, m_BodyMaterial };

            return true;
        }

        #endregion

    }
}

#endif