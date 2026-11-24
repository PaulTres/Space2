using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace Nav3D.Demo
{
    //DEMO kit — the polyline plot: uGUI has no line renderer, so a MaskableGraphic builds one quad per segment. Points are
    //NORMALIZED (0..1 across and up the rect), so the plot rescales with the canvas. Look authored by DemoHUD.ComposeGraph.
    [RequireComponent(typeof(CanvasRenderer))]
    public class DemoGraph : MaskableGraphic
    {
        #region Constants

        //a series is a stream: the oldest points fall off
        const int MAX_POINTS_PER_SERIES = 1024;

        #endregion

        #region Serialized fields

        //the series index is the caller's lane/mode index
        [SerializeField] Color[] m_SeriesTints = new Color[0];
        [SerializeField] float   m_LineWidth   = 2.5f;

        [SerializeField] Color m_ReferenceTint = new Color(1f, 1f, 1f, 0.35f);
        [SerializeField] float m_ReferenceWidth = 1.5f;

        #endregion

        #region Attributes

        readonly List<List<Vector2>> m_Series = new List<List<Vector2>>();

        float m_Reference = -1f;
        bool  m_Dirty;

        #endregion

        #region Properties

        public int SeriesCount => m_SeriesTints.Length;

        //0..1 draws the reference line, anything outside hides it
        public float Reference
        {
            get => m_Reference;
            set
            {
                if (Mathf.Approximately(m_Reference, value))
                    return;

                m_Reference = value;
                m_Dirty     = true;
            }
        }

        #endregion

        #region Unity events

        //rebuilt at most once a frame however many points arrived: a plot fed from FixedUpdate would rebuild several times for nothing
        void LateUpdate()
        {
            if (!m_Dirty)
                return;

            m_Dirty = false;

            SetVerticesDirty();
        }

        #endregion

        #region Public methods

        //edit-time entry point (DemoHUD.ComposeGraph)
        public void Configure(Color[] _SeriesTints, float _LineWidth, Color _ReferenceTint)
        {
            m_SeriesTints   = _SeriesTints;
            m_LineWidth     = _LineWidth;
            m_ReferenceTint = _ReferenceTint;
        }

        public void Clear(int _Series)
        {
            EnsureSeries();

            if (_Series < 0 || _Series >= m_Series.Count)
                return;

            m_Series[_Series].Clear();

            m_Dirty = true;
        }

        public void ClearAll()
        {
            EnsureSeries();

            foreach (List<Vector2> series in m_Series)
                series.Clear();

            m_Dirty = true;
        }

        //_Point is normalized: x 0 = left, 1 = right; y 0 = bottom, 1 = top
        public void Push(int _Series, Vector2 _Point)
        {
            EnsureSeries();

            if (_Series < 0 || _Series >= m_Series.Count)
                return;

            List<Vector2> series = m_Series[_Series];

            if (series.Count >= MAX_POINTS_PER_SERIES)
                series.RemoveAt(0);

            series.Add(new Vector2(Mathf.Clamp01(_Point.x), Mathf.Clamp01(_Point.y)));

            m_Dirty = true;
        }

        #endregion

        #region Service methods

        protected override void OnPopulateMesh(VertexHelper _VertexHelper)
        {
            _VertexHelper.Clear();

            EnsureSeries();

            Rect rect = GetPixelAdjustedRect();

            if (m_Reference >= 0f && m_Reference <= 1f)
            {
                float y = rect.yMin + m_Reference * rect.height;

                AddSegment(_VertexHelper, new Vector2(rect.xMin, y), new Vector2(rect.xMax, y), m_ReferenceWidth, m_ReferenceTint);
            }

            for (int i = 0; i < m_Series.Count; i++)
            {
                List<Vector2> series = m_Series[i];
                Color         tint   = i < m_SeriesTints.Length ? m_SeriesTints[i] : Color.white;

                for (int p = 1; p < series.Count; p++)
                    AddSegment(_VertexHelper, ToRect(series[p - 1], rect), ToRect(series[p], rect), m_LineWidth, tint);
            }
        }

        static Vector2 ToRect(Vector2 _Normalized, Rect _Rect)
        {
            return new Vector2(_Rect.xMin + _Normalized.x * _Rect.width, _Rect.yMin + _Normalized.y * _Rect.height);
        }

        //no joins: at these widths a quad per segment reads as a continuous line
        static void AddSegment(VertexHelper _VertexHelper, Vector2 _From, Vector2 _To, float _Width, Color _Tint)
        {
            Vector2 direction = _To - _From;

            if (direction.sqrMagnitude < 1e-6f)
                return;

            Vector2 normal = new Vector2(-direction.y, direction.x).normalized * (_Width * 0.5f);

            UIVertex vertex = UIVertex.simpleVert;

            vertex.color = _Tint;

            int index = _VertexHelper.currentVertCount;

            vertex.position = _From - normal;
            _VertexHelper.AddVert(vertex);

            vertex.position = _From + normal;
            _VertexHelper.AddVert(vertex);

            vertex.position = _To + normal;
            _VertexHelper.AddVert(vertex);

            vertex.position = _To - normal;
            _VertexHelper.AddVert(vertex);

            _VertexHelper.AddTriangle(index, index + 1, index + 2);
            _VertexHelper.AddTriangle(index + 2, index + 3, index);
        }

        void EnsureSeries()
        {
            while (m_Series.Count < m_SeriesTints.Length)
                m_Series.Add(new List<Vector2>());
        }

        #endregion
    }
}
