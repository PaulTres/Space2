using System;
using UnityEngine;

namespace Nav3D.Demo
{
    //DEMO kit — the placement ghost: the armed palette shape itself, shown where a click would drop it, in one of two
    //pre-made materials (free / blocked). The look is a material swap, never a colour composed in code.
    public sealed class DemoPlacementGhost
    {
        #region Attributes

        readonly Transform m_Parent;
        readonly Material  m_ValidMaterial;
        readonly Material  m_BlockedMaterial;

        //runs on a freshly built body before its renderers are cached: Destroy is deferred, so a renderer dropped after
        //the cache was taken stays in it and the next material swap hits a MissingReferenceException
        readonly Action<GameObject> m_Prepare;

        GameObject    m_Body;
        DemoPlaceable m_Kind;
        Renderer[]    m_Renderers;
        bool          m_Valid;

        #endregion

        #region Constructors

        public DemoPlacementGhost(Transform _Parent, Material _ValidMaterial, Material _BlockedMaterial, Action<GameObject> _Prepare = null)
        {
            m_Parent          = _Parent;
            m_ValidMaterial   = _ValidMaterial;
            m_BlockedMaterial = _BlockedMaterial;
            m_Prepare         = _Prepare;
        }

        #endregion

        #region Public methods

        public void Show(DemoPlaceable _Kind, Vector3 _Point, bool _Valid)
        {
            if (m_Body == null || m_Kind != _Kind)
                Build(_Kind, _Valid);

            m_Body.SetActive(true);

            m_Body.transform.position = _Point;

            if (_Valid == m_Valid)
                return;

            m_Valid = _Valid;

            ApplyMaterial();
        }

        public void Hide()
        {
            if (m_Body != null)
                m_Body.SetActive(false);
        }

        #endregion

        #region Service methods

        void Build(DemoPlaceable _Kind, bool _Valid)
        {
            if (m_Body != null)
                UnityEngine.Object.Destroy(m_Body);

            m_Body      = UnityEngine.Object.Instantiate(_Kind.gameObject, m_Parent);
            m_Body.name = $"Ghost_{_Kind.Label}";
            m_Kind      = _Kind;

            m_Prepare?.Invoke(m_Body);

            m_Renderers = m_Body.GetComponentsInChildren<Renderer>();
            m_Valid     = _Valid;

            ApplyMaterial();
        }

        void ApplyMaterial()
        {
            foreach (Renderer renderer in m_Renderers)
                renderer.sharedMaterial = m_Valid ? m_ValidMaterial : m_BlockedMaterial;
        }

        #endregion
    }
}
