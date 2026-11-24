using UnityEngine;

namespace Nav3D.Demo
{
    //DEMO kit — the root component of a spawnable prefab: what a scene instantiates at runtime. It knows the body's hue, read
    //off the material it wears, and swaps a state material onto every renderer and the trail.
    public class DemoUnit : MonoBehaviour
    {
        #region Attributes

        Color m_Hue;
        bool  m_HueKnown;

        #endregion

        #region Properties

        //readable on the prefab asset as well as on an instance
        public Color Hue
        {
            get
            {
                if (!m_HueKnown)
                {
                    MeshRenderer renderer = GetComponentInChildren<MeshRenderer>(true);

                    m_Hue      = renderer != null && renderer.sharedMaterial != null ? renderer.sharedMaterial.color : Color.white;
                    m_HueKnown = true;
                }

                return m_Hue;
            }
        }

        #endregion

        #region Public methods

        //a state swap — never a colour composed in code
        public void Retint(Material _Material)
        {
            foreach (MeshRenderer renderer in GetComponentsInChildren<MeshRenderer>(true))
                renderer.sharedMaterial = _Material;

            m_HueKnown = false;

            TrailRenderer trail = GetComponent<TrailRenderer>();

            if (trail == null)
                return;

            Color hue = _Material.color;

            trail.startColor = new Color(hue.r, hue.g, hue.b, trail.startColor.a);
            trail.endColor   = new Color(hue.r, hue.g, hue.b, 0f);
        }

        #endregion
    }
}
