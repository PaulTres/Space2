using Nav3D.API;
using System;
using UnityEngine;

namespace Nav3D.Demo
{
    //DEMO kit — a shape the player drops into a scene from a palette, and what it does once dropped: placing one is attaching
    //the obstacle component, which is the operation the scene shows; a placed shape re-adds itself on its own cadence while it
    //is dragged or on rails, wears the selection tint, and takes its obstacle with it when removed. A new shape is a new prefab
    //in the palette list.
    public class DemoPlaceable : MonoBehaviour
    {
        #region Serialized fields

        [Tooltip("The palette button's caption, and the name the HUD reports")]
        [SerializeField] string m_Label = "Shape";

        [Tooltip("Appended to the placement toast; empty for none")]
        [SerializeField] string m_PlacedNote;

        [Tooltip("A placed shape re-adds its obstacle at most this often while it is dragged or on rails")]
        [SerializeField] float m_AutoUpdateMinPeriod = 0.35f;

        //the selection is an emission tint over the shape's own materials: a swap cannot express a highlight over several
        [SerializeField] Color m_SelectedEmission          = new Color(0.35f, 0.8f, 1f);
        [SerializeField] float m_SelectedEmissionIntensity = 0.5f;

        #endregion

        #region Attributes

        //the prefab's own materials, renderer by renderer: what a deselection puts back
        Renderer[] m_Renderers;
        Material[] m_AuthoredMaterials;

        #endregion

        #region Properties

        public string Label => m_Label;

        public string PlacedNote => m_PlacedNote;

        //null until the shape is placed
        public Nav3DObstacleRuntime Obstacle { get; private set; }

        #endregion

        #region Public methods

        //placing is attaching the obstacle component: the octree registers the shape from this moment
        public void Place()
        {
            Obstacle = AttachObstacle();
        }

        //an instance clone of each material while selected — the asset itself is never mutated — and the authored material back
        //on deselection, the instance destroyed with it, or every cycle leaks one
        public void SetSelected(bool _Selected)
        {
            if (m_Renderers == null)
            {
                m_Renderers         = GetComponentsInChildren<Renderer>();
                m_AuthoredMaterials = new Material[m_Renderers.Length];

                for (int i = 0; i < m_Renderers.Length; i++)
                    m_AuthoredMaterials[i] = m_Renderers[i].sharedMaterial;
            }

            for (int i = 0; i < m_Renderers.Length; i++)
            {
                Renderer renderer = m_Renderers[i];

                if (renderer == null)
                    continue;

                if (_Selected)
                {
                    renderer.material.EnableKeyword("_EMISSION");
                    renderer.material.SetColor("_EmissionColor", m_SelectedEmission * m_SelectedEmissionIntensity);

                    continue;
                }

                Material instanced = renderer.sharedMaterial;

                renderer.sharedMaterial = m_AuthoredMaterials[i];

                if (instanced != m_AuthoredMaterials[i])
                    Destroy(instanced);
            }
        }

        //destroying the shape removes its obstacle components, which is the octree's remove operation; _OnRemoved fires when it is out
        public void Remove(Action _OnRemoved)
        {
            if (Obstacle != null && _OnRemoved != null)
                Obstacle.OnObstacleRemoved += _OnRemoved;

            Destroy(gameObject);
        }

        #endregion

        #region Service methods

        //the component the shape is registered through; a shape with parts of its own attaches them first
        protected virtual Nav3DObstacleRuntime AttachObstacle()
        {
            Nav3DObstacleRuntime obstacle = gameObject.AddComponent<Nav3DObstacleRuntime>();

            //placed shapes may later be dragged or put on rails: transform changes re-add
            obstacle.AutoUpdateIfChanged = true;
            obstacle.AutoUpdateMinPeriod = m_AutoUpdateMinPeriod;

            return obstacle;
        }

        #endregion
    }
}
