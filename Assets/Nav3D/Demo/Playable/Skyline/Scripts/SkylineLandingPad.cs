using UnityEngine;

namespace Nav3D.Demo
{
    //A landing pad.
    //Not an obstacle of its own (the tower beneath occupies the volume). This transform is the touchdown point; the hover
    //point above it is where a Nav3D order ends and the helicopter's own descent begins.
    public class SkylineLandingPad : MonoBehaviour
    {
        #region Constants

        #endregion

        #region Serialized fields

        [SerializeField] float m_HoverHeight = 6f;

        //free / claimed are two authored materials swapped, never a colour composed
        [SerializeField] Renderer m_Deck;
        [SerializeField] Material m_FreeMaterial;
        [SerializeField] Material m_ClaimedMaterial;

        #endregion

        #region Properties

        public Vector3 TouchdownPoint => transform.position;

        //where the Nav3D order ends: open air above the deck, free by construction
        public Vector3 HoverPoint => transform.position + Vector3.up * m_HoverHeight;

        public SkylineHelicopter Claimant { get; private set; }

        public bool IsClaimed => Claimant != null;

        #endregion

        #region Unity events

        #endregion

        #region Public methods

        //a claim overwrites any previous one; the previous claimant sees Claimant != itself and re-targets
        public void Claim(SkylineHelicopter _By)
        {
            Claimant = _By;

            ApplyState();
        }

        //only the holder may release: a stale release must not free the pad under the new claimant
        public void Release(SkylineHelicopter _By)
        {
            if (Claimant != _By)
                return;

            Claimant = null;

            ApplyState();
        }

        #endregion

        #region Service methods

        void ApplyState()
        {
            m_Deck.sharedMaterial = Claimant != null ? m_ClaimedMaterial : m_FreeMaterial;
        }

        #endregion
    }
}
