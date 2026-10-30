using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.UI;
namespace Player
{
    public class WeaponDisplay : MonoBehaviour
    {
        [Tooltip("This will be enabled when the weapon is selected.")]
        public Image SelectionIcon;
        public TextMeshProUGUI weaponsName;
        public Image TurretStanceIcon;
        public Image HeatStatusImage;
        public RectTransform ReloadIconsParent;
        public TextMeshProUGUI AmmoText;
        protected RectTransform rectTransform;
        [SerializeField] protected Color emptyColor = Color.white, reloadedColor = Color.orange;
        [SerializeField] protected List<Image> reloadIcons;

        public RectTransform RectTransform
        {
            get
            {
                if (rectTransform == null) rectTransform = GetComponent<RectTransform>();
                return rectTransform;
            }
        }

        public void UpdateReloadIcon(Image image, float value)
        {
            image.fillAmount = value;
            image.color = Color.Lerp(emptyColor, reloadedColor, value);
        }
    }
}
