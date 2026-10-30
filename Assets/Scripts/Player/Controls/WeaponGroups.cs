using UnityEngine.Events;
namespace Player
{
    public abstract class WeaponGroup
    {
        public string Name;
        public abstract float ReloadStatus { get; }
        public UnityEvent OnSelected;
        public UnityEvent OnDeselected;
    }
}
