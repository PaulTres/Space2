using UnityEngine;
using Utilities;

public abstract class EntityBase : ImprovedMonoBehaviour, IRegisterableComponent
{
    /// <summary>
    /// This is used for quick removal from the unit manager lists. Unused by bullets. 
    /// DO NOT FUCK WITH THIS!
    /// </summary>
    public int UnitManagerIndex { get; set; } = -1;
    [field: SerializeField] public int Team { get; set; } = 1;
    [field: SerializeField] public float MaxVelocityMagnitude { get; protected set; } = 20f;
    public abstract float EffectScaleFactor { get; }
    public virtual bool IsDying => false;
    [field: SerializeField] public bool Untargetable { get; protected set; }

    public abstract void SetObjectLayer();
}
