using Effects;
using EventBus;
using HP;
using UnityEngine;
using Utilities;

[RequireComponent(typeof(HPComponent))]
public class DestructableComponent : EntityBase
{
    public override Vector3 LinearVelocity => OwningUnit.LinearVelocity;
    protected Unit unit;
    public Unit OwningUnit
    {
        get
        {
            if (unit == null) unit = Transform.root.GetComponent<Unit>();
            return unit;
        }
    }
    [Tooltip("This object will be enabled when this WeaponStation is killed.")]
    [SerializeField] protected GameObject remains;
    [SerializeField] protected PoolableEffect effectOnDestruction;
    [Tooltip("When the object is destroyed the owning unit will receive this much damage.")]
    [SerializeField] protected float damageOnDestroy;
    [SerializeField] protected float effectScaleFactor = 1;
    public override float EffectScaleFactor => effectScaleFactor;
    protected HPComponent hpComponent;

    protected void OnEnable()
    {
        if (remains != null) remains.SetActive(false);
        Team = OwningUnit.Team;
        UnitManager.Register(this);
        EventBus<ObjectDestroyed>.AddActions(this.GetTransformInstanceID(), Die);
    }

    protected void OnValidate()
    {
        if (remains != null) remains.SetActive(false);
    }

    public override void SetObjectLayer()
    {
        gameObject.layer = GlobalConfig.GetShipLayer(OwningUnit.Team);
    }

    protected void OnDisable()
    {
        EventBus<ObjectDestroyed>.RemoveActions(this.GetTransformInstanceID(), Die);
        UnitManager.Unregister(this);
    }

    public void Die(ObjectDestroyed @event)
    {
        //Debug.Log($"{Transform} is dead.");
        UnitManager.Unregister(this);
        if (remains != null) remains.SetActive(true);
        effectOnDestruction.SpawnEffect(position, effectScaleFactor);
        var damageInfo = new DamageInfo
        {
            position = position,
            Source = @event.Killer,
            IgnoreArmor = true,
            Amount = damageOnDestroy
        };
        Transform.root.TakeDamage(damageInfo);
        gameObject.SetActive(false);
    }

    protected void OnDrawGizmos()
    {
        Gizmos.color = Color.red;
        Gizmos.DrawWireSphere(transform.position, EffectScaleFactor);
    }
}
