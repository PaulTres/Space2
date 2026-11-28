using Effects;
using EventBus;
using HP;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Utilities;

[RequireComponent(typeof(HPComponent))]
[RequireComponent(typeof(UnitDeathEffects))]
[RequireComponent(typeof(ShipParameters))]
public class Unit : EntityBase
{
    #region Fields
    #region Misc
    [Header("Unit")]
    public bool CriticalObjective { get; set; } = false;
    /// <summary>
    /// Fires on every tick. Mainly used to update turrets.
    /// </summary>
    protected Action onTick = delegate { };
    [Tooltip("Telemetry for missiles will be calculated from here. If not set will use the component's transform.")]
    [field: SerializeField] public Transform TelemetryOrigin { get; protected set; }
    [Tooltip("These objects will have their layer changed to better account for collisions.")]
    [SerializeField] protected List<Collider> collisionCheckers;
    [Tooltip("The list of all objects whose layer needs to be changed for taking damage on spawn. Should contain only colliders.")]
    [SerializeField] protected List<DamageReceiver> damageTakingObjects;
    [Tooltip("If true, the spawn effect will play when the unit is enabled.")]
    [field: SerializeField] public bool PlaySpawnEffect { get; set; } = true;
    #endregion

    [field: SerializeField] public ShipParameters Parameters { get; protected set; }
    protected UnitDeathEffects deathEffects;

    #region HP
    protected HPComponent hpComponent;
    public HPComponent HPComponent
    {
        get
        {
            if (hpComponent == null) hpComponent = GetComponent<HPComponent>();
            return hpComponent;
        }
    }
    public float MaxHP => HPComponent != null ? HPComponent.MaxHP : 0;
    public float CurrentHP => HPComponent.CurrentHP;
    public float CurrentHPPercentage => HPComponent.CurrentHPPercentage;
    #endregion

    #region Helpers
    public override float EffectScaleFactor => Parameters.EffectScaleFactor;
    public PoolableEffect SpawnEffect => Parameters.EffectOnSpawn;
    public UnitDeathEffects UnitDeathEffects
    {
        get
        {
            if (deathEffects == null) deathEffects = GetComponentInChildren<UnitDeathEffects>();
            return deathEffects;
        }
    }
    public float SpawnCheckRadius => Parameters.SpawnCheckRadius;
    protected WaitForSeconds wait;
    protected Coroutine tickCoroutine;
    protected Collider[] missileBuffer;
    protected int missileCount = 0;
    protected bool dying;
    public override bool IsDying => dying;
    #endregion
    #endregion

    #region Setup/Cleanup
    protected virtual void Awake()
    {
        if (TelemetryOrigin == null) TelemetryOrigin = Transform;
        wait = new(Parameters.TickInterval);
        ComponentRegister<Unit>.Register(Transform.root, this);
        EventBus<ObjectDestroyed>.AddActions(Extensions.GetTransformInstanceID(this), Die);
        MaxVelocityMagnitude *= GlobalConfig.GetUnitSpeedModifier(Team);
        var signatureModifier = GlobalConfig.GetSignatureModifier(Team);
    }

    protected virtual void OnEnable()
    {
        dying = false;
        missileBuffer = new Collider[GlobalConfig.MaxMissileCheckBufferSize];
        missileCount = 0;
        tickCoroutine = StartCoroutine(Tick());
        UnitManager.Register(this);
        if (PlaySpawnEffect && Parameters.EffectOnSpawn != null)
        {
            Parameters.EffectOnSpawn.SpawnEffect(position, EffectScaleFactor);
        }
    }

    protected virtual void OnValidate()
    {
        if (damageTakingObjects.IsEmpty()) damageTakingObjects = new(GetComponentsInChildren<DamageReceiver>());
        Debug.Assert(GlobalConfig.Testing || damageTakingObjects.Count > 0, $"{Transform} has no objects for taking damage!");
        Debug.Assert(GlobalConfig.Testing || !collisionCheckers.IsEmpty(), $"{Transform} has no objects for handling collisions!");
        if (Parameters == null)
        {
            Parameters = GetComponent<ShipParameters>();
            MaxVelocityMagnitude = Parameters.MaxVelocityMagnitude;
        }
    }

    /// <summary>
    /// Sets the layers of all important objects.
    /// </summary>
    /// <param name="layer"></param>
    public override void SetObjectLayer()
    {
        foreach (var c in damageTakingObjects) c.gameObject.layer = GlobalConfig.GetShipLayer(Team);
        foreach (var c in collisionCheckers) c.gameObject.layer = GlobalConfig.GetCollisionLayer(Team);
    }

    protected virtual void OnDisable()
    {
        if (tickCoroutine != null) StopCoroutine(tickCoroutine);
        UnitManager.Unregister(this);
        missileBuffer = null;
    }

    protected virtual void OnDestroy()
    {
        onTick = null;
        EventBus<ObjectDestroyed>.RemoveBinding(Extensions.GetTransformInstanceID(this));
        ComponentRegister<Unit>.Unregister(Transform.root);
    }
    #endregion

    #region Functionality

    #region Targeting
    /// <summary>
    /// Detect surrounding missiles, fire tick event(turrets), and decrease signature.
    /// </summary>
    /// <returns></returns>
    protected IEnumerator Tick()
    {
        while (true)
        {
            yield return wait;
            GetSurroundingMissiles();
            onTick?.Invoke();
            //DecreaseSignature(Parameters.TickInterval);
        }
    }

    protected virtual void GetSurroundingMissiles()
    {
        missileCount = Physics.OverlapSphereNonAlloc(position, Parameters.MissileTargetingRange, missileBuffer, GlobalConfig.GetEnemyBulletsCheckMask(Team));
    }

    public (int, Collider[]) GetMissiles() => (missileCount, missileBuffer);

    /// <summary>
    /// Returns a list of all units that are on a different team than this unit.
    /// </summary>
    /// <returns></returns>
    public List<EntityBase> GetTargets()
    {
        if (Team == GlobalConfig.PlayerTeam) return UnitManager.GetShips(GlobalConfig.EnemyTeam);
        else if (Team == GlobalConfig.EnemyTeam) return UnitManager.GetShips(GlobalConfig.PlayerTeam);
        Debug.LogError($"{this} is on an invalid team!");
        return new();
    }
    #endregion

    public List<EntityBase> GetAllies() => UnitManager.GetShips(Team);

    /// <summary>
    /// Fires when an object reaches 0 health.
    /// </summary>
    /// <param name="event"></param>
    public virtual void Die(ObjectDestroyed @event)
    {
        if (IsDying) return;
        dying = true;
        UnitDestroyed unitDestroyed = new(this, @event.Killer);
        GameManager.UnitsDestroyed.Add(unitDestroyed);
        EventBus<ObjectDestroyed>.RemoveActions(Extensions.GetTransformInstanceID(this), Die);
        if (tickCoroutine != null) StopCoroutine(tickCoroutine);
        UnitManager.Unregister(this);
        EventBus<UnitDestroyed>.Raise(unitDestroyed);
        UnitDeathEffects.PlayDeathEffects();
    }
    #endregion
}