using Effects;
using EventBus;
using HP;
using UnityEngine;
using Utilities;
namespace Weapons
{
    public class Missile : Projectile
    {
        #region Fields
        protected ImprovedMonoBehaviour target;
        public ImprovedMonoBehaviour Target => target;
        protected float tracking;
        protected HPComponent hpComponent;
        public HPComponent HPComponent
        {
            get
            {
                if (hpComponent == null) hpComponent = GetComponent<HPComponent>();
                return hpComponent;
            }
        }
        protected PoolableEffect onDeathEffect;
        protected float targetedDamage;
        /// <summary>
        /// How much damage is this targeted by. Used to prevent all turrets from 
        /// shooting at the same missile. Replaced old system to account for tanky 
        /// missiles.
        /// </summary>
        public float TargetedDamage
        {
            get => targetedDamage;
            set => targetedDamage = Mathf.Max(value, 0);
        }
        public float MaxHP
        {
            get
            {
                if (HPComponent == null) return 0;
                return HPComponent.MaxHP;
            }
        }
        #endregion

        #region Setup
        protected void Awake()
        {
            EventBus<ObjectDestroyed>.AddActions(Extensions.GetTransformInstanceID(this), Die);
        }

        protected void OnValidate()
        {
            if (HPComponent != null && GetComponent<DamageReceiver>() == null)
            {
                gameObject.AddComponent<DamageReceiver>();
                Debug.LogWarning($"Adding DamageReceiver to {this}.");
            }
        }

        protected void OnDestroy()
        {
            ComponentRegister<Missile>.Unregister(Transform);
            EventBus<ObjectDestroyed>.RemoveBinding(Extensions.GetTransformInstanceID(this));
        }

        protected override void OnDisable()
        {
            SetTarget(null);
            TargetedDamage = 0;
            base.OnDisable();
        }

        public void SetTarget(ImprovedMonoBehaviour newTarget)
        {
            target = newTarget;
        }

        public override void Initialize(BulletData poolableData)
        {
            TargetedDamage = 0;
            ComponentRegister<Missile>.Register(Transform, this);
            base.Initialize(poolableData);
            if (poolableData is not MissileData missileData)
            {
                throw new System.ArgumentException($"Expected {typeof(MissileData)}, " +
                    $"got {poolableData.GetType()}");
            }
            tracking = missileData.Tracking * GlobalConfig.GetTrackingModifier(Team);
            if (missileData.HP > 0 && HPComponent != null) HPComponent.MaxHP = missileData.HP;
            onDeathEffect = missileData.OnDeathEffect;
        }
        #endregion

        #region Functionality
        /// <summary>
        /// Calculates and applies tracking and linear velocity.
        /// </summary>
        /// <param name="dt"></param>
        protected override void RestOfPerformFixedUpdate(float dt)
        {
            if (Target.Active())
            {

                Vector3 directionToTarget = (Target.Transform.position - Transform.position).normalized;

                //Quaternion newRotation = Quaternion.RotateTowards(tr.rotation, Quaternion.l)
                Vector3 newDirection = Vector3.RotateTowards(Transform.forward, directionToTarget,
                    tracking * dt, 0);

                Quaternion prevRotation = Transform.rotation;
                Transform.rotation = Quaternion.LookRotation(newDirection);

                Quaternion rotationDelta = (transform.rotation * Quaternion.Inverse(prevRotation)).normalized;
                rotationDelta.ToAngleAxis(out float angleInDegrees, out Vector3 axis);
                if (angleInDegrees > 180f) angleInDegrees -= 360f;

                angularVelocity = axis * (angleInDegrees * Mathf.Deg2Rad) / dt;
            }
            base.RestOfPerformFixedUpdate(dt);
        }

        /// <summary>
        /// Plays an onDeathEffect if the missile has one. Deactivates the game object.
        /// </summary>
        /// <param name="event"></param>
        public void Die(ObjectDestroyed @event)
        {
            onDeathEffect.SpawnEffect(Transform.position, effectScaleFactor);
            gameObject.SetActive(false);
        }
        #endregion
    }
}