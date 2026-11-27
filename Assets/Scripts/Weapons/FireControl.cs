using System.Collections.Generic;
using UnityEngine;
using Utilities;

namespace Weapons
{

    public abstract class FireControl : ImprovedMonoBehaviour
    {
        protected Vector3 targetPoint;
        /// <summary>
        /// All weapons should aim at this point when firing. It is updated every frame to reflect the current target point based on the weapon's forward direction and any obstacles in the way.
        /// </summary>
        public virtual Vector3 TargetPoint => targetPoint;
        [SerializeField] protected LayerMask targetMask;
        [SerializeField] protected float maxDistance = 100f;
        [SerializeField] protected float checkRadius = .3f;
        [SerializeField] protected List<WeaponBase> weapons = new();
        protected EntityBase target;
        public virtual EntityBase Target => target;
        [SerializeField] protected Transform castOrigin;

        protected virtual void OnValidate()
        {
            if (weapons.IsEmpty())
            {
                weapons = new(GetComponentsInChildren<WeaponBase>());
            }
        }

        protected virtual void OnEnable()
        {
            if (!castOrigin)
            {
                Debug.LogWarning($"{this} has no set cast origin, using transform!");
                castOrigin = Transform;
            }
            GlobalUpdater.RegisterFixedUpdate(PerformUpdate);
        }

        protected virtual void OnDisable()
        {
            GlobalUpdater.DeregisterFixedUpdate(PerformUpdate);
        }

        protected abstract void PerformUpdate(float _);

        public bool Shoot()
        {
            bool result = false;
            for (int i = 0; i < weapons.Count; i++)
            {
                result |= weapons[i].Shoot(TargetPoint, Target);
            }
            return result;
        }

        protected abstract void FindTarget();

        protected void UpdateTargetPoint()
        {
            if (Physics.SphereCast(castOrigin.position, checkRadius, castOrigin.forward, out RaycastHit hitInfo, maxDistance, targetMask))
            {
                targetPoint = hitInfo.point;
            }
            else
            {
                targetPoint = castOrigin.position + castOrigin.forward * maxDistance;
            }
        }
    }
}