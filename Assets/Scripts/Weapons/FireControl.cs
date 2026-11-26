using System.Collections.Generic;
using UnityEngine;
using Utilities;

namespace Weapons
{

    public abstract class FireControl : ImprovedMonoBehaviour
    {
        /// <summary>
        /// All weapons should aim at this point when firing. It is updated every frame to reflect the current target point based on the weapon's forward direction and any obstacles in the way.
        /// </summary>
        public Vector3 TargetPoint { get; protected set; }
        [SerializeField] protected LayerMask targetMask;
        [SerializeField] protected float maxDistance = 100f;
        [SerializeField] protected float checkRadius = .3f;
        [SerializeField] protected List<WeaponBase> weapons = new();
        protected EntityBase target;
        public virtual EntityBase Target => target;

        protected virtual void OnValidate()
        {
            if (weapons.IsEmpty())
            {
                weapons = new(GetComponentsInChildren<WeaponBase>());
            }
        }

        protected void OnEnable()
        {
            GlobalUpdater.RegisterFixedUpdate(PerformUpdate);
        }

        protected void OnDisable()
        {
            GlobalUpdater.DeregisterFixedUpdate(PerformUpdate);
        }

        protected virtual void PerformUpdate(float _)
        {
            UpdateTargetPoint();
        }

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
            if (Physics.SphereCast(Transform.position, checkRadius, Transform.forward, out RaycastHit hitInfo, maxDistance, targetMask))
            {
                TargetPoint = hitInfo.point;
            }
            else
            {
                TargetPoint = Transform.position + Transform.forward * maxDistance;
            }
        }
    }
}