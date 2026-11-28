using UnityEngine;
using Utilities;

namespace Weapons
{
    public class PlayerFireControl : FireControl
    {
        [SerializeField] protected Transform currentTargetIndicator;
        public bool Firing { get; set; }

        protected override void OnEnable()
        {
            base.OnEnable();
            Firing = false;
        }

        protected override void PerformUpdate(float _)
        {
            UpdateTargetInfo();
            if (Firing)
            {
                UpdateTargetPoint();
                Shoot();
            }
        }

        public void CycleTargets() => FindTarget();

        protected override void FindTarget()
        {
            if (GameManager.Player == null || Time.timeScale == 0) return;
            var targets = GameManager.Player.GetTargets();
            EntityBase bestCandidate = null;
            float bestScore = float.MinValue;
            bool targetInFov = false;
            foreach (var target in targets)
            {
                //compute a score for all of these
                if (!target.Active() || target.Untargetable || target.IsEqual(Target)) continue;
                var score = EvaluateTarget(target, out var inFov);
                if (!inFov && targetInFov) continue;
                if (score > bestScore)
                {
                    targetInFov = inFov;
                    bestCandidate = target;
                    bestScore = score;
                }
            }
            if (!target.Active()) TargetSelected(bestCandidate);
            else TargetSelected(bestCandidate.Active() && targetInFov ? bestCandidate : target);

        }

        /// <summary>
        /// Used to evaluate a target for selection. Takes into account distance and angle.
        /// </summary>
        /// <param name="target"></param>
        /// <returns></returns>
        public float EvaluateTarget(EntityBase target, out bool inFov)
        {
            inFov = false;
            if (!GameManager.Player.Active() || !target.Active()) return float.MinValue;

            var direction = target.position - GameManager.Player.position;
            var distance = direction.magnitude;
            var angleScore = Vector3.Dot(direction.normalized, castOrigin.forward);
            inFov = angleScore > 0.1f;
            var distanceScore = (GlobalConfig.PlayerTargetMaxDistance - distance) /
                GlobalConfig.PlayerTargetMaxDistance;
            return angleScore * GlobalConfig.PlayerTargetAngleWeight + distanceScore *
                GlobalConfig.PlayerTargetDistanceWeight;
        }

        /// <summary>
        /// Fires when a new target has been selected.
        /// </summary>
        /// <param name="unit"></param>
        protected void TargetSelected(EntityBase unit)
        {
            if (unit == target) return;
            Debug.Log($"New target selected: {unit}.");
        }

        protected void UpdateTargetInfo()
        {
            if (!target.Active())
            {
                if (currentTargetIndicator != null) currentTargetIndicator.gameObject.SetActive(false);
                FindTarget();
                return;
            }

            if (currentTargetIndicator != null)
            {
                //only activate the indicator when the target is off-screen or far away
                var dirToTarget = target.position - transform.root.position;
                var distance = dirToTarget.magnitude;
                currentTargetIndicator.gameObject.SetActive(
                    distance > 200 ||
                    Vector3.Dot(castOrigin.forward, dirToTarget) < 0.25f);
                //currentTargetIndicator.gameObject.SetActive()
                currentTargetIndicator.LookAt(target.position);
            }
        }
    }
}