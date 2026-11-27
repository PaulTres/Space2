using UnityEngine;
using Utilities;

namespace Weapons
{
    public class AIFireControl : FireControl
    {
        [SerializeField] protected TargetEvaluation targetEvaluation;
        [SerializeField] protected float minUtilityDelta = 0;
        protected Unit unit;
        public FireControl Substitute { get; set; }
        public override Vector3 TargetPoint => Substitute ? Substitute.TargetPoint : targetPoint;
        public override EntityBase Target => Substitute ? Substitute.Target : target;

        protected override void OnValidate()
        {
            base.OnValidate();
            if (unit == null) unit = Transform.root.GetComponent<Unit>();
        }

        protected override void PerformUpdate(float _)
        {
            if (Substitute) return;
            base.PerformUpdate(_);
            FindTarget();
            if (Target)
            {
                castOrigin.LookAt(Target.position);
                Shoot();
            }
        }

        protected override void FindTarget()
        {
            EntityBase bestTarget = null;
            float bestTargetScore = float.MinValue;
            //var escortCandidate = ctx.GetData<ImprovedMonoBehaviour>(Blackboard.Keys.EscortCandidate);
            //if (escortCandidate.Active())
            //{
            //    bestTarget = escortCandidate.Target;
            //    if (bestTarget.Active()) bestTargetScore = escortCandidateTargetMultiplier * TargetEvaluation.Evaluate(ctx.Navigation, bestTarget, MaxRange);
            //}
            foreach (var target in unit.GetTargets())
            {
                if (!target.Active()) continue;
                var score = targetEvaluation.Evaluate(unit, target);
                if (!bestTarget.Active())
                {
                    bestTarget = target;
                    bestTargetScore = score;
                }
                else
                {
                    if (score > bestTargetScore + minUtilityDelta)
                    {
                        bestTarget = target;
                        bestTargetScore = score;
                    }
                }
            }
            target = bestTarget;
        }
    }
}