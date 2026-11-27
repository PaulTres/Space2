using Timers;
using UnityEngine;
using Utilities;

namespace Weapons
{
    public class AIFireControl : FireControl
    {
        #region Fields
        [SerializeField] protected AIFireControlParameters @params;
        protected Unit unit;
        public FireControl Substitute { get; set; }
        public override Vector3 TargetPoint => Substitute ? Substitute.TargetPoint : targetPoint;
        public override EntityBase Target => Substitute ? Substitute.Target : target;
        protected CountdownTimer shotCooldownTimer;
        protected bool shouldShoot => shotCooldownTimer != null && !shotCooldownTimer.IsRunning;
        [SerializeField] protected float currentShotCooldown;
        [SerializeField] protected float readinessValue = -1;
        public float ReadinessValue
        {
            get => readinessValue;
            set
            {
                if (readinessValue != value)
                {
                    readinessValue = value;
                    currentShotCooldown = Mathf.Lerp(@params.MinShotCooldown, @params.MaxShotCooldown, @params.GetCurveValue(readinessValue));
                    //if (transform.root.name.Contains("Shadow") && transform.name.Contains("PD")) Debugging.Log($"Curve value: {@params.GetCurveValue(readinessValue)}");
                }
            }
        }
        #endregion

        #region Setup
        protected override void OnValidate()
        {
            base.OnValidate();
            if (unit == null) unit = Transform.root.GetComponent<Unit>();
        }

        protected override void OnEnable()
        {
            base.OnEnable();
            ReadinessValue = 0;
            shotCooldownTimer ??= new(@params.MaxShotCooldown);
            shotCooldownTimer.Reset(@params.MaxShotCooldown);
            if (@params.FireImmediately) shotCooldownTimer.Start();
        }
        #endregion

        #region Functionality
        protected override void PerformUpdate(float dt)
        {
            if (Substitute) return;
            FindTarget();
            if (Target)
            {
                IncreaseReadiness(dt);
                castOrigin.LookAt(Target.position);
                if (!shouldShoot) return;
                UpdateTargetPoint();
                if (Shoot())
                {
                    shotCooldownTimer.Reset(currentShotCooldown);
                    shotCooldownTimer.Start();
                }
            }
            else DecreaseReadiness(dt);
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
                var score = @params.TargetEvaluation.Evaluate(unit, target);
                if (!bestTarget.Active())
                {
                    bestTarget = target;
                    bestTargetScore = score;
                }
                else
                {
                    if (score > bestTargetScore + @params.MinUtilityDelta)
                    {
                        bestTarget = target;
                        bestTargetScore = score;
                    }
                }
            }
            target = bestTarget;
        }

        /// <summary>
        /// Increases weapon rate of fire.
        /// </summary>
        /// <param name="deltaTime"></param>
        public virtual void IncreaseReadiness(float deltaTime)
        {
            if (@params.RateOfFireRampUp > 0)
            {
                ReadinessValue = Mathf.Min(1, ReadinessValue + deltaTime * @params.RateOfFireRampUp);
            }
        }

        /// <summary>
        /// Decreases weapon rate of fire.
        /// </summary>
        /// <param name="deltaTime"></param>
        public virtual void DecreaseReadiness(float deltaTime)
        {
            if (@params.RateOfFireRampDown > 0)
            {
                ReadinessValue = Mathf.Max(0, ReadinessValue - deltaTime * @params.RateOfFireRampDown);
            }
        }
        #endregion
    }
}