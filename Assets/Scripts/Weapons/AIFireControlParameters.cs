using AI;
using UnityEngine;

namespace Weapons
{
    [CreateAssetMenu(menuName = "AI/Fire control @params", fileName = "AI fire control")]
    public class AIFireControlParameters : ScriptableObject
    {
        [field: SerializeField] public TargetEvaluation TargetEvaluation { get; protected set; }
        [field: SerializeField] public float MinUtilityDelta { get; protected set; } = 0;
        [Tooltip("If true, the AI fire control's cooldown will kick in after the first shot taken.")]
        [field: SerializeField] public bool FireImmediately { get; protected set; } = true;
        [Tooltip("Maximum cooldown between shots. The shot cooldown starts at this value.")]
        [field: SerializeField] public float MaxShotCooldown { get; protected set; } = 5;
        [Tooltip("How quickly the rate of fire of this Weapons increases when continuously firing.")]
        [field: SerializeField] public float RateOfFireRampUp { get; protected set; } = 0.1f;
        [Tooltip("How quickly the rate of fire of this Weapons decreases when not continuously firing.")]
        [field: SerializeField] public float RateOfFireRampDown { get; protected set; } = 1f;
        [Tooltip("Minimum shot cooldown.")]
        [field: SerializeField] public float MinShotCooldown { get; protected set; } = 1;
        [SerializeField]
        protected AnimationCurve readinessCurve = new(
                new Keyframe(0f, 1f),
                new Keyframe(1f, 0f)
            );
        [Tooltip("Editor only. Will reset the value of the curve to a downward slope.")]
        [SerializeField] protected bool resetCurve;

        protected virtual void OnValidate()
        {
            readinessCurve ??= new AnimationCurve(
                new Keyframe(0f, 1f),
                new Keyframe(1f, 0f)
            );
            if (resetCurve)
            {
                resetCurve = false;
                readinessCurve = new AnimationCurve(
                    new Keyframe(0f, 1f),
                    new Keyframe(1f, 0f)
                );
            }
            MinShotCooldown = Mathf.Min(MinShotCooldown, MaxShotCooldown);
        }

        public float GetCurveValue(float value) => Mathf.Clamp01(readinessCurve.Evaluate(value));

    }
}