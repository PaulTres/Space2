using UnityEngine;
using Utilities;
namespace AI
{
    [CreateAssetMenu(fileName = "TargetEvaluation", menuName = "ScriptableObjects/BasicTargetEvaluation")]
    public class BasicTargetEvaluation : TargetEvaluation
    {
        [SerializeField] protected float maxRange = 100;

        public override float Evaluate(Unit shooter, EntityBase target)
        {
            if (!target.Active())
            {
                Debug.LogError($"{this} received invalid target from {shooter}.");
                return float.MinValue;
            }
            var dir = target.Transform.position - shooter.Transform.position;
            return Mathf.Max(maxRange - dir.sqrMagnitude, 0) / maxRange;
        }
    }
}