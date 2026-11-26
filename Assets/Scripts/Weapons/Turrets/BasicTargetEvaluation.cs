using UnityEngine;
using Utilities;
namespace Weapons
{
    [CreateAssetMenu(fileName = "TargetEvaluation", menuName = "Weapons/Turret/TargetEvaluation")]
    public class BasicTargetEvaluation : TargetEvaluation
    {
        [SerializeField] protected float maxRange = 100;

        public override float Evaluate(Unit shooter, ImprovedMonoBehaviour target)
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