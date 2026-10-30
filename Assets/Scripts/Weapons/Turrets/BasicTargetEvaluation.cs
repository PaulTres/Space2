using UnityEngine;
namespace Weapons
{
    [CreateAssetMenu(fileName = "TargetEvaluation", menuName = "Weapons/Turret/TargetEvaluation")]
    public class BasicTargetEvaluation : TargetEvaluation
    {
        public override float Evaluate(Unit shooter, ImprovedMonoBehaviour target, float maxRange)
        {
            var dir = target.Transform.position - shooter.Transform.position;
            return Mathf.Max(maxRange - dir.sqrMagnitude, 0) / maxRange;
        }
    }
}