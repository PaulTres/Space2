using UnityEngine;
namespace Weapons
{
    public abstract class TargetEvaluation : ScriptableObject
    {
        public abstract float Evaluate(Unit shooter, ImprovedMonoBehaviour target, float maxRange);
    }
}