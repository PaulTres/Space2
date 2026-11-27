using UnityEngine;
namespace AI
{
    public abstract class TargetEvaluation : ScriptableObject
    {
        public abstract float Evaluate(Unit shooter, EntityBase target);
    }
}