using AI;
using HybridBT2;
using System;
using UnityEngine;
using Utilities;

[CreateAssetMenu(menuName = "HybridBT2/Services/TargetAcquisition", fileName = "TargetAcquisition")]
public class TargetAcquisitionService : ServiceData
{
    [Tooltip("The minimum difference in utility score between the best target and the next " +
        "best target for the best target to be selected.")]
    public float MinUtilityDelta = 0;
    public TargetEvaluation TargetEvaluation;
    [Tooltip("The score of the target of the escort candidate will be multiplied by this amount.")]
    //[SerializeField] protected float escortCandidateTargetMultiplier = 2;

    public override Action<Node, Blackboard> Action => (n, ctx) =>
    {
        ImprovedMonoBehaviour bestTarget = null;
        float bestTargetScore = float.MinValue;
        //var escortCandidate = ctx.GetData<ImprovedMonoBehaviour>(Blackboard.Keys.EscortCandidate);
        //if (escortCandidate.Active())
        //{
        //    bestTarget = escortCandidate.Target;
        //    if (bestTarget.Active()) bestTargetScore = escortCandidateTargetMultiplier * TargetEvaluation.Evaluate(ctx.Navigation, bestTarget, MaxRange);
        //}
        foreach (var target in ctx.Unit.GetTargets())
        {
            if (!target.Active()) continue;
            if (!bestTarget.Active())
            {
                bestTarget = target;
                bestTargetScore = TargetEvaluation.Evaluate(ctx.Unit, target);
            }
            else
            {
                var score = TargetEvaluation.Evaluate(ctx.Unit, target);
                if (score > bestTargetScore + MinUtilityDelta)
                {
                    bestTarget = target;
                    bestTargetScore = score;
                }
            }
        }
        ctx.SetData(Blackboard.Keys.Target, bestTarget);
    };
}
