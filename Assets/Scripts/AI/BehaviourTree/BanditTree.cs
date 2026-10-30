using EventBus;
using HybridBT2;
using UnityEngine;
using Utilities;
public class BanditTree : BT, IRegisterableComponent
{
    public Blackboard Context => blackboard;
    [SerializeField] protected int maxThreats = 1;
    [Tooltip("Only friendly units with an escort value higher than this are eligible for escort duty. Should always be higher than own escort value.")]
    [SerializeField] protected float minEscortValue = 10;

    #region Setup
    protected override void Awake()
    {
        base.Awake();
        ComponentRegister<BanditTree>.Register(transform, this);
        EventBus<ObjectDestroyed>.AddActions(transform.root.GetInstanceID(), null, Pause);
    }

    protected override void SetupBlackboard()
    {
        Context.SetData(Blackboard.Keys.TargetPosition, Vector3.zero);
    }

    protected virtual void OnDestroy()
    {
        EventBus<ObjectDestroyed>.RemoveActions(transform.root.GetInstanceID(), null, Pause);
        ComponentRegister<BanditTree>.Unregister(transform);
    }
    #endregion

    public void SetTarget(ImprovedMonoBehaviour target) => blackboard.SetData(Blackboard.Keys.Target, target);
}
