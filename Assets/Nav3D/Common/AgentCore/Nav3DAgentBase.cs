using UnityEngine;

namespace Nav3D.Agents
{
    //inherited by every subclass, so one object cannot carry an agent, an evader and a shell at once
    [DisallowMultipleComponent]
    public abstract class Nav3DAgentBase : MonoBehaviour
    {

        #region Attributes

        protected Transform m_CachedTransform;

        #endregion

        #region Properties

        /// <summary>
        /// The avoidance body this component drives — an agent's pipeline, an evader's or a shell's mover.
        /// Null until Nav3D has initialized it. The pursuit task layer resolves a FollowTarget's transform
        /// to its body through this to publish the strike exclusion.
        /// </summary>
        internal virtual Nav3D.Common.IMovable Movable => null;

        #endregion

        #region Public methods

        //runs for every body before any body steps, so a body whose movement is owned by external code publishes one physical
        //instant for the whole tick
        internal virtual void DoBeforeFixedUpdate()
        {
        }

        internal abstract void DoFixedUpdate();

        //runs for every body after all of them have stepped, so a handler raised here cannot affect another body's step
        internal virtual void DoAfterFixedUpdate()
        {
        }

        #endregion

    }
}
