using System;
using UnityEngine;

//It sits in Common rather than in AgentCore, where the rest of the movement machinery lives, because
//SpatialHashMap is constrained to it and that container is a bottom-layer type. The interface itself names
//nothing above this layer, so the move costs nothing; the namespace never changed, and it is public API — an
//outside implementer is unaffected.
namespace Nav3D.Common
{
    public interface IMovable
    {
        #region Events

        event Action<IMovable, Vector3> OnPositionChanged;

        #endregion

        #region Properties

        //Whether is need to avoid the movable.
        bool NeedToBeAvoided { get; }

        /// <summary>
        /// Whether this movable avoids <paramref name="_Other"/> in particular: whether it avoids at all, minus the
        /// one body its current pursuit was told to strike (a FollowTarget order with avoidance of the target off —
        ///). The exclusion is PUBLISHED here so the other side of the pair reads
        /// it and takes the whole manoeuvre: the pair's responsibility shares keep summing to one as a symmetric
        /// function of observed state.
        /// </summary>
        bool IsAvoiding(IMovable _Other);

        #endregion

        #region Public methods

        /// <summary>
        /// The current position of the agent after the last move (usually the same as transform.position).
        /// </summary>
        Vector3 GetPosition();

        /// <summary>
        /// Velocity vector used at las fixed update tick. Used by other movables to compute avoidance velocity.
        /// </summary>
        Vector3 GetLastFrameVelocity();

        /// <summary>
        /// Radius in world units. Used by other movables to compute avoidance velocity.
        /// </summary>
        float GetRadius();

        /// <summary>
        /// Avoidance weight in world units: how much of a shared manoeuvre this movable expects the other
        /// party to take. A pair splits one manoeuvre in inverse proportion to these weights, so the
        /// lighter-weighted side does more of the moving. Radius-derived by default
        /// (radius * Nav3DAgentConfig.AvoidancePriority); equal weights reproduce the even split exactly.
        /// </summary>
        float GetAvoidanceWeight();

        /// <summary>
        /// The maximum allowed movement speed per fixed update tick 
        /// </summary>
        float GetMaxSpeed();

        /// <summary>
        /// How far ahead this movable is willing to start avoiding, in COMBINED body WIDTHS. The horizon in
        /// time is derived from it per encounter; a pair negotiates on
        /// the smaller of the two values, which is what keeps both sides seeing each other at the same moment.
        /// </summary>
        float GetAvoidanceLookahead();
        
        float GetStaticObstaclesDangerDistance();

        void SetNeighborMovablesDirty(bool  _Dirty);
        void SetNeighborObstaclesDirty(bool _Dirty);

        #endregion
    }
}
