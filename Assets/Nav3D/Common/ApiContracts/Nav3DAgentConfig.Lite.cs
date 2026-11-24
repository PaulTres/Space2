#if LITE_VERSION

namespace Nav3D.API
{
    public partial class Nav3DAgentConfig
    {

        #region Properties

        /// <summary>
        /// External Control is available only in Nav3D Pro. This getter always returns NAV3D;
        /// the setter preserves the authored value for use after upgrading.
        /// </summary>
        public AgentControlMode ControlMode
        {
            get => AgentControlMode.NAV3D;
            set => m_ControlMode = value;
        }

        /// <summary>
        /// Whether the agent performs local avoidance (agents + static obstacles) while following
        /// its path. Replaces the removed MotionNavigationType (COMBINED = enabled, GLOBAL = disabled).
        /// Always false in Nav3D Lite (the serialized value is preserved but inert).
        /// </summary>
        public bool LocalAvoidanceEnabled
        {
            get => false;
            set => m_LocalAvoidanceEnabled = value;
        }

        /// <summary>
        /// The effective pathfinding clearance radius: Radius * PathfindingRadiusScale when
        /// radius-aware pathfinding is enabled, 0 (point pathfinding) otherwise. Always 0 in Nav3D Lite.
        /// </summary>
        public float PathfindingRadius => 0f;

        /// <summary>
        /// Reconciles radius-aware pathfinding with static-obstacle avoidance automatically (on by
        /// default). The delivered path carries the clearance radius it was measured to keep (its certificate),
        /// and the avoidance layer's static constraints stand down while the agent provably stays inside that
        /// certified corridor — so the agent flies the guaranteed line at full speed even where it grazes
        /// geometry the pathfinding already accounted for. The constraints return smoothly as the agent drifts
        /// off the path, and instantly when the world stops matching the certificate (a moved obstacle) or the
        /// path carries no certificate at all (a degraded fallback path). Turn off to hand-tune the
        /// two layers against each other via PathfindingRadiusScale and AvoidanceLookahead instead.
        /// Inert without RadiusAwarePathfinding + LocalAvoidanceEnabled + AvoidStaticObstacles.
        /// </summary>
        public bool CertifiedClearanceGate
        {
            get => false;
            set => m_CertifiedClearanceGate = value;
        }

        /// <summary>
        /// The clearance-certification cap handed to pathfinding requests: PathfindingRadius plus one
        /// body radius of margin while the certified-clearance gate can consume it, 0 (no measurement) otherwise.
        /// </summary>
        public float PathfindingCertificationCap => 0f;

        /// <summary>
        /// How the agent's scalar speed evolves along its path (direction is always
        /// path-following + local avoidance regardless of mode). Always CONSTANT in Nav3D Lite
        /// (the serialized value is preserved but inert).
        /// </summary>
        public AgentMotionMode MotionMode
        {
            get => AgentMotionMode.CONSTANT;
            set => m_MotionMode = value;
        }

        #endregion

    }
}

#endif
