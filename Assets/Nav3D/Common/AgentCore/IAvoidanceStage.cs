using UnityEngine;

namespace Nav3D.Agents
{
    /// <summary>
    /// The local-avoidance seam of <see cref="AgentMotionPipeline"/>: takes the path-following preferred velocity and
    /// returns the velocity the agent uses this tick. Null means no local avoidance (always in Nav3D Lite). The
    /// production implementation is OrcaAvoidanceStage.
    /// </summary>
    public interface IAvoidanceStage
    {
        /// <summary>
        /// Computes this tick's velocity from the path-following preferred velocity.
        /// </summary>
        Vector3 Apply(Vector3 _VPref);

        #if UNITY_EDITOR

        //the avoidance-only velocity of the last Apply call (debug gizmo)
        Vector3 LastAvoidanceVelocity { get; }

        void DrawNearestMovers();
        void DrawNearestTriangles();
        void DrawTrianglesUpdateInfo();

        #endif
    }
}
