using Nav3D.API;

namespace Nav3D.Agents
{
    /// <summary>
    /// What an assembly that implements local avoidance offers this layer.
    /// </summary>
    partial interface IAvoidanceProvider
    {
        #region Public methods

        IAvoidanceStage CreateAgentStage(AgentMotionPipeline _Pipeline, Nav3DAgentConfig _Config);

        #endregion
    }
}
