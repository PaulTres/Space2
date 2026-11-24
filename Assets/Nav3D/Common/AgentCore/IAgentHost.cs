namespace Nav3D.Agents
{
    /// <summary>
    /// The pipeline's minimal view of its host entity. It replaces a back-reference to the concrete Nav3DAgent, which
    /// would name an API type from this layer; the API layer maps a movable back to its agent by downcasting <see
    /// cref="AgentMotionPipeline.Host"/>.
    /// </summary>
    public interface IAgentHost
    {
        //whether the host currently executes an order (its movement logic is alive) — drives the
        //pipeline's NeedToBeAvoided
        bool Operates { get; }
    }
}
