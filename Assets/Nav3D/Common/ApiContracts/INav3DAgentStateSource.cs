namespace Nav3D.API
{
    /// <summary>
    /// Supplies the measured state of an agent whose movement is owned by the game's own controller
    /// (<see cref="AgentControlMode.EXTERNAL"/>). Nav3D calls it on the main thread once per physics tick, before computing the
    /// agent's desired velocity, and once when an order starts; it must return the physical state of the current tick.
    /// </summary>
    public interface INav3DAgentStateSource
    {
        Nav3DAgentState ReadState();
    }
}