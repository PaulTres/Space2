namespace Nav3D.API
{
    /// <summary>
    /// Who moves an agent's body: Nav3D itself, or the game's own controller acting on the velocity Nav3D advises.
    /// </summary>
    public enum AgentControlMode
    {
        //Nav3D writes the agent's position and rotation every physics tick (the default)
        NAV3D,

        //Nav3D only publishes Nav3DAgent.DesiredVelocity and reads the body's state back through a state source
        EXTERNAL
    }
}