namespace Nav3D.API
{
    /// <summary>
    /// How an agent's scalar speed evolves along its path (the speed stage of the motion pipeline).
    /// Direction is always path-following + local avoidance regardless of mode.
    /// </summary>
    public enum AgentMotionMode
    {
        //the config Speed every tick (the default)
        CONSTANT,
        //sectioned easing: a start section ramps to cruise, a final section brakes to the target,
        //cruise in between (per-section curves; see the eased motion config parameters)
        EASED,
        //trapezoidal kinematics: Acceleration up to Speed, braking into the last target at Deceleration
        ACCELERATED,
        //flight model: thrust inertia + turn slowdown + banking (pursuit steering)
        FLIGHT
    }
}
