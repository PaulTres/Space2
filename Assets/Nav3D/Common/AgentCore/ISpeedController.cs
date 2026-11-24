namespace Nav3D.Agents
{
    /// <summary>
    /// The scalar-speed stage of <see cref="AgentMotionPipeline"/>: this tick's speed along the path in units per
    /// second. Direction never comes from here; the pipeline converts the speed to a per-tick distance once. Advance
    /// is called only on ticks the agent actually moves, so a stateful controller never progresses through a skipped
    /// tick.
    /// </summary>
    public interface ISpeedController
    {
        //true when Advance consumes the remaining/total path-length arguments — lets the pipeline skip
        //computing them for modes that don't care
        bool NeedsPathContext { get; }

        /// <summary>
        /// Computes this tick's scalar speed along the path, units/second. The path context (remaining
        /// arc distance to the path end + total path arc length, both world units) is supplied only when
        /// <see cref="NeedsPathContext"/> is true, zeros otherwise.
        /// </summary>
        float Advance(float _DeltaTime, float _RemainingDistance, float _TotalPathLength);

        /// <summary>
        /// A new order began (not a re-plan within one): restarts order-scoped state (ramps, end
        /// behavior) from the given entry speed in units/second. Re-plans preserve ramp/velocity state
        /// precisely by never calling this — that is the "no re-ramp from zero mid-flight" rule.
        /// </summary>
        void OnNewOrder(float _EntrySpeed, bool _LoopedOrder);
    }
}
