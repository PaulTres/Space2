namespace Nav3D.Common
{
    /// <summary>
    /// Drives arc-length distance along a curve for one on-rail motion behavior (constant speed, eased tween,
    /// trapezoidal acceleration, ...). Owns its own progress state (clock, direction);
    /// decoupled from any concrete curve type or MonoBehaviour so it can be reused outside the Spline3D demo
    /// toolkit it was extracted from. Call <see cref="Reset"/> before first use and whenever the run restarts.
    /// </summary>
    interface ISpeedProfile
    {
        //signed speed along +s from the most recent Advance call, world units/second
        float CurrentSpeed { get; }

        void Reset(float _StartDistance);

        //overrides the current arc-length distance without touching progress state (clock/direction);
        //mirrors what a direct external position jump would have done pre-extraction
        void SetDistance(float _Distance);
    }
}
