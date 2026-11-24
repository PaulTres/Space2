namespace Nav3D.Common
{
    /// <summary>
    /// How a speed profile behaves at the ends of its travel range.
    /// </summary>
    public enum MotionEndBehavior
    {
        //finish at the end
        STOP,
        //wrap around
        LOOP,
        //reverse direction at the ends
        PING_PONG
    }
}
