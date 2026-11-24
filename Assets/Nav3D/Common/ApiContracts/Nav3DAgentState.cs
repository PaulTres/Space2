using UnityEngine;

namespace Nav3D.API
{
    /// <summary>
    /// The measured state of an externally controlled agent at the current physics tick.
    /// </summary>
    public readonly struct Nav3DAgentState
    {

        #region Properties

        public Vector3 Position { get; }

        /// <summary>
        /// World-space velocity in units per second.
        /// </summary>
        public Vector3 Velocity { get; }

        #endregion

        #region Constructors

        public Nav3DAgentState(Vector3 _Position, Vector3 _Velocity)
        {
            Position = _Position;
            Velocity = _Velocity;
        }

        #endregion

    }
}