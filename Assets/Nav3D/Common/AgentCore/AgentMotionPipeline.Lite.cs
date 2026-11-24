#if LITE_VERSION

using UnityEngine;

namespace Nav3D.Agents
{
    public partial class AgentMotionPipeline
    {
        internal bool ExternalControl => false;

        public Vector3 GetLastFrameVelocity() => m_LastVelocity;

        float CurrentSpeedUnitsPerSecond => m_LastVelocity.magnitude / Time.fixedDeltaTime;
    }
}

#endif
