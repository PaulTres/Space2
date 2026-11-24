#if LITE_VERSION

using System;
using UnityEngine;

namespace Nav3D.API
{
    public partial class Nav3DAgent
    {
        //The shared contract preserves source compatibility; External Control is unavailable here.
        public Vector3 DesiredVelocity => Vector3.zero;
        public Vector3 SteeringTarget => m_CachedTransform.position;
        public event Action<Vector3> OnDesiredVelocityUpdated
        {
            add { }
            remove { }
        }

        public void SetStateSource(INav3DAgentStateSource _Source) { }

        bool ExternalControl => false;
        Vector3 BodyPosition => m_CachedTransform.position;
        Vector3 SampledPosition => m_CachedTransform.position;
    }
}

#endif
