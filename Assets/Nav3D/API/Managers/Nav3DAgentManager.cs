using Nav3D.Agents;
using Nav3D.Common;
using Nav3D.LocalAvoidance;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Nav3D.API
{
    public static class Nav3DAgentManager
    {

        #region Public methods

        /// <summary>
        /// Get agents inside the bounds.
        /// </summary>
        public static Nav3DAgent[] GetAgentsInBounds(Bounds _Bounds)
        {
            if (!Nav3DManager.Inited)
                throw new Nav3DManager.Nav3DManagerNotInitializedException();

            //the spatial-hash query returns a bucket-granular SUPERSET of the bounds, so filter to the
            //exact positions here
            return AgentManager.Instance.GetMovablesInBounds(_Bounds)
                               .Where(_Movable => _Movable is AgentMotionPipeline pipeline &&
                                                  pipeline.Host is Nav3DAgent              &&
                                                  _Bounds.Contains(_Movable.GetPosition())
                                    )
                               .Select(_Movable => (Nav3DAgent) ((AgentMotionPipeline) _Movable).Host)
                               .ToArray();
        }

        /// <summary>
        /// Get agents inside the sphere.
        /// </summary>
        public static Nav3DAgent[] GetAgentsInSphere(Vector3 _Center, float _Radius)
        {
            if (!Nav3DManager.Inited)
                throw new Nav3DManager.Nav3DManagerNotInitializedException();

            //the hash's range query already applies the exact distance filter
            List<IMovable> movables = new List<IMovable>();

            AgentManager.Instance.GetMovablesInRange(_Center, _Radius, movables);

            List<Nav3DAgent> agents = new List<Nav3DAgent>(movables.Count);

            foreach (IMovable movable in movables)
            {
                if (movable is AgentMotionPipeline pipeline && pipeline.Host is Nav3DAgent agent)
                    agents.Add(agent);
            }

            return agents.ToArray();
        }

        #endregion

    }
}