using UnityEngine;

namespace Nav3D.Demo
{
    //Teleports a trailed body without a segment across the jump. TrailRenderer.Clear() re-seeds from
    //where the body stands at the call, so the only order that holds is move first, Clear() second.
    public static class DemoTrail
    {
        #region Public methods

        public static void Teleport(Transform _Body, Vector3 _Position)
        {
            _Body.position = _Position;

            TrailRenderer trail = _Body.GetComponent<TrailRenderer>();

            if (trail != null)
                trail.Clear();
        }

        #endregion
    }
}
