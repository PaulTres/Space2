using Nav3D.API;
using UnityEngine;

namespace Nav3D.Demo
{
    //DEMO kit — the world cursor under the mouse: a physics hit first, else a point at an adjustable depth along the ray.
    //Safe before Nav3D init: everything is reported free until then.
    public class DemoCursor3D : MonoBehaviour
    {

        #region Constants

        const float MIN_FALLBACK_DEPTH = 1f;
        const float MAX_FALLBACK_DEPTH = 200f;

        #endregion

        #region Serialized fields

        [SerializeField] Camera m_Camera;

        [SerializeField] float m_FallbackDepth = 15f;

        //the reach of the physics ray and the ceiling of the fallback depth; a 500 m world authors its own reach, or a click on a
        //distant ridge silently becomes a depth-plane point in mid-air
        [SerializeField] float m_RayDistance = MAX_FALLBACK_DEPTH;

        #endregion

        #region Properties

        public float FallbackDepth
        {
            get => m_FallbackDepth;
            set => m_FallbackDepth = Mathf.Clamp(value, MIN_FALLBACK_DEPTH, m_RayDistance);
        }

        #endregion

        #region Public methods

        //for scenes whose click surface is a volume (a shell, a shoal band) and resolve the ray themselves
        public Ray ScreenRay
        {
            get
            {
                Camera camera = m_Camera != null ? m_Camera : Camera.main;

                return camera != null ? camera.ScreenPointToRay(DemoInput.MousePosition) : new Ray(Vector3.zero, Vector3.forward);
            }
        }

        //true for a physics hit, false for the fallback depth-plane point; the point is valid either way
        public bool TryGetPoint(out Vector3 _Point)
        {
            Camera camera = m_Camera != null ? m_Camera : Camera.main;

            if (camera == null)
            {
                _Point = Vector3.zero;

                return false;
            }

            Ray ray = camera.ScreenPointToRay(DemoInput.MousePosition);

            if (Physics.Raycast(ray, out RaycastHit hit, m_RayDistance))
            {
                _Point = hit.point;

                return true;
            }

            _Point = ray.GetPoint(m_FallbackDepth);

            return false;
        }

        public bool IsPointFree(Vector3 _Point)
        {
            return !Nav3DManager.Inited || !Nav3DManager.IsPointInsideOccupiedVolume(_Point);
        }

        public void AdjustDepth(float _Delta)
        {
            FallbackDepth = m_FallbackDepth + _Delta;
        }

        //the height where the ray passes over the column at _Column — the press-drag-release height mechanic (LMB anchors XZ,
        //holding drags the height along the column); without a camera, the column's own height
        public float ColumnHeightUnderRay(Vector3 _Column)
        {
            Camera camera = m_Camera != null ? m_Camera : Camera.main;

            if (camera == null)
                return _Column.y;

            Ray ray = camera.ScreenPointToRay(DemoInput.MousePosition);

            Vector2 rayOriginXZ    = new Vector2(ray.origin.x, ray.origin.z);
            Vector2 rayDirectionXZ = new Vector2(ray.direction.x, ray.direction.z);
            Vector2 columnXZ       = new Vector2(_Column.x, _Column.z);

            float t = rayDirectionXZ.sqrMagnitude > 1e-6f
                ? Vector2.Dot(columnXZ - rayOriginXZ, rayDirectionXZ) / rayDirectionXZ.sqrMagnitude
                : 1f;

            return ray.GetPoint(Mathf.Max(t, 0f)).y;
        }

        #endregion

    }
}