using UnityEngine;
using Utilities;
namespace Player
{
    [RequireComponent(typeof(LineRenderer))]
    public class BoundingBoxDrawer : ImprovedMonoBehaviour
    {
        public Transform Viewer => throw new System.NotImplementedException();
        [SerializeField] protected Renderer objectRenderer;
        [SerializeField] protected LineRenderer lineRenderer;
        [Tooltip("Padding around the bounding box of the object.")]
        [SerializeField] protected float boxPadding = 1;

        protected void OnValidate()
        {
            if (lineRenderer == null) lineRenderer = GetComponent<LineRenderer>();
        }

        protected void OnEnable()
        {
            if (objectRenderer == null || lineRenderer == null) return;
            GlobalUpdater.RegisterUpdate(PerformUpdate);
        }

        protected void OnDisable()
        {
            GlobalUpdater.DeregisterUpdate(PerformUpdate);
        }

        protected void PerformUpdate(float _)
        {
            if (objectRenderer == null || lineRenderer == null) return;

            transform.rotation = Quaternion.LookRotation(-Viewer.forward, transform.up);
            //Debug.DrawRay(transform.position, transform.forward * 3, Color.blue);
            var topLeft = objectRenderer.bounds.ClosestPoint(objectRenderer.bounds.center - transform.right + transform.up);
            var topRight = objectRenderer.bounds.ClosestPoint(objectRenderer.bounds.center + transform.right + transform.up);
            var bottomLeft = objectRenderer.bounds.ClosestPoint(objectRenderer.bounds.center - transform.right - transform.up);
            var bottomRight = objectRenderer.bounds.ClosestPoint(objectRenderer.bounds.center + transform.right - transform.up);
            topLeft += boxPadding * (topLeft - objectRenderer.bounds.center);
            topRight += boxPadding * (topRight - objectRenderer.bounds.center);
            bottomLeft += boxPadding * (bottomLeft - objectRenderer.bounds.center);
            bottomRight += boxPadding * (bottomRight - objectRenderer.bounds.center);

            //lineRenderer.positionCount = 5;
            lineRenderer.SetPosition(0, topLeft);
            lineRenderer.SetPosition(1, topRight);
            lineRenderer.SetPosition(2, bottomRight);
            lineRenderer.SetPosition(3, bottomLeft);
            lineRenderer.SetPosition(4, topLeft);
        }
    }
}