using UnityEngine;

namespace Nav3D.Demo
{
    //Keeps a world-space label facing the camera.
    public class FaceCameraLabel : MonoBehaviour
    {
        #region Unity events

        void LateUpdate()
        {
            if (Camera.main != null)
                transform.rotation = Camera.main.transform.rotation;
        }

        #endregion
    }
}
