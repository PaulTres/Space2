using EventBus;
using Level;
using UnityEngine;
using UnityEngine.InputSystem;
using Utilities;
namespace Player
{
    public class PlayerCameraControls : ImprovedMonoBehaviour, PlayerControls.ICameraControlsActions
    {
        #region Fields
        [field: SerializeField] public Transform CameraPivot { get; protected set; }
        [SerializeField] protected Transform freelookCameraPivot;
        [SerializeField] protected Transform cameraTransform;
        [Range(1, 10)]
        [SerializeField] protected float zoomSpeed = 1;
        [Range(-100, 0)]
        [SerializeField] protected float minZoom = -10;
        [Range(0, 50)]
        [SerializeField] protected float maxZoom = 5;
        public bool FreeLook { get; protected set; }
        protected Vector3 defaultCamPos;
        protected bool callbacksEnabled;
        protected float rollInput;
        #endregion

        #region Setup
        protected void OnEnable()
        {
            GameManager.PlayerCamera = cameraTransform.GetComponent<Camera>();
            defaultCamPos = cameraTransform.localPosition;
            GameManager.PlayerCamera.fieldOfView = GlobalConfig.FOV;
            FreeLook = false;
            callbacksEnabled = false;
            MissionManager.TryGetInstance(true).ExecuteAction(EnableCamera, 0.1f);
            MissionManager.TryGetInstance(true).ExecuteAction(EnableCallbacks, 1f);
            Cursor.lockState = CursorLockMode.Locked;
            Cursor.visible = false;
            GlobalUpdater.RegisterLateUpdate(PerformLateUpdate);
            ResetCamera();
            EventBus<PauseScreen.PauseEvent>.AddActions(OnPauseEvent);
        }

        protected void OnDisable()
        {
            EventBus<PauseScreen.PauseEvent>.RemoveActions(OnPauseEvent);
            GameManager.PlayerCamera = null;
            Cursor.lockState = CursorLockMode.None;
            Cursor.visible = true;
            GlobalUpdater.DeregisterLateUpdate(PerformLateUpdate);
            DisableCallbacks();
        }

        /// <summary>
        /// Delayed by .1 seconds for smoother viewing of jump effects.
        /// </summary>
        public void EnableCamera()
        {
            ResetCamera();
        }
        #endregion

        #region Functionality

        #region Controls
        public void EnableCallbacks()
        {
            GameManager.InputActions.CameraControls.AddCallbacks(this);
            callbacksEnabled = true;
        }

        protected void DisableCallbacks()
        {
            callbacksEnabled = false;
            GameManager.InputActions.CameraControls.RemoveCallbacks(this);
        }

        public void OnZoom(InputAction.CallbackContext context)
        {
            cameraTransform.localPosition = new Vector3(cameraTransform.localPosition.x, cameraTransform.localPosition.y,
               Mathf.Clamp(cameraTransform.localPosition.z + context.ReadValue<float>() * zoomSpeed, minZoom, maxZoom));
        }

        public void OnResetCamera(InputAction.CallbackContext context)
        {
            if (context.started && !FreeLook) ResetCamera();
        }

        public void OnRoll(InputAction.CallbackContext context)
        {
            rollInput = context.ReadValue<float>();
        }

        public void OnFreeLook(InputAction.CallbackContext context)
        {
            FreeLook = context.ReadValue<float>() > 0.5f;
        }

        public void OnMouseLook(InputAction.CallbackContext context) { }

        public void OnPauseEvent(PauseScreen.PauseEvent pauseEvent)
        {
            if (pauseEvent.Paused)
            {
                DisableCallbacks();
                Cursor.lockState = CursorLockMode.None;
                Cursor.visible = true;
            }
            else
            {
                Cursor.lockState = CursorLockMode.Locked;
                Cursor.visible = false;
                EnableCallbacks();
            }
        }
        #endregion

        protected void ResetCamera()
        {
            cameraTransform.localPosition = defaultCamPos;
            freelookCameraPivot.localRotation = Quaternion.identity;
        }

        //camera movement
        public void PerformLateUpdate(float deltaTime)
        {
            if (!GameManager.PlayerCamera.enabled && !callbacksEnabled) return;
            var lookDelta = GameManager.InputActions.CameraControls.MouseLook.ReadValue<Vector2>();

            var rotationInput = new Vector3(
                GlobalConfig.YMultiplier * deltaTime * lookDelta.y * GlobalConfig.XSensitivity,
                deltaTime * lookDelta.x * GlobalConfig.YSensitivity,
                deltaTime * rollInput * GlobalConfig.ZSensitivity);
            if (FreeLook)
            {
                freelookCameraPivot.Rotate(rotationInput);
            }
            else
            {
                CameraPivot.Rotate(rotationInput);
            }
        }
        #endregion
    }
}
