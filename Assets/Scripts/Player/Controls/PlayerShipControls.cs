using EventBus;
using Level;
using UnityEngine;
using UnityEngine.InputSystem;
using Utilities;
namespace Player
{
    [RequireComponent(typeof(PlayerCameraControls))]
    public class PlayerShipControls : Ship, PlayerControls.IShipControlsActions
    {
        #region Fields
        [Header("Player controls")]
        [SerializeField] protected Transform shipBody;
        [Header("UI")]
        [SerializeField] protected GameObject missionFailureScreenPrefab;
        protected Vector3 movementVector;
        public override Vector3 forward => shipBody.forward;
        public override Vector3 right => shipBody.right;
        public override Vector3 up => shipBody.up;
        public bool CannotDie { get; set; } = false;
        public event System.Action<bool> OnDampenersChanged;
        protected bool callbacksEnabled;
        protected PlayerCameraControls cameraControls;
        #endregion

        #region Setup
        protected override void OnValidate()
        {
            base.OnValidate();
            if (cameraControls != null) cameraControls = transform.GetOrAddComponent<PlayerCameraControls>();
        }

        protected override void OnEnable()
        {
            GameManager.PlayerDead = false;
            CannotDie = false;
            callbacksEnabled = false;
            MissionManager.TryGetInstance(true).ExecuteAction(EnableCallbacks, 1f);
            GameManager.Player = this;
            Dampeners = true;
            GlobalUpdater.RegisterFixedUpdate(PerformFixedUpdate);
            GlobalUpdater.RegisterUpdate(PerformUpdate);
            EventBus<PauseScreen.PauseEvent>.AddActions(OnPauseEvent);
            base.OnEnable();
        }

        protected override void OnDisable()
        {
            base.OnDisable();
            EventBus<PauseScreen.PauseEvent>.RemoveActions(OnPauseEvent);
            Cursor.lockState = CursorLockMode.None;
            Cursor.visible = true;
            DisableCallbacks();
            GlobalUpdater.DeregisterFixedUpdate(PerformFixedUpdate);
            GlobalUpdater.DeregisterUpdate(PerformUpdate);
            if (IsDying)
            {
                //spawn mission failure screen
                GameManager.PlayerDead = true;
                Instantiate(missionFailureScreenPrefab, transform.position, transform.rotation);
            }
            OnDampenersChanged = null;
        }
        #endregion

        #region Functionality

        #region Controls
        /// <summary>
        /// Delayed by 1 second to ensure smoother player spawning.
        /// </summary>
        public void EnableCallbacks()
        {
            GameManager.InputActions.ShipControls.AddCallbacks(this);
            callbacksEnabled = true;
        }

        protected void DisableCallbacks()
        {
            callbacksEnabled = false;
            GameManager.InputActions.ShipControls.RemoveCallbacks(this);
        }

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

        public void OnFindTarget(InputAction.CallbackContext context)
        {
            if (context.started) throw new System.NotImplementedException();
        }

        public void OnMove(InputAction.CallbackContext context)
        {
            movementVector = context.ReadValue<Vector3>();
        }

        public void OnToggleDampeners(InputAction.CallbackContext context)
        {
            if (context.started)
            {
                Dampeners = !Dampeners;
                OnDampenersChanged?.Invoke(Dampeners);
            }
        }
        #endregion

        public void PerformFixedUpdate(float deltaTime)
        {
            ApplyThrust(movementVector, deltaTime);
        }

        public void PerformUpdate(float deltaTime)
        {
            if (!cameraControls.FreeLook)
            {
                Quaternion previousRotation = shipBody.rotation;

                //rotate the ship body to match the rotation of the camera pivot
                shipBody.rotation = Quaternion.RotateTowards(shipBody.rotation, cameraControls.CameraPivot.rotation, deltaTime * RotationSpeed);

                Quaternion rotationDelta = (shipBody.rotation *
                    Quaternion.Inverse(previousRotation)).normalized;
                rotationDelta.ToAngleAxis(out float angleInDegrees, out Vector3 axis);
                if (angleInDegrees > 180f) angleInDegrees -= 360f;
                angularVelocity = axis * (angleInDegrees * Mathf.Deg2Rad) / deltaTime;
            }
        }

        public override void Die(ObjectDestroyed @event)
        {
            if (CannotDie) return;
            GameManager.Player = null;
            MissionManager.TryGetInstance(true).StopAllTimers();
            UISystem.ShowText(new(Vector3.zero, 5, "Mission failed!", Color.red));
            DisableCallbacks();
            Soundtrack.FadeSoundtrack(GlobalConfig.MissionFailureEndDuration);
            base.Die(@event);
        }

        public void MissionFailed()
        {
            dying = true;
            gameObject.SetActive(false);
        }
        #endregion
    }
}
