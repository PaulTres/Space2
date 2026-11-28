using AudioSystem;
using EventBus;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;
using Weapons;
namespace Player
{
    public class PlayerWeaponControls : MonoBehaviour, PlayerControls.IWeaponsActions
    {
        protected int currentWeaponIndex;
        public event System.Action<(int oldValue, int newValue)> OnWeaponsCycled = delegate { };
        [SerializeField] protected SoundData errorSound;
        [SerializeField] protected PlayerFireControl fireControl;
        [SerializeField] protected List<AIFireControl> fireControls = new();

        protected void OnValidate()
        {
            fireControl = GetComponentInChildren<PlayerFireControl>();
            fireControls = new(GetComponentsInChildren<AIFireControl>(true));
        }

        protected void OnEnable()
        {
            EventBus<PauseScreen.PauseEvent>.AddActions(OnPauseEvent);
            currentWeaponIndex = 0;
            fireControls[currentWeaponIndex].Substitute = fireControl;
            GameManager.InputActions.Weapons.SetCallbacks(this);
        }

        protected void OnDisable()
        {
            EventBus<PauseScreen.PauseEvent>.RemoveActions(OnPauseEvent);
            GameManager.InputActions.Weapons.RemoveCallbacks(this);
        }

        public void OnPauseEvent(PauseScreen.PauseEvent pauseEvent)
        {
            if (pauseEvent.Paused) GameManager.InputActions.Weapons.RemoveCallbacks(this);
            else GameManager.InputActions.Weapons.SetCallbacks(this);
        }

        public void OnCycle(InputAction.CallbackContext context)
        {
            if (context.started)
            {
                var newValue = (currentWeaponIndex + 1) % (fireControls.Count);
                OnWeaponsCycled?.Invoke((currentWeaponIndex, newValue));
                fireControls[currentWeaponIndex].Substitute = null;
                fireControls[newValue].Substitute = fireControl;
                currentWeaponIndex = newValue;
            }
        }

        public void OnShoot(InputAction.CallbackContext context)
        {
            fireControl.Firing = context.ReadValue<float>() > 0.5f;
        }

        public void OnFindTarget(InputAction.CallbackContext context)
        {
            if (context.started) fireControl.CycleTargets();
        }
    }
}
