using AudioSystem;
using EventBus;
using UnityEngine;
using UnityEngine.InputSystem;
namespace Player
{
    public class PlayerWeapons : MonoBehaviour, PlayerControls.IWeaponsActions
    {
        [SerializeField] protected Transform weaponParent;
        protected int currentWeaponIndex;
        public event System.Action<(int oldValue, int newValue)> OnWeaponsCycled = delegate { };
        public event System.Action<int> OnTurretGroupStanceChange = delegate { };
        [SerializeField] protected SoundData errorSound;

        protected void OnEnable()
        {
            EventBus<PauseScreen.PauseEvent>.AddActions(OnPauseEvent);
            currentWeaponIndex = 0;
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
                throw new System.NotImplementedException();
                //var newValue = (currentWeaponIndex + 1) % (LauncherGroups.Count + TurretGroups.Count);
                //OnWeaponsCycled?.Invoke((currentWeaponIndex, newValue));
                //GetWeaponGroup(currentWeaponIndex)?.OnDeselected?.Invoke();
                //GetWeaponGroup(newValue)?.OnSelected?.Invoke();
                //currentWeaponIndex = newValue;
            }
        }

        public void OnExecuteAction(InputAction.CallbackContext context)
        {
            throw new System.NotImplementedException();
            //if (currentWeaponIndex < LauncherGroups.Count)
            //{
            //    if (context.canceled)
            //    {
            //        //Debug.Log("Firing all");
            //        FireAllMissiles(currentWeaponIndex);
            //    }
            //    else if (context.performed)
            //    {
            //        //Debug.Log("Tap firing");
            //        TapFireMissiles(currentWeaponIndex);
            //    }
            //}
            //else if (context.started)
            //{
            //    //this is a turret group
            //    CycleTurretStance(currentWeaponIndex - LauncherGroups.Count);
            //}
        }

        public void OnFindTarget(InputAction.CallbackContext context)
        {
            if (context.started)
            {
                throw new System.NotImplementedException();
            }
        }
    }
}
