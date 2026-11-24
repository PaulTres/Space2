using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.SceneManagement;

#if ENABLE_INPUT_SYSTEM && !ENABLE_LEGACY_INPUT_MANAGER && NAV3D_INPUT_SYSTEM

using UnityEngine.InputSystem;
using UnityEngine.InputSystem.Controls;
using UnityEngine.InputSystem.UI;

#endif

namespace Nav3D.Demo
{
    //The demo suite's one door to the keyboard and mouse: the Input Manager where the project keeps it (Unity's
    //default up to 2022.3, and "Both"), the Input System package where it is the only handler (the Unity 6 templates).
    //Values are in the Input Manager's units either way, so a scene's sensitivities read the same under both.
    public static class DemoInput
    {

        #region Constants

        //the Input Manager's default sensitivity of its "Mouse X" / "Mouse Y" / "Mouse ScrollWheel" axes
        const float MOUSE_AXIS_SENSITIVITY = 0.1f;

        #endregion

        #if ENABLE_INPUT_SYSTEM && !ENABLE_LEGACY_INPUT_MANAGER && NAV3D_INPUT_SYSTEM

        #region Properties

        public static Vector3 MousePosition => Mouse.current != null ? (Vector3) Mouse.current.position.ReadValue() : Vector3.zero;

        public static Vector2 MouseScrollDelta => Mouse.current != null ? Mouse.current.scroll.ReadValue() : Vector2.zero;

        public static Vector2 MouseDelta => Mouse.current != null ? Mouse.current.delta.ReadValue() * MOUSE_AXIS_SENSITIVITY : Vector2.zero;

        public static float ScrollWheel => MouseScrollDelta.y * MOUSE_AXIS_SENSITIVITY;

        public static bool AnyKeyDown => Keyboard.current != null && Keyboard.current.anyKey.wasPressedThisFrame;

        #endregion

        #region Public methods

        public static bool GetKey(KeyCode _Key)
        {
            KeyControl control = ControlOf(_Key);

            return control != null && control.isPressed;
        }

        public static bool GetKeyDown(KeyCode _Key)
        {
            KeyControl control = ControlOf(_Key);

            return control != null && control.wasPressedThisFrame;
        }

        public static bool GetMouseButton(int _Button)
        {
            ButtonControl control = MouseButton(_Button);

            return control != null && control.isPressed;
        }

        public static bool GetMouseButtonDown(int _Button)
        {
            ButtonControl control = MouseButton(_Button);

            return control != null && control.wasPressedThisFrame;
        }

        public static bool GetMouseButtonUp(int _Button)
        {
            ButtonControl control = MouseButton(_Button);

            return control != null && control.wasReleasedThisFrame;
        }

        #endregion

        #region Service methods

        //The scenes are authored with the Input Manager's StandaloneInputModule on their EventSystem; where the Input
        //Manager is off that module reads nothing, so the HUD takes the Input System's module instead.
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        static void ReplaceInputModules()
        {
            ReplaceInputModules(SceneManager.GetActiveScene());

            SceneManager.sceneLoaded += (_Scene, _Mode) => ReplaceInputModules(_Scene);
        }

        static void ReplaceInputModules(Scene _Scene)
        {
            foreach (GameObject root in _Scene.GetRootGameObjects())
            {
                foreach (StandaloneInputModule module in root.GetComponentsInChildren<StandaloneInputModule>(true))
                {
                    GameObject holder = module.gameObject;

                    Object.Destroy(module);

                    holder.AddComponent<InputSystemUIInputModule>();
                }
            }
        }

        static ButtonControl MouseButton(int _Button)
        {
            if (Mouse.current == null)
                return null;

            switch (_Button)
            {
                case 0:  return Mouse.current.leftButton;
                case 1:  return Mouse.current.rightButton;
                case 2:  return Mouse.current.middleButton;
                default: return null;
            }
        }

        static KeyControl ControlOf(KeyCode _Key)
        {
            if (Keyboard.current == null || _Key == KeyCode.None)
                return null;

            if (_Key >= KeyCode.A && _Key <= KeyCode.Z)
                return Keyboard.current[Key.A + (_Key - KeyCode.A)];

            if (_Key >= KeyCode.Alpha0 && _Key <= KeyCode.Alpha9)
                return Keyboard.current[Key.Digit0 + (_Key - KeyCode.Alpha0)];

            if (_Key >= KeyCode.F1 && _Key <= KeyCode.F12)
                return Keyboard.current[Key.F1 + (_Key - KeyCode.F1)];

            switch (_Key)
            {
                case KeyCode.Space:        return Keyboard.current.spaceKey;
                case KeyCode.Return:       return Keyboard.current.enterKey;
                case KeyCode.Tab:          return Keyboard.current.tabKey;
                case KeyCode.Backspace:    return Keyboard.current.backspaceKey;
                case KeyCode.Delete:       return Keyboard.current.deleteKey;
                case KeyCode.Escape:       return Keyboard.current.escapeKey;
                case KeyCode.LeftShift:    return Keyboard.current.leftShiftKey;
                case KeyCode.RightShift:   return Keyboard.current.rightShiftKey;
                case KeyCode.LeftControl:  return Keyboard.current.leftCtrlKey;
                case KeyCode.RightControl: return Keyboard.current.rightCtrlKey;
                case KeyCode.LeftAlt:      return Keyboard.current.leftAltKey;
                case KeyCode.RightAlt:     return Keyboard.current.rightAltKey;
                case KeyCode.LeftCommand:  return Keyboard.current.leftCommandKey;
                case KeyCode.RightCommand: return Keyboard.current.rightCommandKey;
                case KeyCode.UpArrow:      return Keyboard.current.upArrowKey;
                case KeyCode.DownArrow:    return Keyboard.current.downArrowKey;
                case KeyCode.LeftArrow:    return Keyboard.current.leftArrowKey;
                case KeyCode.RightArrow:   return Keyboard.current.rightArrowKey;
                default:                   return null;
            }
        }

        #endregion

        #else

        #region Properties

        public static Vector3 MousePosition => Input.mousePosition;

        public static Vector2 MouseScrollDelta => Input.mouseScrollDelta;

        public static Vector2 MouseDelta => new Vector2(Input.GetAxis("Mouse X"), Input.GetAxis("Mouse Y"));

        public static float ScrollWheel => Input.GetAxis("Mouse ScrollWheel");

        public static bool AnyKeyDown => Input.anyKeyDown;

        #endregion

        #region Public methods

        public static bool GetKey(KeyCode _Key) => Input.GetKey(_Key);

        public static bool GetKeyDown(KeyCode _Key) => Input.GetKeyDown(_Key);

        public static bool GetMouseButton(int _Button) => Input.GetMouseButton(_Button);

        public static bool GetMouseButtonDown(int _Button) => Input.GetMouseButtonDown(_Button);

        public static bool GetMouseButtonUp(int _Button) => Input.GetMouseButtonUp(_Button);

        #endregion

        #endif

    }
}