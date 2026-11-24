using UnityEngine;

namespace Nav3D.Demo
{
    //DEMO kit — the aim ring: a software cursor the hand moves and the camera may not.
    //While the hand moves, the delta goes onto the ring's own last screen position (the projection would be a rubber band under
    //an aligning camera); while it rests, the world direction is kept bit for bit; a frame over STALL_FRAME_SECONDS carries no command.
    public sealed class DemoAimRing
    {

        #region Constants

        const float STALL_FRAME_SECONDS = 0.1f;

        #endregion

        #region Attributes

        Vector3? m_Direction;
        Vector2? m_ScreenPoint;

        #endregion

        #region Properties

        //null until first steered
        public Vector3? Direction => m_Direction;

        //null while the hand does not own it
        public Vector2? ScreenPoint => m_ScreenPoint;

        public bool HandMoved { get; private set; }

        #endregion

        #region Public methods

        //the screen anchor is stale once the camera moved without the ring: dropped, the world direction kept
        public void ReleaseScreen()
        {
            m_ScreenPoint = null;
            HandMoved     = false;
        }

        //the next steer starts from the sight, so a settling camera is not pulled back off it
        public void Reseat()
        {
            m_Direction   = null;
            m_ScreenPoint = null;
            HandMoved     = false;
        }

        //_SightDirection: where a ring without a direction is born, and where one the camera turned its back on returns
        public Vector2 Steer(Camera _Camera, Vector3 _SightDirection, float _ReferenceDistance, float _PixelsPerMouseUnit, float _ScreenInset)
        {
            Vector3 cameraPosition = _Camera.transform.position;
            Vector3 direction      = m_Direction ?? _SightDirection;
            Vector3 projected      = _Camera.WorldToScreenPoint(cameraPosition + direction * _ReferenceDistance);

            if (projected.z <= 0f)
            {
                direction = _SightDirection;
                projected = _Camera.WorldToScreenPoint(cameraPosition + direction * _ReferenceDistance);
            }

            Vector2 delta = Time.unscaledDeltaTime > STALL_FRAME_SECONDS
                ? Vector2.zero
                : new Vector2(DemoInput.MouseDelta.x, DemoInput.MouseDelta.y) * _PixelsPerMouseUnit;

            HandMoved = delta != Vector2.zero;

            Vector2 anchor = HandMoved && m_ScreenPoint != null ? m_ScreenPoint.Value : projected;

            Vector2 held = new Vector2(
                    Mathf.Clamp(anchor.x + delta.x, _ScreenInset, Screen.width  - _ScreenInset),
                    Mathf.Clamp(anchor.y + delta.y, _ScreenInset, Screen.height - _ScreenInset)
                );

            //re-read only when the held point is not the projection of the current direction: the hand moved it, or the edge held it
            if (held != (Vector2) projected)
                direction = _Camera.ScreenPointToRay(held).direction;

            m_Direction   = direction;
            m_ScreenPoint = held;

            return held;
        }

        //re-applied every frame: the editor and the OS release the lock on their own (focus lost, Esc)
        public static void ApplyCursorLock(bool _Locked)
        {
            Cursor.lockState = _Locked ? CursorLockMode.Locked : CursorLockMode.None;
            Cursor.visible   = !_Locked;
        }

        #endregion

    }
}
