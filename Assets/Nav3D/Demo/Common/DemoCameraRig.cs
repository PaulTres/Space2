using UnityEngine;

namespace Nav3D.Demo
{
    //DEMO kit — the fly/orbit camera: RMB look, WASD/QE fly, Shift boost, wheel zoom unless a scene captured the wheel, idle orbit
    //after a timeout. A follow target makes it a chase camera; the opt-in extras below are off by default. Reads unscaled time:
    //the player keeps the camera while agent time is paused, but the rig starts nothing of its own (Paused).
    public class DemoCameraRig : MonoBehaviour
    {

        #region Constants

        const float IDLE_ORBIT_DELAY         = 10f;
        const float ORBIT_DEGREES_PER_SECOND = 5f;
        const float PITCH_LIMIT              = 89f;

        const float FOLLOW_ALIGN_RATE = 2.5f;

        //pitch eases faster than yaw: a mouse-steered vehicle can only aim within half a FOV of where the camera looks, so a camera
        //slow to pitch caps the climb the pilot can ask for
        const float FOLLOW_ALIGN_PITCH_RATE = 6f;

        //look-ahead: a rider hands the rig a point further along its route and the view eases toward it (aligned to the target's
        //forward, a chase camera reaches every turn late). MAX_ANGLE keeps the target in shot on a hairpin; MIN_SQR_DISTANCE drops a lead on the focus
        const float FOLLOW_LOOK_AHEAD_MAX_ANGLE      = 35f;
        const float FOLLOW_LOOK_AHEAD_MIN_SQR_LENGTH = 0.01f;

        //the orbit arm is swept so the camera stops on the near surface. radius must exceed the near plane's half-diagonal (0.6 covers
        //the default 0.3 near clip at 60° FOV) or the clip plane cuts into the rock; MIN_DISTANCE keeps a buried arm off the hull;
        //RESTORE_RATE is one-way — pulling in is instant, easing out stops a flight past a spire from pumping the camera
        const float FOLLOW_COLLISION_RADIUS       = 0.6f;
        const float FOLLOW_COLLISION_MIN_DISTANCE = 2f;
        const float FOLLOW_COLLISION_RESTORE_RATE = 30f;

        //exponential ease: a camera 300 u out and one 3 u out both arrive in about two seconds
        const float DETACH_RETURN_RATE          = 3f;
        const float DETACH_RETURN_SNAP_DISTANCE = 0.4f;
        const float DETACH_RETURN_SNAP_ANGLE    = 1.5f;

        #endregion

        #region Serialized fields

        [SerializeField] float m_MoveSpeed       = 8f;
        [SerializeField] float m_BoostFactor     = 3f;
        [SerializeField] float m_LookSensitivity = 3f;
        [SerializeField] float m_ZoomSpeed       = 10f;

        [SerializeField] bool m_IdleOrbitEnabled = false;

        //locked: the authored pose is the shot, wins over every mode. A follow target makes the rig a chase camera: RMB orbits,
        //the wheel sets the distance, WASD does nothing, the arm is swept against the world
        [SerializeField] bool m_Locked;

        [SerializeField] Transform m_FollowTarget;

        //follow-align: while RMB is not held the orbit eases in behind the target's forward on both axes
        [SerializeField] bool m_FollowAlign;

        //rear view: held, the camera snaps to the target's far side looking back; instant both ways, wins over RMB and follow-align
        [SerializeField] KeyCode m_RearViewKey = KeyCode.None;

        //how long the hand may rest after a SteerToward before follow-align takes the camera back; 0 = at once. Skyline sets
        //seconds: its ring is a designator, and the view holds where the player put it
        [SerializeField] float m_FollowRealignDelay;

        //elevation: degrees above the target's line the settled chase camera rides — one number, because pitching the resting view
        //down both lifts the camera and tilts it, so the target keeps its place in the frame; follow-align only. Detach: W/S fly the
        //camera off its arm, A/D slide it, and after every input has rested for the return delay it flies back; not swept while out
        [SerializeField] float   m_FollowElevationAngle;
        [SerializeField] bool    m_FollowDetachEnabled;
        [SerializeField] float   m_FollowReturnDelay = 4f;
        [SerializeField] float   m_DetachedMoveSpeed = 30f;
        [SerializeField] float   m_FollowDistance    = 46f;
        [SerializeField] float   m_FollowFocusHeight = 4f;
        [SerializeField] float   m_FollowMinDistance = 16f;
        [SerializeField] float   m_FollowMaxDistance = 140f;
        [SerializeField] Vector3 m_OrbitFocus        = Vector3.zero;

        #endregion

        #region Attributes

        float m_Yaw;
        float m_Pitch;
        float m_IdleTime;
        float m_FollowCollisionDistance;

        //live for one frame; consumed by UpdateFollow
        Vector2? m_SteerPoint;
        bool     m_SteerInputThisFrame;
        float    m_LastSteerInputTime = float.NegativeInfinity;
        Camera   m_Camera;

        //one frame; none = no known route, the align falls back to the target's forward
        Vector3? m_LookAheadPoint;

        bool  m_Detached;
        bool  m_Returning;
        float m_LastDetachedInputTime;

        //the arm length the scene was authored with: what a camera coming back onto the arm is given, whatever the wheel did since
        float m_AuthoredFollowDistance;

        #endregion

        #region Properties

        public bool WheelCaptured { get; set; }

        //consumers steering by the camera freeze their input for the glance, or the vehicle would chase the flipped view
        public bool RearViewHeld => m_RearViewKey != KeyCode.None && m_FollowTarget != null && DemoInput.GetKey(m_RearViewKey);

        public bool Detached => m_Detached;

        //the player keeps RMB, the wheel, the detach keys and the rear view while paused; what the rig does on its own (idle orbit,
        //follow-align, the flight back) stops, and its clocks are HELD, not merely ignored (Update)
        bool Paused => Mathf.Approximately(Time.timeScale, 0f);

        //the view belongs to the last steer request: the hand moved within the realign delay, or the camera is detached and the ring
        //is its only look control. A ring scene shows its sight marks for exactly this long
        public bool SteerHeld => m_FollowTarget != null &&
                                 ((m_Detached && !m_Returning) || Time.unscaledTime - m_LastSteerInputTime < m_FollowRealignDelay);

        //follow mode only: back to the default view now — a detached camera starts its flight back, an attached one aligns without the
        //realign delay. A ring rider must re-seat its ring in the same breath, or its next SteerToward pulls the view back off
        public void ResetView()
        {
            if (m_FollowTarget == null)
                return;

            m_SteerPoint          = null;
            m_SteerInputThisFrame = false;
            m_LastSteerInputTime  = float.NegativeInfinity;

            if (m_Detached)
            {
                BeginReturn();

                m_LastDetachedInputTime = float.NegativeInfinity;
            }
        }

        #endregion

        #region Public methods

        //follow mode only, every frame the ring is live; _InputThisFrame = the hand moved, the realign delay counts from the last one
        public void SteerToward(Vector2 _ScreenPoint, bool _InputThisFrame)
        {
            m_SteerPoint          = _ScreenPoint;
            m_SteerInputThisFrame = _InputThisFrame;

            if (_InputThisFrame)
                m_LastSteerInputTime = Time.unscaledTime;
        }

        //follow-align only, one frame: the align eases toward this point instead of the target's forward, within
        //FOLLOW_LOOK_AHEAD_MAX_ANGLE of it. Never called = plain follow-align
        public void LookAhead(Vector3 _WorldPoint)
        {
            m_LookAheadPoint = _WorldPoint;
        }

        #endregion

        #region Unity events

        void Start()
        {
            Vector3 euler = transform.rotation.eulerAngles;

            m_Yaw   = euler.y;
            m_Pitch = euler.x > 180f ? euler.x - 360f : euler.x;

            m_FollowCollisionDistance = m_FollowDistance;
            m_AuthoredFollowDistance  = m_FollowDistance;
        }

        void Update()
        {
            if (m_Locked)
                return;

            float deltaTime = Time.unscaledDeltaTime;

            if (m_FollowTarget != null)
            {
                UpdateFollow();

                return;
            }

            bool hadInput = false;

            if (DemoInput.GetMouseButton(1))
            {
                m_Yaw   += DemoInput.MouseDelta.x * m_LookSensitivity;
                m_Pitch -= DemoInput.MouseDelta.y * m_LookSensitivity;
                m_Pitch =  Mathf.Clamp(m_Pitch, -PITCH_LIMIT, PITCH_LIMIT);

                transform.rotation = Quaternion.Euler(m_Pitch, m_Yaw, 0f);

                hadInput = true;
            }

            Vector3 move = Vector3.zero;

            if (DemoInput.GetKey(KeyCode.W))
                move += transform.forward;

            if (DemoInput.GetKey(KeyCode.S))
                move -= transform.forward;

            if (DemoInput.GetKey(KeyCode.D))
                move += transform.right;

            if (DemoInput.GetKey(KeyCode.A))
                move -= transform.right;

            if (DemoInput.GetKey(KeyCode.E))
                move += Vector3.up;

            if (DemoInput.GetKey(KeyCode.Q))
                move -= Vector3.up;

            if (move != Vector3.zero)
            {
                float speed = m_MoveSpeed * (DemoInput.GetKey(KeyCode.LeftShift) ? m_BoostFactor : 1f);

                transform.position += move.normalized * (speed * deltaTime);

                hadInput = true;
            }

            if (!WheelCaptured)
            {
                float wheel = DemoInput.ScrollWheel;

                if (!Mathf.Approximately(wheel, 0f))
                {
                    transform.position += transform.forward * (wheel * m_ZoomSpeed);

                    hadInput = true;
                }
            }

            if (hadInput)
            {
                m_IdleTime = 0f;

                return;
            }

            if (Paused)
                return;

            m_IdleTime += deltaTime;

            if (!m_IdleOrbitEnabled || m_IdleTime < IDLE_ORBIT_DELAY)
                return;

            transform.RotateAround(m_OrbitFocus, Vector3.up, ORBIT_DEGREES_PER_SECOND * deltaTime);
            transform.LookAt(m_OrbitFocus);

            Vector3 orbitEuler = transform.rotation.eulerAngles;

            m_Yaw   = orbitEuler.y;
            m_Pitch = orbitEuler.x > 180f ? orbitEuler.x - 360f : orbitEuler.x;
        }

        #endregion

        #region Service methods

        void UpdateFollow()
        {
            bool    rearView  = RearViewHeld;
            bool    paused    = Paused;
            float   deltaTime = Time.unscaledDeltaTime;
            float   wheel     = WheelCaptured ? 0f : DemoInput.ScrollWheel;
            Vector3 focus     = m_FollowTarget.position + Vector3.up * m_FollowFocusHeight;

            //a pause is not the player leaving the view alone: both clocks are carried forward so the delays resume where the pause
            //found them; a "never" stamp (float.NegativeInfinity) carries
            if (paused)
            {
                m_LastSteerInputTime    += deltaTime;
                m_LastDetachedInputTime += deltaTime;
            }

            if (m_FollowDetachEnabled)
                UpdateDetach(deltaTime, wheel);

            if (rearView)
            {
                //releasing the glance restores exactly the view held before it
            }
            else if (DemoInput.GetMouseButton(1))
            {
                m_Yaw   += DemoInput.MouseDelta.x * m_LookSensitivity;
                m_Pitch -= DemoInput.MouseDelta.y * m_LookSensitivity;
                m_Pitch =  Mathf.Clamp(m_Pitch, -PITCH_LIMIT, PITCH_LIMIT);
            }
            else if (m_SteerPoint != null && ((m_Detached && !m_Returning) || Time.unscaledTime - m_LastSteerInputTime < m_FollowRealignDelay))
            {
                //the ring is world-anchored while the hand rests, so the turn ends with the ring on the sight; a detached camera keeps
                //turning after the ring for as long as it is out
                if (m_Camera == null)
                    m_Camera = GetComponent<Camera>();

                Vector3 direction   = m_Camera.ScreenPointToRay(m_SteerPoint.Value).direction;
                float   targetYaw   = Mathf.Atan2(direction.x, direction.z)          * Mathf.Rad2Deg;
                float   targetPitch = -Mathf.Asin(Mathf.Clamp(direction.y, -1f, 1f)) * Mathf.Rad2Deg;

                m_Yaw   = Mathf.LerpAngle(m_Yaw, targetYaw, FOLLOW_ALIGN_RATE           * deltaTime);
                m_Pitch = Mathf.LerpAngle(m_Pitch, targetPitch, FOLLOW_ALIGN_PITCH_RATE * deltaTime);
                m_Pitch = Mathf.Clamp(m_Pitch, -PITCH_LIMIT, PITCH_LIMIT);
            }
            else if (m_FollowAlign && !paused && (!m_Detached || m_Returning))
            {
                //both axes: aligning yaw only left the pitch wherever it was last put, and the steering ray could never leave that band.
                //A camera on its way back aligns the same way, so it arrives looking where the arm looks
                float targetYaw   = m_FollowTarget.eulerAngles.y;
                float targetPitch = m_FollowTarget.eulerAngles.x;

                if (m_LookAheadPoint != null)
                {
                    Vector3 lead = m_LookAheadPoint.Value - focus;

                    if (lead.sqrMagnitude > FOLLOW_LOOK_AHEAD_MIN_SQR_LENGTH)
                    {
                        //the lead may pull the view off the target's nose, but only so far
                        Vector3 direction = Vector3.RotateTowards(m_FollowTarget.forward, lead.normalized, FOLLOW_LOOK_AHEAD_MAX_ANGLE * Mathf.Deg2Rad, 0f);

                        targetYaw   = Mathf.Atan2(direction.x, direction.z)          * Mathf.Rad2Deg;
                        targetPitch = -Mathf.Asin(Mathf.Clamp(direction.y, -1f, 1f)) * Mathf.Rad2Deg;
                    }
                }

                //on an orbit arm one number is both the height over the target and the tilt that keeps it framed
                targetPitch += m_FollowElevationAngle;

                m_Yaw   = Mathf.LerpAngle(m_Yaw, targetYaw, FOLLOW_ALIGN_RATE           * deltaTime);
                m_Pitch = Mathf.LerpAngle(m_Pitch, targetPitch, FOLLOW_ALIGN_PITCH_RATE * deltaTime);
                m_Pitch = Mathf.Clamp(m_Pitch, -PITCH_LIMIT, PITCH_LIMIT);
            }

            //the wheel: the arm's length on the arm, a step along the view off it
            if (!Mathf.Approximately(wheel, 0f) && !m_Detached)
                m_FollowDistance = Mathf.Clamp(m_FollowDistance - wheel * m_ZoomSpeed, m_FollowMinDistance, m_FollowMaxDistance);

            //a steer request and a look-ahead point live one frame
            m_SteerPoint          = null;
            m_SteerInputThisFrame = false;
            m_LookAheadPoint      = null;

            //mirrored orbit: the camera stands on the far side and looks back through the target
            Quaternion rotation = Quaternion.Euler(m_Pitch, m_Yaw + (rearView ? 180f : 0f), 0f);
            Vector3    arm      = rotation * Vector3.back;

            if (m_Detached)
            {
                UpdateDetachedPose(rotation, focus, arm, deltaTime, paused);

                return;
            }

            transform.rotation = rotation;
            transform.position = focus + arm * ResolveArmLength(focus, arm);
        }

        //W/S fly the camera along its own forward, A/D along its right; every input resets the rest clock, and an input during
        //the flight back cancels it
        void UpdateDetach(float _DeltaTime, float _Wheel)
        {
            float axis     = (DemoInput.GetKey(KeyCode.W) ? 1f : 0f) - (DemoInput.GetKey(KeyCode.S) ? 1f : 0f);
            float strafe   = (DemoInput.GetKey(KeyCode.D) ? 1f : 0f) - (DemoInput.GetKey(KeyCode.A) ? 1f : 0f);
            bool  keyInput = !Mathf.Approximately(axis, 0f) || !Mathf.Approximately(strafe, 0f);

            if (keyInput && !m_Detached)
            {
                m_Detached  = true;
                m_Returning = false;
            }

            if (!m_Detached)
                return;

            bool anyInput = keyInput || m_SteerInputThisFrame || DemoInput.GetMouseButton(1) || !Mathf.Approximately(_Wheel, 0f);

            if (anyInput)
            {
                m_LastDetachedInputTime = Time.unscaledTime;
                m_Returning             = false;
            }
            else if (!m_Returning && Time.unscaledTime - m_LastDetachedInputTime >= m_FollowReturnDelay)
            {
                BeginReturn();
            }

            if (m_Returning)
                return;

            float speed = m_DetachedMoveSpeed * (DemoInput.GetKey(KeyCode.LeftShift) ? m_BoostFactor : 1f);

            transform.position += transform.forward * (axis * speed * _DeltaTime + _Wheel * m_ZoomSpeed) + transform.right * (strafe * speed * _DeltaTime);
        }

        //the arm goes back to its authored length: the wheel is a framing choice for a moment, and the attached camera is the
        //scene's default shot. Restored at the start of the flight, so the camera eases onto the authored arm instead of jumping at the snap
        void BeginReturn()
        {
            m_Returning      = true;
            m_FollowDistance = m_AuthoredFollowDistance;
        }

        //off the arm the rotation is the camera's own; on the way back both ease onto the arm's pose
        void UpdateDetachedPose(Quaternion _Rotation, Vector3 _Focus, Vector3 _Arm, float _DeltaTime, bool _Paused)
        {
            if (!m_Returning)
            {
                transform.rotation = _Rotation;

                return;
            }

            //a flight back that a pause caught in the air hangs there until the scene runs again
            if (_Paused)
                return;

            Vector3 wanted = _Focus + _Arm * ResolveArmLength(_Focus, _Arm);
            float   ease   = 1f     - Mathf.Exp(-DETACH_RETURN_RATE * _DeltaTime);

            transform.position = Vector3.Lerp(transform.position, wanted, ease);
            transform.rotation = Quaternion.Slerp(transform.rotation, _Rotation, ease);

            if (Vector3.Distance(transform.position, wanted)    > DETACH_RETURN_SNAP_DISTANCE ||
                Quaternion.Angle(transform.rotation, _Rotation) > DETACH_RETURN_SNAP_ANGLE)
                return;

            m_Detached  = false;
            m_Returning = false;

            transform.rotation = _Rotation;
            transform.position = wanted;
        }

        //unfiltered cast: a follow-mode demo scene has no colliders that are not solid world (agents, pickups, rings and the plane
        //are colliderless by the kit rule), so a layer mask would be a knob with one setting
        float ResolveArmLength(Vector3 _Focus, Vector3 _Arm)
        {
            float wanted = m_FollowDistance;

            //an arm that starts already buried reports distance 0 (Unity ignores the initial overlap's
            //geometry), so the floor below is what keeps the camera off the hull rather than an early out
            if (Physics.SphereCast(_Focus, FOLLOW_COLLISION_RADIUS, _Arm, out RaycastHit hit, wanted, Physics.AllLayers, QueryTriggerInteraction.Ignore))
                wanted = Mathf.Max(hit.distance, FOLLOW_COLLISION_MIN_DISTANCE);

            m_FollowCollisionDistance = wanted < m_FollowCollisionDistance
                ? wanted
                : Mathf.MoveTowards(m_FollowCollisionDistance, wanted, FOLLOW_COLLISION_RESTORE_RATE * Time.unscaledDeltaTime);

            return m_FollowCollisionDistance;
        }

        #endregion

    }
}
