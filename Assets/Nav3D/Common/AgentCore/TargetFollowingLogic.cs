using System;
using UnityEngine;
using Nav3D.API;
using Nav3D.Common;
using Nav3D.Obstacles;

namespace Nav3D.Agents
{
    /// <summary>
    /// The FollowTarget task layer: the target's velocity is
    /// estimated every tick and paths are planned to a predicted intercept point; within range with clear line of
    /// sight the logic stops re-planning and aims the pipeline directly at the live predicted point; continuous
    /// follow holds distance with a proportional arrive law.
    /// </summary>
    public class TargetFollowingLogic : MovementLogic
    {
        #region Constants

        readonly string LOG_CTOR = $"{nameof(TargetFollowingLogic)}.ctor";

        //Prediction horizon ceiling: past this the estimate is guesswork (the target can turn), and a
        //clamped lead still shortens the chase — an uncapped one overshoots on every course change.
        const float PREDICTION_MAX_SECONDS = 2f;

        //Per-tick EMA factor for the target-velocity estimate: ~90 % of a step change adopted within
        //~9 ticks (0.18 s at 50 Hz) — fast enough to track manoeuvres, slow enough to eat jitter.
        const float TARGET_VELOCITY_SMOOTHING = 0.25f;

        //A per-tick target displacement beyond this is a teleport, not motion: reset the velocity
        //estimate instead of predicting along an impossible jump.
        const float TELEPORT_DISTANCE = 2f;

        //Line-of-sight re-test cadence — the test costs one octree segment query, mirroring the
        //pipeline's blocked-carrot throttle (AgentMotionPipeline.CARROT_CHECK_PERIOD_TICKS rationale).
        const int LOS_CHECK_PERIOD_TICKS = 3;

        //Direct pursuit engages within cruise-speed × this window (floored below): close enough that a
        //re-planned path contributes nothing but staleness, short enough that the segment query stays
        //cheap and a long approach still rides a proper path around geometry.
        const float DIRECT_PURSUIT_SECONDS = 0.75f;
        const float DIRECT_RANGE_MIN       = 3f;

        //Exit hysteresis so the mode does not flap at the range boundary.
        const float DIRECT_EXIT_RANGE_FACTOR = 1.25f;

        //The arrive ramp width in body radii (floored by the reach distance): full cruise outside
        //reach + ramp, zero at reach — the proportional law both brakes the approach and resumes it as
        //the target pulls away, with no reached/not-reached state to flip.
        const float ARRIVE_RAMP_RADII = 1.5f;

        #endregion

        #region Attributes

        readonly Transform m_Target;

        readonly Action m_OnReach;
        readonly Action m_OnFollowingFailed;

        readonly float m_SqrOffsetUpdate;
        readonly float m_ReachDistance;

        readonly bool m_FollowContinuously;

        //the strike switch (FollowTarget's _AvoidTarget = false): the pursuit is meant to reach its target
        //(a kamikaze, a missile), so the target's own avoidance body is excluded from this agent's local
        //avoidance and the exclusion is published on the pipeline for the target to read
        //. The body is resolved from the target's Nav3DAgentBase component and
        //re-asserted every tick, because a scene-placed shell's mover exists only once Nav3D has initialized.
        readonly bool          m_AvoidTarget;
        readonly Nav3DAgentBase m_TargetBody;

        //the goal the current path was planned toward (a predicted intercept point at plan time)
        Vector3 m_PlannedGoal;

        //this tick's live target position (sampled once in DoOnFrameStart) and the velocity estimate
        Vector3 m_LastTargetSample;
        Vector3 m_TargetVelocity;

        //terminal direct-pursuit state
        bool m_DirectPursuit;
        bool m_LosClear;
        int  m_LosCheckCountdown;

        #endregion

        #region Constructors

        public TargetFollowingLogic(
                Transform                 _AgentTransform,
                AgentMotionPipeline       _Mover,
                PathfindingRequest        _Request,
                Transform                 _Target,
                float                     _TargetOffsetUpdate,
                float                     _ReachDistance,
                bool                      _FollowContinuously,
                bool                      _AvoidTarget,
                Action                    _OnReach,
                Action<PathfindingResult> _OnPathUpdated,
                Action                    _OnFollowingFailed,
                Action<PathfindingResult> _OnPathfindingFail,
                Log                       _Log = null
            ) :
            base(_AgentTransform, _Mover, _Request, _ReachDistance, _OnPathUpdated, _OnPathfindingFail, _Log)
        {
            m_Target            = _Target;
            m_OnReach           = _OnReach;
            m_OnFollowingFailed = _OnFollowingFailed;

            m_SqrOffsetUpdate    = _TargetOffsetUpdate * _TargetOffsetUpdate;
            m_ReachDistance      = _ReachDistance;
            m_FollowContinuously = _FollowContinuously;
            m_AvoidTarget        = _AvoidTarget;

            //a target without a Nav3D body (a plain transform) has nothing to exclude — the switch is inert
            m_TargetBody = _AvoidTarget || _Target == null ? null : ResolveTargetBody(_Target);

            m_Log?.Write(LOG_CTOR);
        }

        #endregion

        #region Public methods

        //the body of A target transform (the strike switch's second bug): the body a pursuit is told
        //to strike is the Nav3D component that REPRESENTS the target transform — on the transform itself, on an
        //ancestor (the order names a hardpoint of an agent), or on a descendant (a rig carries its shell on a
        //child: Skyrift's biplane does, and the first cut read the root alone, found nothing, published no
        //exclusion, and every kamikaze was held by ORCA at the radius sum outside its detonation range — the
        //very hold the switch exists to remove). Self and ancestors first, then the subtree; null = plain.
        public static Nav3DAgentBase ResolveTargetBody(Transform _Target)
        {
            if (_Target == null)
                return null;

            Nav3DAgentBase body = _Target.GetComponentInParent<Nav3DAgentBase>();

            return body != null ? body : _Target.GetComponentInChildren<Nav3DAgentBase>();
        }

        public override void Init(Nav3DPath _Path)
        {
            m_Path             = _Path;
            m_PlannedGoal      = m_Target.position;
            m_LastTargetSample = m_Target.position;
            m_TargetVelocity   = Vector3.zero;

            m_Mover.BeginMoveTo(m_Request, m_Path, m_OnPathUpdated, DoOnPathfindingFailed);

            base.Init(_Path);
        }

        public override void DoOnFrameStart()
        {
            if (!IsValid)
                return;

            if (m_Target == null || !m_Target.gameObject.activeInHierarchy)
            {
                Invalidate();
                m_Mover.SetStrikeTarget(null);
                m_OnFollowingFailed?.Invoke();
                return;
            }

            //the strike exclusion is re-asserted per tick: the target's body may appear after the order
            //started (a shell initializes with Nav3D) and may be swapped by a Radius/config change
            if (!m_AvoidTarget && m_TargetBody != null)
                m_Mover.SetStrikeTarget(m_TargetBody.Movable);

            Vector3 targetPosition = m_Target.position;

            UpdateTargetVelocity(targetPosition);
            UpdateDirectPursuit(targetPosition);

            //while homing directly there is nothing to keep fresh — re-planning is exactly the churn
            //the direct mode exists to remove
            if (m_DirectPursuit)
                return;

            Vector3 predictedGoal = PredictInterceptPoint(targetPosition);

            if ((m_PlannedGoal - predictedGoal).sqrMagnitude > m_SqrOffsetUpdate)
                ReplanTo(predictedGoal);
        }

        public override Vector3 GetFrameVelocity()
        {
            if (!IsValid)
                return Vector3.zero;

            if (m_DirectPursuit)
            {
                //continuous follow holds distance with the arrive law; a one-shot follow flies the
                //approach at whatever its motion mode commands (the reach callback ends it)
                float speedCap = m_FollowContinuously ? ArriveSpeedCap() : -1f;

                m_Mover.SetDirectAimForTick(PredictInterceptPoint(m_LastTargetSample), speedCap);
            }

            return m_Mover.GetFrameVelocity();
        }

        //the order ends: the exclusion ends with it — whatever the agent does next avoids everyone again
        public override void Dispose()
        {
            m_Mover.SetStrikeTarget(null);

            base.Dispose();
        }

        public override void DoOnTargetReached()
        {
            //continuous follow has no terminal state at all: the arrive law slows the approach into the
            //reach distance and speeds it up again as the target pulls away
            if (m_FollowContinuously)
                return;

            m_Mover.DisposePath();
            Dispose();

            m_OnReach?.Invoke();
        }

        public override void DoOnDeviatedFromPath()
        {
            ReplanTo(PredictInterceptPoint(m_LastTargetSample));
        }

        #endregion

        #region Service methods

        //reach is a fact about the live target, not about the point the current path was planned toward
        //(judging against the stale plan goal froze pursuers short of an escaping target)
        protected override bool CheckTargetReached()
        {
            return (m_Mover.CurrentPosition - m_LastTargetSample).sqrMagnitude < m_SqrTargetReachDistance;
        }

        void ReplanTo(Vector3 _Goal)
        {
            m_PlannedGoal = _Goal;

            m_Mover.MoveToUsingExistingPath(
                    m_Request with { Points = new[] { m_Mover.CurrentPosition, _Goal } },
                    m_OnPathUpdated,
                    DoOnPathfindingFailed
                );
        }

        void UpdateTargetVelocity(Vector3 _TargetPosition)
        {
            Vector3 step = _TargetPosition - m_LastTargetSample;

            if (step.magnitude > TELEPORT_DISTANCE)
            {
                //a teleport invalidates both the estimate and the current line of sight
                m_TargetVelocity    = Vector3.zero;
                m_LosCheckCountdown = 0;
            }
            else
            {
                m_TargetVelocity = Vector3.Lerp(m_TargetVelocity, step / Time.fixedDeltaTime, TARGET_VELOCITY_SMOOTHING);
            }

            m_LastTargetSample = _TargetPosition;
        }

        void UpdateDirectPursuit(Vector3 _TargetPosition)
        {
            float distance = (m_Mover.CurrentPosition - _TargetPosition).magnitude;
            float range    = Mathf.Max(DIRECT_RANGE_MIN, m_Mover.CruiseSpeed * DIRECT_PURSUIT_SECONDS);

            if (--m_LosCheckCountdown <= 0)
            {
                //REFUSED means a publish holds the octree or is queued for it; the check is retried on the next tick
                //instead of blocking this one, and m_LosClear keeps the verdict it already had — which is exactly the
                //staleness the LOS_CHECK_PERIOD_TICKS cadence tolerates anyway.
                m_LosCheckCountdown = TryUpdateLineOfSight(_TargetPosition) ? LOS_CHECK_PERIOD_TICKS : 1;
            }

            if (m_DirectPursuit)
            {
                if (distance <= range * DIRECT_EXIT_RANGE_FACTOR && m_LosClear)
                    return;

                //back to path mode with a fresh route — the one the direct leg made stale
                m_DirectPursuit = false;

                ReplanTo(PredictInterceptPoint(_TargetPosition));
            }
            else if (distance <= range && m_LosClear)
            {
                m_DirectPursuit = true;
            }
        }

        //Writes m_LosClear and reports whether the octree answered at all. False = refused (a publish is in flight),
        //m_LosClear untouched.
        bool TryUpdateLineOfSight(Vector3 _TargetPosition)
        {
            ObstacleManager manager = ObstacleManager.Instance;

            if (manager == null)
            {
                m_LosClear = false;

                return true;
            }

            if (!manager.TrySegmentIntersectOccupiedLeaf(new Segment3(m_Mover.CurrentPosition, _TargetPosition), out bool intersects))
                return false;

            m_LosClear = !intersects;

            return true;
        }

        //Closed-form intercept: the t solving |Δp + v_target·t| = cruise·t (smaller positive root via the
        //numerically stable form), clamped to the prediction ceiling; tail-chase fallback when no
        //intercept exists (target as fast or diverging).
        Vector3 PredictInterceptPoint(Vector3 _TargetPosition)
        {
            float cruise = Mathf.Max(m_Mover.CruiseSpeed, 0.01f);

            Vector3 delta       = _TargetPosition - m_Mover.CurrentPosition;
            float   sqrDistance = delta.sqrMagnitude;

            float a = m_TargetVelocity.sqrMagnitude - cruise * cruise;
            float b = 2f * Vector3.Dot(delta, m_TargetVelocity);

            float interceptTime;

            if (Mathf.Abs(a) < 1e-4f)
            {
                interceptTime = b < -1e-4f ? -sqrDistance / b : Mathf.Sqrt(sqrDistance) / cruise;
            }
            else
            {
                float discriminant = b * b - 4f * a * sqrDistance;

                if (discriminant < 0f || -b + Mathf.Sqrt(discriminant) < 1e-4f)
                    interceptTime = Mathf.Sqrt(sqrDistance) / cruise;
                else
                    interceptTime = 2f * sqrDistance / (-b + Mathf.Sqrt(discriminant));
            }

            interceptTime = Mathf.Clamp(interceptTime, 0f, PREDICTION_MAX_SECONDS);

            return _TargetPosition + m_TargetVelocity * interceptTime;
        }

        //v = cruise · clamp01((d − reach) / ramp): zero at the reach distance, full cruise one ramp
        //width beyond it — the trailing distance self-stabilizes where the capped speed matches the
        //target's own
        float ArriveSpeedCap()
        {
            float distance  = (m_Mover.CurrentPosition - m_LastTargetSample).magnitude;
            float rampWidth = Mathf.Max(m_ReachDistance, m_Mover.BodyRadius * ARRIVE_RAMP_RADII);

            return m_Mover.CruiseSpeed * Mathf.Clamp01((distance - m_ReachDistance) / rampWidth);
        }

        #endregion
    }
}
