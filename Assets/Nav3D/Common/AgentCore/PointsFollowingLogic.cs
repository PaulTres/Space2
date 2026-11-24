using System;
using UnityEngine;
using Nav3D.API;
using Nav3D.Common;

namespace Nav3D.Agents
{
    public class PointsFollowingLogic : MovementLogic
    {
        #region Constants

        readonly string LOG_CTOR              = $"{nameof(PointsFollowingLogic)}.ctor";
        readonly string LOG_ON_TARGET_REACHED = $"{nameof(PointsFollowingLogic)}.{nameof(DoOnTargetReached)}, Loop:{{0}}";

        #endregion

        #region Attributes
        
        readonly Action<Vector3> m_OnPointPassed;
        readonly Action<int>     m_OnPointIndexPassed;
        readonly Action          m_OnFinished;
        
        int  m_LastPassedIndex = -1;
        bool m_IsLastPointReached;
        bool m_IsAnyTargetPassed; //Except the first one because movement take start from it

        #endregion

        #region Constructors

        public PointsFollowingLogic(
                Transform                 _AgentTransform,
                AgentMotionPipeline       _Mover,
                PathfindingRequest        _Request,
                float                     _ReachDistance,
                Action<Vector3>           _OnPointPassed,
                Action                    _OnFinished,
                Action<PathfindingResult> _OnPathUpdated,
                Action<PathfindingResult> _OnPathfindingFail,
                Log                       _Log               = null,
                Action<int>               _OnPointIndexPassed = null
            ) : base(_AgentTransform, _Mover, _Request, _ReachDistance, _OnPathUpdated, _OnPathfindingFail, _Log)
        {
            m_OnPointPassed      = _OnPointPassed;
            m_OnPointIndexPassed = _OnPointIndexPassed;
            m_OnFinished         = _OnFinished;
            
            m_Log?.Write(LOG_CTOR);
        }

        #endregion

        #region Public methods

        public override void Init(Nav3DPath _Path)
        {
            m_Path              = _Path;
            m_IsAnyTargetPassed = false;

            m_Mover.BeginMoveToPoints(
                    m_Request,
                    m_Path,
                    m_OnPathUpdated,
                    OnTargetPassed,
                    LastPointReached,
                    DoOnPathfindingFailed
                );

            base.Init(_Path);
        }

        public override Vector3 GetFrameVelocity()
        {
            if (!IsValid)
                return Vector3.zero;

            return m_Mover.GetFrameVelocity();
        }

        public override void DoOnDeviatedFromPath()
        {
        }

        public override void DoOnTargetReached()
        {
            m_Log?.WriteFormat(LOG_ON_TARGET_REACHED, m_Request.Loop);

            if (m_Request.Loop)
            {
                //Keep the last pass across multi-target lap boundaries: a pending replan can adopt it again.
                if (m_Mover.HasSingleTargetSource)
                    m_LastPassedIndex = -1;
                m_Mover.ReinitPathFollowingData();

                m_IsLastPointReached = false;
            }
            else
            {
                m_Mover.DisposePath();
                Dispose();

                m_OnFinished?.Invoke();
            }
        }

        #endregion

        #region Service methods

        void OnTargetPassed(Vector3 _Target)
        {
            //read before any callback: a caller's handler may end the order, and the follow data with it
            int index = m_Mover.PassedTargetSource;

            //Drop the transient opening leg only after passing an authored waypoint. Adopting the ring while still
            //outside it can project the follower past its first waypoint, especially with an explicit closing duplicate.
            if (index >= 0 && m_Request.Loop && m_Request.Points.Length > 2 && _Target != m_Request.Points[0] && !m_IsAnyTargetPassed)
            {
                m_Mover.RestartFollowingForCurrentPath();
                m_IsAnyTargetPassed = true;
            }

            m_OnPointPassed?.Invoke(_Target);

            //An inserted point is not the caller's, and a target certified again by a re-plan's adoption is not a new pass.
            if (index < 0 || index == m_LastPassedIndex)
                return;

            m_LastPassedIndex = index;
            m_OnPointIndexPassed?.Invoke(index);
        }

        void LastPointReached()
        {
            m_IsLastPointReached = true;
        }

        protected override bool CheckTargetReached()
        {
            return m_IsLastPointReached;
        }

        #endregion
    }
}
