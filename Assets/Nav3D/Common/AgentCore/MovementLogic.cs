using System;
using UnityEngine;
using Nav3D.API;
using Nav3D.Common;

namespace Nav3D.Agents
{
    public abstract class MovementLogic : IDisposable
    {

        #region Attributes

        protected          Nav3DPath          m_Path;
        protected readonly PathfindingRequest m_Request;
        readonly           Transform          m_AgentTransform;

        protected readonly float m_SqrTargetReachDistance;

        readonly           Action<PathfindingResult> m_OnPathfindingFail;
        protected readonly AgentMotionPipeline       m_Mover;

        protected readonly Action<PathfindingResult> m_OnPathUpdated;

        protected readonly Log m_Log;

        //a follow-target re-plan delivers one failure twice (the one-shot callback and the event)
        PathfindingResult m_LastReportedFailure;

        #endregion

        #region Properties

        public bool IsValid { get; private set; }

        public Vector3[] CurrentPath => m_Mover?.CurrentPath;

        internal Nav3DPath Path => m_Mover?.Path;

        internal GameObject AgentObject => m_AgentTransform != null ? m_AgentTransform.gameObject : null;

        #endregion

        #region Constructors

        protected MovementLogic(
                Transform                 _AgentTransform,
                AgentMotionPipeline       _Mover,
                PathfindingRequest        _Request,
                float                     _ReachDistance,
                Action<PathfindingResult> _OnPathUpdated,
                Action<PathfindingResult> _OnPathfindingFail,
                Log                       _Log
            )
        {
            m_Log = _Log;

            m_Request           = _Request;
            m_AgentTransform    = _AgentTransform;
            m_Mover             = _Mover;
            m_OnPathUpdated     = _OnPathUpdated;
            m_OnPathfindingFail = _OnPathfindingFail;

            m_SqrTargetReachDistance = _ReachDistance * _ReachDistance;

            //an externally controlled body's position is the state source's sample, never the (interpolated) transform
            if (!m_Mover.ExternalControl)
                m_Mover.SetCurrentPosition(_AgentTransform.position);
        }

        #endregion

        #region Public methods

        public virtual void Init(Nav3DPath _Path)
        {
            IsValid = true;
        }

        public virtual void DoOnFrameStart()
        {
        }

        public virtual void DoOnFrameEnd()
        {
            if (!m_Mover.ExternalControl)
                m_Mover.SetCurrentPosition(m_AgentTransform.position);

            if (!IsValid)
                return;

            if (m_Mover.IsDeviatedFromPathLastFrame())
            {
                DoOnDeviatedFromPath();

                return;
            }

            if (CheckTargetReached())
            {
                DoOnTargetReached();
            }
        }

        public abstract void DoOnDeviatedFromPath();
        public abstract void DoOnTargetReached();

        public abstract Vector3 GetFrameVelocity();

        public virtual void Dispose()
        {
            Invalidate();
            m_Mover.DisposePath();
        }

        #endregion

        #region Service methods

        protected void DoOnPathfindingFailed(PathfindingResult _Result)
        {
            if (_Result.Result == PathfindingResultCode.CANCELLED)
                return;

            Invalidate();

            if (m_OnPathfindingFail != null)
            {
                m_OnPathfindingFail.Invoke(_Result);
            }
            else if (!ReferenceEquals(_Result, m_LastReportedFailure))
            {
                m_LastReportedFailure = _Result;

                Nav3DPath.ReportUnreachableTarget(m_Request, _Result, AgentObject);
            }
        }

        protected abstract bool CheckTargetReached();

        protected void Invalidate()
        {
            IsValid = false;
        }

        #endregion

    }
}