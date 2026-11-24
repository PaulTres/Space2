using Nav3D.Agents;
using Nav3D.API;
using System;
using System.Threading.Tasks;

namespace Nav3D.Common.AgentCore
{
    public class AgentOrder
    {
        #region Attributes

        readonly PathfindingRequest m_Request;

        Nav3DPath m_Path;

        Action<Nav3DPath>         m_OnPathfindingSuccess;
        Action<PathfindingResult> m_OnPathfindingFail;

        //set by Cancel (a newer order superseded this one) and read by the pathfinding continuation
        //on a threadpool thread — a cancelled order must deliver no callback, not a stale one
        volatile bool m_Cancelled;

        #endregion

        #region Properties

        public MovementLogic MovementLogic { get; }

        #endregion

        #region Constructors

        public AgentOrder(PathfindingRequest _Request, MovementLogic _MovementLogic)
        {
            m_Request     = _Request;
            MovementLogic = _MovementLogic;
        }

        #endregion

        #region Public methods

        public void Init(Action<Nav3DPath> _OnReady, Action<PathfindingResult> _OnPathfindingFail, Log _Log = null)
        {
            m_OnPathfindingSuccess = _OnReady;
            m_OnPathfindingFail    = _OnPathfindingFail;

            m_Path = new Nav3DPath(_Log) { ReportsToConsole = false };
            m_Path.Find(m_Request).ContinueWith(OnPathfindingFinished, TaskContinuationOptions.OnlyOnRanToCompletion);
        }

        public void Cancel()
        {
            m_Cancelled = true;

            m_Path.Dispose();
        }

        #endregion

        #region Service methods

        void OnPathfindingFinished(Task<PathfindingResult> _PathfindingTask)
        {
            //a superseded order stays silent — its owner has already issued a newer one, and a stale
            //callback (of either kind) would race it
            if (m_Cancelled)
                return;

            //the FAIL branch comes first: a failed search leaves
            //the path invalid by definition (Find resets IsValid and only DoOnSuccess restores it),
            //so the validity guard below — meant for the path-died-while-in-flight race — swallowed
            //every pathfinding failure with it: MoveTo's _OnPathfindingFail was unreachable for its
            //primary purpose, and a failed order looked like an order that never answered
            if (_PathfindingTask.Result.Failed)
            {
                if (m_OnPathfindingFail != null)
                    ThreadDispatcher.BeginInvoke(m_OnPathfindingFail, _PathfindingTask.Result);
                else
                    ThreadDispatcher.BeginInvoke(() => Nav3DPath.ReportUnreachableTarget(m_Request, _PathfindingTask.Result, MovementLogic.AgentObject));

                return;
            }

            //something invalidated the path while the pathfinding task was in flight
            if (!m_Path.IsValid)
                return;

            ThreadDispatcher.BeginInvoke(m_OnPathfindingSuccess, m_Path);
        }

        #endregion
    }
}