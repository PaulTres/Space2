using System;
using System.Threading;
using System.Threading.Tasks;
using Nav3D.API;
using Nav3D.Common;
using PathfindingMethod =
    System.Func<
        Nav3D.API.PathfindingRequest,
        System.Threading.CancellationToken,
        System.Threading.CancellationToken,
        System.Action<string>,
        Nav3D.Common.Log,
        System.Threading.Tasks.Task<Nav3D.API.PathfindingResult>>;

namespace Nav3D.Pathfinding
{
    class PathfindingOrder : IExecutable
    {
        #region Constants

        readonly string LOG_CTOR                 = $"{nameof(PathfindingOrder)}.ctor (ID:{{0}}): Points {{1}}, Loop: {{2}}";
        readonly string LOG_EXECUTION_STARTED    = $"{nameof(PathfindingOrder)}.{nameof(Execute)}: for points {{0}}";
        readonly string LOG_PATHFINDING_STARTED  = $"{nameof(PathfindingOrder)}: Pathfinding task started: for points {{0}}";
        readonly string LOG_PATHFINDING_FINISHED = $"{nameof(PathfindingOrder)}: Pathfinding task finished: for points {{0}}";

        #endregion

        #region Attributes

        readonly PathfindingRequest m_Request;

        readonly CancellationToken m_CancellationToken;

        readonly PathfindingMethod m_PathfindingMethod;

        string       m_Status;
        readonly Log m_Log;

        readonly TaskCompletionSource<PathfindingResult> m_TCS;

        #endregion

        #region Properties

        public Task<PathfindingResult> Task              => m_TCS.Task;
        public CancellationToken       CancellationToken => m_CancellationToken;

        #endregion

        #region Constructors

        public PathfindingOrder(PathfindingRequest _Request, CancellationToken _CancellationToken, PathfindingMethod _PathfindingMethod, Log _Log = null)
        {
            m_Request           = _Request;
            m_CancellationToken = _CancellationToken;
            m_PathfindingMethod = _PathfindingMethod;

            m_TCS = new TaskCompletionSource<PathfindingResult>(m_CancellationToken);

            (m_Log = _Log)?.WriteFormat(LOG_CTOR, GetHashCode(), UtilsCommon.GetPointsString(m_Request.Points), m_Request.Loop);
        }

        #endregion

        #region Public methods

        public string GetExecutingStatus()
        {
            return $"{m_Request.RequesterID}: {m_Status}";
        }

        public void Execute(Action _OnStart, Action _OnResolve)
        {
            _OnStart?.Invoke();

            const string STATUS_EXECUTION_CALLED         = "Execute method called";
            const string STATUS_EXECUTING_TASK_STARTED   = "Pathfinding task started";
            const string STATUS_EXECUTING_TASK_FINISHED  = "Pathfinding task finished";
            const string STATUS_EXECUTING_TASK_CANCELLED = "Pathfinding task cancelled";

            m_Status = STATUS_EXECUTION_CALLED;
            m_Log?.WriteFormat(LOG_EXECUTION_STARTED, UtilsCommon.GetPointsString(m_Request.Points));

            CancellationTokenSource timeoutCancellationTokenSource = new CancellationTokenSource();
            CancellationToken       cancellationTokenTimeout       = timeoutCancellationTokenSource.Token;

            m_Status = STATUS_EXECUTING_TASK_STARTED;

            m_Log?.WriteFormat(LOG_PATHFINDING_STARTED, UtilsCommon.GetPointsString(m_Request.Points));

            timeoutCancellationTokenSource.CancelAfter(m_Request.Timeout);

            PathfindingResult result = m_PathfindingMethod.Invoke(
                                                               m_Request,
                                                               m_CancellationToken,
                                                               cancellationTokenTimeout,
                                                               _Status => m_Status = _Status,
                                                               m_Log
                                                           )
                                                          .GetAwaiter()
                                                          .GetResult();

            if (m_CancellationToken.IsCancellationRequested)
            {
                m_TCS.SetCanceled();

                _OnResolve.Invoke();
                m_Status = STATUS_EXECUTING_TASK_CANCELLED;
            }
            else
            {
                m_TCS.SetResult(result);

                _OnResolve.Invoke();
                m_Status = STATUS_EXECUTING_TASK_FINISHED;
            }


            m_Log?.WriteFormat(LOG_PATHFINDING_FINISHED, UtilsCommon.GetPointsString(m_Request.Points));
        }

        #endregion
    }
}