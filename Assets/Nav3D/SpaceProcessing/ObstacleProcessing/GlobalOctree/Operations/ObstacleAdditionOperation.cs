using Nav3D.API;
using Nav3D.Common;
using Nav3D.SpaceProcessing.ObstacleProcessing.UtilityTypes;
using System;
using System.Threading;
using AdditionMethod =
    System.Action<Nav3D.SpaceProcessing.ObstacleProcessing.UtilityTypes.ObstacleInfo,
        Nav3D.API.ObstacleAdditionProgress,
        System.Threading.CancellationToken,
        System.Action,
        bool>;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.Operations
{
    class ObstacleAdditionOperation : IExecutable
    {
        #region Attributes

        readonly Action                   m_OnFinish;
        readonly AdditionMethod           m_AdditionMethod;
        readonly Func<ObstacleInfo>       m_GetObstacleInfo;
        readonly ObstacleAdditionProgress m_AdditionProgress;
        readonly bool                     m_EditMode;

        #endregion

        #region Properties

        public CancellationToken CancellationToken { get; }

        #endregion

        #region Constructors

        public ObstacleAdditionOperation(
                Func<ObstacleInfo>       _GetObstacleInfo,
                ObstacleAdditionProgress _AdditionProgress,
                AdditionMethod           _AdditionMethod,
                CancellationToken        _CancellationToken,
                Action                   _OnFinish,
                bool                     _EditMode
            )
        {
            m_GetObstacleInfo  = _GetObstacleInfo;
            m_AdditionProgress = _AdditionProgress;
            CancellationToken  = _CancellationToken;
            m_AdditionMethod   = _AdditionMethod;
            m_OnFinish         = _OnFinish;
            m_EditMode         = _EditMode;
        }

        #endregion

        #region Public methods

        public void Execute(Action _OnStart, Action _OnResolve)
        {
            _OnStart?.Invoke();
            m_AdditionMethod?.Invoke(m_GetObstacleInfo(), m_AdditionProgress, CancellationToken, m_OnFinish, m_EditMode);
        }
        public string GetExecutingStatus()
        {
            return string.Empty;
        }

        #endregion
    }
}