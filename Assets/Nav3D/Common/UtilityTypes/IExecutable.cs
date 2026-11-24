using System;
using System.Threading;

namespace Nav3D.Common
{
    interface IExecutable
    {
        #region Properties
        
        public CancellationToken CancellationToken { get; }
        
        #endregion
        
        #region Public methods

        public void   Execute(Action _OnStart, Action _OnResolve);
        public string GetExecutingStatus();

        #endregion
    }
}