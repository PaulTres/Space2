using System;
using Nav3D.Common;

namespace Nav3D.API
{
    /// <summary>
    /// The "is the Nav3D runtime up" flag, and the guard the public entry points raise when it is not.
    /// It lives here rather than beside Nav3DManager because the guarded entry points sit in lower assemblies;
    /// Nav3DManager is the only writer. The exception is registered rather than constructed because
    /// Nav3DManagerNotInitializedException is shipped surface a customer may already catch.
    /// </summary>
    static class Nav3DRuntimeGate
    {
        #region Constants

        const string NEED_INIT_ERROR = "Nav3D runtime is not initialized. Call Nav3DManager.InitNav3DRuntime() first.";

        #endregion

        #region Attributes

        static Func<Exception> s_NotInitializedExceptionFactory;

        static Action s_PreInitedCallbacks;

        #endregion

        #region Properties

        public static bool Inited { get; private set; }

        public static bool PreInited { get; private set; }

        #endregion

        #region Public methods

        public static void SetInited(bool _Inited)
        {
            Inited = _Inited;
        }

        public static void SetPreInited(bool _PreInited)
        {
            PreInited = _PreInited;

            Action callbacks = s_PreInitedCallbacks;

            //one-shot in both directions: firing consumes the queue, and a teardown drops whatever never fired
            s_PreInitedCallbacks = null;

            if (_PreInited)
                callbacks?.Invoke();
        }

        /// <summary>
        /// Runs <paramref name="_Callback"/> once the Nav3D managers exist — immediately if they already do.
        /// </summary>
        //The lowered half of Nav3DManager.OnNav3DPreInit, and it exists because subsystems under the entry point
        //need that moment too: a resolution region enabled in a scene must wait for it before it can register with
        //its manager, and it cannot reach up to the event. Same semantics as the public one — one-shot, and the
        //callback is dispatched to the main thread, because everything that waits for pre-init touches Unity
        //objects. Nav3DManager stays the only writer of the flag that drives it.
        public static void InvokeWhenPreInited(Action _Callback)
        {
            if (_Callback == null)
                return;

            if (PreInited)
            {
                ThreadDispatcher.BeginInvoke(_Callback);

                return;
            }

            s_PreInitedCallbacks += () => ThreadDispatcher.BeginInvoke(_Callback);
        }

        public static void SetNotInitializedExceptionFactory(Func<Exception> _Factory)
        {
            s_NotInitializedExceptionFactory = _Factory;
        }

        public static void ThrowIfNotInited()
        {
            if (Inited)
                return;

            throw s_NotInitializedExceptionFactory?.Invoke() ?? new InvalidOperationException(NEED_INIT_ERROR);
        }

        #endregion
    }
}
