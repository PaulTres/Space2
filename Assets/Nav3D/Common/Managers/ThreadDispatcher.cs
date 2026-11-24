using System;
using System.Collections.Concurrent;
using UnityEngine;

namespace Nav3D.Common
{
    [DisallowMultipleComponent]
    partial class ThreadDispatcher : MonoBehaviour
    {

        #region Factory

        public static ThreadDispatcher Instance => Singleton<ThreadDispatcher>.Instance;

        #endregion

        #region Attributes

        readonly ConcurrentQueue<Action> m_ActionQueue = new ConcurrentQueue<Action>();

        //The live instance captured on the main thread (Awake/Initialize). BeginInvoke is called from worker
        //threads where Singleton.Instance's lazy scene lookup is illegal; worker threads read this field
        //instead of resolving the singleton.
        static ThreadDispatcher s_MainThreadInstance;

        //Unity main thread id, captured by Initialize (which runs on the main thread).
        static int s_MainThreadId = -1;

        #endregion

        #region Properties

        public static bool Doomed { get; private set; } = false;

        //True on the Unity main thread. Before Initialize captures the id, every call site is necessarily
        //main-thread code (workers only exist after Nav3D init), so an uninitialized dispatcher reports true —
        //this lets callers invoke main-thread-contract callbacks INLINE on early paths where BeginInvoke would
        //silently drop them (no instance to enqueue on).
        public static bool IsMainThread => s_MainThreadId == -1 || System.Threading.Thread.CurrentThread.ManagedThreadId == s_MainThreadId;

        #endregion

        #region Public methods

        /// <summary>
        /// Must be inited from main thread!
        /// </summary>
        public void Initialize()
        {
            //also here, not only in Awake: edit mode runs no Awake, and the last play session's OnDestroy left it set
            Doomed = false;

            s_MainThreadInstance = this;
            s_MainThreadId       = System.Threading.Thread.CurrentThread.ManagedThreadId;

            Nav3DMainThreadCost.MarkMainThread();
        }

        public void Uninitialize()
        {
            UtilsCommon.SmartDestroyImmediateWithHolder(this);
        }

        static partial void BeginInvokeEditor(ThreadDispatcher _Instance, Action _Action, ref bool _Handled);

        public static void BeginInvoke(Action _Action)
        {
            if (_Action == null)
                return;

            //read the main-thread-captured instance; never resolve the singleton on a worker thread
            ThreadDispatcher instance = s_MainThreadInstance;

            if (instance == null)
                return;

            bool handled = false;
            BeginInvokeEditor(instance, _Action, ref handled);

            if (!handled)
                instance.m_ActionQueue.Enqueue(_Action);
        }

        public static void BeginInvoke<T>(Action<T> _Action, T _Arg)
        {
            BeginInvoke(() => _Action(_Arg));
        }

        #endregion

        #region Dev instrumentation hooks

        //Implemented in ThreadDispatcher.DevTimings.cs ([Nav3DDevOnly], deleted by the release preparer — an
        //unimplemented partial void compiles to nothing, call sites included). They name every dispatched action
        //as a profiler sample, so a long Nav3D.Dispatcher frame says which callback it ran.
        partial void DevActionBegin(Action _Action);
        partial void DevActionEnd();

        #endregion

        #region Engine methods

        partial void UpdateEditor(ref bool _Handled);

        void Update()
        {
            bool handled = false;
            UpdateEditor(ref handled);

            if (handled)
                return;

            UnityEngine.Profiling.Profiler.BeginSample("Nav3D.Dispatcher");

            //the drain is one of the main-thread slices the adaptive search concurrency watches (Nav3DMainThreadCost)
            long drainBegin = Nav3DMainThreadCost.Begin();

            while (m_ActionQueue.TryDequeue(out Action action))
            {
                if (action != null)
                {
                    try
                    {
                        DevActionBegin(action);
                        action.Invoke();
                        DevActionEnd();
                    }
                    catch (Exception ex)
                    {
                        UnityEngine.Debug.LogErrorFormat("Error on event invocation: {0}", ex);
                    }
                }
            }

            Nav3DMainThreadCost.End(drainBegin);

            UnityEngine.Profiling.Profiler.EndSample();
        }

        void Awake()
        {
            Doomed = false;

            //capture the instance on the main thread as early as possible for worker-thread BeginInvoke
            s_MainThreadInstance = this;

            //Awake is the main thread — the one place that is certain to run there (Initialize is not always called)
            Nav3DMainThreadCost.MarkMainThread();
        }

        void OnDestroy()
        {
            Doomed = true;

            //Drop both static references to this instance so nothing keeps it (and its action queue) rooted past
            //this destroy — s_MainThreadInstance is a second, Singleton<T>-independent static this type keeps for
            //worker-thread BeginInvoke access (see its declaration above).
            Singleton<ThreadDispatcher>.ClearInstance(this);

            if (ReferenceEquals(s_MainThreadInstance, this))
                s_MainThreadInstance = null;
        }

        #endregion

    }
}