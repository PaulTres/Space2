#if UNITY_EDITOR
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;

namespace Nav3D.Common
{
    partial class ThreadDispatcher
    {
        bool                                                   m_VerboseInvocation;
        readonly ConcurrentQueue<KeyValuePair<string, Action>> m_ActionQueueEditor = new ConcurrentQueue<KeyValuePair<string, Action>>();

        public void Initialize(bool _VerboseInvocation)
        {
            m_VerboseInvocation = _VerboseInvocation;
        }

        static partial void BeginInvokeEditor(ThreadDispatcher _Instance, Action _Action, ref bool _Handled)
        {
            _Instance.m_ActionQueueEditor.Enqueue(new KeyValuePair<string, Action>(Environment.StackTrace, _Action));
            _Handled = true;
        }

        partial void UpdateEditor(ref bool _Handled)
        {
            while (m_ActionQueueEditor.TryDequeue(out KeyValuePair<string, Action> keyValuePair))
            {
                if (keyValuePair.Value != null)
                {
                    try
                    {
                        if (m_VerboseInvocation)
                            UnityEngine.Debug.Log($"{nameof(ThreadDispatcher)}.QueueUpdate(): StackTrace: {keyValuePair.Key}");

                        keyValuePair.Value.Invoke();
                    }
                    catch (Exception ex)
                    {
                        UnityEngine.Debug.LogErrorFormat("Error on event invocation: {0}", ex);
                    }
                }
            }
            _Handled = true;
        }
    }
}
#endif
