using System.Diagnostics;
using UnityEngine;

namespace Nav3D.Common
{
    //What Nav3D costs the main thread per frame: every main-thread slice Nav3D owns brackets itself with Begin/End
    //and the sum is kept per Unity frame. It is the signal the adaptive search concurrency reacts to — a whole frame
    //can spike for reasons that are nobody's here, and Nav3D backs off only for time it can see is its own. Main
    //thread only: the frame counter is Time.frameCount, which throws off the main thread, so this class keeps its own
    //idea of which thread is main (MarkMainThread, from ThreadDispatcher.Awake) and records nothing elsewhere.
    //Leaning on ThreadDispatcher.IsMainThread instead would let a pool thread reach Time.frameCount inside a lock
    //acquisition and leave the lock held.
    static class Nav3DMainThreadCost
    {
        #region Attributes (static)

        static readonly double s_TicksToMs = 1000d / Stopwatch.Frequency;

        static int    s_MainThreadId = -1;
        static int    s_Frame        = -1;
        static double s_CurrentMs;
        static int    s_LastFrame    = -1;
        static double s_LastMs;

        #endregion

        #region Properties (static)

        //false until MarkMainThread has run — an unknown thread is never assumed to be the main one here
        public static bool IsMainThread => s_MainThreadId != -1 && System.Threading.Thread.CurrentThread.ManagedThreadId == s_MainThreadId;

        //the sum for the frame before the current one (the current frame's is still being written); zero when
        //nothing of Nav3D's ran on the main thread in it
        public static double PreviousFrameMs
        {
            get
            {
                int previous = Time.frameCount - 1;

                if (s_Frame == previous)
                    return s_CurrentMs;

                return s_LastFrame == previous ? s_LastMs : 0d;
            }
        }

        #endregion

        #region Public methods (static)

        //called on the main thread from ThreadDispatcher.Awake (and harmlessly again on every scene load)
        public static void MarkMainThread()
        {
            s_MainThreadId = System.Threading.Thread.CurrentThread.ManagedThreadId;
        }

        public static long Begin()
        {
            return Stopwatch.GetTimestamp();
        }

        //a slice ended on any thread but the main one is dropped, not counted and not an error
        public static void End(long _Begin)
        {
            if (!IsMainThread)
                return;

            Add((Stopwatch.GetTimestamp() - _Begin) * s_TicksToMs);
        }

        #endregion

        #region Service methods (static)

        static void Add(double _Ms)
        {
            int frame = Time.frameCount;

            if (frame != s_Frame)
            {
                s_LastFrame = s_Frame;
                s_LastMs    = s_CurrentMs;
                s_Frame     = frame;
                s_CurrentMs = 0d;
            }

            s_CurrentMs += _Ms;
        }

        #endregion
    }
}
