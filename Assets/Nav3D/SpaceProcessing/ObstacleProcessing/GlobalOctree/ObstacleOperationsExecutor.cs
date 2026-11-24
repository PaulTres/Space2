using Nav3D.Common;
using System;
using System.Collections.Concurrent;
using System.Threading;
using System.Threading.Tasks;
using ObstacleAdditionOperation = Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.Operations.ObstacleAdditionOperation;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree
{
    partial class ObstacleOperationsExecutor
    {
        #region Attributes

        readonly ConcurrentDictionary<int, ObstacleAdditionOperation> m_AdditionPendingOrders = new ConcurrentDictionary<int, ObstacleAdditionOperation>();

        //quiescence bookkeeping. An operation is outstanding from the moment it is scheduled until its body returns
        //(or its task is cancelled before the body ever ran). The pending maps cannot answer this on their own: a typed
        //order leaves its map when its body STARTS, and the two Force enqueues never enter one at all. Nothing in the
        //engine reads these — they exist so a dev reader can tell "the octree is settled" from "an operation is in
        //flight", which the WRONG_RESOLUTION_SIZE check needs to be meaningful at all.
        int  m_OutstandingOperations;
        long m_ScheduledOperations;

        readonly TaskFactory   m_TaskFactory;
        readonly TaskScheduler m_TaskScheduler;

        #endregion

        #region Construction

        public ObstacleOperationsExecutor()
        {
            m_TaskScheduler = new ConcurrentExclusiveSchedulerPair(TaskScheduler.Default, 1).ConcurrentScheduler;
            m_TaskFactory   = new TaskFactory(m_TaskScheduler);
        }

        #endregion

        #region Properties

        //No operation is queued, running, or scheduled-but-not-yet-started. Operations are serialized, so this is the
        //octree's settled state: everything requested so far has been published (or cancelled).
        public bool IsIdle => Volatile.Read(ref m_OutstandingOperations) == 0;

        //Monotonic count of operations ever scheduled. IsIdle alone cannot tell a reader that nothing ran during a walk
        //— an operation can start and finish inside it. Sampling this before and after, with IsIdle true at both ends,
        //can: the pair is unchanged only if no operation was scheduled in between.
        public long ScheduledOperationsCount => Interlocked.Read(ref m_ScheduledOperations);

        #endregion

        #region Public methods

        public void EnqueueAdditionalOrderForce(ObstacleAdditionOperation _AdditionOperation)
        {
            if (!_AdditionOperation.CancellationToken.IsCancellationRequested)
                Schedule(_AdditionOperation, null);
        }

        public void EnqueueAdditionalOrder(int _ID, ObstacleAdditionOperation _AdditionOperation)
        {
            if (_AdditionOperation.CancellationToken.IsCancellationRequested || !m_AdditionPendingOrders.TryAdd(_ID, _AdditionOperation))
                return;

            Action removeFromPending = () =>
            {
                m_AdditionPendingOrders.TryRemove(_ID, out _);
            };

            Schedule(_AdditionOperation, removeFromPending);
        }

        //Runs _Action behind every operation already scheduled — the scheduler is exclusive, so "after" is structural,
        //not a wait — and cannot be cancelled: it is for work that must happen because everything else was
        public void ScheduleAfterAll(Action _Action)
        {
            m_TaskFactory.StartNew(_Action, CancellationToken.None, TaskCreationOptions.LongRunning, m_TaskScheduler);
        }

        #endregion

        #region Service methods

        //the one place an operation reaches the serialized scheduler. Behaviour is exactly what every enqueue did by
        //hand before: run the operation's body on the exclusive scheduler, passing _OnStart (the pending-map release)
        //into Execute, and release the pending entry too when the task is cancelled before its body ever ran. What it
        //adds is the outstanding count, incremented here — at scheduling — and decremented exactly once, on whichever
        //of the two paths ends the operation, so a reader never sees "idle" while an operation is still queued.
        void Schedule(IExecutable _Operation, Action _OnStart)
        {
            Interlocked.Increment(ref m_OutstandingOperations);
            Interlocked.Increment(ref m_ScheduledOperations);

            int released = 0;

            Action release = () =>
            {
                if (Interlocked.Exchange(ref released, 1) == 0)
                    Interlocked.Decrement(ref m_OutstandingOperations);
            };

            m_TaskFactory.StartNew(
                              () =>
                              {
                                  try
                                  {
                                      _Operation.Execute(_OnStart, null);
                                  }
                                  finally
                                  {
                                      release();
                                  }
                              },
                              _Operation.CancellationToken,
                              TaskCreationOptions.LongRunning,
                              m_TaskScheduler
                          )
                         .ContinueWith(
                              _ =>
                              {
                                  _OnStart?.Invoke();
                                  release();
                              },
                              TaskContinuationOptions.OnlyOnCanceled
                          );
        }

        #endregion
    }
}
