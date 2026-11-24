using System.Linq;
using System.Threading;
using System.Threading.Tasks;

namespace Nav3D.Common
{
    class OrdersExecutor<O> where O : IExecutable
    {
        #region Constants

        const string LOG_ORDER_ENQUEUED = "{0} instance ({1}) has enqueued";

        //a gated order re-checks its token this often, so a cancellation issued while it waits is not held until the
        //next completion
        const int GATE_WAIT_SLICE_MS = 20;

        #endregion

        #region Attributes

        readonly int m_MaxConcurrencyLevel;
        volatile int m_CurrentAliveOrders;

        //the live limit. The scheduler's concurrency is fixed at construction and is the ceiling; this is
        //how many orders may be EXECUTING at once right now, adjustable at any moment from any thread. An order that
        //finds the gate full waits on the monitor (it already holds one of the ceiling's threads, which is what
        //those threads are for) and is released by the next completion or by a raised limit. The pathfinding manager
        //lowers it while the main thread's frame time spikes and raises it back while frames are smooth —
        //.
        readonly object m_GateLock = new object();
        int             m_ConcurrencyLimit;
        int             m_GatedRunning;

        readonly ConcurrentHashSet<O> m_OrdersRunning = new ConcurrentHashSet<O>();

        readonly TaskFactory   m_TaskFactory;
        readonly TaskScheduler m_TaskScheduler;

        #endregion

        #region Properties

        public int MaxConcurrencyLevel => m_MaxConcurrencyLevel;
        public int CurrentAliveOrders  => m_CurrentAliveOrders;

        public int ConcurrencyLimit
        {
            get
            {
                lock (m_GateLock)
                    return m_ConcurrencyLimit;
            }
        }

        #endregion

        #region Constructors

        public OrdersExecutor(int _MaxConcurrencyLevel)
        {
            m_MaxConcurrencyLevel = _MaxConcurrencyLevel;
            m_ConcurrencyLimit    = _MaxConcurrencyLevel;

            m_TaskScheduler = new ConcurrentExclusiveSchedulerPair(TaskScheduler.Default, _MaxConcurrencyLevel).ConcurrentScheduler;
            m_TaskFactory   = new TaskFactory(m_TaskScheduler);
        }

        #endregion

        #region Public methods

        public void EnqueueOrder(O _Order, Log _Log = null)
        {
            _Log?.WriteFormat(LOG_ORDER_ENQUEUED, _Order.GetType().Name, _Order.GetHashCode());

            UpdateQueue(_Order);
        }

        public string GetRunningOrdersInfo()
        {
            return string.Join("\n", m_OrdersRunning.Select(_Order => _Order.GetExecutingStatus()));
        }

        //Clamped to [1, MaxConcurrencyLevel]. Lowering never interrupts an executing order — the excess drains as
        //orders complete; raising wakes the orders waiting at the gate.
        public void SetConcurrencyLimit(int _Limit)
        {
            int limit = System.Math.Max(1, System.Math.Min(_Limit, m_MaxConcurrencyLevel));

            lock (m_GateLock)
            {
                if (m_ConcurrencyLimit == limit)
                    return;

                m_ConcurrencyLimit = limit;

                Monitor.PulseAll(m_GateLock);
            }
        }

        #endregion

        #region Service methods

        void UpdateQueue(O _Order)
        {
            void MarkOrderAsRunning()
            {
                Interlocked.Increment(ref m_CurrentAliveOrders);
                
                m_OrdersRunning.TryAdd(_Order);
            }

            void UnmarkOrderAsRunning()
            {
                Interlocked.Decrement(ref m_CurrentAliveOrders);

                m_OrdersRunning.TryRemove(_Order);
            }

            void ExecuteGated()
            {
                //a cancelled order does not queue at the gate: Execute must still run so the order resolves (its
                //body sees the cancelled token and answers at once), it just does not wait for a slot to do so
                lock (m_GateLock)
                {
                    while (m_GatedRunning >= m_ConcurrencyLimit && !_Order.CancellationToken.IsCancellationRequested)
                        Monitor.Wait(m_GateLock, GATE_WAIT_SLICE_MS);

                    m_GatedRunning++;
                }

                try
                {
                    _Order.Execute(MarkOrderAsRunning, UnmarkOrderAsRunning);
                }
                finally
                {
                    lock (m_GateLock)
                    {
                        m_GatedRunning--;

                        Monitor.PulseAll(m_GateLock);
                    }
                }
            }

            m_TaskFactory.StartNew(ExecuteGated, _Order.CancellationToken, TaskCreationOptions.LongRunning, m_TaskScheduler);
        }

        #endregion
    }
}
