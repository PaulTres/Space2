using System.Collections;
using System.Collections.Concurrent;

namespace Nav3D.Common
{
    class LimitedSizeQueue<T>
    {
        #region Attributes

        object m_LockObject = new object();

        int                m_Limit;
        ConcurrentQueue<T> m_Queue;

        #endregion

        #region Properties

        public int Count => m_Queue.Count;
        public int Limit => m_Limit;

        #endregion

        #region Constructors

        public LimitedSizeQueue(int _MaxSize)
        {
            m_Limit = _MaxSize;
            m_Queue = new ConcurrentQueue<T>();

        }

        #endregion

        #region Public methods

        public void Enqueue(T _Value)
        {
            m_Queue.Enqueue(_Value);


            lock (m_LockObject)
            {
                while (m_Queue.Count > m_Limit && m_Queue.TryDequeue(out _)) ;
            }
        }

        public IEnumerator GetEnumerator()
        {
            return m_Queue.GetEnumerator();
        }

        #endregion
    }
}