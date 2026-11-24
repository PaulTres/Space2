using System;
using System.Collections.Generic;

namespace Nav3D.Pathfinding
{
    class MinHeap<T> where T : IComparable<T>
    {
        #region Constants

        const int DEFAULT_HEAP_SIZE = 32;

        #endregion

        #region Attributes

        List<T> m_Data;

        #endregion

        #region Properties

        public int Count => m_Data.Count;

        #endregion

        #region Constructors

        public MinHeap()
        {
            m_Data = new List<T>(DEFAULT_HEAP_SIZE);
        }

        #endregion

        #region Public methods

        //empties the heap and keeps its storage, for a heap reused across searches
        public void Clear() => m_Data.Clear();

        public void Add(T _Item)
        {
            int index  = m_Data.Count;
            int parent = (index - 1) >> 1;
            ;
            m_Data.Add(_Item);

            while (index > 0 && _Item.CompareTo(m_Data[parent]) < 0)
            {
                //There is no need to do a “swap”,
                //because the value with the number “index”
                //is stored either in one of the child elements
                //or it is an “item”.
                m_Data[index] = m_Data[parent];
                index         = parent;
                parent        = (index - 1) >> 1;
                ;
            }

            m_Data[index] = _Item;
        }

        public T PopMin()
        {
            if (m_Data.Count == 0)
                throw new InvalidOperationException("Cannot pop min, heap is empty.");
            T res = m_Data[0];

            m_Data[0] = m_Data[^1];
            int parent = 0;
            T   item   = m_Data[parent];

            while (true)
            {
                int leftChild = (parent << 1) + 1;
                if (leftChild >= m_Data.Count)
                    break;

                int rightChild = (parent << 1) + 2;
                int minChildIndex;

                if (rightChild                                      >= m_Data.Count ||
                    m_Data[leftChild].CompareTo(m_Data[rightChild]) < 0)
                    minChildIndex = leftChild;
                else
                    minChildIndex = rightChild;

                if (item.CompareTo(m_Data[minChildIndex]) < 0)
                    break;

                //There is no need to do a “swap”,
                //because the value with the number “parent”
                //is stored either in its parent element
                //or it is an removed min element.
                m_Data[parent] = m_Data[minChildIndex];
                parent         = minChildIndex;
            }

            m_Data[parent] = item;
            m_Data.RemoveAt(m_Data.Count - 1);

            return res;
        }

        #endregion
    }
}