using System.Collections.Generic;
using System.Collections.Concurrent;
using System.Collections;

namespace Nav3D.Common
{
    class ConcurrentHashSet<T> : IEnumerable<T>
    {
        #region Attributes

        readonly ConcurrentDictionary<T, byte> m_Dictionary = new ConcurrentDictionary<T, byte>();

        #endregion

        #region IEnumerable

        public IEnumerator<T> GetEnumerator()
        {
            //enumerate the dictionary directly: the Keys property allocates a snapshot list on every call
            foreach (KeyValuePair<T, byte> pair in m_Dictionary)
                yield return pair.Key;
        }

        IEnumerator IEnumerable.GetEnumerator()
        {
            return GetEnumerator();
        }

        #endregion

        #region Public methods

        public bool TryAdd(T _Value)
        {
            return m_Dictionary.TryAdd(_Value, 0);
        }

        public bool TryRemove(T _Value)
        {
            return m_Dictionary.TryRemove(_Value, out _);
        }

        public bool Contains(T _Value)
        {
            return m_Dictionary.ContainsKey(_Value);
        }

        #endregion
    }
}