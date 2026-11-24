using System;
using System.Collections.Generic;
using System.Linq;

namespace Nav3D.Common
{
    public static class ExtensionIEnumerable
    {
        //specific list copy implementation

        #region Public methods

        public static List<T> Copy<T>(this List<T> _List)
        {
            return new List<T>(_List);
        }

        public static int FindIndex<T>(this T[] _Array, Predicate<T> _Match)
        {
            for (int i = 0; i < _Array.Length; i++)
            {
                if (_Match(_Array[i]))
                    return i;
            }

            return -1;
        }

        public static void RemoveLast<T>(this List<T> _List)
        {
            if (_List == null || !_List.Any())
            {
                throw new ArgumentException("List must not to be null or empty");
            }
            
            _List.RemoveAt(_List.Count - 1);
        }
        
        public static T Second<T>(this List<T> _List)
        {
            if (_List == null || _List.Count < 2)
            {
                throw new ArgumentException("List must be not null and has length greater then one");
            }

            return _List[1];
        }

        public static T[] Copy<T>(this T[] _Array)
        {
            T[] copy = new T[_Array.Length];
            _Array.CopyTo(copy, 0);

            return copy;
        }

        public static void AddRange<T>(this HashSet<T> _HashSet, IEnumerable<T> _Enumerable)
        {
            _Enumerable.ForEach(_Element => _HashSet.Add(_Element));
        }
        
        public static void ForEach<T>(this IEnumerable<T> _Enumerable, Action<T> _Action)
        {
            foreach (T item in _Enumerable)
                _Action(item);
        }

        public static T MinBy<T>(this IEnumerable<T> _Enumerable, Func<T, float> _MinFunc)
        {
            return _Enumerable.OrderBy(_MinFunc).First();
        }

        public static bool IsNullOrEmpty<T>(this IEnumerable<T> _Enumerable)
        {
            return _Enumerable == null || !_Enumerable.Any();

        #endregion
        }
    }
}