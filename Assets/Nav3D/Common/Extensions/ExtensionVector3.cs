using System.Diagnostics.CodeAnalysis;
using UnityEngine;
using System.Globalization;

namespace Nav3D.Common
{
    static class ExtensionVector3
    {
        #region Constants

        #endregion
        
        #region Static methods

        public static string ToStringExt(this Vector3 _Vector)
        {
            return $"({_Vector.x.ToString(CultureInfo.InvariantCulture)}, " +
                   $"{_Vector.y.ToString(CultureInfo.InvariantCulture)}, "  +
                   $"{_Vector.z.ToString(CultureInfo.InvariantCulture)})";
        }

        [SuppressMessage("ReSharper", "CompareOfFloatsByEqualityOperator")]
        public static int LexCompare(this Vector3 _A, Vector3 _B)
        {
            if (_A.x != _B.x) return _A.x.CompareTo(_B.x);
            if (_A.y != _B.y) return _A.y.CompareTo(_B.y);
            return _A.z.CompareTo(_B.z);
        }
        
        #endregion
    }
}