using UnityEngine;

namespace Nav3D.Common
{
    static class ExtensionVector3Int
    {
        #region Static methods

        public static bool IsNeighborBucket(this Vector3Int _Bucket, Vector3Int _Other)
        {
            if (Mathf.Abs(_Bucket.x - _Other.x) > 1)
                return false;

            if (Mathf.Abs(_Bucket.y - _Other.y) > 1)
                return false;

            if (Mathf.Abs(_Bucket.z - _Other.z) > 1)
                return false;

            return true;
        }

        #endregion
    }
}