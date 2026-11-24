#if LITE_VERSION

using System;
using UnityEngine;

namespace Nav3D.API
{
    public static partial class Nav3DManager
    {

        #region Constants

        static readonly string UPDATE_OBSTACLES_UNSUPPORTED_WARNING =
            $"{nameof(UpdateObstacles)} is supported only in Nav3D Pro. The navigation space stays as it was captured. " +
            $"To change it in Nav3D Lite, dispose Nav3D ({nameof(Nav3DInitializer)}.{nameof(Nav3DInitializer.Utilize)}) and initialize it again.";

        #endregion

        #region Attributes

        static bool s_UpdateObstaclesWarned;

        #endregion

        #region Public methods

        /// <summary>
        /// Updates every runtime obstacle whose geometry crosses a world-space volume. In Nav3D Lite the navigation
        /// space is fixed after initialization; the call only warns, returns 0 and raises _OnFinish at once.
        /// </summary>
        public static int UpdateObstacles(Bounds _Volume, Action _OnFinish = null)
        {
            if (!s_UpdateObstaclesWarned)
            {
                s_UpdateObstaclesWarned = true;

                Debug.LogWarning(UPDATE_OBSTACLES_UNSUPPORTED_WARNING);
            }

            _OnFinish?.Invoke();

            return 0;
        }

        #endregion

    }
}

#endif
