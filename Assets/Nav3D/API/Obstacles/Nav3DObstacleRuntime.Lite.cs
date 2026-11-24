#if LITE_VERSION

using Nav3D.Common;
using Nav3D.Obstacles;
using Nav3D.SpaceProcessing.ObstacleProcessing.UtilityTypes;
using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Nav3D.API
{

    public abstract partial class Nav3DObstacle
    {

        #region Constants

        static readonly string DISABLE_UNSUPPORTED_WARNING =
            "Disabling an obstacle after Nav3D initialization is supported only in Nav3D Pro. \"{0}\" stays in the navigation space. " +
            $"To change the navigation space in Nav3D Lite, dispose Nav3D ({nameof(Nav3DInitializer)}.{nameof(Nav3DInitializer.Utilize)}) and initialize it again.";

        #endregion

        #region Service methods

        partial void DisableHook()
        {
            //only a CAPTURED obstacle has anything to stay in; teardown (scene unload, play-mode exit) disables
            //everything and is not a user asking for a change
            if (m_ObstacleInfo == null || !Nav3DManager.Inited || ObstacleManager.Doomed)
                return;

            Debug.LogWarning(string.Format(DISABLE_UNSUPPORTED_WARNING, gameObject.name));
        }

        //the merged obstacle carries one ID; every captured component takes it so that IsObstacleRegistered, the gizmo
        //drawing and InstanceID all answer for the obstacle the component actually belongs to
        internal void AssignSharedObstacleID(int _ID)
        {
            m_InstanceID = _ID;
        }

        #endregion

    }

    public partial class Nav3DObstacleRuntime
    {

        #region Constants

        static readonly string ENABLE_UNSUPPORTED_WARNING =
            "Enabling an obstacle after Nav3D initialization is supported only in Nav3D Pro. \"{0}\" is not part of the navigation space. " +
            "In Nav3D Lite the scene's obstacles are captured once, when Nav3D initializes: enable them before that, or dispose Nav3D and initialize it again.";

        static readonly string AUTO_UPDATE_UNSUPPORTED_WARNING =
            $"{nameof(AutoUpdateIfChanged)} is supported only in Nav3D Pro. \"{{0}}\" stays where it was captured; the setting is preserved but has no effect.";

        static readonly string UPDATE_UNSUPPORTED_WARNING =
            $"{nameof(RequestUpdate)} is supported only in Nav3D Pro. \"{{0}}\" stays as it was captured. " +
            $"To change the navigation space in Nav3D Lite, dispose Nav3D ({nameof(Nav3DInitializer)}.{nameof(Nav3DInitializer.Utilize)}) and initialize it again.";

        #endregion

        #region Attributes

        bool m_AutoUpdateWarned;
        bool m_UpdateWarned;

        #endregion

        #region Properties

        /// <summary>
        /// When enabled, transform changes re-add the obstacle automatically (throttled by
        /// AutoUpdateMinPeriod). Exposed for runtime-created obstacles, which cannot set the serialized flag in the
        /// inspector. Always false in Nav3D Lite (the serialized value is preserved but inert).
        /// </summary>
        public bool AutoUpdateIfChanged
        {
            get => false;
            set
            {
                m_AutoUpdateIfChanged = value;

                if (value)
                    WarnAutoUpdateUnavailable();
            }
        }

        #endregion

        #region Public methods

        /// <summary>
        /// Re-reads the obstacle's geometry and pose and applies them to the navigation space as one move
        /// operation (the obstacle is never absent midway). For a mesh edited in place — a deformed or partially destroyed
        /// body — which the transform-driven auto-update cannot see. Nav3D detects changed geometry chunks and reuses
        /// unchanged ones; geometry capture and change detection still inspect the obstacle. Not throttled: call after
        /// completing an edit, rather than every frame. Does nothing before the obstacle's addition has
        /// been requested. In Nav3D Lite the navigation space is fixed after initialization; the call only warns.
        /// </summary>
        public void RequestUpdate()
        {
            WarnUpdateUnavailable();
        }

        /// <summary>
        /// Like RequestUpdate(), for geometry that changed only inside a known world-space volume. In Nav3D Lite the
        /// navigation space is fixed after initialization; the call only warns.
        /// </summary>
        public void RequestUpdate(Bounds _ChangedVolume)
        {
            WarnUpdateUnavailable();
        }

        #endregion

        #region Service methods

        partial void EnableHook()
        {
            //A captured component arrives here too: at scene load its OnEnable may run after the initializer's Awake
            //(which captured it — the capture scans the loaded scene, not the OnEnable order), and again after a
            //disable/enable pair. The base DoEnable's Init() keeps an id already assigned, so the merged obstacle's
            //shared identity survives it; re-assert it all the same and say nothing: the obstacle is in the navigation
            //space.
            if (m_ObstacleInfo != null)
            {
                AssignSharedObstacleID(m_ObstacleInfo.ID);

                return;
            }

            //before initialization the capture will pick this component up; after it the space is fixed
            if (!Nav3DManager.PreInited || ObstacleManager.Doomed)
                return;

            Debug.LogWarning(string.Format(ENABLE_UNSUPPORTED_WARNING, gameObject.name));
        }

        void WarnAutoUpdateUnavailable()
        {
            if (m_AutoUpdateWarned)
                return;

            m_AutoUpdateWarned = true;

            Debug.LogWarning(string.Format(AUTO_UPDATE_UNSUPPORTED_WARNING, gameObject.name));
        }

        void WarnUpdateUnavailable()
        {
            if (m_UpdateWarned)
                return;

            m_UpdateWarned = true;

            Debug.LogWarning(string.Format(UPDATE_UNSUPPORTED_WARNING, gameObject.name));
        }

        internal void SetCaptured(ObstacleInfo _MergedObstacleInfo, ObstacleAdditionProgress _AdditionProgress)
        {
            m_ObstacleInfo = _MergedObstacleInfo;
            m_AdditionProgress = _AdditionProgress;
        }

        #endregion

    }

    internal static class Nav3DLiteObstacleCapture
    {

        #region Constants

        static readonly string NO_OBSTACLE_INFOS_ERROR =
            "There is no obstacle data components found for GameObject: \"{0}\". Make sure that the game object or " +
            $"its children in the transform hierarchy has attached {nameof(MeshFilter)} component with the selected " +
            $"{nameof(Mesh)}, or {nameof(Terrain)} component and that \"Auto add child transforms\" parameter is "     +
            $"enabled. Note that children carrying their own {nameof(Nav3DObstacleRuntime)} component are excluded: " +
            "a nested obstacle component owns its subtree exclusively.";

        static readonly string CAPTURE_REPORT = "Nav3D: navigation space built from {0} obstacle component(s), {1} triangles, in {2} ms.";

        #endregion

        #region Public methods

        //Scans the scene rather than keeping a registration list: components already enabled never see OnEnable again,
        //and a dispose + re-initialize must capture the scene exactly as it is at that moment.
        public static void Capture(float _MaxAgentRadius, Action _OnFinished)
        {
            System.Diagnostics.Stopwatch captureWatch = System.Diagnostics.Stopwatch.StartNew();

            List<Nav3DObstacleRuntime> captured = new List<Nav3DObstacleRuntime>();
            HashSet<Triangle>          triangles = new HashSet<Triangle>();
            HashSet<Triangle>          hollow = new HashSet<Triangle>();

            IEnumerable<Nav3DObstacleRuntime> present = UtilsSceneObjects.FindAll<Nav3DObstacleRuntime>()
                                                                   .Where(_Obstacle => _Obstacle.enabled && _Obstacle.gameObject.activeInHierarchy);

            foreach (Nav3DObstacleRuntime obstacle in present)
            {
                obstacle.Init();

                if (!obstacle.TryGetObstacleInfo(_MaxAgentRadius, out (ObstacleInfo ObstacleInfo, string Imprint)? obstacleInfo))
                {
                    Debug.LogError(string.Format(NO_OBSTACLE_INFOS_ERROR, obstacle.gameObject.name));

                    continue;
                }

                triangles.AddRange(obstacleInfo!.Value.ObstacleInfo.Triangles);
                captured.Add(obstacle);

                //the merged obstacle keeps each component's own choice: a hollow component's shells stay navigable
                if (obstacle.HollowInterior)
                    hollow.AddRange(obstacleInfo.Value.ObstacleInfo.Triangles);
            }

            if (captured.Count == 0)
            {
                _OnFinished?.Invoke();

                return;
            }

            int obstacleID = captured[0].InstanceID;

            foreach (Nav3DObstacleRuntime obstacle in captured)
            {
                obstacle.AssignSharedObstacleID(obstacleID);
            }

            ObstacleInfo merged = new ObstacleInfo(obstacleID, triangles, _MaxAgentRadius, hollow.Count > 0 ? hollow.Contains : null);

            //completion is dispatched on the main thread by the addition worker
            ObstacleAdditionProgress additionProgress = ObstacleManager.Instance.AddObstacle(
                    () => merged,
                    ObstacleManager.CancellationToken,
                    () =>
                    {
                        if (!ObstacleManager.Doomed)
                        {
                            foreach (Nav3DObstacleRuntime obstacle in captured)
                            {
                                obstacle.InvokeOnObstacleAdded();
                            }

                            Debug.Log(string.Format(CAPTURE_REPORT, captured.Count, merged.Triangles.Count, captureWatch.ElapsedMilliseconds));
                        }

                        _OnFinished?.Invoke();
                    }
                );

            foreach (Nav3DObstacleRuntime obstacle in captured)
            {
                obstacle.SetCaptured(merged, additionProgress);
            }
        }

        #endregion

    }
}

#endif
