#if UNITY_EDITOR

using Nav3D.Common;
using Nav3D.Common.Debug;
using System.Linq;
using UnityEditor;
using UnityEngine;

namespace Nav3D.API
{
    public partial class Nav3DInitializer
    {

        #region Constants

        //How long (seconds) the root-grid preview stays visible after the last change of
        //'Octree Min Bucket Size' or 'Root size'.
        const double PREVIEW_DURATION = 5d;

        //Above this many root cells we draw only the grid-snapped bounding box instead of every cell,
        //to keep the scene view responsive.
        const long PREVIEW_MAX_CELLS = 4096;

        #endregion

        #region Static attributes

        static readonly Color PREVIEW_CELL_COLOR       = new Color(0.25f, 0.85f, 1f, 0.9f);
        static readonly Color PREVIEW_BOUNDS_COLOR     = new Color(1f, 0.6f, 0.2f, 0.9f);
        static readonly Color PREVIEW_MIN_BUCKET_COLOR = new Color(1f, 0.95f, 0.3f, 0.95f);

        static double s_PreviewUntil;
        static bool   s_PreviewPumpRegistered;

        #endregion

        #region Public methods

        //Called by the settings inspector whenever 'Octree Min Bucket Size' or 'Root size' changes,
        //to (re)arm the transient root-grid scene preview.
        internal static void PingRootGridPreview()
        {
            s_PreviewUntil = EditorApplication.timeSinceStartup + PREVIEW_DURATION;

            if (!s_PreviewPumpRegistered)
            {
                s_PreviewPumpRegistered  =  true;
                EditorApplication.update += PreviewRepaintPump;
            }

            SceneView.RepaintAll();
        }

        #endregion

        [UnityEditor.MenuItem(MENU_PATH)]
        internal static void CreateOnScene()
        {
            UtilsGameObject.InstantiateGOWithComponent<Nav3DInitializer>(nameof(Nav3DInitializer));
        }

        #region Preview drawing

        static bool PreviewActive => EditorApplication.timeSinceStartup < s_PreviewUntil;

        //Drives continuous scene-view repaints while the preview window is open (edit mode does not repaint
        //every frame otherwise), then unsubscribes itself so it costs nothing once the window elapses.
        static void PreviewRepaintPump()
        {
            if (EditorApplication.timeSinceStartup >= s_PreviewUntil)
            {
                EditorApplication.update -= PreviewRepaintPump;
                s_PreviewPumpRegistered  =  false;
                SceneView.RepaintAll();
                return;
            }

            SceneView.RepaintAll();
        }

        void OnDrawGizmosSelected()
        {
            if (!PreviewActive)
                return;

            DrawRootGridPreview();
        }

        void DrawRootGridPreview()
        {
            float rootSize = Nav3DWorldSettings.GetMaxBucketSize(OctreeMinBucketSize, OctreeLayersCount);

            if (rootSize <= 0f)
                return;

            //Cover the combined bounds of scene obstacles; fall back to a single cell around this component
            //when there are no obstacles yet, so the size is still conveyed.
            if (!TryGetObstaclesBounds(out Bounds bounds))
                bounds = new Bounds(transform.position, Vector3.zero);

            Vector3Int minIndex = UtilsSpatialGrid.GetBucketIndex(bounds.min, rootSize);
            Vector3Int maxIndex = UtilsSpatialGrid.GetBucketIndex(bounds.max, rootSize);

            long countX = (long) (maxIndex.x - minIndex.x) + 1;
            long countY = (long) (maxIndex.y - minIndex.y) + 1;
            long countZ = (long) (maxIndex.z - minIndex.z) + 1;
            long total  = countX * countY * countZ;

            Bounds snapped = new Bounds();

            snapped.SetMinMax(
                    new Vector3(minIndex.x, minIndex.y, minIndex.z)             * rootSize,
                    new Vector3(maxIndex.x + 1, maxIndex.y + 1, maxIndex.z + 1) * rootSize
                );

            //A min-bucket cube sits in the min corner of every drawn root (or of the bounding box when the roots are
            //too many to draw), so the two sizes the user is choosing between are seen side by side, at scale.
            using (UtilsGizmos.ColorPermanence)
            {
                if (total > PREVIEW_MAX_CELLS)
                {
                    Gizmos.color = PREVIEW_BOUNDS_COLOR;
                    Gizmos.DrawWireCube(snapped.center, snapped.size);

                    DrawMinBucketCube(snapped.min);
                }
                else
                {
                    for (int x = minIndex.x; x <= maxIndex.x; x++)
                        for (int y = minIndex.y; y <= maxIndex.y; y++)
                            for (int z = minIndex.z; z <= maxIndex.z; z++)
                            {
                                Bounds cell = UtilsSpatialGrid.GetBucketBounds(new Vector3Int(x, y, z), rootSize);

                                Gizmos.color = PREVIEW_CELL_COLOR;
                                Gizmos.DrawWireCube(cell.center, cell.size);

                                DrawMinBucketCube(cell.min);
                            }
                }
            }

            string label = $"Root size: {rootSize:0.###}  ({OctreeMinBucketSize:0.###} × 2^{OctreeLayersCount - 1})\n" +
                           $"Min bucket: {OctreeMinBucketSize:0.###}  (the small cube in each root's corner)\n"        +
                           $"Roots: {countX}×{countY}×{countZ} = {total}";

            if (total > PREVIEW_MAX_CELLS)
                label += "  (too many to draw individually)";

            Handles.Label(new Vector3(snapped.center.x, snapped.max.y, snapped.center.z), label, EditorStyles.whiteBoldLabel);
        }

        void DrawMinBucketCube(Vector3 _Corner)
        {
            Vector3 size = Vector3.one * OctreeMinBucketSize;

            Gizmos.color = PREVIEW_MIN_BUCKET_COLOR;
            Gizmos.DrawWireCube(_Corner + size * 0.5f, size);
        }

        static bool TryGetObstaclesBounds(out Bounds _Bounds)
        {
            _Bounds = default;
            bool any = false;

            foreach (Nav3DObstacle obstacle in UtilsSceneObjects.FindAll<Nav3DObstacle>())
            {
                foreach (MeshRenderer meshRenderer in obstacle.GetComponentsInChildren<MeshRenderer>())
                {
                    if (!any)
                    {
                        _Bounds = meshRenderer.bounds;
                        any     = true;
                    }
                    else
                    {
                        _Bounds.Encapsulate(meshRenderer.bounds);
                    }
                }
            }

            return any;
        }

        #endregion

        partial void CheckEditorAwakeGuard(ref bool _Skip)
        {
            if (!Application.isPlaying || !enabled)
            {
                //edit mode only. Here a duplicate is a copy the user has just made (Ctrl+D, paste, a second
                //Create Initializer), so it goes on the spot - the menu path promises duplication is not
                //possible, and nothing is lost that was not made a moment ago. In play mode - and in a player
                //build, where this whole partial is compiled out - Awake()'s first-come ownership takes over
                //instead: it reports and leaves the component alone.
                if (UtilsSceneObjects.FindAll<Nav3DInitializer>().Any(_Instance => _Instance != this))
                {
                    Debug.LogError(INSTANCE_HAS_EXIST_ON_SCENE_ERROR, this);

                    UtilsCommon.SmartDestroy(this);
                }

                _Skip = true;
            }
        }
    }
}

#endif