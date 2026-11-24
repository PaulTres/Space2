using System.Collections.Generic;
using UnityEngine;
using Nav3D.Obstacles;
using Nav3D.Common;
using Nav3D.SpaceProcessing.ObstacleProcessing.UtilityTypes;
using System.Linq;
using System;
using System.Text;
using System.Threading;

namespace Nav3D.API
{
    //inherited by both subclasses, so one object cannot carry a Runtime and a PreBaked at once
    [DisallowMultipleComponent]
    [ExecuteInEditMode]
    public abstract partial class Nav3DObstacle : MonoBehaviour
    {

        #region Constants

        readonly string INITIALIZER_NOT_FOUND_ERROR = $"Serialization error. {nameof(Nav3DInitializer)} component is not found on scene.";

        #endregion

        #region Serialized fields

        [SerializeField] int                     m_ReducedLayersCount;
        [SerializeField] bool                    m_ProcessChildren = true;
        [SerializeField] bool                    m_HollowInterior;
        [SerializeField] ObstacleGeometrySources m_GeometrySources = ExtensionTransform.LEGACY_GEOMETRY_SOURCES;

        #endregion

        #region Attributes

        //Nav3D's own obstacle identity, drawn once per component instance from the counter below. It is NOT Unity's
        //instance id: the octree keys obstacles on this int and the pre-baked binary stores positions in the id list the
        //loader passes, so the scheme stays the asset's own whatever Unity does to its object ids.
        int m_InstanceID;

        static int s_LastInstanceID;

        internal Transform               m_Transform;
        internal ObstacleInfo            m_ObstacleInfo;
        internal CancellationTokenSource m_ProcessingTokenSource;

        //the obstacle-manager epoch the source is linked to: a scene switch can enable this obstacle while the outgoing
        //epoch's (cancelled) token is still the current one, and a source linked to it cancels every operation it issues
        CancellationToken m_LinkedManagerToken;

        #endregion

        #region Events

        public event Action OnObstacleAdded;

        public event Action OnObstacleRemoved;

        #endregion

        #region Properties

        internal int InstanceID => m_InstanceID;

        /// <summary>
        /// Whether the space enclosed by this obstacle's closed meshes stays navigable. Off (the default), the inside of a
        /// closed body is solid: no path enters it, and an endpoint there is moved to the nearest free point outside. Turn
        /// it on for a body agents live inside — a closed tank, a hull modelled as one box. Read when the obstacle is
        /// processed, so a change applies from its next update.
        /// </summary>
        public bool HollowInterior
        {
            get => m_HollowInterior;
            set => m_HollowInterior = value;
        }

        /// <summary>
        /// Which components give the obstacle its geometry: colliders, MeshFilter meshes, terrains, or several of them.
        /// When its GameObjects carry only one of these kinds, that kind is used whatever this says; when they carry
        /// several and this names none of them, the first by priority is used — collider, then mesh, then terrain. A
        /// component added in the Inspector starts at AUTO; one added by a script, or saved before this setting existed,
        /// reads meshes and terrains until this is set. Read when the obstacle is processed, so a change applies from its
        /// next update.
        /// </summary>
        public ObstacleGeometrySources GeometrySources
        {
            get => m_GeometrySources;
            set => m_GeometrySources = value;
        }

        //the kinds of source the obstacle's GameObjects carry, read as if the obstacle were active, and the ones its capture
        //reads of them
        internal ObstacleGeometrySources PresentGeometrySources => CachedTransform.GetPresentGeometrySources(m_ProcessChildren, IsNestedObstacleOwner, true);

        internal ObstacleGeometrySources UsedGeometrySources => ExtensionTransform.ResolveGeometrySources(m_GeometrySources, PresentGeometrySources);

        //what ObstacleInfo carries for the octree: null keeps every closed shell solid
        internal Predicate<Triangle> HollowTriangles => m_HollowInterior ? ObstacleInfo.AllTriangles : null;

        public abstract IObstacleAdditionProgress AdditionProgress { get; }

        internal Transform CachedTransform => m_Transform = m_Transform != null ? m_Transform : transform;

        //the token an operation is issued with, linked to the obstacle manager's current epoch
        internal CancellationToken ProcessingToken
        {
            get
            {
                if (m_ProcessingTokenSource == null || m_LinkedManagerToken != ObstacleManager.CancellationToken)
                    ResetCancellationToken();

                return m_ProcessingTokenSource.Token;
            }
        }

        #endregion

        #region Public methods

        internal void InvokeOnObstacleAdded()
        {
            OnObstacleAddedEditor();

            OnObstacleAdded?.Invoke();
        }

        internal void InvokeOnObstacleRemoved()
        {
            OnObstacleRemovedEditor();

            OnObstacleRemoved?.Invoke();
        }

        public void Init()
        {
            //Sticky: the loader and the component's own OnEnable both call Init on a pre-baked obstacle, and the id the
            //loader handed the octree must still be the one the component adds itself under.
            if (m_InstanceID == 0)
            {
                m_InstanceID = Interlocked.Increment(ref s_LastInstanceID);

                RegisterGizmosCacheEditor();
            }

            ResetCancellationToken();
        }

        public bool TryGetObstacleInfo(out (ObstacleInfo ObstacleInfo, string Imprint)? _ObstacleInfo)
        {
            Nav3DInitializer initializer = UtilsSceneObjects.FindAny<Nav3DInitializer>();

            if (initializer == null)
                throw new Exception(INITIALIZER_NOT_FOUND_ERROR);

            return TryGetObstacleInfo(initializer.MaxAgentRadius, out _ObstacleInfo);
        }

        //The overload for a caller that already knows the world's MaxAgentRadius (resolving the initializer per obstacle
        //is a scene search each time; a caller capturing many obstacles at once resolves it once).
        public bool TryGetObstacleInfo(float _MaxAgentRadius, out (ObstacleInfo ObstacleInfo, string Imprint)? _ObstacleInfo)
        {
            List<(string Imprint, HashSet<Triangle> Triangles)> obstacleData = CachedTransform.GetObstacleData(m_ProcessChildren, IsNestedObstacleOwner, m_GeometrySources);

            if (!obstacleData.Any())
            {
                _ObstacleInfo = null;
                return false;
            }

            HashSet<Triangle> triangles            = new HashSet<Triangle>();
            StringBuilder     imprintStringBuilder = new StringBuilder();

            foreach ((string Imprint, HashSet<Triangle> Triangles) obstacleDataPiece in obstacleData)
            {
                triangles.AddRange(obstacleDataPiece.Triangles);
                imprintStringBuilder.Append(obstacleDataPiece.Imprint);
            }

            _ObstacleInfo = (new ObstacleInfo(m_InstanceID, triangles, _MaxAgentRadius, HollowTriangles), imprintStringBuilder.ToString());
            return true;
        }

        #endregion

        #region Service methods

        partial void OnObstacleAddedEditor();

        partial void OnObstacleRemovedEditor();

        partial void RegisterGizmosCacheEditor();

        partial void UnregisterGizmosCacheEditor();

        //SUBTREE ownership RULE: a descendant carrying
        //its own obstacle component owns its subtree exclusively, and every traversal here prunes such subtrees.
        //Keyed on component presence, not enabled state — that is what lets a nested component act as a door
        //(enable/disable = its subtree's octree presence) or as a decor excluder (never enabled = never registered).
        //Ownership is read at capture time, so nested owners must exist before the ancestor is enabled.
        static bool IsNestedObstacleOwner(Transform _Transform)
        {
            return _Transform.TryGetComponent(out Nav3DObstacle _);
        }

        internal virtual void DoEnable()
        {
            Init();
        }

        internal virtual void DoUpdate()
        {
        }

        partial void CheckEditorDoDisableGuard(ref bool _Skip);

        partial void DisableHook();

        partial void DestroyHook();

        void DoDisable()
        {
            bool skip = false;
            CheckEditorDoDisableGuard(ref skip);

            if (skip)
                return;

            ResetCancellationToken();

            DisableHook();
        }

        //a cancelled source cannot be reused, and both the addition and the removal need one
        void ResetCancellationToken()
        {
            m_ProcessingTokenSource?.Cancel();

            m_LinkedManagerToken    = ObstacleManager.CancellationToken;
            m_ProcessingTokenSource = CancellationTokenSource.CreateLinkedTokenSource(m_LinkedManagerToken);
        }

        #endregion

        #region Unity methods

        void Update()
        {
            DoUpdate();
        }

        void OnEnable()
        {
            DoEnable();
        }

        void OnDisable()
        {
            DoDisable();
        }

        void OnDestroy()
        {
            DestroyHook();

            UnregisterGizmosCacheEditor();
        }

        #endregion

    }
}
