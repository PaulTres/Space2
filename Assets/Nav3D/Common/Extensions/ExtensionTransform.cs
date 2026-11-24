using System;
using System.Collections.Generic;
using Nav3D.API;
using UnityEngine;

namespace Nav3D.Common
{
    static class ExtensionTransform
    {

        #region Constants

        //what an obstacle read before colliders became a source; a component added from code, or saved before, keeps it
        public const ObstacleGeometrySources LEGACY_GEOMETRY_SOURCES = ObstacleGeometrySources.MESH | ObstacleGeometrySources.TERRAIN;

        static readonly ObstacleGeometrySources[] GEOMETRY_SOURCES_BY_PRIORITY =
        {
            ObstacleGeometrySources.COLLIDER,
            ObstacleGeometrySources.MESH,
            ObstacleGeometrySources.TERRAIN
        };

        #endregion

        #region Public methods

        //A deferred obstacle-geometry capture: the same walk as GetObstacleData, but each mesh or collider piece is kept
        //as its arrays plus the matrix that places them, and the world-space triangles are built later by BuildTriangles
        //on whatever thread asks. A terrain piece is built at once, from the height map. _PruneSubtree skips a child
        //(with its subtree) for which it returns true: the obstacle layer's subtree-ownership rule arrives as a
        //delegate because this class sits below the API layer.
        public sealed class ObstacleGeometryCapture
        {

            #region Nested types

            public readonly struct Piece
            {
                public Piece(string _Imprint, Matrix4x4 _LocalToWorld, Vector3[] _Vertices, int[] _Indices, HashSet<Triangle> _ReadyTriangles)
                {
                    Imprint        = _Imprint;
                    LocalToWorld   = _LocalToWorld;
                    Vertices       = _Vertices;
                    Indices        = _Indices;
                    ReadyTriangles = _ReadyTriangles;
                }

                public string            Imprint        { get; }
                public Matrix4x4         LocalToWorld   { get; }
                public Vector3[]         Vertices       { get; }
                public int[]             Indices        { get; }
                public HashSet<Triangle> ReadyTriangles { get; }
            }

            #endregion

            #region Properties

            public List<Piece> Pieces { get; } = new List<Piece>();

            public string Imprint
            {
                get
                {
                    System.Text.StringBuilder imprint = new System.Text.StringBuilder();

                    foreach (Piece piece in Pieces)
                        imprint.Append(piece.Imprint);

                    return imprint.ToString();
                }
            }

            #endregion

            #region Public methods

            //thread-free: pure math over the copied arrays
            public HashSet<Triangle> BuildTriangles()
            {
                HashSet<Triangle> triangles = new HashSet<Triangle>();

                foreach (Piece piece in Pieces)
                {
                    if (piece.ReadyTriangles != null)
                        triangles.UnionWith(piece.ReadyTriangles);
                    else
                        triangles.UnionWith(UtilsCommon.GetMeshTransformTriangles(piece.LocalToWorld, piece.Vertices, piece.Indices));
                }

                return triangles;
            }

            #endregion

        }

        public static ObstacleGeometryCapture CaptureObstacleGeometry(
                this Transform          _Transform,
                bool                    _FactorInChildren,
                Func<Transform, bool>   _PruneSubtree = null,
                ObstacleGeometrySources _Selection    = LEGACY_GEOMETRY_SOURCES
            )
        {
            ObstacleGeometryCapture capture = new ObstacleGeometryCapture();

            List<(Component Source, int Depth)> sources = new List<(Component Source, int Depth)>();
            CollectGeometrySources(_Transform, _FactorInChildren, _PruneSubtree, _Selection, sources);

            for (int i = 0; i < sources.Count; i++)
                capture.Pieces.Add(CapturePiece(sources[i].Source, GetImprint(sources[i].Source, i, sources[i].Depth, _FactorInChildren)));

            return capture;
        }

        public static List<(string Imprint, HashSet<Triangle> Triangles)> GetObstacleData(
                this Transform          _Transform,
                bool                    _FactorInChildren,
                Func<Transform, bool>   _PruneSubtree = null,
                ObstacleGeometrySources _Selection    = LEGACY_GEOMETRY_SOURCES
            )
        {
            List<(string Imprint, HashSet<Triangle> Triangles)> result = new List<(string Imprint, HashSet<Triangle> Triangles)>();

            List<(Component Source, int Depth)> sources = new List<(Component Source, int Depth)>();
            CollectGeometrySources(_Transform, _FactorInChildren, _PruneSubtree, _Selection, sources);

            for (int i = 0; i < sources.Count; i++)
                result.Add((GetImprint(sources[i].Source, i, sources[i].Depth, _FactorInChildren), BuildTriangles(sources[i].Source)));

            return result;
        }

        //The kinds of geometry source that give geometry in the subtree a capture reads. _AsIfActive reads the subtree as it
        //will be once the root is active — an inspector shows an inactive object, or a prefab asset, what it will read.
        public static ObstacleGeometrySources GetPresentGeometrySources(
                this Transform        _Transform,
                bool                  _FactorInChildren,
                Func<Transform, bool> _PruneSubtree = null,
                bool                  _AsIfActive   = false
            )
        {
            List<(Component Source, int Depth)> components = new List<(Component Source, int Depth)>();
            CollectGeometrySourceComponents(_Transform, 0, _FactorInChildren, _PruneSubtree, _AsIfActive, components);

            return GetPresentGeometrySources(components);
        }

        //one kind present is used whatever the selection; otherwise the selected ones present, or else the first present
        public static ObstacleGeometrySources ResolveGeometrySources(ObstacleGeometrySources _Selection, ObstacleGeometrySources _Present)
        {
            ObstacleGeometrySources selected = _Selection & _Present;

            if (selected != ObstacleGeometrySources.AUTO)
                return selected;

            foreach (ObstacleGeometrySources kind in GEOMETRY_SOURCES_BY_PRIORITY)
            {
                if ((_Present & kind) != 0)
                    return kind;
            }

            return ObstacleGeometrySources.AUTO;
        }

        //the components that give the capture its geometry under _Selection, in the capture's order, each with its depth
        public static void CollectGeometrySources(
                this Transform                      _Transform,
                bool                                _FactorInChildren,
                Func<Transform, bool>               _PruneSubtree,
                ObstacleGeometrySources             _Selection,
                List<(Component Source, int Depth)> _Result
            )
        {
            List<(Component Source, int Depth)> components = new List<(Component Source, int Depth)>();
            CollectGeometrySourceComponents(_Transform, _FactorInChildren, _PruneSubtree, components);

            ObstacleGeometrySources sources = ResolveGeometrySources(_Selection, GetPresentGeometrySources(components));

            foreach ((Component Source, int Depth) component in components)
            {
                if ((GetGeometrySourceKind(component.Source) & sources) != 0 && GivesGeometry(component.Source))
                    _Result.Add(component);
            }
        }

        //Every collider, MeshFilter and Terrain on the active GameObjects a capture reads, whether or not it gives
        //geometry now: what a runtime obstacle watches for a component that is removed, disabled or enabled.
        public static void CollectGeometrySourceComponents(
                this Transform                      _Transform,
                bool                                _FactorInChildren,
                Func<Transform, bool>               _PruneSubtree,
                List<(Component Source, int Depth)> _Result
            )
        {
            CollectGeometrySourceComponents(_Transform, 0, _FactorInChildren, _PruneSubtree, false, _Result);
        }

        //a MeshFilter cannot be disabled
        public static bool IsGeometrySourceEnabled(Component _Source)
        {
            switch (_Source)
            {
                case Collider collider:
                    return collider.enabled;

                case Terrain terrain:
                    return terrain.enabled;

                default:
                    return true;
            }
        }

        //Here we assume that each obstacle contains either a mesh or a terrain, and not both.
        /*public static bool TryGetObstacleInfo(
                this Transform         _Transform,
                Nav3DObstacle          _ObstacleRootController,
                out ObstacleInfoSingle _AdditionData
            )
        {
            if (TryGetObstacleMeshData(_Transform, _ObstacleRootController, out _AdditionData))
                return true;

            if (TryGetObstacleTerrainData(_Transform, _ObstacleRootController, out _AdditionData))
                return true;

            UnityEngine.Debug.LogWarning(string.Format(MESH_ERROR, _Transform.name, _Transform.UnityIdText()));

            return false;
        }*/

        //_PruneSubtree: same contract as in GetObstacleData — a child matching it is skipped with its whole
        //subtree, the root itself is never tested
        public static List<Transform> GetAllChildren(this Transform _Root, bool _CheckActive = true, Func<Transform, bool> _PruneSubtree = null)
        {
            List<Transform> transformList = new List<Transform>();

            GetAllChildren(_Root, transformList, _CheckActive, _PruneSubtree);

            return transformList;
        }

        #endregion

        #region Service methods

        static void CollectGeometrySourceComponents(
                Transform                           _Transform,
                int                                 _Depth,
                bool                                _FactorInChildren,
                Func<Transform, bool>               _PruneSubtree,
                bool                                _AsIfActive,
                List<(Component Source, int Depth)> _Result
            )
        {
            //as if active: the root counts whatever its state, a descendant by its own flag
            bool active = _AsIfActive ? _Depth == 0 || _Transform.gameObject.activeSelf : _Transform.gameObject.activeInHierarchy;

            if (!active)
                return;

            foreach (Collider collider in _Transform.GetComponents<Collider>())
            {
                if (ColliderGeometry.IsSupported(collider))
                    _Result.Add((collider, _Depth));
            }

            if (_Transform.TryGetComponent(out MeshFilter meshFilter))
                _Result.Add((meshFilter, _Depth));

            if (_Transform.TryGetComponent(out Terrain terrain))
                _Result.Add((terrain, _Depth));

            if (!_FactorInChildren)
                return;

            foreach (Transform child in _Transform)
            {
                if (_PruneSubtree != null && _PruneSubtree(child))
                    continue;

                CollectGeometrySourceComponents(child, _Depth + 1, true, _PruneSubtree, _AsIfActive, _Result);
            }
        }

        static ObstacleGeometrySources GetPresentGeometrySources(List<(Component Source, int Depth)> _Components)
        {
            ObstacleGeometrySources present = ObstacleGeometrySources.AUTO;

            foreach ((Component Source, int Depth) component in _Components)
            {
                if (GivesGeometry(component.Source))
                    present |= GetGeometrySourceKind(component.Source);
            }

            return present;
        }

        static ObstacleGeometrySources GetGeometrySourceKind(Component _Source)
        {
            switch (_Source)
            {
                case Collider _:
                    return ObstacleGeometrySources.COLLIDER;

                case MeshFilter _:
                    return ObstacleGeometrySources.MESH;

                default:
                    return ObstacleGeometrySources.TERRAIN;
            }
        }

        static bool GivesGeometry(Component _Source)
        {
            switch (_Source)
            {
                case MeshCollider meshCollider:
                    return meshCollider.enabled && !meshCollider.isTrigger && HasTriangles(meshCollider.sharedMesh);

                case TerrainCollider terrainCollider:
                    return terrainCollider.enabled && !terrainCollider.isTrigger && HasSize(terrainCollider.terrainData);

                case Collider collider:
                    return collider.enabled && !collider.isTrigger;

                case MeshFilter meshFilter:
                    return HasTriangles(meshFilter.sharedMesh);

                case Terrain terrain:
                    return terrain.enabled && HasSize(terrain.terrainData);

                default:
                    return false;
            }
        }

        //without copying the index array (Mesh.triangles is a fresh managed copy on every read)
        static bool HasTriangles(Mesh _Mesh)
        {
            if (_Mesh == null)
                return false;

            for (int subMesh = 0; subMesh < _Mesh.subMeshCount; subMesh++)
            {
                if (_Mesh.GetIndexCount(subMesh) > 0)
                    return true;
            }

            return false;
        }

        static bool HasSize(TerrainData _TerrainData)
        {
            return _TerrainData != null && _TerrainData.size != Vector3.zero;
        }

        //Without children the imprint carries no depth, as it always has: a bake keeps validating against the same
        //strings. A mesh and a terrain are imprinted as they were before colliders became a source.
        static string GetImprint(Component _Source, int _Index, int _Depth, bool _FactorInChildren)
        {
            string kind = _Source is Collider collider ? $"collider{ColliderGeometry.Describe(collider)}" : _Source is MeshFilter ? "mesh" : "terrain";

            return $"{_Index}{kind}{GetTransformImprintStr(_Source.transform)}{(_FactorInChildren ? _Depth.ToString() : string.Empty)}";
        }

        //the deferred form: a Unity read of the arrays and the matrix now, the triangles later on any thread
        static ObstacleGeometryCapture.Piece CapturePiece(Component _Source, string _Imprint)
        {
            Transform transform = _Source.transform;

            switch (_Source)
            {
                case MeshFilter meshFilter:
                    return new ObstacleGeometryCapture.Piece(_Imprint, transform.localToWorldMatrix, meshFilter.sharedMesh.vertices, meshFilter.sharedMesh.triangles, null);

                case MeshCollider meshCollider:
                    return new ObstacleGeometryCapture.Piece(_Imprint, transform.localToWorldMatrix, meshCollider.sharedMesh.vertices, meshCollider.sharedMesh.triangles, null);

                case Terrain terrain:
                    return new ObstacleGeometryCapture.Piece(_Imprint, Matrix4x4.identity, null, null, UtilsCommon.GetTerrainTransformTriangles(transform, terrain.terrainData));

                case TerrainCollider terrainCollider:
                    return new ObstacleGeometryCapture.Piece(
                            _Imprint,
                            Matrix4x4.identity,
                            null,
                            null,
                            UtilsCommon.GetTerrainTransformTriangles(transform, terrainCollider.terrainData)
                        );

                default:
                    ColliderGeometry.TryGetPrimitive((Collider) _Source, out Matrix4x4 localToWorld, out Vector3[] vertices, out int[] indices);

                    return new ObstacleGeometryCapture.Piece(_Imprint, localToWorld, vertices, indices, null);
            }
        }

        //the eager form, for the bake: a mesh keeps the transform overload its baked data was built with
        static HashSet<Triangle> BuildTriangles(Component _Source)
        {
            Transform transform = _Source.transform;

            switch (_Source)
            {
                case MeshFilter meshFilter:
                    return UtilsCommon.GetMeshTransformTriangles(transform, meshFilter.sharedMesh);

                case MeshCollider meshCollider:
                    return UtilsCommon.GetMeshTransformTriangles(transform, meshCollider.sharedMesh);

                case Terrain terrain:
                    return UtilsCommon.GetTerrainTransformTriangles(transform, terrain.terrainData);

                case TerrainCollider terrainCollider:
                    return UtilsCommon.GetTerrainTransformTriangles(transform, terrainCollider.terrainData);

                default:
                    ColliderGeometry.TryGetPrimitive((Collider) _Source, out Matrix4x4 localToWorld, out Vector3[] vertices, out int[] indices);

                    return UtilsCommon.GetMeshTransformTriangles(localToWorld, vertices, indices);
            }
        }

        static string GetTransformImprintStr(Transform _Transform)
        {
            return $"{_Transform.position.ToStringExt()}|{_Transform.rotation.ToStringExt()}|{_Transform.lossyScale.ToStringExt()}";
        }

        /*static bool TryGetObstacleMeshData(
                this Transform         _Transform,
                Nav3DObstacle          _ObstacleRootController,
                out ObstacleInfoSingle _AdditionData
            )
        {
            if (!_Transform.TryGetComponent(out MeshFilter meshFilter) || meshFilter.sharedMesh.triangles.Length == 0)
            {
                _AdditionData = null;

                return false;
            }

            Mesh obstacleMesh = meshFilter.sharedMesh;

            _AdditionData = new ObstacleInfoMesh(
                    _ObstacleRootController.RootsSizeMode,
                    _ObstacleRootController.ReducedLayersCount,
                    _ObstacleRootController.InstanceID,
                    obstacleMesh.vertices,
                    obstacleMesh.triangles,
                    _Transform.position,
                    _Transform.lossyScale,
                    _Transform.rotation
                );

            return true;
        }

        static bool TryGetObstacleTerrainData(
                this Transform         _Transform,
                Nav3DObstacle          _ObstacleRootController,
                out ObstacleInfoSingle _AdditionData
            )
        {
            if (!_Transform.TryGetComponent(out Terrain terrain))
            {
                _AdditionData = null;
                return false;
            }

            TerrainData terrainData         = terrain.terrainData;
            int         heightMapResolution = terrainData.heightmapResolution;

            _AdditionData = new ObstacleInfoTerrain(
                    _ObstacleRootController.RootsSizeMode,
                    _ObstacleRootController.ReducedLayersCount,
                    _ObstacleRootController.InstanceID,
                    terrainData.GetHeights(0, 0, heightMapResolution, heightMapResolution),
                    heightMapResolution,
                    terrainData.size,
                    _Transform.position
                );

            return true;
        }*/

        //Parses transform tree. Resulting list contains root transform and all nested children.
        static void GetAllChildren(Transform _Transform, List<Transform> _Children, bool _CheckActive, Func<Transform, bool> _PruneSubtree)
        {
            if (_CheckActive && !_Transform.gameObject.activeInHierarchy)
                return;

            if (_Children != null)
                _Children.Add(_Transform);
            else
                _Children = new List<Transform> { _Transform };

            foreach (Transform child in _Transform)
            {
                if (_PruneSubtree != null && _PruneSubtree(child))
                    continue;

                GetAllChildren(child, _Children, _CheckActive, _PruneSubtree);
            }
        }

        #endregion

    }
}