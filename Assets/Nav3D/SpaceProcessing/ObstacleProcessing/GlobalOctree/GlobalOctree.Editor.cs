#if UNITY_EDITOR

using Nav3D.API;
using Nav3D.Obstacles;
using Nav3D.Common;
using Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.Elements;
using Nav3D.SpaceProcessing.ObstacleProcessing.UtilityTypes;
using System;
using System.Collections.Generic;
using System.Linq;
using UnityEditor;
using UnityEngine;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree
{
    partial class GlobalOctree
    {

        #region Properties

        public int LayersCount => m_LayersCount;

        #endregion

        #region Public methods

        public void FillGizmosDrawDataForAll(
                Common.Debug.GizmosDrawData _GizmosDrawData,
                bool                        _DrawOccupiedLeaves,
                bool                        _DrawFreeLeaves,
                bool                        _DrawGraph,
                bool                        _DrawRoots,
                int                         _DrawOccupiedLeavesAll,
                int                         _DrawOccupiedLeavesLayerNumber,
                int                         _DrawFreeLeavesAll,
                int                         _DrawFreeLeavesLayerNumber,
                int                         _DrawGraphNodesAll,
                int                         _DrawGraphLayerNumber
            )
        {
            Node[]            roots;
            Node[]            allRoots;
            TriangleCluster[] clusters;

            //Snapshot the live tree and cluster maps under the read lock. A concurrent obstacle operation mutates
            //m_RootsGrid and these maps only inside its write-locked publish, so taking the read lock here prevents
            //enumerating any of them mid-mutation (which would otherwise throw "Collection was modified" on the
            //gizmo thread). Read the live m_RootsGrid, not m_ActiveRootsGrid — the latter points at an operation's
            //private staging grid while it is building. An operation never mutates an already-published root in
            //place (it replaces roots at publish), so the captured node objects stay valid to draw after release.
            m_NavTree.EnterReadLock();

            try
            {
                clusters = m_GeometryRegistry.SafeClustersByID.SelectMany(_Kvp => _Kvp.Value).ToArray();

                roots = clusters.SelectMany(_Cluster => m_GeometryRegistry.TryGetRootsOfCluster(_Cluster, out Vector3Int[] rootIndices)
                                                ? rootIndices.Select(_Idx => m_NavTree.LiveRootsGrid.TryGetNode(0, _Idx, out Node root) ? root : null
                                                              )
                                                             .Where(_Root => _Root != null)
                                                : Enumerable.Empty<Node>()
                                 )
                                .Distinct()
                                .ToArray();

                allRoots = _DrawRoots ? m_NavTree.LiveRootsGrid.GetAllAtLevel(0) : Array.Empty<Node>();
            }
            finally
            {
                m_NavTree.ExitReadLock();
            }

            foreach (Node root in roots)
            {
                root.FillGizmosDrawData(
                        _GizmosDrawData,
                        _DrawOccupiedLeaves,
                        _DrawFreeLeaves,
                        _DrawGraph,
                        _DrawOccupiedLeavesAll,
                        _DrawOccupiedLeavesLayerNumber,
                        _DrawFreeLeavesAll,
                        _DrawFreeLeavesLayerNumber,
                        _DrawGraphNodesAll,
                        _DrawGraphLayerNumber
                    );
            }

            if (_DrawRoots)
            {
                foreach (Node root in allRoots)
                {
                    GUIStyle style = new GUIStyle
                    {
                        normal = { textColor = Color.cyan }
                    };

                    _GizmosDrawData.Add(new Common.Debug.GizmosWireCube(root.Bounds.center, root.Bounds.size, Color.cyan));
                    _GizmosDrawData.Add(new Common.Debug.GizmosLabel(root.Bounds.center, $"[ID: {root.ID}]\n[{root.Index}]", style));
                }
            }

            foreach (TriangleCluster cluster in clusters)
            {
                _GizmosDrawData.Add(new Common.Debug.GizmosWireCube(cluster.Bounds.center, cluster.Bounds.size, Color.white));
            }
        }

        public void FillGizmosDrawData(
                Common.Debug.GizmosDrawData _GizmosDrawData,
                int                         _ObstacleID,
                bool                        _DrawOccupiedLeaves,
                bool                        _DrawFreeLeaves,
                bool                        _DrawGraph,
                bool                        _DrawRoots,
                int                         _DrawOccupiedLeavesAll,
                int                         _DrawOccupiedLeavesLayerNumber,
                int                         _DrawFreeLeavesAll,
                int                         _DrawFreeLeavesLayerNumber,
                int                         _DrawGraphNodesAll,
                int                         _DrawGraphLayerNumber
            )
        {
            TriangleCluster[] clusters;
            Node[]            roots;

            //Snapshot under the read lock (see FillGizmosDrawDataForAll) and resolve roots from the live
            //m_RootsGrid — not m_ActiveRootsGrid, which may be an operation's private staging grid mid-build.
            m_NavTree.EnterReadLock();

            try
            {
                if (!m_GeometryRegistry.TryGetSafeClusters(_ObstacleID, out List<TriangleCluster> clustersList))
                    return;

                clusters = clustersList.ToArray();

                roots = clusters.SelectMany(_Cluster => m_GeometryRegistry.TryGetRootsOfCluster(_Cluster, out Vector3Int[] rootIndices)
                                                ? rootIndices.Select(_Idx => m_NavTree.LiveRootsGrid.TryGetNode(0, _Idx, out Node root) ? root : null
                                                              )
                                                             .Where(_Root => _Root != null)
                                                : Enumerable.Empty<Node>()
                                 )
                                .Distinct()
                                .ToArray();
            }
            finally
            {
                m_NavTree.ExitReadLock();
            }

            foreach (Node root in roots)
            {
                root.FillGizmosDrawData(
                        _GizmosDrawData,
                        _DrawOccupiedLeaves,
                        _DrawFreeLeaves,
                        _DrawGraph,
                        _DrawOccupiedLeavesAll,
                        _DrawOccupiedLeavesLayerNumber,
                        _DrawFreeLeavesAll,
                        _DrawFreeLeavesLayerNumber,
                        _DrawGraphNodesAll,
                        _DrawGraphLayerNumber
                    );

                if (_DrawRoots)
                {
                    GUIStyle style = new GUIStyle
                    {
                        normal = { textColor = Color.cyan }
                    };

                    _GizmosDrawData.Add(new Common.Debug.GizmosWireCube(root.Bounds.center, root.Bounds.size, Color.cyan));
                    _GizmosDrawData.Add(new Common.Debug.GizmosLabel(root.Bounds.center, $"[ID: {root.ID}]", style));
                }
            }

            foreach (TriangleCluster cluster in clusters)
            {
                _GizmosDrawData.Add(new Common.Debug.GizmosWireCube(cluster.Bounds.center, cluster.Bounds.size, Color.white));
            }
        }

        //Draws the navigation-graph open FACES (the faces a node may exit through to free space toward a goal): a
        //coloured outline of each open face plus a short outward normal stub. This is the visual check for the
        //ClassifyOpenFaces classification. Snapshots the live free edge leaves under the read lock (like
        //FillGizmosDrawDataForAll) so it never enumerates the tree mid-publish.
        public void FillOpenFacesGizmos(Common.Debug.GizmosDrawData _GizmosDrawData)
        {
            Leaf[] edgeLeaves;

            m_NavTree.EnterReadLock();

            try
            {
                List<Leaf>    collected = new List<Leaf>();
                HashSet<Leaf> scratch   = new HashSet<Leaf>();

                foreach (Node root in m_NavTree.LiveRootsGrid.GetAllAtLevel(0))
                {
                    scratch.Clear();
                    root.GetAllContainedFreeLeaves(scratch);

                    foreach (Leaf leaf in scratch)
                    {
                        if (leaf.IsEdgeNode)
                            collected.Add(leaf);
                    }
                }

                edgeLeaves = collected.ToArray();
            }
            finally
            {
                m_NavTree.ExitReadLock();
            }

            foreach (Leaf leaf in edgeLeaves)
            {
                for (int bit = 0; bit < 6; bit++)
                {
                    if (leaf.IsFaceOpen(bit))
                        AddOpenFaceGizmo(_GizmosDrawData, leaf, bit);
                }
            }
        }

        #endregion

        #region Service methods

        static void AddOpenFaceGizmo(Common.Debug.GizmosDrawData _GizmosDrawData, Leaf _Leaf, int _FaceBit)
        {
            Color color = new Color(0.1f, 1f, 0.4f);

            Bounds bounds = _Leaf.Bounds;
            float  half   = bounds.extents.x;

            (Vector3 normal, Vector3 tangentA, Vector3 tangentB) = FaceAxes(_FaceBit);

            Vector3 faceCenter = bounds.center + normal * half;

            Vector3 c0 = faceCenter + (tangentA  + tangentB) * half;
            Vector3 c1 = faceCenter + (tangentA  - tangentB) * half;
            Vector3 c2 = faceCenter + (-tangentA - tangentB) * half;
            Vector3 c3 = faceCenter + (-tangentA + tangentB) * half;

            //face outline
            _GizmosDrawData.Add(new Common.Debug.GizmosLine(c0, c1, color));
            _GizmosDrawData.Add(new Common.Debug.GizmosLine(c1, c2, color));
            _GizmosDrawData.Add(new Common.Debug.GizmosLine(c2, c3, color));
            _GizmosDrawData.Add(new Common.Debug.GizmosLine(c3, c0, color));
            //diagonals + outward normal stub, so an open face reads clearly at a glance
            _GizmosDrawData.Add(new Common.Debug.GizmosLine(c0, c2, color));
            _GizmosDrawData.Add(new Common.Debug.GizmosLine(c1, c3, color));
            _GizmosDrawData.Add(new Common.Debug.GizmosLine(faceCenter, faceCenter + normal * half * 0.6f, color));
        }

        //FaceBit (0..5 = +x,-x,+y,-y,+z,-z, matching UtilsSpatialGrid.FacesDirections) → outward normal + two in-face tangents.
        static (Vector3 Normal, Vector3 TangentA, Vector3 TangentB) FaceAxes(int _FaceBit)
        {
            switch (_FaceBit)
            {
                case 0:
                    return (Vector3.right, Vector3.up, Vector3.forward);
                case 1:
                    return (Vector3.left, Vector3.up, Vector3.forward);
                case 2:
                    return (Vector3.up, Vector3.right, Vector3.forward);
                case 3:
                    return (Vector3.down, Vector3.right, Vector3.forward);
                case 4:
                    return (Vector3.forward, Vector3.right, Vector3.up);
                default:
                    return (Vector3.back, Vector3.right, Vector3.up);
            }
        }

        partial void ResetGizmosDataForObstacles(params int[] _IDs)
        {
            try
            {
                ThreadDispatcher.BeginInvoke(() =>
                        {
                            foreach (int id in _IDs)
                            {
                                if (ObstacleGizmosCacheRegistry.TryGet(id, out IObstacleGizmosCache cache))
                                    cache.ClearGizmosCache();
                            }
                        }
                    );
            }
            catch (Exception exception)
            {
                UnityEngine.Debug.LogError(exception.Message);
            }
        }

        #endregion

    }
}

#endif