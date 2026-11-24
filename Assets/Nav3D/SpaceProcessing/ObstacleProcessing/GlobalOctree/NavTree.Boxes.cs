using Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.Boxes;
using Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.Elements;
using System.Collections.Generic;
using System.Diagnostics;
using UnityEngine;
using NodeSpatialGrid = Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.SpatialStorages.NodeSpatialGrid;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree
{
    //The tree's box graph (roadmap A14): per root, its free space partitioned into boxes with the portals between them,
    //and per pair of face-adjacent roots, the portals across their shared face. Built by the operation on its staged
    //roots before the write lock (BuildStagedBoxes, from StagingTransaction.Publish) and installed under the lock in the
    //same publish (InstallStagedBoxes), so the graph is always the graph of the published tree — no stale window, no
    //stamps. Only the roots whose occupancy the publish changed are rebuilt; a face table is rebuilt when either of its
    //two roots is. The store is read unlocked by the building operation (operations are serialized and only a publish
    //writes it) and under the read lock by everyone else.
    internal sealed partial class NavTree
    {
        #region Nested types

        //what one publish installs: the rebuilt roots' records (null = the root is gone) and the face tables that touch
        //them (null = one of the two roots is gone)
        internal sealed class StagedBoxes
        {
            public Dictionary<Vector3Int, BoxRootRecord>                  Records      { get; } = new Dictionary<Vector3Int, BoxRootRecord>();
            public Dictionary<(Vector3Int Lower, int Axis), BoxPortal[]> Faces        { get; } = new Dictionary<(Vector3Int, int), BoxPortal[]>();
            public double                                                 Milliseconds { get; set; }
        }

        #endregion

        #region Attributes

        static readonly Vector3Int[] s_BoxAxes = { Vector3Int.right, Vector3Int.up, new Vector3Int(0, 0, 1) };

        readonly Dictionary<Vector3Int, BoxRootRecord>                  m_BoxRecords = new Dictionary<Vector3Int, BoxRootRecord>();
        readonly Dictionary<(Vector3Int Lower, int Axis), BoxPortal[]> m_BoxFaces   = new Dictionary<(Vector3Int, int), BoxPortal[]>();

        //the builds so far, for the probes
        long   m_BoxBuilds;
        long   m_BoxRootsBuilt;
        double m_BoxBuildMilliseconds;
        double m_BoxBuildMaxMilliseconds;

        #endregion

        #region Properties

        //for the probes, read on a quiescent world
        internal IReadOnlyDictionary<Vector3Int, BoxRootRecord>                  BoxRecords => m_BoxRecords;
        internal IReadOnlyDictionary<(Vector3Int Lower, int Axis), BoxPortal[]> BoxFaces   => m_BoxFaces;

        internal (long Builds, long RootsBuilt, double TotalMilliseconds, double MaxMilliseconds) BoxBuildStats =>
            (m_BoxBuilds, m_BoxRootsBuilt, m_BoxBuildMilliseconds, m_BoxBuildMaxMilliseconds);

        #endregion

        #region Service methods

        //the operation's worker, before the write lock: the staged roots are the transaction's own, and the live records
        //it reads for unchanged neighbours are written only by a publish, which cannot run while this operation does
        internal StagedBoxes BuildStagedBoxes(HashSet<Vector3Int> _ChangedRootIndices, NodeSpatialGrid _StagingGrid)
        {
            if (_ChangedRootIndices.Count == 0)
                return null;

            Stopwatch   watch  = Stopwatch.StartNew();
            StagedBoxes staged = new StagedBoxes();

            foreach (Vector3Int rootIndex in _ChangedRootIndices)
            {
                staged.Records[rootIndex] = _StagingGrid.TryGetNode(0, rootIndex, out Node root) && root != null
                                                ? BoxPartition.Build(root, m_Octree.RootSize)
                                                : null;
            }

            foreach (Vector3Int rootIndex in _ChangedRootIndices)
            {
                for (int axis = 0; axis < 3; axis++)
                {
                    StageFace(staged, rootIndex, axis);
                    StageFace(staged, rootIndex - s_BoxAxes[axis], axis);
                }
            }

            staged.Milliseconds = watch.Elapsed.TotalMilliseconds;

            return staged;
        }

        //under the write lock, in the publish that installs the staged roots
        internal void InstallStagedBoxes(StagedBoxes _Staged)
        {
            if (_Staged == null)
                return;

            foreach (KeyValuePair<Vector3Int, BoxRootRecord> record in _Staged.Records)
            {
                if (record.Value == null)
                    m_BoxRecords.Remove(record.Key);
                else
                    m_BoxRecords[record.Key] = record.Value;
            }

            foreach (KeyValuePair<(Vector3Int Lower, int Axis), BoxPortal[]> face in _Staged.Faces)
            {
                if (face.Value == null || face.Value.Length == 0)
                    m_BoxFaces.Remove(face.Key);
                else
                    m_BoxFaces[face.Key] = face.Value;
            }

            m_BoxBuilds++;
            m_BoxRootsBuilt           += _Staged.Records.Count;
            m_BoxBuildMilliseconds    += _Staged.Milliseconds;
            m_BoxBuildMaxMilliseconds =  System.Math.Max(m_BoxBuildMaxMilliseconds, _Staged.Milliseconds);
        }

        //Copies the boxes and portals out under one read-lock hold — a few thousand boxes a world, nothing to chunk. A
        //portal is drawn as the navigation graph's edge: box centre, the portal rectangle's centre, the other box's centre.
        //Roots outside _Result.Scope contribute nothing; a face portal is kept when either of its roots is in scope.
        internal void SnapshotBoxes(NavBoxSnapshot _Result)
        {
            AcquireReadLock();
            DevReadHoldLabel("SnapshotBoxes");
            try
            {
                foreach (BoxRootRecord record in m_BoxRecords.Values)
                {
                    if (!InBoxScope(_Result, record))
                        continue;

                    for (int box = 0; box < record.Boxes.Length; box++)
                        _Result.Boxes.Add(record.WorldBounds(box));

                    foreach (BoxPortal portal in record.InnerPortals)
                    {
                        _Result.Portals.Add(new NavGraphEdge(record.WorldBounds(portal.A).center, portal.Rect.center, record.WorldBounds(portal.B).center));
                        _Result.PortalRects.Add(portal.Rect);
                    }
                }

                foreach (KeyValuePair<(Vector3Int Lower, int Axis), BoxPortal[]> face in m_BoxFaces)
                {
                    if (!m_BoxRecords.TryGetValue(face.Key.Lower, out BoxRootRecord lower) ||
                        !m_BoxRecords.TryGetValue(face.Key.Lower + s_BoxAxes[face.Key.Axis], out BoxRootRecord upper))
                        continue;

                    if (!InBoxScope(_Result, lower) && !InBoxScope(_Result, upper))
                        continue;

                    foreach (BoxPortal portal in face.Value)
                    {
                        _Result.Portals.Add(new NavGraphEdge(lower.WorldBounds(portal.A).center, portal.Rect.center, upper.WorldBounds(portal.B).center));
                        _Result.PortalRects.Add(portal.Rect);
                    }
                }
            }
            finally
            {
                ReleaseReadLock();
            }
        }

        //the search's reads, under the read lock the search already holds

        internal BoxRootRecord BoxRecordOf(Vector3Int _RootIndex) => m_BoxRecords.TryGetValue(_RootIndex, out BoxRootRecord record) ? record : null;

        internal BoxPortal[] BoxFaceTable(Vector3Int _Lower, int _Axis) => m_BoxFaces.TryGetValue((_Lower, _Axis), out BoxPortal[] portals) ? portals : null;

        internal bool TryLocateBox(Vector3 _Point, out BoxRootRecord _Record, out int _Box)
        {
            _Box    = -1;
            _Record = BoxRecordOf(Vector3Int.FloorToInt(_Point / m_Octree.RootSize));

            if (_Record != null)
                _Box = _Record.Locate(_Point);

            return _Box >= 0;
        }

        //the epoch's death, under SeverWorld's write lock
        void ClearBoxes()
        {
            m_BoxRecords.Clear();
            m_BoxFaces.Clear();
        }

        void StageFace(StagedBoxes _Staged, Vector3Int _Lower, int _Axis)
        {
            (Vector3Int, int) key = (_Lower, _Axis);

            if (_Staged.Faces.ContainsKey(key))
                return;

            BoxRootRecord lower = StagedOrLiveRecord(_Staged, _Lower);
            BoxRootRecord upper = StagedOrLiveRecord(_Staged, _Lower + s_BoxAxes[_Axis]);

            _Staged.Faces[key] = lower != null && upper != null ? BoxPartition.FacePortals(lower, upper, _Axis) : null;
        }

        BoxRootRecord StagedOrLiveRecord(StagedBoxes _Staged, Vector3Int _RootIndex)
        {
            if (_Staged.Records.TryGetValue(_RootIndex, out BoxRootRecord staged))
                return staged;

            return m_BoxRecords.TryGetValue(_RootIndex, out BoxRootRecord live) ? live : null;
        }

        static bool InBoxScope(NavBoxSnapshot _Snapshot, BoxRootRecord _Record)
        {
            if (_Snapshot.Scope == null)
                return true;

            Bounds root = new Bounds(_Record.Origin + Vector3.one * (_Record.RootSize * 0.5f), Vector3.one * _Record.RootSize);

            return _Snapshot.Scope.Value.Intersects(root);
        }

        #endregion
    }
}
