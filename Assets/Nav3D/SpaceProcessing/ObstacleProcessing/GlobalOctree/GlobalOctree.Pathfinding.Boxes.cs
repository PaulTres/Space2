using Nav3D.API;
using Nav3D.Common;
using Nav3D.Obstacles;
using Nav3D.Pathfinding;
using Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.Boxes;
using System;
using System.Collections.Generic;
using System.Threading;
using UnityEngine;
using Leaf = Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.Elements.Leaf;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree
{
    //The point lane's search over the box graph (roadmap A14, step 2): an A* whose nodes are the free boxes the publish
    //installed (NavTree.Boxes.cs), each keeping the point it was best entered at, joined through the rectangles two boxes
    //share. A box lying on a root face with no root behind it may also shoot straight at the goal, or into the next region
    //a blocked shot meets — the flat search's free-space edge, so what it can reach outside the roots this search reaches
    //too. The route is then pulled taut through the rectangles it crossed; a box is convex and free, so every leg inside
    //one is free by construction. Runs under the read lock the attempt already holds; the published box graph is the
    //published tree's, so nothing here is ever stale.
    partial class GlobalOctree
    {
        #region Nested types

        readonly struct BoxHeapEntry : IComparable<BoxHeapEntry>
        {
            public BoxHeapEntry(float _F, int _Node)
            {
                F    = _F;
                Node = _Node;
            }

            public float F    { get; }
            public int   Node { get; }

            public int CompareTo(BoxHeapEntry _Other) => F.CompareTo(_Other.F);
        }

        //one box as the search met it
        sealed class BoxNode
        {
            public BoxRootRecord Record   { get; set; }
            public int           Box      { get; set; }
            public float         G        { get; set; } = float.PositiveInfinity;
            public Vector3       At       { get; set; }
            public int           Parent   { get; set; } = -1;
            public bool          Closed   { get; set; }
            public bool          Straight { get; set; }
            public Bounds        Portal   { get; set; }
            public int           Axis     { get; set; }

            //the points the route turned through in the previous box on its way here: the point on its outer face a shot into
            //this box left from, or — at a radius — the one or two points a body rounded a bend inside it by
            public Vector3?      Via      { get; set; }
            public Vector3?      Via2     { get; set; }

            //the turning point lies outside the previous box — an approach lined up with the portal from behind it — and is
            //not moved by the refinement, which keeps a turning point inside its box
            public bool          ViaFixed { get; set; }
        }

        #endregion

        #region Constants

        //the node id of the goal in the open list
        const int BOX_GOAL = -1;

        //a portal point is kept this share of the pitch inside the rectangle's edges, off the lines where a box meets an
        //obstacle; a leg lying in one face of its box is pulled this share of the pitch inside it
        const float BOX_PORTAL_INSET = 0.05f;

        //at a radius a portal is shrunk by the radius and this share of the pitch more: a point exactly a radius off two walls
        //is contact with both, and a leg leaving it can read as touching one of them
        const float BOX_RADIUS_MARGIN = 0.05f;
        const float BOX_FACE_INSET   = 0.1f;

        //a body's route over boxes longer than this multiple of the straight line is searched once more with a node per box
        //AND entry, the shorter delivered — the leaf search's detour escalation, at box granularity
        const float BOX_DETOUR_ESCALATION = 1.5f;

        //a body's search keeps a node per box AND entry for a box whose largest side is at least BOX_ENTRY_NODE_SPAN radii (or
        //pitches, the larger), and for one of BOX_ENTRY_NODE_SMALL_SPAN or more with at most BOX_ENTRY_NODE_MAX_PORTALS inner
        //portals: one entry point prices a large box, and the search then picks a neighbouring, longer corridor (ROADMAP P54).
        //A box with many portals is what makes a node per entry expensive — at a plain 48 a city scene's box search ran slower
        //than the leaf search
        const float BOX_ENTRY_NODE_SPAN        = 64f;
        const float BOX_ENTRY_NODE_SMALL_SPAN  = 56f;
        const int   BOX_ENTRY_NODE_MAX_PORTALS = 8;

        const int   BOX_REFINE_SWEEPS    = 64;
        const float BOX_REFINE_TOLERANCE = 0.01f;

        static readonly Vector3Int[] s_BoxSearchAxes = { Vector3Int.right, Vector3Int.up, new Vector3Int(0, 0, 1) };

        #endregion

        #region Attributes (static)

        //the dev A/B arm, on by default: off, the point lane searches the leaves as before
        internal static bool BoxSearchEnabled { get; set; } = true;

        #endregion

        #region Service methods — the box search

        //Where the route enters the box graph: the start itself when it lies in a box — a free leaf, or the void inside a
        //root, which GetClosestOrEmbracingLeaf reports as no leaf at all; the free point it was moved to (it lies in _StartLeaf); and for a
        //start outside every root, the free leaf the straight line toward the goal enters first, exactly as the leaf search
        //seeds. false = no anchor, and the leaf search takes the request (it has one more recovery to try).
        bool TryAnchorInBoxes(Vector3 _Start, Leaf _StartLeaf, Vector3 _GoalPoint, out Vector3 _Anchor)
        {
            //a start inside an obstacle has already been moved to the nearest free point, which lies in _StartLeaf
            if (_StartLeaf != null)
            {
                _Anchor = _Start;

                return true;
            }

            if (m_NavTree.TryLocateBox(_Start, out _, out _))
            {
                _Anchor = _Start;

                return true;
            }

            Leaf entry = m_NavTree.RayMarchEntryLeaf(_Start, _GoalPoint);

            _Anchor = default;

            return entry != null && !entry.Occupied && TryShotEntryPoint(_Start, _GoalPoint, entry, 0f, out _Anchor);
        }

        //Where a blocked straight line from _From toward _Goal is carried into the region it meets: the middle of the line's
        //run through the entry leaf, which lies on the line itself, else the leaf's centre, which does not — whichever the
        //body reaches from _From along a clear segment or capsule; false when neither. The march finds the leaf by sampling
        //the line, so the hop to it was never tested, and a hop to the centre can clip what the line passed.
        bool TryShotEntryPoint(Vector3 _From, Vector3 _Goal, Leaf _Entry, float _Radius, out Vector3 _Point)
        {
            Bounds  leaf      = _Entry.Bounds;
            Vector3 direction = _Goal - _From;
            float   enter     = 0f, exit = 1f;

            for (int axis = 0; axis < 3 && enter <= exit; axis++)
            {
                if (Mathf.Abs(direction[axis]) < 1e-9f)
                {
                    if (_From[axis] < leaf.min[axis] || _From[axis] > leaf.max[axis])
                        enter = 2f;

                    continue;
                }

                float t0 = (leaf.min[axis] - _From[axis]) / direction[axis];
                float t1 = (leaf.max[axis] - _From[axis]) / direction[axis];

                enter = Mathf.Max(enter, Mathf.Min(t0, t1));
                exit  = Mathf.Min(exit,  Mathf.Max(t0, t1));
            }

            if (enter <= exit)
            {
                _Point = _From + direction * (0.5f * (enter + exit));

                if (!HopBlocked(_From, _Point, _Radius))
                    return true;
            }

            _Point = leaf.center;

            return !HopBlocked(_From, _Point, _Radius);
        }

        bool HopBlocked(Vector3 _From, Vector3 _To, float _Radius)
        {
            if (_Radius <= 0f)
                return m_NavTree.SegmentIntersectOccupiedLeafInternal(new Segment3(_From, _To));

            bool blocked = false;

            BoxLegBlocked(_From, _To, _Radius, ref blocked);

            return blocked;
        }

        //a straight segment of a box route: into an authored sealed volume (inflated by the radius, as the leaf search's Expand
        //does), or — at a radius — through occupied space by the capsule
        bool SegmentBlocked(Vector3 _From, Vector3 _To, float _Radius, SealedVolumeSnapshot _Sealed)
        {
            if (!_Sealed.IsEmpty && _Sealed.SegmentEnters(_From, _To, _Radius))
                return true;

            bool blocked = false;

            if (_Radius > 0f)
                BoxLegBlocked(_From, _To, _Radius, ref blocked);

            return blocked;
        }

        //false = the boxes cannot take this request (the start lies outside every root, or its box is not found) and the
        //caller searches the leaves; true = _Result is the answer, a route or a refusal. At a _Radius above zero every leg
        //is tested by capsule, since the body reaches out of the box it travels in, and a portal is crossed only where the
        //rectangle shrunk by the radius is not empty; the search is then no longer exhaustive — a box closes with one entry
        //point, and another might have let the body through — so its refusal is not final and the leaf search takes it.
        bool TryFindPathOverBoxes(
                Vector3                                              _Start,
                Vector3                                              _Anchor,
                Vector3                                              _GoalPoint,
                Vector3                                              _Target,
                float                                                _Radius,
                CancellationToken                                    _CancellationTokenExternal,
                CancellationToken                                    _CancellationTokenTimeout,
                out (List<Vector3> Path, PathfindingResultCode Code) _Result,
                Action<List<(BoxRootRecord Record, int Box)>>        _OnCorridor = null,
                bool                                                 _PerEntry   = false
            )
        {
            _Result = default;

            bool   volumetric = _Radius > 0f;
            string lane       = volumetric ? "box vol" : "box";

            if (!m_NavTree.TryLocateBox(_Anchor, out BoxRootRecord startRecord, out int startBox))
                return false;

            m_NavTree.TryLocateBox(_GoalPoint, out BoxRootRecord goalRecord, out int goalBox);

            List<BoxNode>                               nodes         = new List<BoxNode>();
            Dictionary<(Vector3Int, int, Vector3), int> ids           = new Dictionary<(Vector3Int, int, Vector3), int>();
            MinHeap<BoxHeapEntry>                       open          = new MinHeap<BoxHeapEntry>();
            SealedVolumeSnapshot                        sealedVolumes = SealedVolumeSnapshot.Current;

            //A node is a box. A body closed into a box at one entry may find every way on blocked where another entry leaves
            //freely; a node per box AND entry answers that, at six times the nodes, so it runs for every box only as the detour
            //escalation below, and otherwise for the large boxes alone (BOX_ENTRY_NODE_SPAN and the rest); the turns through a box
            //(TryPortalPointVolumetric) answer most of the same case at one node a box
            int IdOf(BoxRootRecord _Record, int _Box, Vector3 _Entry)
            {
                bool perEntry = volumetric && (_PerEntry || LargeBox(_Record, _Box));

                (Vector3Int, int, Vector3) key = (_Record.RootIndex, _Box, perEntry ? _Entry : Vector3.zero);

                if (ids.TryGetValue(key, out int id))
                    return id;

                id = nodes.Count;

                nodes.Add(new BoxNode { Record = _Record, Box = _Box });
                ids.Add(key, id);

                return id;
            }

            bool LargeBox(BoxRootRecord _Record, int _Box)
            {
                Vector3 size  = _Record.WorldBounds(_Box).size;
                float   side  = Mathf.Max(size.x, Mathf.Max(size.y, size.z));
                float   scale = Mathf.Max(_Radius, _Record.Pitch);

                return side >= BOX_ENTRY_NODE_SPAN * scale ||
                       side >= BOX_ENTRY_NODE_SMALL_SPAN * scale && _Record.PortalsOfBox[_Box].Length <= BOX_ENTRY_NODE_MAX_PORTALS;
            }

            float    goalG        = float.PositiveInfinity;
            int      goalParent   = -1;
            bool     goalStraight = false;
            Vector3? goalVia      = null;
            Vector3? goalVia2     = null;
            int   expansions = 0;

            int start = IdOf(startRecord, startBox, _Anchor);

            nodes[start].G  = 0f;
            nodes[start].At = _Anchor;

            open.Add(new BoxHeapEntry(Vector3.Distance(_Anchor, _GoalPoint), start));

            void OfferGoal(int _From, float _Cost, bool _Straight, Vector3? _Via = null, Vector3? _Via2 = null)
            {
                if (_Cost >= goalG)
                    return;

                goalG        = _Cost;
                goalParent   = _From;
                goalStraight = _Straight;
                goalVia      = _Via;
                goalVia2     = _Via2;

                open.Add(new BoxHeapEntry(_Cost, BOX_GOAL));
            }

            void Relax(int _From, BoxRootRecord _Record, int _Box, Bounds _Portal, int _Axis, bool _Straight, Vector3 _StraightPoint, Vector3? _Via = null)
            {
                Vector3? via2     = null;
                bool     viaFixed = false;

                int     id    = IdOf(_Record, _Box, _Straight ? _StraightPoint : _Portal.center);
                BoxNode node  = nodes[id];
                BoxNode from  = nodes[_From];

                if (node.Closed)
                    return;

                Vector3 point;

                if (_Straight)
                    point = _StraightPoint;
                else if (!volumetric)
                    point = PortalPoint(Inset(_Portal, _Record.Pitch * BOX_PORTAL_INSET), from.At, _GoalPoint);
                else if (!TryPortalPointVolumetric(_Portal, _Axis, _Radius, _Record.Pitch, from.At, _GoalPoint, from.Record.WorldBounds(from.Box), sealedVolumes, out point, out _Via,
                                                   out via2, out viaFixed))
                    return;

                //a box is free, but a sealed volume's partly covered cells are free too: a point's leg is tested against the volumes
                //(a body's legs were, with its radius, among the candidates)
                if (!volumetric && !_Straight && !sealedVolumes.IsEmpty && sealedVolumes.SegmentEnters(from.At, point))
                    return;

                float cost = !_Via.HasValue ? from.G + Vector3.Distance(from.At, point)
                             : !via2.HasValue ? from.G + Vector3.Distance(from.At, _Via.Value) + Vector3.Distance(_Via.Value, point)
                             : from.G + Vector3.Distance(from.At, _Via.Value) + Vector3.Distance(_Via.Value, via2.Value) + Vector3.Distance(via2.Value, point);

                if (cost >= node.G)
                    return;

                node.G        = cost;
                node.At       = point;
                node.Parent   = _From;
                node.Straight = _Straight;
                node.Portal   = _Portal;
                node.Axis     = _Axis;
                node.Via      = _Via;
                node.Via2     = via2;
                node.ViaFixed = viaFixed;

                open.Add(new BoxHeapEntry(cost + Vector3.Distance(point, _GoalPoint), id));
            }

            while (open.Count > 0)
            {
                if (_CancellationTokenTimeout.IsCancellationRequested)
                {
                    _Result = (null, PathfindingResultCode.TIMEOUT);

                    return true;
                }

                if (_CancellationTokenExternal.IsCancellationRequested)
                {
                    _Result = (null, PathfindingResultCode.CANCELLED);

                    return true;
                }

                BoxHeapEntry entry = open.PopMin();

                if (entry.Node == BOX_GOAL)
                {
                    m_NavTree.DevNoteSearchOutcomeHook(lane, PathfindingResultCode.SUCCEEDED, false, expansions);

                    if (volumetric && !_PerEntry)
                        DevNoteBoxRouted(nodes);

                    _Result = (BuildBoxWaypoints(nodes, goalParent, goalStraight, goalVia, goalVia2, _Start, _Anchor, _GoalPoint, _Target, _Radius, sealedVolumes), PathfindingResultCode.SUCCEEDED);

                    if (_OnCorridor != null)
                    {
                        List<(BoxRootRecord, int)> corridor = new List<(BoxRootRecord, int)>();

                        for (int node = goalParent; node >= 0; node = nodes[node].Parent)
                            corridor.Add((nodes[node].Record, nodes[node].Box));

                        _OnCorridor(corridor);
                    }

                    if (volumetric && !_PerEntry && Length(_Result.Path) > BOX_DETOUR_ESCALATION * Vector3.Distance(_Start, _GoalPoint) &&
                        TryFindPathOverBoxes(_Start, _Anchor, _GoalPoint, _Target, _Radius, _CancellationTokenExternal, _CancellationTokenTimeout,
                                             out (List<Vector3> Path, PathfindingResultCode Code) escalated, _OnCorridor, true) &&
                        escalated.Code == PathfindingResultCode.SUCCEEDED && Length(escalated.Path) < Length(_Result.Path))
                        _Result = escalated;

                    return true;
                }

                BoxNode current = nodes[entry.Node];

                if (current.Closed)
                    continue;

                current.Closed = true;
                expansions++;

                BoxRootRecord record = current.Record;
                BoxCells      cells  = record.Boxes[current.Box];

                //arrival: the goal lies in this box, which is convex and free; a body may also make the last hop from a
                //box within two radii of it, as the leaf search lets a fitting neighbour of the goal's leaf make it
                bool goalHere = goalRecord != null && ReferenceEquals(record, goalRecord) && current.Box == goalBox;

                if (goalHere || volumetric && (record.WorldBounds(current.Box).ClosestPoint(_GoalPoint) - _GoalPoint).sqrMagnitude <= 4f * _Radius * _Radius)
                {
                    if (!LegBlocked(current.At, _GoalPoint))
                    {
                        OfferGoal(entry.Node, current.G + Vector3.Distance(current.At, _GoalPoint), false);
                    }
                    else if (volumetric && TryArrivalTurn(current, out Vector3 turn, out Vector3? turn2))
                    {
                        float cost = !turn2.HasValue
                                ? Vector3.Distance(current.At, turn) + Vector3.Distance(turn, _GoalPoint)
                                : Vector3.Distance(current.At, turn) + Vector3.Distance(turn, turn2.Value) + Vector3.Distance(turn2.Value, _GoalPoint);

                        OfferGoal(entry.Node, current.G + cost, false, turn, turn2);
                    }
                }

                foreach (int portalIndex in record.PortalsOfBox[current.Box])
                {
                    BoxPortal portal = record.InnerPortals[portalIndex];

                    Relax(entry.Node, record, portal.A == current.Box ? portal.B : portal.A, portal.Rect, portal.Axis, false, default);
                }

                int shotFace = DominantFaceBit(_GoalPoint - current.At);

                for (int axis = 0; axis < 3; axis++)
                {
                    if (cells.Max[axis] == record.Cells)
                    {
                        BoxRootRecord upper = m_NavTree.BoxRecordOf(record.RootIndex + s_BoxSearchAxes[axis]);

                        if (upper == null)
                            TryShoot(entry.Node, axis, true, shotFace == axis * 2);
                        else
                        {
                            foreach (BoxPortal portal in m_NavTree.BoxFaceTable(record.RootIndex, axis) ?? Array.Empty<BoxPortal>())
                            {
                                if (portal.A == current.Box)
                                    Relax(entry.Node, upper, portal.B, portal.Rect, portal.Axis, false, default);
                            }
                        }
                    }

                    if (cells.Min[axis] == 0)
                    {
                        Vector3Int    lowerIndex = record.RootIndex - s_BoxSearchAxes[axis];
                        BoxRootRecord lower      = m_NavTree.BoxRecordOf(lowerIndex);

                        if (lower == null)
                            TryShoot(entry.Node, axis, false, shotFace == axis * 2 + 1);
                        else
                        {
                            foreach (BoxPortal portal in m_NavTree.BoxFaceTable(lowerIndex, axis) ?? Array.Empty<BoxPortal>())
                            {
                                if (portal.B == current.Box)
                                    Relax(entry.Node, lower, portal.A, portal.Rect, portal.Axis, false, default);
                            }
                        }
                    }
                }
            }

            m_NavTree.DevNoteSearchOutcomeHook(lane, PathfindingResultCode.PATH_DOES_NOT_EXIST, false, expansions);

            if (!volumetric)
            {
                _Result = (null, PathfindingResultCode.PATH_DOES_NOT_EXIST);

                return true;
            }

            //At a radius the refusal proves nothing, but the point search's does: a body's route is a point's route too, so
            //when not even a point reaches the goal from a start lying in a box, no body does — and this is the request that
            //would flood its whole component on the leaves. Otherwise the leaf search takes it.
            //and when the point search routes, its corridor is where the leaf search is confined to first (the caller's)
            if (_Anchor == _Start &&
                TryFindPathOverBoxes(_Start, _Anchor, _GoalPoint, _Target, 0f, _CancellationTokenExternal, _CancellationTokenTimeout,
                                     out (List<Vector3> Path, PathfindingResultCode Code) point, _OnCorridor) &&
                point.Code != PathfindingResultCode.SUCCEEDED)
            {
                _Result = point;

                return true;
            }

            return false;

            bool LegBlocked(Vector3 _A, Vector3 _B) => SegmentBlocked(_A, _B, _Radius, sealedVolumes);

            //A body its entry point cannot carry straight to the goal may turn inside the box first, as the leaf search's last
            //hop leaves from anywhere in the region and not from one point: at the point of the box, shrunk by the radius, nearest
            //the goal, or at its centre; else through two points, the first the entry carried off the face it lies on to the
            //box's middle (or the centre), the second that nearest point — an entry on a face with geometry beyond it reaches
            //nothing near the goal in one leg, and without a turn the route goes round to another box to arrive from.
            bool TryArrivalTurn(BoxNode _Box, out Vector3 _Turn, out Vector3? _Turn2)
            {
                Bounds  box    = _Box.Record.WorldBounds(_Box.Box);
                Vector3 margin = Vector3.one * (_Radius + _Box.Record.Pitch * BOX_PORTAL_INSET);
                Vector3 min    = Vector3.Min(box.min + margin, box.center);
                Vector3 max    = Vector3.Max(box.max - margin, box.center);
                Vector3 near   = Vector3.Min(Vector3.Max(_GoalPoint, min), max);
                Vector3 lifted = _Box.At;

                for (int axis = 0; axis < 3; axis++)
                {
                    if (Mathf.Abs(lifted[axis] - box.min[axis]) < 1e-3f || Mathf.Abs(lifted[axis] - box.max[axis]) < 1e-3f)
                        lifted[axis] = box.center[axis];
                }

                _Turn2 = null;

                bool Fits(Vector3 _P)
                {
                    bool fits = true;

                    BoxPointFits(_P, _Radius, ref fits);

                    return fits;
                }

                foreach (Vector3 turn in new[] { near, box.center })
                {
                    if (Fits(turn) && !LegBlocked(_Box.At, turn) && !LegBlocked(turn, _GoalPoint))
                    {
                        _Turn = turn;

                        return true;
                    }
                }

                if (Fits(near) && !LegBlocked(near, _GoalPoint))
                {
                    foreach (Vector3 first in new[] { lifted, box.center })
                    {
                        if (Fits(first) && !LegBlocked(_Box.At, first) && !LegBlocked(first, near))
                        {
                            _Turn  = first;
                            _Turn2 = near;

                            return true;
                        }
                    }
                }

                _Turn = default;

                return false;
            }

            //The free-space edge: from a box lying on a root face with no root behind it, through that face when the goal lies
            //beyond it, from the point of the face the detour to the goal is shortest through (the leg to it stays in the box,
            //which is convex and free). A clear shot reaches the goal; when that face is also the one the direction to the goal
            //leaves through, a blocked shot enters the region it meets. Every face the goal lies beyond shoots, not only the
            //dominant one: a straight line leaves a cube through whichever face it meets, and a shot through the dominant face
            //alone misses routes the leaf search finds from the leaf the line starts in.
            void TryShoot(int _From, int _Axis, bool _Upper, bool _Dominant)
            {
                BoxNode from = nodes[_From];
                Bounds  box  = from.Record.WorldBounds(from.Box);
                float   side = _Upper ? box.max[_Axis] : box.min[_Axis];

                if ((_Upper ? _GoalPoint[_Axis] - side : side - _GoalPoint[_Axis]) <= 0f)
                    return;

                Vector3 min = box.min, max = box.max;

                min[_Axis] = max[_Axis] = side;

                Bounds face = volumetric
                                  ? PortalArea(new Bounds((min + max) * 0.5f, max - min), _Axis, _Radius, from.Record.Pitch, out bool _)
                                  : Inset(new Bounds((min + max) * 0.5f, max - min), from.Record.Pitch * BOX_PORTAL_INSET);

                Vector3 via = TautPoint(face, _Axis, from.At, _GoalPoint);

                //never through an authored sealed volume, as in the leaf search's Expand: neither the leg to the shot's point
                //nor the shot itself, whose blocked form also carries the route into the region it meets
                if (sealedVolumes.SegmentEnters(from.At, via, _Radius) || sealedVolumes.SegmentEnters(via, _GoalPoint, _Radius))
                    return;

                if (volumetric && LegBlocked(from.At, via))
                    return;

                float toVia = Vector3.Distance(from.At, via);

                if (volumetric ? !LegBlocked(via, _GoalPoint) : !m_NavTree.SegmentIntersectOccupiedLeafInternal(new Segment3(via, _GoalPoint)))
                {
                    OfferGoal(_From, from.G + toVia + Vector3.Distance(via, _GoalPoint), true, via);

                    return;
                }

                if (!_Dominant)
                    return;

                Leaf entryLeaf = null;

                if (volumetric)
                    BoxShotEntry(via, _GoalPoint, _Radius, ref entryLeaf);
                else
                    entryLeaf = m_NavTree.RayMarchEntryLeaf(via, _GoalPoint);

                if (entryLeaf == null || entryLeaf.Occupied || !TryShotEntryPoint(via, _GoalPoint, entryLeaf, _Radius, out Vector3 entryPoint))
                    return;

                if (m_NavTree.TryLocateBox(entryPoint, out BoxRootRecord entryRecord, out int entryBox) && !(ReferenceEquals(entryRecord, from.Record) && entryBox == from.Box))
                    Relax(_From, entryRecord, entryBox, default, 0, true, entryPoint, via);
            }
        }

        //One point of a route as it is refined and emitted: where it is, whether the refinement may move it (a point on a
        //portal) and over which rectangle, and the box the leg leaving it runs through (null: a straight shot, which leaves
        //every box).
        struct BoxRoutePoint
        {
            public Vector3 Point;
            public BoxNode Portal;
            public BoxNode LegBox;

            //a point a body turned a bend through: movable inside its box, each move tested
            public BoxNode TurnBox;
        }

        //the chain from the start box to the goal, pulled taut through the portals it crossed, a leg lying in one face of
        //its box pulled inside it; the start, the goal point and the target framing it as the leaf search frames its route.
        //A point the search routed through on the way to the next portal — a shot's point on its box's outer face, or a
        //box's centre a body turned a corner through — stays where the search tested it; every leg to or from it lies in
        //one convex box. At a radius the pulled route keeps its points r inside their rectangles and is tested leg by leg:
        //the body is not held by the boxes, so a leg the pull moved may graze what the search's own legs cleared, and then
        //the route the search tested is delivered as it was found. No leg is pulled off a face then — each was tested
        //where it lies.
        List<Vector3> BuildBoxWaypoints(List<BoxNode> _Nodes, int _Last, bool _GoalStraight, Vector3? _GoalVia, Vector3? _GoalVia2, Vector3 _Start, Vector3 _Anchor,
                Vector3 _GoalPoint, Vector3 _Target, float _Radius, SealedVolumeSnapshot _Sealed)
        {
            List<int> chain = new List<int>();

            for (int node = _Last; node >= 0; node = _Nodes[node].Parent)
                chain.Add(node);

            chain.Reverse();

            List<BoxRoutePoint> route = new List<BoxRoutePoint> { new BoxRoutePoint { Point = _Anchor, LegBox = _Nodes[chain[0]] } };

            void Enter(Vector3? _Via, bool _Straight, Vector3 _At, BoxNode _Node, BoxNode _Next, bool _ViaFixed = false)
            {
                BoxRoutePoint last = route[route.Count - 1];

                if (_Via.HasValue)
                {
                    //a shot's point on the face (the leg after it leaves the box), a body's turning point inside the box, or its
                    //lined-up approach, which may lie outside it (no leg pulled off a face, nothing moved)
                    bool inBox = !_Straight && !_ViaFixed;

                    route.Add(new BoxRoutePoint { Point = _Via.Value, LegBox = inBox ? last.LegBox : null, TurnBox = inBox ? last.LegBox : null });
                }
                else if (_Straight)
                {
                    last.LegBox           = null;
                    route[route.Count - 1] = last;
                }

                route.Add(new BoxRoutePoint { Point = _At, Portal = _Straight ? null : _Node, LegBox = _Next });
            }

            for (int i = 1; i < chain.Count; i++)
            {
                BoxNode node = _Nodes[chain[i]];

                //a second turning point sits inside the previous box, after the first
                if (node.Via2.HasValue)
                    route.Add(new BoxRoutePoint { Point = node.Via.Value, LegBox = route[route.Count - 1].LegBox, TurnBox = route[route.Count - 1].LegBox });

                Enter(node.Via2 ?? node.Via, node.Straight, node.At, node, node, node.ViaFixed);
            }

            //an arrival's second turning point, like a node's, sits after the first inside the same box
            if (_GoalVia2.HasValue)
                route.Add(new BoxRoutePoint { Point = _GoalVia.Value, LegBox = route[route.Count - 1].LegBox, TurnBox = route[route.Count - 1].LegBox });

            Enter(_GoalVia2 ?? _GoalVia, _GoalStraight, _GoalPoint, null, null);

            //the pull moves points off the ones the search tested: a body's legs, and a point's where a sealed volume is near
            bool checkPulled = _Radius > 0f || !_Sealed.IsEmpty;

            List<BoxRoutePoint> found = checkPulled ? new List<BoxRoutePoint>(route) : null;

            RefineThroughPortals(route, _Radius);

            //a moved turning point moves the portal points' taut places, and back: two rounds
            for (int round = 0; _Radius > 0f && round < 2; round++)
            {
                RefineTurns(route, _Radius, _Sealed);
                RefineThroughPortals(route, _Radius);
            }

            if (checkPulled)
            {
                for (int i = 0; i + 1 < route.Count; i++)
                {
                    bool blocked = SegmentBlocked(route[i].Point, route[i + 1].Point, _Radius, _Sealed);

                    if (blocked)
                    {
                        route = found;

                        break;
                    }
                }
            }

            List<Vector3> path = new List<Vector3>(route.Count * 2 + 2);

            if ((_Anchor - _Start).sqrMagnitude > PATHFINDING_EPSILON)
                path.Add(_Start);

            path.Add(route[0].Point);

            for (int i = 0; i + 1 < route.Count; i++)
            {
                Vector3 from = route[i].Point, to = route[i + 1].Point;
                BoxNode leg  = route[i].LegBox;

                if (leg != null && _Radius <= 0f)
                {
                    Bounds  box   = leg.Record.WorldBounds(leg.Box);
                    float   inset = leg.Record.Pitch * BOX_FACE_INSET;
                    Vector3 shift = Vector3.zero;

                    for (int axis = 0; axis < 3; axis++)
                    {
                        if (Mathf.Abs(from[axis] - box.min[axis]) < 1e-4f && Mathf.Abs(to[axis] - box.min[axis]) < 1e-4f)
                            shift[axis] = inset;
                        else if (Mathf.Abs(from[axis] - box.max[axis]) < 1e-4f && Mathf.Abs(to[axis] - box.max[axis]) < 1e-4f)
                            shift[axis] = -inset;
                    }

                    if (shift != Vector3.zero)
                    {
                        path.Add(from + shift);
                        path.Add(to + shift);
                    }
                }

                if ((to - path[path.Count - 1]).sqrMagnitude > PATHFINDING_EPSILON || i + 2 == route.Count)
                    path.Add(to);
            }

            if (_GoalPoint != _Target)
                path.Add(_Target);

            return path;
        }

        //A body's turning points, each moved toward the shortest detour between its neighbours inside its box shrunk by the
        //radius, a move kept only when both its legs stay clear: the portal refinement cannot move them, and left at a box's
        //centre they cost length the pulled route then has to carry
        void RefineTurns(List<BoxRoutePoint> _Route, float _Radius, SealedVolumeSnapshot _Sealed)
        {
            for (int sweep = 0; sweep < 4; sweep++)
            {
                bool moved = false;

                for (int i = 1; i + 1 < _Route.Count; i++)
                {
                    BoxRoutePoint point = _Route[i];

                    if (point.TurnBox == null)
                        continue;

                    Bounds  core = point.TurnBox.Record.WorldBounds(point.TurnBox.Box);
                    Vector3 min  = core.min, max = core.max;

                    for (int axis = 0; axis < 3; axis++)
                    {
                        float shrink = Mathf.Min(_Radius, 0.5f * (max[axis] - min[axis]));

                        min[axis] += shrink;
                        max[axis] -= shrink;
                    }

                    Vector3 a = _Route[i - 1].Point, b = _Route[i + 1].Point;

                    Vector3 Clamp(Vector3 _P) => Vector3.Min(Vector3.Max(_P, min), max);

                    float Detour(float _T)
                    {
                        Vector3 x = Clamp(Vector3.Lerp(a, b, _T));

                        return Vector3.Distance(a, x) + Vector3.Distance(x, b);
                    }

                    float lo = 0f, hi = 1f;

                    for (int step = 0; step < 24; step++)
                    {
                        float m1 = lo + (hi - lo) / 3f, m2 = hi - (hi - lo) / 3f;

                        if (Detour(m1) <= Detour(m2))
                            hi = m2;
                        else
                            lo = m1;
                    }

                    Vector3 target = Clamp(Vector3.Lerp(a, b, 0.5f * (lo + hi)));

                    //toward the taut point as far as the body lets it: the whole way, else half of it, else a quarter
                    for (float share = 1f; share > 0.2f; share *= 0.5f)
                    {
                        Vector3 next = Vector3.Lerp(point.Point, target, share);

                        if ((next - point.Point).sqrMagnitude < PATHFINDING_EPSILON ||
                            Vector3.Distance(a, next) + Vector3.Distance(next, b) >= Vector3.Distance(a, point.Point) + Vector3.Distance(point.Point, b))
                            break;

                        if (SegmentBlocked(a, next, _Radius, _Sealed) || SegmentBlocked(next, b, _Radius, _Sealed))
                            continue;

                        point.Point = next;
                        _Route[i]   = point;
                        moved       = true;

                        break;
                    }
                }

                if (!moved)
                    break;
            }
        }

        //Gauss-Seidel over the points on portals, alternating direction, each moved to the exact minimum of its rectangle
        //between its two neighbours: the length of a route through a fixed sequence of convex sets is convex, so the sweeps
        //reach the shortest route through the corridor. The other points stay where they are.
        static void RefineThroughPortals(List<BoxRoutePoint> _Route, float _Radius)
        {
            int last = _Route.Count - 2;

            if (last < 1)
                return;

            float tolerance = BOX_REFINE_TOLERANCE * (_Route[0].LegBox?.Record.Pitch ?? 1f);

            for (int sweep = 0; sweep < BOX_REFINE_SWEEPS; sweep++)
            {
                float moved = 0f;

                for (int step = 0; step < last; step++)
                {
                    int           i     = sweep % 2 == 0 ? step + 1 : last - step;
                    BoxRoutePoint point = _Route[i];
                    BoxNode       node  = point.Portal;

                    if (node == null)
                        continue;

                    Bounds area = _Radius > 0f
                                      ? PortalArea(node.Portal, node.Axis, _Radius, node.Record.Pitch, out bool _)
                                      : Inset(node.Portal, node.Record.Pitch * BOX_PORTAL_INSET);

                    Vector3 next = TautPoint(area, node.Axis, _Route[i - 1].Point, _Route[i + 1].Point);

                    moved       = Mathf.Max(moved, (next - point.Point).magnitude);
                    point.Point = next;
                    _Route[i]   = point;
                }

                if (moved < tolerance)
                    break;
            }
        }

        //the point of the (already inset) portal rectangle that makes the detour from _From to _Goal shortest of three
        //clamps — toward the goal, toward the current point, toward their midpoint
        static Vector3 PortalPoint(Bounds _Rect, Vector3 _From, Vector3 _Goal)
        {
            Vector3 min = _Rect.min, max = _Rect.max;

            Vector3 Clamp(Vector3 _Point) => Vector3.Min(Vector3.Max(_Point, min), max);

            Vector3 best     = Clamp(_Goal);
            float   bestCost = Vector3.Distance(_From, best) + Vector3.Distance(best, _Goal);

            void Consider(Vector3 _Candidate)
            {
                float cost = Vector3.Distance(_From, _Candidate) + Vector3.Distance(_Candidate, _Goal);

                if (cost < bestCost)
                {
                    best     = _Candidate;
                    bestCost = cost;
                }
            }

            Consider(Clamp(_From));
            Consider(Clamp(0.5f * (_From + _Goal)));

            return best;
        }

        //At a radius: where a body crosses the portal from _From, the capsule deciding. The candidates are the rectangle's (shrunk by
        //the radius) exact point of the shortest detour from _From to the goal, its three clamps and its centre; then — only when none of them is reachable, or the rectangle is
        //narrower than the body — its 3 x 3 grid of corners and edge middles, as the leaf search samples a face: on a narrow
        //rectangle the rim is where an aperture's axis runs (an exact-fit hole is tiled by two or more boxes, each portal of
        //it narrower than the body), and a body flying along the plane two roots share has its axis on the rim itself. Last,
        //turns inside the box being left: the entry point may hug the wall a bend turns around — chosen toward the goal, it
        //sits a radius off the inner wall, and no straight leg from it clears the corner.
        bool TryPortalPointVolumetric(Bounds _Portal, int _Axis, float _Radius, float _Pitch, Vector3 _From, Vector3 _Goal, Bounds _Box, SealedVolumeSnapshot _Sealed, out Vector3 _Point, out Vector3? _Via,
                out Vector3? _Via2, out bool _ViaFixed)
        {
            _Point    = default;
            _Via      = null;
            _Via2     = null;
            _ViaFixed = false;

            Bounds area = PortalArea(_Portal, _Axis, _Radius, _Pitch, out _);

            Vector3 min = area.min, max = area.max;

            Vector3 Clamp(Vector3 _P) => Vector3.Min(Vector3.Max(_P, min), max);

            float Detour(Vector3 _P) => Vector3.Distance(_From, _P) + Vector3.Distance(_P, _Goal);

            //the exact point first: entered at a clamp, a large box's onward legs are priced from a point the shortest detour does not
            //pass through, and the search closed it on that price and took a neighbouring, longer corridor (the differential harness,
            //TIGHT and SPARSE; on their failing pairs 9 length differences became 6)
            List<Vector3> first = new List<Vector3> { TautPoint(area, _Axis, _From, _Goal), Clamp(_Goal), Clamp(_From), Clamp(0.5f * (_From + _Goal)), area.center };
            List<Vector3> grid  = new List<Vector3>(8);

            int u = (_Axis + 1) % 3, v = (_Axis + 2) % 3;

            for (int a = 0; a < 3; a++)
            for (int b = 0; b < 3; b++)
            {
                if (a == 1 && b == 1)
                    continue;

                Vector3 sample = area.center;

                sample[u] = Mathf.Lerp(min[u], max[u], a * 0.5f);
                sample[v] = Mathf.Lerp(min[v], max[v], b * 0.5f);

                grid.Add(sample);
            }

            first.Sort((_A, _B) => Detour(_A).CompareTo(Detour(_B)));
            grid.Sort((_A, _B) => Detour(_A).CompareTo(Detour(_B)));

            //a point the body does not fit at is no candidate and no turning point: one descent each, remembered, in front of
            //the capsule tests a failing relaxation would otherwise pay for every pair of them
            Dictionary<Vector3, bool> fits = new Dictionary<Vector3, bool>();

            bool Fits(Vector3 _P)
            {
                if (fits.TryGetValue(_P, out bool fit))
                    return fit;

                fit = true;

                BoxPointFits(_P, _Radius, ref fit);

                fits[_P] = fit;

                return fit;
            }

            bool LegBlocked(Vector3 _A, Vector3 _B) => !Fits(_B) || SegmentBlocked(_A, _B, _Radius, _Sealed);

            bool TryDirect(List<Vector3> _Candidates, out Vector3 _Found)
            {
                for (int i = 0; i < _Candidates.Count; i++)
                {
                    if (i > 0 && (_Candidates[i] - _Candidates[i - 1]).sqrMagnitude < PATHFINDING_EPSILON)
                        continue;

                    if (!LegBlocked(_From, _Candidates[i]))
                    {
                        _Found = _Candidates[i];

                        return true;
                    }
                }

                _Found = default;

                return false;
            }

            if (TryDirect(first, out _Point) || TryDirect(grid, out _Point))
            {
                return true;
            }

            List<Vector3> candidates = new List<Vector3>(first);

            candidates.AddRange(grid);

            //a turn through a point of the box, which the box, convex, holds and the route keeps: its centre, or the candidate
            //carried to the box's far side along the portal's axis, or to the box's middle along it — the corner of an L
            foreach (Vector3 candidate in candidates)
            {
                if (!Fits(candidate))
                    continue;

                Vector3 far = candidate, middle = candidate;

                far[_Axis]    = Mathf.Abs(_Box.min[_Axis] - candidate[_Axis]) > Mathf.Abs(_Box.max[_Axis] - candidate[_Axis]) ? _Box.min[_Axis] : _Box.max[_Axis];
                middle[_Axis] = _Box.center[_Axis];

                foreach (Vector3 via in new[] { _Box.center, far, middle })
                {
                    if (LegBlocked(_From, via) || LegBlocked(via, candidate))
                        continue;

                    _Point = candidate;
                    _Via   = via;

                    return true;
                }
            }

            //and a bend at each end of the box — a shaft entered from one side and left from the other: the entry carried to the
            //box's middle along the portal's axis, then the candidate carried there
            Vector3 entryMiddle = _From;

            entryMiddle[_Axis] = _Box.center[_Axis];

            if (!LegBlocked(_From, entryMiddle))
            {
                foreach (Vector3 candidate in candidates)
                {
                    if (!Fits(candidate))
                        continue;

                    Vector3 middle = candidate;

                    middle[_Axis] = _Box.center[_Axis];

                    if (LegBlocked(entryMiddle, middle) || LegBlocked(middle, candidate))
                        continue;

                    _Point = candidate;
                    _Via   = entryMiddle;
                    _Via2  = middle;

                    return true;
                }
            }

            //and an approach lined up with the portal: a point on the candidate's normal behind it, two, four or eight radii
            //back — an aperture the body fits exactly is entered only along its axis, and the box in front of it may be too thin
            //to line up in. The point may lie beyond this box; the capsule decides, and the refinement leaves it where it is.
            float side = Mathf.Sign(_From[_Axis] - _Portal.center[_Axis]);

            if (side != 0f)
            {
                foreach (Vector3 candidate in candidates)
                {
                    if (!Fits(candidate))
                        continue;

                    for (float back = 2f; back <= 8f; back *= 2f)
                    {
                        Vector3 approach = candidate;

                        approach[_Axis] += side * back * _Radius;

                        if (LegBlocked(_From, approach) || LegBlocked(approach, candidate))
                            continue;

                        _Point    = candidate;
                        _Via      = approach;
                        _ViaFixed = true;

                        return true;
                    }
                }
            }

            return false;
        }

        //the portal shrunk by _Radius in its plane; when the shrink leaves nothing (_Usable false: the rectangle is narrower
        //than the body) the whole rectangle is what is left to try. No inset off the rim at a radius: the shrink keeps a
        //body off it, and on a narrow portal the rim IS where an exact-fit aperture's axis runs
        static Bounds PortalArea(Bounds _Portal, int _Axis, float _Radius, float _Pitch, out bool _Usable)
        {
            float shrink = _Radius + _Pitch * BOX_RADIUS_MARGIN;

            Vector3 min = _Portal.min, max = _Portal.max;

            _Usable = true;

            for (int axis = 0; axis < 3; axis++)
            {
                if (axis != _Axis && max[axis] - min[axis] < 2f * shrink)
                    _Usable = false;
            }

            if (!_Usable)
                return _Portal;

            for (int axis = 0; axis < 3; axis++)
            {
                if (axis == _Axis)
                    continue;

                min[axis] += shrink;
                max[axis] -= shrink;
            }

            return new Bounds((min + max) * 0.5f, max - min);
        }

        //the point of an axis-aligned rectangle minimising |_A - x| + |x - _B|: where the segment from _A to _B (mirrored
        //through the rectangle's plane when both lie on one side) crosses the plane, if that is inside; otherwise the best
        //of its four edges
        static Vector3 TautPoint(Bounds _Rect, int _Axis, Vector3 _A, Vector3 _B)
        {
            Vector3 min    = _Rect.min, max = _Rect.max;
            int     axis   = _Axis;
            float   plane  = 0.5f * (min[axis] + max[axis]);
            Vector3 mirror = _B;
            float   da     = _A[axis] - plane;
            float   db     = _B[axis] - plane;

            if (da * db > 0f)
            {
                mirror[axis] = plane - db;
                db           = -db;
            }

            float   t     = Mathf.Abs(da - db) > 1e-6f ? da / (da - db) : 0.5f;
            Vector3 cross = _A + (mirror - _A) * t;

            cross[axis] = plane;

            int u = (axis + 1) % 3, v = (axis + 2) % 3;

            if (cross[u] >= min[u] && cross[u] <= max[u] && cross[v] >= min[v] && cross[v] <= max[v])
                return cross;

            Vector3 c00 = min, c11 = max;

            c00[axis] = plane;
            c11[axis] = plane;

            Vector3 c10 = c00, c01 = c00;

            c10[u] = c11[u];
            c01[v] = c11[v];

            Vector3 best     = cross;
            float   bestCost = float.PositiveInfinity;

            void Edge(Vector3 _P, Vector3 _Q)
            {
                Vector3 candidate = TautPointOnSegment(_P, _Q, _A, _B);
                float   cost      = Vector3.Distance(_A, candidate) + Vector3.Distance(candidate, _B);

                if (cost < bestCost)
                {
                    best     = candidate;
                    bestCost = cost;
                }
            }

            Edge(c00, c10);
            Edge(c00, c01);
            Edge(c10, c11);
            Edge(c01, c11);

            return best;
        }

        //unfold _B about the segment's line into _A's half-plane: the straight line between them meets the line at the optimum
        static Vector3 TautPointOnSegment(Vector3 _P, Vector3 _Q, Vector3 _A, Vector3 _B)
        {
            Vector3 d      = _Q - _P;
            float   length = d.magnitude;

            if (length < 1e-6f)
                return _P;

            d /= length;

            float ta = Vector3.Dot(_A - _P, d), tb = Vector3.Dot(_B - _P, d);
            float ra = (_A - _P - d * ta).magnitude, rb = (_B - _P - d * tb).magnitude;
            float t  = ra + rb > 1e-6f ? ta + (tb - ta) * ra / (ra + rb) : 0.5f * (ta + tb);

            return _P + d * Mathf.Clamp(t, 0f, length);
        }

        static Bounds Inset(Bounds _Rect, float _Inset)
        {
            Vector3 min = _Rect.min, max = _Rect.max;

            for (int axis = 0; axis < 3; axis++)
            {
                float inset = Mathf.Min(_Inset, 0.5f * (max[axis] - min[axis]));

                min[axis] += inset;
                max[axis] -= inset;
            }

            return new Bounds((min + max) * 0.5f, max - min);
        }

        #endregion

        #region Service methods — the corridor

        //The boxes a found route crossed and every box joined to one of them, per root as world bounds: where a search that
        //refines the route is let go. The ring is the room a body needs to cut a corner the route's own boxes do not hold.
        Dictionary<Vector3Int, List<Bounds>> CorridorBounds(HashSet<(BoxRootRecord Record, int Box)> _Boxes)
        {
            HashSet<(BoxRootRecord, int)> ring = new HashSet<(BoxRootRecord, int)>(_Boxes);

            foreach ((BoxRootRecord record, int box) in _Boxes)
            {
                foreach (int portal in record.PortalsOfBox[box])
                {
                    BoxPortal p = record.InnerPortals[portal];

                    ring.Add((record, p.A == box ? p.B : p.A));
                }

                BoxCells cells = record.Boxes[box];

                for (int axis = 0; axis < 3; axis++)
                {
                    if (cells.Max[axis] == record.Cells && m_NavTree.BoxRecordOf(record.RootIndex + s_BoxSearchAxes[axis]) is BoxRootRecord upper)
                    {
                        foreach (BoxPortal p in m_NavTree.BoxFaceTable(record.RootIndex, axis) ?? Array.Empty<BoxPortal>())
                            if (p.A == box)
                                ring.Add((upper, p.B));
                    }

                    if (cells.Min[axis] == 0 && m_NavTree.BoxRecordOf(record.RootIndex - s_BoxSearchAxes[axis]) is BoxRootRecord lower)
                    {
                        foreach (BoxPortal p in m_NavTree.BoxFaceTable(lower.RootIndex, axis) ?? Array.Empty<BoxPortal>())
                            if (p.B == box)
                                ring.Add((lower, p.A));
                    }
                }
            }

            Dictionary<Vector3Int, List<Bounds>> byRoot = new Dictionary<Vector3Int, List<Bounds>>();

            foreach ((BoxRootRecord record, int box) in ring)
            {
                if (!byRoot.TryGetValue(record.RootIndex, out List<Bounds> list))
                    byRoot[record.RootIndex] = list = new List<Bounds>();

                list.Add(record.WorldBounds(box));
            }

            return byRoot;
        }

        #endregion

        #region Dev verification (implemented in GlobalOctree.Pathfinding.Boxes.DevExplain.cs)

        //the boxes the last radius-aware search on this thread closed before it routed, for a probe that asks why its corridor
        //is not the leaf route's
        partial void DevNoteBoxRouted(List<BoxNode> _Nodes);

        #endregion

        #region Service methods — the radius-aware half (GlobalOctree.Pathfinding.Boxes.Volumetric.cs)

        //whether a body of _Radius sweeping from _A to _B meets occupied space
        partial void BoxLegBlocked(Vector3 _A, Vector3 _B, float _Radius, ref bool _Blocked);

        //whether a body of _Radius fits at a point at all — one descent, the cheap filter in front of the capsule tests
        partial void BoxPointFits(Vector3 _Point, float _Radius, ref bool _Fits);

        //the free leaf a blocked shot of a body of _Radius enters, as the radius-aware leaf search's free-space edge finds it
        partial void BoxShotEntry(Vector3 _From, Vector3 _GoalPoint, float _Radius, ref Leaf _Entry);

        #endregion
    }
}
