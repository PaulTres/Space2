using Nav3D.API;
using System;
using System.Collections.Generic;
using System.Threading;
using UnityEngine;

namespace Nav3D.Pathfinding
{
    //An edge produced by the successor function for a node: either another nav node, or the goal having been
    //reached. ViaStraight distinguishes a free-space straight-shot edge, which needs no contact-point detailing on
    //reconstruction, from an in-region face-adjacency edge.
    //generic over the node type, which is the whole point of the parameter: the point lane's node is a free leaf,
    //while the radius-aware lane must also stand on a portal — the clearest point of the face two leaves share —
    //because in a passage tiled by an even number of leaves no leaf centre lies on the axis, and the passage would
    //otherwise carry half the radius its geometry admits.
    readonly struct NavEdge<TNode>
    {
        public readonly TNode To;
        public readonly float Cost;
        public readonly bool  IsGoal;
        public readonly bool  ViaStraight;

        NavEdge(TNode _To, float _Cost, bool _IsGoal, bool _ViaStraight)
        {
            To          = _To;
            Cost        = _Cost;
            IsGoal      = _IsGoal;
            ViaStraight = _ViaStraight;
        }

        public static NavEdge<TNode> ToNode(TNode _To,   float _Cost, bool _ViaStraight) => new NavEdge<TNode>(_To, _Cost, false, _ViaStraight);
        public static NavEdge<TNode> ToGoal(float _Cost, bool  _ViaStraight) => new NavEdge<TNode>(default, _Cost, true, _ViaStraight);
    }

    //The reconstructed nav-graph portion of a path: the node chain from the seed to the node from which the goal was
    //reached, plus, per step, whether it was a straight-shot edge (no contact point) or a face edge.
    readonly struct GlobalAStarResult<TNode>
    {
        public readonly PathfindingResultCode Code;
        public readonly List<TNode>           Chain; //seed node … goal-reaching node
        public readonly List<bool>            StepStraight; //StepStraight[i] = edge Chain[i]→Chain[i+1] is a straight shot

        //The run stood aside for a waiting tree change rather than finishing (see GlobalAStar.YieldRequested). It is
        //not a failure and must never be reported as one: the caller releases the tree read lock, lets the publish
        //through and then either resumes the run (GlobalAStar.Resume) or runs the search again on the published tree.
        public readonly bool Yielded;

        public GlobalAStarResult(PathfindingResultCode _Code, List<TNode> _Chain, List<bool> _StepStraight, bool _Yielded = false)
        {
            Code         = _Code;
            Chain        = _Chain;
            StepStraight = _StepStraight;
            Yielded      = _Yielded;
        }

        public static GlobalAStarResult<TNode> Yield() =>
            new GlobalAStarResult<TNode>(PathfindingResultCode.CANCELLED, null, null, true);
    }

    //Root-agnostic A* over the navigation graph. It knows nothing about octree roots or free/occupied space: it pops
    //the min-f node, asks the injected Successors function for edges, and relaxes. All spatial knowledge lives in
    //Successors (and the injected Heuristic). The goal is reached when a successor edge with IsGoal is relaxed and
    //popped — that lets free-space straight-shot edges and in-region face edges live in one contiguous frontier.
    partial class GlobalAStar<TNode> : IDisposable
    {
        #region Nested types

        //The containers one run fills, kept per thread between runs so a search does not allocate its frontier, its
        //costs and its successor lists afresh. A search runs synchronously on one thread from Run to Dispose,
        //resumes included, so a thread-static pool needs no lock; the InUse flag covers a second search on the same
        //thread while one is still held, which gets fresh containers instead.
        sealed class Scratch
        {
            public readonly MinHeap<Priced>                                                  Open   = new MinHeap<Priced>();
            public readonly Dictionary<TNode, float>                                         GScore = new Dictionary<TNode, float>();
            public readonly Dictionary<TNode, (TNode Parent, bool HasParent, bool Straight)> Parent = new Dictionary<TNode, (TNode, bool, bool)>();
            public readonly HashSet<TNode>                                                   Closed = new HashSet<TNode>();
            public readonly List<NavEdge<TNode>>                                             Edges  = new List<NavEdge<TNode>>(16);

            public bool InUse;
        }

        readonly struct Priced : IComparable<Priced>
        {
            public readonly TNode Node; //meaningless when IsGoal
            public readonly float F;
            public readonly bool  IsGoal;

            Priced(TNode _Node, float _F, bool _IsGoal)
            {
                Node   = _Node;
                F      = _F;
                IsGoal = _IsGoal;
            }

            public static Priced ForNode(TNode _Node, float _F) => new Priced(_Node, _F, false);
            public static Priced ForGoal(float _F) => new Priced(default, _F, true);

            public int CompareTo(Priced _Other)
            {
                float dif = F - _Other.F;

                if (Mathf.Abs(dif) < 0.000001f)
                    return 0;

                return dif > 0 ? 1 : -1;
            }
        }

        #endregion

        #region Constants

        //must be a power of two — the yield poll masks with (PERIOD - 1)
        const int YIELD_CHECK_PERIOD = 32;

        //A run that grew its containers past this many nodes does not hand them back to the pool: a no-path search can
        //open a whole component (190 K leaves measured on Skyrift), and keeping that capacity alive on every search
        //thread would trade the garbage for a permanent few megabytes per thread. The next run on the thread starts
        //from fresh containers and pools those instead.
        const int SCRATCH_KEEP_LIMIT = 32768;

        #endregion

        #region Attributes

        //stride counter for the yield poll; per instance, and one A* instance serves exactly one search
        int m_YieldCheckTick;

        [ThreadStatic] static Scratch t_Scratch;

        //The run's state lives on the instance rather than in Run's frame so that a run which stood aside for a
        //publish can be resumed (Resume) instead of restarted. One instance serves exactly one search.
        Scratch                                                          m_Scratch;
        MinHeap<Priced>                                                  m_Open;
        Dictionary<TNode, float>                                         m_GScore;
        Dictionary<TNode, (TNode Parent, bool HasParent, bool Straight)> m_Parent;
        HashSet<TNode>                                                   m_Closed;
        float                                                            m_GGoal;
        (TNode Node, bool HasNode, bool Straight)                        m_GoalLink;

        #endregion

        #region Properties

        //Injected, octree-aware. Fills the list it is handed (empty on entry, owned by the run and reused for every node)
        //with the successor edges of a node (in-region neighbours + optional straight shot).
        public Action<TNode, List<NavEdge<TNode>>> Successors { get; set; }

        //Injected. h(leaf) — default euclidean distance from the leaf to the goal point. The user can supply any.
        public Func<TNode, float> Heuristic { get; set; }

        public CancellationToken CancellationTokenTimeout { get; set; }
        public CancellationToken CancellationTokenControl { get; set; }

        //pathfinding is second in line behind A tree change. Injected by the octree, where it reads "a publish is
        //waiting for the write lock". A search holds the tree read lock for its whole run, so without this the
        //publish waits for the longest search in flight — and, because ReaderWriterLockSlim prefers writers, every
        //reader arriving meanwhile waits with it. Polled between expansions; true makes the run return
        //GlobalAStarResult.Yield() for the caller to resume or restart after the publish. Null = never yields.
        public Func<bool> YieldRequested { get; set; }

        //Optional dev-debug hook, invoked once per leaf the moment it is first opened, in expansion order — the
        //data the A* visualization animates. Null on the production path, where the per-open `?.Invoke` costs one
        //predicted-not-taken branch and no allocation.
        public Action<TNode> OnLeafOpened { get; set; }

        //How many nodes the last Run closed (popped and expanded) — the search's own size, independent of the
        //machine it ran on. Read by the octree's dev outcome histogram (which searches are the long
        //ones); one increment per expansion on the hot loop, nothing else.
        public int Expansions { get; private set; }

        //Every node the run has put a cost on — a superset of the nodes it holds references to (the heap, the parent
        //links and the goal link only ever name nodes that were given a g first). The caller of Resume checks them
        //against what the publish it stood aside for replaced. Only meaningful after a run that yielded.
        public IEnumerable<TNode> OpenedNodes => m_GScore.Keys;

        #endregion

        #region Dev hooks

        //Implemented in GlobalAStar.DevTrace.cs, which is [Nav3DDevOnly] and therefore deleted by the release
        //preparer — at which point an unimplemented partial void compiles to nothing and every call site below
        //vanishes with it.

        partial void DevTraceRunStart(TNode _SeedNode, float _SeedG);
        partial void DevTracePop(TNode _Node, bool _IsGoal, float _F);
        partial void DevTracePopSkippedClosed(TNode _Node);
        partial void DevTraceEdges(TNode _Node, List<NavEdge<TNode>> _Edges);
        partial void DevTraceEdgeOutcome(TNode _To, bool _IsGoal, float _TentativeG, string _Outcome);
        partial void DevTraceRunEnd(PathfindingResultCode _Code);

        #endregion

        #region Public methods

        //Runs the search seeded at _SeedNode (the node the path enters the graph at) with accrued cost _SeedG.
        //_SeedStraight marks whether the edge from the path origin into _SeedNode was a straight shot (it never needs
        //a contact point). Returns the node chain + per-step straightness; the caller turns it into world waypoints.
        public GlobalAStarResult<TNode> Run(TNode _SeedNode, float _SeedG, bool _SeedStraight)
        {
            RentScratch();

            //a second Run on one instance starts from empty containers like the first
            m_Scratch.Open.Clear();
            m_Scratch.GScore.Clear();
            m_Scratch.Parent.Clear();
            m_Scratch.Closed.Clear();

            m_Open     = m_Scratch.Open;
            m_GScore   = m_Scratch.GScore;
            m_Parent   = m_Scratch.Parent;
            m_Closed   = m_Scratch.Closed;
            m_GGoal    = float.MaxValue;
            m_GoalLink = (default, false, false);

            Expansions = 0;

            m_GScore[_SeedNode] = _SeedG;
            m_Parent[_SeedNode] = (default, false, _SeedStraight);
            m_Open.Add(Priced.ForNode(_SeedNode, _SeedG + Heuristic(_SeedNode)));
            OnLeafOpened?.Invoke(_SeedNode);
            DevTraceRunStart(_SeedNode, _SeedG);

            return Continue();
        }

        //Continues a run that returned GlobalAStarResult.Yield(), from exactly where it stood aside. Sound only if
        //nothing the run holds was replaced meanwhile — the caller's check against OpenedNodes, see
        //GlobalOctree.FindPathAttempt. Edges the run relaxed before the publish keep their pre-publish cost and
        //verdict; the path layer re-examines a route that a change published during its search crosses.
        public GlobalAStarResult<TNode> Resume()
        {
            //a full stride before the next poll, so a run that meets one publish after another still advances
            m_YieldCheckTick = 0;

            return Continue();
        }

        //Hands the run's containers back to this thread's pool. The result of the run stays valid — Reconstruct builds
        //it into lists of its own — but OpenedNodes and Resume do not.
        public void Dispose()
        {
            Scratch scratch = m_Scratch;

            if (scratch == null)
                return;

            m_Scratch = null;
            m_Open    = null;
            m_GScore  = null;
            m_Parent  = null;
            m_Closed  = null;

            if (scratch.GScore.Count > SCRATCH_KEEP_LIMIT)
            {
                if (ReferenceEquals(t_Scratch, scratch))
                    t_Scratch = null;

                return;
            }

            scratch.Open.Clear();
            scratch.GScore.Clear();
            scratch.Parent.Clear();
            scratch.Closed.Clear();
            scratch.Edges.Clear();

            scratch.InUse = false;
            t_Scratch ??= scratch;
        }

        #endregion

        #region Service methods

        GlobalAStarResult<TNode> Continue()
        {
            MinHeap<Priced>                                                  open   = m_Open;
            Dictionary<TNode, float>                                         gScore = m_GScore;
            Dictionary<TNode, (TNode Parent, bool HasParent, bool Straight)> parent = m_Parent;
            HashSet<TNode>                                                   closed = m_Closed;

            while (open.Count != 0)
            {
                if (CancellationTokenTimeout.IsCancellationRequested)
                {
                    DevTraceRunEnd(PathfindingResultCode.TIMEOUT);

                    return new GlobalAStarResult<TNode>(PathfindingResultCode.TIMEOUT, null, null);
                }

                if (CancellationTokenControl.IsCancellationRequested)
                {
                    DevTraceRunEnd(PathfindingResultCode.CANCELLED);

                    return new GlobalAStarResult<TNode>(PathfindingResultCode.CANCELLED, null, null);
                }

                //Asked every YIELD_CHECK_PERIOD expansions rather than every one: an expansion is ~1 µs, so the
                //publish's extra wait is bounded by tens of microseconds either way, while the delegate call itself
                //stays off the hot loop's critical path.
                if (YieldRequested != null && (++m_YieldCheckTick & (YIELD_CHECK_PERIOD - 1)) == 0 && YieldRequested())
                {
                    DevTraceRunEnd(PathfindingResultCode.CANCELLED);

                    return GlobalAStarResult<TNode>.Yield();
                }

                Priced current = open.PopMin();

                DevTracePop(current.Node, current.IsGoal, current.F);

                if (current.IsGoal)
                {
                    DevTraceRunEnd(PathfindingResultCode.SUCCEEDED);

                    return Reconstruct(parent, m_GoalLink);
                }

                TNode node = current.Node;

                if (!closed.Add(node))
                {
                    DevTracePopSkippedClosed(node);

                    continue;
                }

                Expansions++;

                //A node's cost is always the best one known for it, never the one the popped heap entry was priced
                //with. That is also why superseded entries need no test of their own: the cheaper entry has a lower
                //f, pops first and closes the node, and the one it superseded is caught by the closed set above.
                //Do not re-derive the test by comparing the popped f against a recomputed g + h — f was rounded on
                //its way into a float field, so a node whose f rounded up would fail by half an ulp and die
                //permanently.
                gScore.TryGetValue(node, out float gNode);

                List<NavEdge<TNode>> edges = m_Scratch.Edges;

                edges.Clear();

                Successors(node, edges);

                DevTraceEdges(node, edges);

                foreach (NavEdge<TNode> edge in edges)
                {
                    float tentativeG = gNode + edge.Cost;

                    if (edge.IsGoal)
                    {
                        if (tentativeG < m_GGoal)
                        {
                            m_GGoal    = tentativeG;
                            m_GoalLink = (node, true, edge.ViaStraight);
                            open.Add(Priced.ForGoal(m_GGoal));

                            DevTraceEdgeOutcome(default, true, tentativeG, "goal edge pushed");
                        }
                        else
                        {
                            DevTraceEdgeOutcome(default, true, tentativeG, "goal edge dropped (costlier)");
                        }

                        continue;
                    }

                    TNode to = edge.To;

                    if (closed.Contains(to))
                    {
                        DevTraceEdgeOutcome(to, false, tentativeG, "closed");

                        continue;
                    }

                    if (!gScore.TryGetValue(to, out float gOld) || tentativeG < gOld)
                    {
                        gScore[to] = tentativeG;
                        parent[to] = (node, true, edge.ViaStraight);
                        open.Add(Priced.ForNode(to, tentativeG + Heuristic(to)));
                        OnLeafOpened?.Invoke(to);

                        DevTraceEdgeOutcome(to, false, tentativeG, "relaxed");
                    }
                    else
                    {
                        DevTraceEdgeOutcome(to, false, tentativeG, "not improved");
                    }
                }
            }

            DevTraceRunEnd(PathfindingResultCode.PATH_DOES_NOT_EXIST);

            return new GlobalAStarResult<TNode>(PathfindingResultCode.PATH_DOES_NOT_EXIST, null, null);
        }

        void RentScratch()
        {
            if (m_Scratch != null)
                return;

            Scratch pooled = t_Scratch;

            if (pooled != null && !pooled.InUse)
            {
                m_Scratch = pooled;
            }
            else
            {
                m_Scratch = new Scratch();

                t_Scratch ??= m_Scratch;
            }

            m_Scratch.InUse = true;
        }

        static GlobalAStarResult<TNode> Reconstruct(
                Dictionary<TNode, (TNode Parent, bool HasParent, bool Straight)> _Parent,
                (TNode Node, bool HasNode, bool Straight)                        _GoalLink
            )
        {
            List<TNode> chain        = new List<TNode>();
            List<bool>  intoStraight = new List<bool>(); //intoStraight[i] = straightness of the edge that reached chain[i]

            //the chain is walked by the recorded parent link rather than by a null node: TNode may be a value type,
            //so "no parent" has to be a flag, not a reference comparison
            TNode cur    = _GoalLink.Node;
            bool  hasCur = _GoalLink.HasNode;

            while (hasCur)
            {
                chain.Add(cur);

                (TNode parent, bool hasParent, bool straight) = _Parent[cur];

                intoStraight.Add(straight);

                cur    = parent;
                hasCur = hasParent;
            }

            chain.Reverse();
            intoStraight.Reverse();

            //Per-step straightness: edge chain[i] → chain[i+1] equals the straightness recorded "into" chain[i+1].
            List<bool> stepStraight = new List<bool>(chain.Count);
            for (int i = 1; i < chain.Count; i++)
                stepStraight.Add(intoStraight[i]);

            //the final leg chain[last] → goal
            stepStraight.Add(_GoalLink.Straight);

            return new GlobalAStarResult<TNode>(PathfindingResultCode.SUCCEEDED, chain, stepStraight);
        }

        #endregion
    }
}
