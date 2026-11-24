using System;
using System.Collections.Generic;
using System.Threading;
using UnityEngine;
using Node = Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.Elements.Node;
using Leaf = Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.Elements.Leaf;
using LeavesSpatialGrid = Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.SpatialStorages.LeavesSpatialGrid;
using NodeSpatialGrid = Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.SpatialStorages.NodeSpatialGrid;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree
{
    //Extracts the "off-tree build -> single-patch publish" skeleton that AddObstacle, RemoveObstacleInternal,
    //UpdateObstaclesInBounds and AddObstacleFromCache each hand-rolled identically. Every op still computes its own
    //footprint and business logic; only the mechanical ring/clone/stage/alias -> repair/lock/install/classify/
    //deferred-changes -> alias-restore sequence lives here, so the standing rules (no publish after cancel, alias
    //always restored, repair always precedes publish) hold structurally instead of by per-op convention.
    //Standalone rather than nested in GlobalOctree, matching RootBuilder/GeometryRegistry/NavTree, and holding a
    //back-reference for the handful of members it needs.
    //Deliberately not owning ConstructionProgress, the op timers or the per-op cancellation checkpoints: their
    //call-site position already differs across the four ops, and folding them in would silently change what each
    //op's timing window covers.
    internal sealed class StagingTransaction : IDisposable
    {
        #region Constants

        //the publish'S own share of the bar, step by step. The phase reported nothing at all, so a user's loading
        //bar stood still through it; it has no counter to divide, so each step reports where it leaves the phase
        //as it completes. The steps are weighted by what the phase actually costs, which is one of them:
        //`ClassifyOpenFacesStaged` is 342 ms of a ~350 ms publish, `ComputeRingBackRefRepoints` 0.01 ms and the
        //write-locked install 0.24 ms.
        const float PUBLISH_PROGRESS_CLASSIFIED          = 0.90f;
        const float PUBLISH_PROGRESS_BACK_REFS_COMPUTED  = 0.93f;
        const float PUBLISH_PROGRESS_ORIGINALS_COLLECTED = 0.95f;
        const float PUBLISH_PROGRESS_INSTALLED           = 0.97f;

        #endregion

        #region Nested types

        //one verdict per footprint root that EXISTS in the live tree, decided in the constructor before anything is
        //cloned — the binding form of the narrowing plan. The clone is cut from these verdicts and the rebuild
        //follows them; RebuildRoots may not decide differently per root, because a divergence writes into leaves the
        //clone left shared with the live tree (RestoreWholeRootRebuild escalates instead of reconciling once the
        //clone is actually narrowed). A root absent from the map (built into fresh space) has nothing to clone and
        //keeps the legacy rebuild-time decisions.
        public enum RootRebuildVerdict
        {
            //rebuilt from scratch wholesale — the default, and what every verdictless root effectively gets
            WHOLE      = 0,
            //rebuilt cell by cell against the planned RebuiltCellsByRoot entry; the clone is scoped to the same plan
            NARROWED   = 1,
            //the existing subtree is kept in place and only complemented (the oversized-neighbour lane)
            COMPLEMENT = 2,
            //no surviving geometry justifies the root — the publish DROPS it, so nothing needs cloning or building
            DOOMED     = 3,
        }

        //One replaced cell, addressable both ways: as a volume (the classification asks "does this leaf touch it")
        //and as a tree position (the free-graph repair asks "which leaves sit in it and across its faces", which
        //needs a descent, not a box).
        public readonly struct RebuiltCell
        {
            public Vector3Int Index  { get; }
            public int        Layer  { get; }
            public Bounds     Bounds { get; }

            public RebuiltCell(Vector3Int _Index, int _Layer, Bounds _Bounds)
            {
                Index  = _Index;
                Layer  = _Layer;
                Bounds = _Bounds;
            }
        }

        #endregion

        #region Attributes

        readonly GlobalOctree m_Octree;

        //whether the ring was copied in full — the precondition for widening RebuiltRootIndices later
        readonly bool m_FullRingClone;

        //The op's construction grid, registered by the worker so Dispose severs it with the other hubs (P28): it
        //maps every leaf the operation built or seeded individually, so a conservatively pinned stale grid would
        //retain a subtree's worth of leaves past even the detached-subtree teardown, which only unlinks the graph
        LeavesSpatialGrid m_ConstructionGrid;

        //Built on first use by IsClone — the clone side of OriginalToClone, as a set (the scratch's set; filled
        //lazily because most operations never ask for it).
        HashSet<Leaf> m_Clones;
        bool          m_ClonesBuilt;

        //the operation's reusable buffers, taken in the constructor and released in Dispose (OperationScratch)
        readonly OperationScratch m_Scratch;

        //set at the end of a completed Publish — the commit point; a transaction disposed without it did not
        //complete (cancelled, failed, early return) and its scratch is discarded rather than reused
        bool m_Published;

        //the binding narrowing plan, decided in the constructor; null when the operation supplied no delta
        readonly Dictionary<Vector3Int, RootRebuildVerdict> m_Verdicts;

        //Memoized IsSharedWithLiveTree: the connection pass asks the ownership question once per write and a leaf
        //owns ~6 edges, so the memo turns six live-tree descents into one. The scratch's dictionary, cleared with it.
        readonly Dictionary<Leaf, bool> m_SharedOwnerAnswers;

        #endregion

        #region Properties

        public CancellationToken      CancellationToken       { get; }
        public HashSet<Vector3Int>    RingRootIndices         { get; }

        //the footprint, kept as the operation stated it — before Phase 3's narrowing removes cell-rebuilt roots from
        //RebuiltRootIndices. This is the set the ring's boundary layer is cut against, by both the clone and the
        //open-face classification: one definition, because the two drifting apart cost a silent stale-open-face
        //defect once already.
        public HashSet<Vector3Int>    FootprintRootIndices    { get; }

        //the ROOTS A boundary layer is OWED toward — the single set the ring clone, the narrowed footprint clone,
        //the open-face classification and the clone-anatomy oracle all cut "faces toward" against (the
        //one-definition rule, now held by construction: computed once, here, frozen). Under a bound plan it is the
        //footprint roots whose subtree is replaced or dropped wholesale (verdict != NARROWED): only there does
        //adjacency change across the whole mutual face. A face between two NARROWED roots changes only against
        //their replaced CELLS, which the cells predicate (CollectCellsAround) already covers on both sides —
        //cutting boundary layers against the whole footprint instead is what made the narrowed clone degenerate to
        //a whole clone on a corner-placed obstacle (all 8 footprint roots narrowed, every leaf on a mutual face).
        //Without a plan it stays the whole footprint — the legacy-conservative set the P10 fix installed.
        public HashSet<Vector3Int>    BoundaryFaceRootIndices { get; }

        public HashSet<Vector3Int>    CloneSetIndices         { get; }
        public NodeSpatialGrid        StagingGrid             { get; }
        public Dictionary<Leaf, Leaf> OriginalToClone         { get; }

        //The roots whose SUBTREE this operation may replace, drop or rebuild — its own footprint by default, i.e.
        //everything in the clone set except the ring, whose clones are only ever re-WIRED (FreeNeighbor edges), never
        //structurally rebuilt. Two passes are scoped by this rather than by the whole clone set, and both are only
        //correct because of that distinction: the open-face classification (a ring leaf's adjacency can change only
        //across a face looking into a rebuilt root) and the free-graph repair (only a leaf belonging to a rebuilt
        //root can be a discarded clone).
        //
        //A caller that installs subtrees it did not clone must widen this to the clone set — the pre-baked add does
        //exactly that, installing bake-time nodes whose leaves are neither clones nor classified. Widening is a
        //method rather than a setter because it is only sound together with a full ring clone: the partial clone
        //copies the ring leaves facing this set as it stood at construction time, so a later widening would leave
        //leaves that must be written to shared with the live tree (see WidenRebuiltRootsToCloneSet).
        public HashSet<Vector3Int>    RebuiltRootIndices      { get; private set; }

        //The CELLS a narrowed rebuild replaced, for the roots it narrowed (which it removed from RebuiltRootIndices
        //as it did so). Empty means "nothing was narrowed" and every scoped pass reads exactly as it did before.
        //The two passes scoped by the rebuilt set — the open-face classification and the free-graph repair — take
        //this list beside it, because after the narrowing "this operation replaced something here" is no
        //longer answerable at root granularity.
        //Grouped by the root they NARROWED, not as one flat list, and that is a performance contract rather than a
        //convenience: both passes ask "did this operation replace anything at/next to this leaf" once per leaf and
        //once per ref, so a flat list turns into millions of box tests per operation. Measured when it was flat: the
        //whole operation went 233 -> 751 ms. A cell always lies inside the root that narrowed, so the leaf's own root
        //(plus its six face neighbours, for the "touches across a face" question) is all any test has to look at.
        public Dictionary<Vector3Int, List<RebuiltCell>> RebuiltCellsByRoot { get; } = new Dictionary<Vector3Int, List<RebuiltCell>>();

        public bool HasNarrowedCells => RebuiltCellsByRoot.Count > 0;

        //Free-neighbour edits the construction passes (the free-graph repair, the edge-connection pass, the grid's
        //connection pass) asked for on leaves this operation does not replace — i.e. leaves SHARED with the live
        //tree, which readers are traversing right now. They are applied inside the publish's write lock instead of
        //on the spot; see EditFreeNeighbor for why this is a rerouting rather than a narrowing. Non-empty exactly
        //when the footprint clone was narrowed (Phase 2) — these are the writes that would otherwise have landed in
        //the live tree unlocked.
        public List<(Leaf Owner, Leaf Neighbour, bool Add)> DeferredSharedGraphEdits { get; }

        //true when the constructor computed per-root verdicts (an operation with a geometry delta and a partial ring
        //clone) — the precondition for every "the plan is binding" consumer
        public bool PlanBound => m_Verdicts != null;

        //Set by NavTree.CloneRootsForStaging when it actually scoped at least one footprint root's clone (narrowed
        //predicate or a skipped doomed root). From that moment a plan/execution divergence is no longer repairable:
        //a whole-root rebuild of a partially-cloned root would write into shared leaves, so
        //RestoreWholeRootRebuild throws instead of reconciling.
        public bool CloneNarrowed { get; private set; }

        //replaces the raw List<Action> deferredReaderMutations each op used to declare locally; still a plain
        //list of lambdas (GeometryRegistry owns the reader-visible map mutations themselves; this is only the
        //deferral vehicle) — existing
        //helpers that take a List<Action> (RemoveClusterData, ProcessIntersectingCachedClusters) accept this
        //property directly, unchanged.
        public List<Action> DeferredRegistryChanges { get; }

        //the undo side of the same split. A registry change an operation applies INLINE — the root index
        //and cluster ownership, which the construction reads back — must be put back if the operation never
        //publishes, or it outlives an operation that changed nothing else. Filled by RemoveClusterData, applied by
        //Dispose on every path that did not publish.
        public List<Action> RegistryRollbacks { get; }

        #endregion

        #region Constructors

        //Clones _FootprintRootIndices plus a one-root face-neighbour ring into a private staging grid and points
        //construction at it.
        //_FullRingClone = true copies every ring root in full instead of only its boundary layer. Pass it when the
        //operation cannot be described by the boundary predicate — it installs subtrees it never cloned, or widens
        //RebuiltRootIndices afterwards. The pre-baked add is that operation and the only one.
        //_Delta is what lets the rebuild, and the clone with it, be narrowed: each live footprint root gets a
        //binding verdict against the live tree, the clone is cut from those verdicts and the rebuild follows them.
        //One plan, decided once — each side deriving its own is how a rebuild reaches a leaf the clone left shared,
        //and that write would land in the tree readers traverse.
        public StagingTransaction(
                GlobalOctree               _Octree,
                HashSet<Vector3Int>        _FootprintRootIndices,
                CancellationToken          _CancellationToken,
                bool                       _FullRingClone = false,
                GlobalOctree.GeometryDelta _Delta         = null
            )
        {
            m_Octree           = _Octree;
            CancellationToken  = _CancellationToken;
            m_FullRingClone    = _FullRingClone;

            //the scratch first: every hub below is one of its buffers
            m_Scratch = _Octree.OperationScratch;
            m_Scratch.Begin();

            m_Clones                 = m_Scratch.Clones;
            DeferredSharedGraphEdits = m_Scratch.DeferredSharedGraphEdits;
            DeferredRegistryChanges  = m_Scratch.DeferredRegistryChanges;
            RegistryRollbacks        = m_Scratch.RegistryRollbacks;
            m_SharedOwnerAnswers     = m_Scratch.SharedOwnerAnswers;

            FootprintRootIndices = new HashSet<Vector3Int>(_FootprintRootIndices);

            RingRootIndices = _Octree.ComputeBoundaryRing(_FootprintRootIndices);

            CloneSetIndices = new HashSet<Vector3Int>(_FootprintRootIndices);
            CloneSetIndices.UnionWith(RingRootIndices);

            RebuiltRootIndices = new HashSet<Vector3Int>(_FootprintRootIndices);

            //the plan comes first, and on the live tree: the clone is cut from it, and so is the rebuild. One set,
            //decided once — the alternative (each deriving its own) is how a rebuild reaches a leaf the clone left
            //shared, and that write would land in the tree readers traverse. The verdict order mirrors RebuildRoots'
            //branch order exactly: complement outranks narrowing, and a root that reverted to empty space is a
            //whole-root drop decision that narrowing must never claim.
            if (!_FullRingClone && _Delta != null && _Delta.RemovedClusters.Count + _Delta.AddedClusters.Count > 0)
            {
                m_Verdicts = new Dictionary<Vector3Int, RootRebuildVerdict>();

                Bounds[] cutVolume = GlobalOctree.BuildCutVolume(_Delta);

                foreach (Vector3Int rootIndex in _FootprintRootIndices)
                {
                    if (!_Octree.NavTree.LiveRootsGrid.TryGetNode(0, rootIndex, out Node liveRoot))
                        continue;

                    Bounds rootBounds = Common.UtilsSpatialGrid.GetBucketBounds(rootIndex, _Octree.RootSize);

                    if (_Octree.IsRootComplementEligible(rootBounds, _Delta))
                    {
                        m_Verdicts[rootIndex] = RootRebuildVerdict.COMPLEMENT;

                        continue;
                    }

                    if (!_Octree.RootHasSurvivingGeometry(rootBounds, _Delta))
                    {
                        m_Verdicts[rootIndex] = RootRebuildVerdict.DOOMED;

                        continue;
                    }

                    if (_Octree.TryPlanNarrowedRebuild(rootIndex, liveRoot, cutVolume, out List<RebuiltCell> cells))
                    {
                        m_Verdicts[rootIndex] = RootRebuildVerdict.NARROWED;

                        NarrowRebuiltRoot(rootIndex, cells);

                        continue;
                    }

                    m_Verdicts[rootIndex] = RootRebuildVerdict.WHOLE;
                }
            }

            //frozen here, after the verdicts: see the property — every "faces toward" cut asks this one set
            if (m_Verdicts == null)
                BoundaryFaceRootIndices = FootprintRootIndices;
            else
            {
                BoundaryFaceRootIndices = new HashSet<Vector3Int>();

                foreach (Vector3Int rootIndex in _FootprintRootIndices)
                {
                    if (!m_Verdicts.TryGetValue(rootIndex, out RootRebuildVerdict rootVerdict) || rootVerdict != RootRebuildVerdict.NARROWED)
                        BoundaryFaceRootIndices.Add(rootIndex);
                }
            }

            //The ring's boundary layer is cut against the whole footprint, never against the post-narrowing rebuilt
            //set. Narrowing removes a root from that set, and passing the shrunken set here silently stopped cloning
            //the ring leaves facing it — while `FormEdgeConnectionsForAddedRoots` still wires every edge leaf on
            //both sides of that root's faces. The result was 626 writes per operation into leaves SHARED with the
            //live tree: correct in the end (Run A stayed clean, the final graph is the same) and completely wrong in
            //the middle, because readers traverse those leaves while the operation is still building.
            //Caught by `RuntimeObstaclesRigOracles.RunCloneAnatomy`, which is the only gate that sees it.
            OriginalToClone = m_Scratch.OriginalToClone;

            Dictionary<Vector3Int, Node> stagedClones = _Octree.NavTree.CloneRootsForStaging(
                    CloneSetIndices,
                    RingRootIndices,
                    BoundaryFaceRootIndices,
                    m_FullRingClone,
                    this,
                    OriginalToClone,
                    m_Scratch.NeighborBuffer
                );

            StagingGrid = new NodeSpatialGrid(_Octree.ActiveLayersCount);
            foreach (KeyValuePair<Vector3Int, Node> stagedClone in stagedClones)
                StagingGrid.AddNode(stagedClone.Value);

            _Octree.ActiveRootsGrid = StagingGrid;

            //dev, off by default: fingerprints every staged free leaf — clones and the ring interior leaves the
            //partial clone SHARES with the live tree — so the publish can say which of them this operation actually
            //wrote to. The shared ones are the load-bearing half: a write to one of those is a write into the tree
            //readers are traversing.
            _Octree.NavTree.DevCloneAnatomySnapshotHook(CloneSetIndices, OriginalToClone, StagingGrid);
        }

        #endregion

        #region Public methods

        public void MarkCloneNarrowed()
        {
            CloneNarrowed = true;
        }

        public bool TryGetVerdict(Vector3Int _RootIndex, out RootRebuildVerdict _Verdict)
        {
            if (m_Verdicts != null && m_Verdicts.TryGetValue(_RootIndex, out _Verdict))
                return true;

            _Verdict = RootRebuildVerdict.WHOLE;

            return false;
        }

        public bool IsDoomedRoot(Vector3Int _RootIndex)
        {
            return m_Verdicts != null && m_Verdicts.TryGetValue(_RootIndex, out RootRebuildVerdict verdict) && verdict == RootRebuildVerdict.DOOMED;
        }

        //the operation's construction-grid factory: sized from the previous operations' grids (OperationScratch)
        public LeavesSpatialGrid RentConstructionGrid()
        {
            LeavesSpatialGrid grid = m_Scratch.RentConstructionGrid(m_Octree.ActiveLayersCount);

            RegisterConstructionGrid(grid);

            return grid;
        }

        //Records that a root was rebuilt cell by cell rather than wholesale: it leaves the rebuilt-root set and its
        //cells join the cell list, so the classification and the repair narrow with it instead of walking every leaf
        //the root holds.
        //REFUSES once the rebuilt set has been widened to the clone set: after widening the two are the same object,
        //and removing a root here would silently shrink the clone set — the thing that governs cloning, ring
        //repointing and what the publish installs.
        public void NarrowRebuiltRoot(Vector3Int _RootIndex, List<RebuiltCell> _Cells)
        {
            if (ReferenceEquals(RebuiltRootIndices, CloneSetIndices))
                throw new InvalidOperationException("StagingTransaction: a rebuilt root cannot be narrowed after the rebuilt set has been widened to the clone set.");

            RebuiltRootIndices.Remove(_RootIndex);

            RebuiltCellsByRoot[_RootIndex] = _Cells;
        }

        //is this leaf one this operation REPLACES? `OriginalToClone` is the authority on what a clone is
        //, and
        //this is the same identity question `ComputeRingBackRefRepoints` asks — kept here so both ask it of one set
        //rather than each building its own, and so the answer cannot drift between them.
        //Built lazily: the dictionary is final once the constructor returns, and most operations never ask.
        public bool IsClone(Leaf _Leaf)
        {
            if (!m_ClonesBuilt)
            {
                m_ClonesBuilt = true;

                foreach (Leaf clone in OriginalToClone.Values)
                    m_Clones.Add(clone);
            }

            return m_Clones.Contains(_Leaf);
        }

        //A free-NEIGHBOUR edit, ROUTED by ownership — the mechanism the free-graph repair needs before the clone
        //can be narrowed.
        //
        //The repair severs and re-points refs, writing to the leaf on the far end of a ref: a two-step graph
        //neighbourhood of a rebuilt cell, which through a coarse leaf reaches a root-size away in space. **No
        //geometric clone predicate can bound that**, which is why two attempts at narrowing the clone by refining a
        //predicate both landed unlocked writes in the live tree. So the write set is not bounded, it is REROUTED: an
        //edit whose owner is a leaf this operation replaces happens inline on a clone no reader can reach, and an
        //edit whose owner is SHARED with the live tree is queued and applied inside the publish's write lock —
        //exactly what the ring's back-refs already do.
        //
        //"Shared" is not simply "not a clone": the repair also reaches leaves this operation CONSTRUCTED or
        //INSTALLED verbatim, which are in no clone map yet unreachable by readers, so writing them inline is as safe
        //as writing a clone. Deferring those would falsely fire the "edits deferred to the publish" acceptance
        //counter, so a non-clone owner is asked the exact question — is this very object reachable in the live
        //tree? — and defers only on yes.
        public void EditFreeNeighbor(Leaf _Owner, Leaf _Neighbour, bool _Add)
        {
            if (IsClone(_Owner) || !IsSharedWithLiveTreeCached(_Owner))
            {
                if (_Add)
                    _Owner.AddFreeNeighbor(_Neighbour);
                else
                    _Owner.RemoveFreeNeighbor(_Neighbour);

                return;
            }

            //A no-OP edit on a shared leaf is dropped, not queued: the connection pass re-wires whole seeded
            //neighbourhoods and most of what it asks for on a shared leaf is an edge the leaf already carries.
            //Queueing those would grow the write-locked publish by thousands of idempotent applications per
            //operation. Sound unlocked for the same reason IsSharedWithLiveTree is: operations are serialized,
            //so nothing else edits this set between the test and the publish.
            if (_Add == _Owner.ContainsFreeNeighbor(_Neighbour))
                return;

            DeferredSharedGraphEdits.Add((_Owner, _Neighbour, _Add));
        }

        //The mirror of NarrowRebuiltRoot. It exists because the plan and the execution can disagree: narrowing is
        //planned in this constructor against the live tree, but RebuildRoots decides per root what it actually does,
        //and three of its branches rebuild a narrowed root whole anyway. Left unsaid, the transaction keeps claiming
        //that only a few cells changed while the whole root was reconstructed, and every pass scoped by these sets
        //then skips leaves the operation replaced — a measured defect, since the complement branch re-IDs the
        //existing subtree and carries its old open-face bits into a root whose occupancy just changed.
        //No-op for a root that was never narrowed, so callers may call it unconditionally.
        //under A bound plan this is an assertion rather than A reconciliation: the verdicts come from the same live
        //tree RebuildRoots later reads, so arriving here means plan and execution disagreed, which is a code defect.
        //Once the clone was actually cut from the plan, rebuilding the root whole would write into leaves the clone
        //left SHARED with the live tree, so the operation must die here — an aborted op leaves the live tree
        //untouched, which is strictly better than an unlocked write into a tree readers are traversing.
        public void RestoreWholeRootRebuild(Vector3Int _RootIndex)
        {
            if (!RebuiltCellsByRoot.Remove(_RootIndex))
                return;

            if (CloneNarrowed)
                throw new InvalidOperationException(
                        $"StagingTransaction: root {_RootIndex} was planned as NARROWED and its clone was scoped to that plan, "  +
                        "but RebuildRoots tried to rebuild it whole — the plan is binding once the clone is narrowed, so this " +
                        "operation is aborted instead of writing into leaves shared with the live tree."
                    );

            if (PlanBound)
                Debug.LogError(
                        $"[Nav3D] StagingTransaction: root {_RootIndex} was planned as NARROWED but rebuilt WHOLE — the plan "   +
                        "and the execution diverged. Sound while the footprint clone is whole, but it means a verdict predicate " +
                        "disagrees with RebuildRoots; investigate before enabling the narrowed clone."
                    );

            RebuiltRootIndices.Add(_RootIndex);
        }

        //Every cell replaced in this root or in one of its six face neighbours — the only cells that can affect a
        //leaf of this root, since a leaf is only ever adjacent across a face. Built once per root by the passes that
        //walk leaves, never per leaf.
        public List<RebuiltCell> CollectCellsAround(Vector3Int _RootIndex)
        {
            List<RebuiltCell> around = null;

            if (RebuiltCellsByRoot.TryGetValue(_RootIndex, out List<RebuiltCell> own))
                around = new List<RebuiltCell>(own);

            foreach (Vector3Int direction in Common.UtilsSpatialGrid.FacesDirections)
            {
                if (!RebuiltCellsByRoot.TryGetValue(_RootIndex + direction, out List<RebuiltCell> neighbourCells))
                    continue;

                around ??= new List<RebuiltCell>();
                around.AddRange(neighbourCells);
            }

            return around;
        }

        //Every cell this operation replaced, flat — the free-graph repair walks them directly instead of walking the
        //clone set's leaves.
        public IEnumerable<RebuiltCell> AllRebuiltCells()
        {
            foreach (KeyValuePair<Vector3Int, List<RebuiltCell>> rootCells in RebuiltCellsByRoot)
            {
                foreach (RebuiltCell cell in rootCells.Value)
                    yield return cell;
            }
        }

        public void WidenRebuiltRootsToCloneSet()
        {
            if (!m_FullRingClone)
                throw new InvalidOperationException("StagingTransaction: RebuiltRootIndices can only be widened on a transaction constructed with _FullRingClone: true.");

            RebuiltRootIndices = CloneSetIndices;
        }

        //Write-locked install → classify → run every queued deferred registry change, in that order — the only
        //write-locked section, identical across all four ops.
        //
        //NOTE: does not call RepairStagedRegionFreeGraph — that call is not uniform across the four ops (region
        //update never needed it: geometry is unchanged, only subdivision depth, so no clone-set ref can go
        //stale). Callers that need it (add / remove / prebaked-add) call
        //RepairStagedRegionFreeGraph(CloneSetIndices, StagingGrid) explicitly, immediately before Publish(),
        //exactly where the pre-extraction code did.
        //
        //Structural no-publish-after-cancel: every call site already checks its CancellationToken before
        //reaching this call (unchanged from the pre-extraction code), so this check is a zero-risk defensive backstop
        //that makes the invariant hold by construction rather than by every future call site remembering to
        //check first.
        public void Publish()
        {
            if (CancellationToken.IsCancellationRequested)
                return;

            //Open-face classification runs here, on the staged tree, outside the write lock. It is a pure function of
            //the staged region and writes only to clone leaves no reader can reach yet, so holding the lock for it
            //bought nothing — and it was 85 ms of the ~113 ms hold, i.e. the dominant stall every main-thread agent
            //query paid on every obstacle change.
            m_Octree.DevPublishPhaseStartHook();
            m_Octree.NavTree.ClassifyOpenFacesStaged(CloneSetIndices, RebuiltRootIndices, this, StagingGrid);
            m_Octree.DevPublishPhaseEndHook("ClassifyOpenFacesStaged (unlocked)");

            m_Octree.ConstructionProgress?.SetPublishProgress(PUBLISH_PROGRESS_CLASSIFIED);

            //Same treatment, same reason: searching for the outer back-refs the publish must retarget is proportional
            //to the clone set (~150 k leaf/ref visits per publish) while APPLYING them is proportional to what
            //actually changes (~1.3 k). Once the classification moved off the lock, that search was the whole
            //remaining reader stall — 27.58 ms of a 27.59 ms hold. It writes nothing, so it does not need the lock.
            m_Octree.DevPublishPhaseStartHook();
            List<(Leaf Outer, Leaf Original, Leaf Clone)> ringBackRefRepoints =
                m_Octree.NavTree.ComputeRingBackRefRepoints(CloneSetIndices, RingRootIndices, this, HasNarrowedCells, m_Scratch.RingBackRefRepoints, m_Scratch.NeighborBuffer);
            m_Octree.DevPublishPhaseEndHook("ComputeRingBackRefRepoints (unlocked)");

            m_Octree.ConstructionProgress?.SetPublishProgress(PUBLISH_PROGRESS_BACK_REFS_COMPUTED);

            //dev, off by default: compares every staged free leaf against the fingerprint taken in the constructor.
            //Deliberately here — after construction, the free-graph repair and the classification, and before the
            //write lock. That window is exactly the invariant: nothing running unlocked may write to a leaf the
            //partial clone left SHARED with the live tree. What happens after this point is the publish itself,
            //which mutates shared and outer leaves by design, under the lock, as the atomic patch.
            m_Octree.NavTree.DevCloneAnatomyCheckHook(CloneSetIndices, RingRootIndices, BoundaryFaceRootIndices, m_FullRingClone, StagingGrid);

            //P28 detached-subtree teardown, pre-lock half: remember the live footprint originals this publish
            //replaces — a read-only look at the live grid, sound unlocked because operations are serialized
            //(the ComputeRingBackRefRepoints discipline). Under a narrowed clone a footprint root's subtree is
            //mostly SHARED with the staged root, so only the planned cells' originals are collected for it — the
            //whole-subtree teardown is reserved for roots that are replaced or dropped wholesale.
            List<Node> replacedOriginals = m_Octree.NavTree.CollectReplacedOriginals(this, m_Scratch.ReplacedOriginals);

            m_Octree.ConstructionProgress?.SetPublishProgress(PUBLISH_PROGRESS_ORIGINALS_COLLECTED);

            //the box graph of the roots whose occupancy changed, from the staged roots, outside the lock like the passes
            //above; installed under it below, so the published box graph is always the published tree's
            HashSet<Vector3Int> occupancyChanged = OccupancyChangedRootIndices();

            m_Octree.DevPublishPhaseStartHook();
            NavTree.StagedBoxes stagedBoxes = m_Octree.NavTree.BuildStagedBoxes(occupancyChanged, StagingGrid);
            m_Octree.DevPublishPhaseEndHook("BuildStagedBoxes (unlocked)");

            m_Octree.NavTree.EnterWriteLock();
            try
            {
                m_Octree.DevOpTimerPublishStartHook();

                m_Octree.DevPublishPhaseStartHook();
                m_Octree.NavTree.PublishStagedRegion(CloneSetIndices, StagingGrid, ringBackRefRepoints, DeferredSharedGraphEdits, occupancyChanged);
                m_Octree.NavTree.InstallStagedBoxes(stagedBoxes);
                m_Octree.DevPublishPhaseEndHook("PublishStagedRegion");

                m_Octree.DevPublishPhaseStartHook();

                foreach (Action change in DeferredRegistryChanges)
                    change();

                m_Octree.DevPublishPhaseEndHook("DeferredRegistryChanges");
            }
            finally
            {
                m_Octree.DevOpTimerPublishEndHook();
                m_Octree.NavTree.ExitWriteLock();
            }

            m_Octree.ConstructionProgress?.SetPublishProgress(PUBLISH_PROGRESS_INSTALLED);

            //P28, post-lock half: the collected originals are now unreachable from the live tree and from every
            //reader (the write lock drained them; readers do not carry node refs across lock releases, and a resuming
            //search holds none in a replaced root — NavTree.AnyLeafReplacedSince). The teardown severs their internal
            //links so a single conservative stack/register pin retains one node instead of a whole island-sized
            //component.
            NavTree.TearDownDetachedSubtrees(replacedOriginals, m_Scratch.TeardownStack);

            m_Octree.ConstructionProgress?.SetPublishProgress(1f);

            //the commit point: from here the operation is complete and its scratch may be reused
            m_Published = true;
        }

        //Restores the construction alias and discards any unfinished op timer — the exact two statements every
        //op's outer `finally` used to hand-roll. Runs on every exit path (normal return, early return,
        //exception) via the caller's `using` declaration, so a failed/cancelled operation never leaves
        //construction pointing at a dead staging grid.
        public void Dispose()
        {
            m_Octree.ActiveRootsGrid = m_Octree.NavTree.LiveRootsGrid;
            m_Octree.DevOpTimerAbandonHook();

            //an operation that did not publish must leave the registry as it found it. The live tree is
            //already untouched on this path — nothing was installed — but the inline half of the cluster
            //unregistration was applied during construction, and leaving it applied is what produced the
            //"Removal error: KeyNotFoundException" seen in play: SafeClustersByID kept listing a cluster
            //ObstaclesByCluster no longer knew. Reverse order, mirroring how the changes were made, and unlocked —
            //deliberately, by symmetry: what it undoes are the build-ONLY maps (the root index and cluster
            //ownership), which the inline half itself writes without the lock because no reader reads them; the
            //reader-visible half never ran on this path, since it is the deferred list the publish applies.
            if (!m_Published && GlobalOctree.RegistryRollbackEnabled)
            {
                for (int i = RegistryRollbacks.Count - 1; i >= 0; i--)
                    RegistryRollbacks[i]();
            }

            //P28 hub severing (same reasoning as NavTree.TearDownDetachedSubtrees):
            //these collections map thousands of node references each, and under the conservative collector a
            //single stale stack/register slot pinning any one of them would retain everything it maps long after
            //the operation. The operation is over on every path that reaches Dispose — sever them. Since the
            //scratch reuse the severing is the scratch's End: contents dropped, capacity kept, the
            //buffers themselves live on for the next operation — or, after an operation that did not complete
            //(no Publish, or cancelled), dropped wholesale so a straggling construction task cannot write into the
            //next operation's buffers (OperationScratch's discard rule).
            StagingGrid.Clear();

            if (m_ConstructionGrid != null)
            {
                m_Scratch.ReturnConstructionGrid(m_ConstructionGrid);
                m_ConstructionGrid.Clear();
            }

            m_ConstructionGrid = null;

            m_Scratch.End(_Discard: !m_Published || CancellationToken.IsCancellationRequested);
        }

        //see m_ConstructionGrid — the worker hands its per-op grid over so the Dispose severing covers it
        public void RegisterConstructionGrid(LeavesSpatialGrid _Grid)
        {
            m_ConstructionGrid = _Grid;
        }

        #endregion

        #region Service methods

        bool IsSharedWithLiveTreeCached(Leaf _Leaf)
        {
            if (!m_SharedOwnerAnswers.TryGetValue(_Leaf, out bool shared))
            {
                shared = IsSharedWithLiveTree(_Leaf);

                m_SharedOwnerAnswers[_Leaf] = shared;
            }

            return shared;
        }

        //Is this very leaf object what the live tree presents at its own cell? True exactly for a leaf the partial
        //clone left SHARED with the live tree (a ring interior leaf today) — and for a live original a staging clone
        //replaced, whose edit is equally reader-visible until the publish swaps it out. A reference compare, not a
        //position compare: a fresh leaf built at the same cell is a different object and reads false. The descent is
        //read-only on the live tree, unlocked — the same discipline as ComputeRingBackRefRepoints and the narrowing
        //plan, safe because operations are serialized and readers never mutate. Paid only for non-clone owners,
        //which the repair reaches rarely.
        bool IsSharedWithLiveTree(Leaf _Leaf)
        {
            return m_Octree.NavTree.LiveRootsGrid.TryGetNode(0, _Leaf.RootIndex, out Node liveRoot)
                   && liveRoot != null
                   && ReferenceEquals(liveRoot.GetClosestOrEmbracingLeaf(_Leaf.Bounds.center), _Leaf);
        }

        //The roots whose occupancy this operation may have changed, as the rebuild actually did it: the roots
        //rebuilt whole (RebuiltRootIndices — which RestoreWholeRootRebuild keeps truthful for the complement and
        //drop branches, and which the pre-baked add widens to the whole clone set) plus the roots rebuilt cell-wise
        //(RebuiltCellsByRoot's keys). Ring roots are only re-wired and are deliberately not in it. Every reader-side
        //annotation that depends on occupancy (the volumetric clearance store) is invalidated from this set and
        //nothing else, so a new operation that changes occupancy anywhere else must land it in one of these two
        //sets.
        HashSet<Vector3Int> OccupancyChangedRootIndices()
        {
            if (RebuiltCellsByRoot.Count == 0)
                return RebuiltRootIndices;

            HashSet<Vector3Int> changed = new HashSet<Vector3Int>(RebuiltRootIndices);

            changed.UnionWith(RebuiltCellsByRoot.Keys);

            return changed;
        }

        #endregion
    }
}
