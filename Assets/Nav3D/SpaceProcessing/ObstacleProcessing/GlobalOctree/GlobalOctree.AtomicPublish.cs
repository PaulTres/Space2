using System.Collections.Generic;
using Nav3D.Common;
using UnityEngine;
using Node = Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.Elements.Node;
using Leaf = Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.Elements.Leaf;
using NodeSpatialGrid = Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.SpatialStorages.NodeSpatialGrid;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree
{
    //Atomic off-tree build → single-patch publish.
    //
    //An obstacle operation builds all its changes off the live tree, on clones of the affected region, so readers
    //keep traversing the old tree during the heavy build; the live tree transitions to the new state in one shot
    //under the write lock (PublishStagedRegion). The region staged is the affected roots plus a one-root
    //face-neighbour ring: construction connects affected edge leaves to the ring, so those ring leaves must be
    //clones too (otherwise the connection would mutate the live tree).
    //A ring root is cloned only down to its boundary layer, though — the leaves against a face looking into a
    //rebuilt root — and its interior is SHARED with the live tree, because that is the only part an operation ever
    //writes to (measured before it was relied on). So "inside the clone set" is no longer the same
    //question as "replaced by this publish": the authority is the original→clone map, and every reference held by
    //a leaf this publish does not replace — an outer leaf beyond the ring, or a shared ring interior leaf — is
    //what needs repointing.
    partial class GlobalOctree
    {
        #region Constants

        //how often the free-graph repair pushes its progress: one report per this many staged free leaves
        const int REPAIR_PROGRESS_REPORT_STRIDE = 2048;

        #endregion

        #region Service methods

        //CloneRootsForStaging and PublishStagedRegion moved to NavTree.cs — both are the only methods
        //here that touch the live m_RootsGrid; everything below operates purely on the caller-supplied staging grid.

        //The one-root-thick face-neighbour ring around an affected root set: every face neighbour of an affected
        //root that is not itself affected. Construction connects affected edge leaves to these roots, so they are
        //cloned into staging (the connection then never touches the live tree); at publish only their outward refs
        //to leaves beyond the ring need repointing (see PublishStagedRegion).
        //internal (not private): StagingTransaction.cs (a standalone class) needs this.
        internal HashSet<Vector3Int> ComputeBoundaryRing(HashSet<Vector3Int> _AffectedRootIndices)
        {
            HashSet<Vector3Int> ring = new HashSet<Vector3Int>();

            foreach (Vector3Int affected in _AffectedRootIndices)
            foreach (Vector3Int direction in UtilsSpatialGrid.FacesDirections)
            {
                Vector3Int neighbor = affected + direction;

                if (!_AffectedRootIndices.Contains(neighbor))
                    ring.Add(neighbor);
            }

            return ring;
        }

        //The root (layer-0) index the leaf belongs to, regardless of the leaf's own layer — derived from the leaf's
        //own grid index rather than from its world position (see Node.RootIndex; the free-graph repair asks this
        //per referenced neighbour).
        Vector3Int RootIndexOfLeaf(Leaf _Leaf)
        {
            return _Leaf.RootIndex;
        }

        //Restores the free-leaf graph invariant over the staged clone set, run on the staging grid just before
        //publish.
        //
        //Reconstruction REPLACES clone leaves through several paths (mutual roots, decomposed clusters, orphaned
        //roots, non-raw roots, PruneExcessiveRoots). Each severs the refs of the leaves it discards, but coverage
        //across paths is incomplete: a surviving clone-set leaf can still hold a FreeNeighbor ref to a discarded
        //clone that was rebuilt into a new object at the same cell — a DANGLING_REF, seen where a shared root is
        //only partially rebuilt. The publish's own ring back-ref pass repoints ring->outer refs only, so it does
        //not catch these.
        //
        //For every staged terminal free leaf, drop each FreeNeighbor whose cell is inside the clone set but is no
        //longer the staging tree's live leaf there, then reconnect the survivor to whatever leaf now occupies that
        //cell when the two are genuinely face-adjacent and free. Refs to leaves outside the clone set belong to the
        //publish's ring back-ref pass and are left untouched. Idempotent: on a clean build nothing is stale.
        //
        //ORDERING is part of this PASS: it must run before StagingTransaction.Publish(), whose first act is to read
        //the staged clones' FreeNeighbors unlocked to work out which outer back-refs to retarget. Repairing
        //afterwards would compute that list against refs this pass then changes.
        //
        //scope. _ReplaceableRootIndices is the transaction's rebuilt set, not its clone set: a ref can only be
        //stale if the leaf it points at was discarded, and leaves are discarded exactly where subtrees are
        //replaced. A ring root's clones are re-wired but never rebuilt, so refs aimed into one are sound by
        //construction and need no verification descent — which is what this pass costs, and roughly 70 % of the
        //clone set is ring. A root rebuilt cell by cell lists its cells in _ReplaceableCellBounds instead; an empty
        //list narrows nothing. A caller that installs subtrees it did not clone must pass the whole clone set — the
        //pre-baked add does, since its baked leaves reference bake-time neighbours including originals the staging
        //clones have replaced.
        //internal, not private: the standalone add/remove workers call it immediately before Publish().
        internal void RepairStagedRegionFreeGraph(
                HashSet<Vector3Int> _CloneSetIndices,
                HashSet<Vector3Int> _ReplaceableRootIndices,
                StagingTransaction  _Transaction,
                NodeSpatialGrid     _StagingGrid
            )
        {
            //the operation's scratch walk set and snapshot buffer; a
            //transaction-less caller keeps local ones
            OperationScratch scratch = _Transaction != null ? ActiveScratch : null;

            HashSet<Leaf> stagedFreeLeaves = scratch?.RepairLeaves ?? new HashSet<Leaf>();
            List<Leaf>    neighborBuffer   = scratch?.NeighborBuffer ?? new List<Leaf>();

            stagedFreeLeaves.Clear();

            //A leaf can only hold a stale ref if something replaced sits across one of its faces, so a root that
            //neither was replaced wholesale nor lies against a replaced cell has nothing to verify. Skipping those
            //roots skips their whole leaf walk, which is most of what this pass used to cost after the rebuild
            //narrowed.
            bool narrowed = _Transaction != null && _Transaction.HasNarrowedCells;

            //experiment (`GlobalOctree.RepairScopeSkipsCreatedRootsEnabled`, ships OFF). A ref can only be stale if
            //the leaf it points at was DISCARDED, and this pass's replaceable set is the operation's whole rebuilt
            //footprint — which on a first build of a world is entirely roots that did not exist before. Nothing was
            //discarded there: those roots were CREATED, not replaced. Measured on a Skyline first load: 1.5 s of
            //tree descents over 506 552 leaves producing zero edits. Narrowing the set to roots the live tree
            //actually holds makes the pass a no-op exactly where it can find nothing, and leaves every update
            //untouched. The live grid is read unlocked, which is the discipline every other pre-publish pass uses
            //(operations are serialized, readers never mutate).
            if (RepairScopeSkipsCreatedRootsEnabled && !narrowed)
            {
                HashSet<Vector3Int> replacedLive = new HashSet<Vector3Int>();

                foreach (Vector3Int rootIndex in _ReplaceableRootIndices)
                {
                    if (m_NavTree.LiveRootsGrid.TryGetNode(0, rootIndex, out Node liveRoot) && liveRoot != null)
                        replacedLive.Add(rootIndex);
                }

                DevCountWork("repair: replaceable roots that existed live", replacedLive.Count);

                if (replacedLive.Count == 0)
                {
                    //the pass costs nothing here, so its whole share of the loading bar is handed over at once
                    //rather than left as a gap the bar would sit in
                    ConstructionProgress?.SetGraphRepairProgress(1, 1);

                    DevCountWork("repair: staged free leaves walked", 0);
                    DevCountWork("repair: edits deferred to the publish (shared-leaf owners)", _Transaction?.DeferredSharedGraphEdits.Count ?? 0);

                    return;
                }

                _ReplaceableRootIndices = replacedLive;
            }

            //Two brackets over this pass, because its two halves answer to different levers: the gather is a walk of
            //the staged region (narrowing shrinks it, parallelism could split it) and the VERIFY is one tree descent
            //per free-neighbour ref (which a "this root had no live original" test could skip outright).
            DevStepStart();

            //the source set, and narrowing it is the whole point — the predicate below was narrowed first and bought
            //nothing, because this pass's cost is the walk, not the test (measured: 117 ms of a 270 ms operation with
            //the predicate already cell-scoped). A ref can only be stale if the leaf it points at was discarded, and
            //discarded leaves are exactly the ones the rebuilt cells replaced; refs are face-adjacency edges, so the
            //leaf holding such a ref is either inside one of those cells or directly across one of its faces. Those
            //two sets are what this gathers — everything else in the clone set is provably unaffected and no longer
            //walked at all.
            if (narrowed)
            {
                foreach (StagingTransaction.RebuiltCell cell in _Transaction.AllRebuiltCells())
                {
                    FindNodeCoveringCell(cell.Index, cell.Layer)?.GetAllContainedFreeLeaves(stagedFreeLeaves);

                    foreach (Vector3Int direction in UtilsSpatialGrid.FacesDirections)
                        FindNodeCoveringCell(cell.Index + direction, cell.Layer)?.GetEdgeLeaves(-direction, stagedFreeLeaves);
                }

                //roots this operation still replaces wholesale (a footprint root the narrowing refused) keep the
                //original treatment — their every leaf can hold a stale ref
                foreach (Vector3Int rootIndex in _ReplaceableRootIndices)
                {
                    if (_StagingGrid.TryGetNode(0, rootIndex, out Node replacedRoot))
                        replacedRoot.GetAllContainedFreeLeaves(stagedFreeLeaves);

                    foreach (Vector3Int direction in UtilsSpatialGrid.FacesDirections)
                    {
                        if (_StagingGrid.TryGetNode(0, rootIndex + direction, out Node neighbourRoot))
                            neighbourRoot.GetEdgeLeaves(-direction, stagedFreeLeaves);
                    }
                }
            }
            else
            {
                foreach (Vector3Int rootIndex in _CloneSetIndices)
                    if (_StagingGrid.TryGetNode(0, rootIndex, out Node root))
                        root.GetAllContainedFreeLeaves(stagedFreeLeaves);
            }

            DevStepEnd("repair: gather staged free leaves");

            DevStepStart();

            Leaf StagedLeafAt(Vector3 _Point)
            {
                return _StagingGrid.TryGetNode(0, UtilsSpatialGrid.GetBucketIndex(_Point, RootSize), out Node root)
                    ? root.GetClosestOrEmbracingLeaf(_Point)
                    : null;
            }

            //This pass is a quarter of a large add (1 572 ms of 6 877 on Skyline) and used to report nothing, so a
            //user's loading bar sat still through it. The gathered set is known here, so the walk over it is the
            //counter.
            int repairedLeaves = 0;
            int repairTotal    = stagedFreeLeaves.Count;

            foreach (Leaf leaf in stagedFreeLeaves)
            {
                //reported in steps, not per leaf: the set runs to hundreds of thousands and no bar shows more
                if (++repairedLeaves % REPAIR_PROGRESS_REPORT_STRIDE == 0)
                    ConstructionProgress?.SetGraphRepairProgress(repairedLeaves, repairTotal);

                //a snapshot (into the scratch buffer), so mutating the underlying set below is safe
                leaf.CopyFreeNeighborsTo(neighborBuffer);

                foreach (Leaf neighbor in neighborBuffer)
                {

                    //Refs that cannot be stale, skipped before the expensive part:
                    //  - crossing outside the clone set (ring→outer live leaf) — the publish's ring back-ref pass
                    //    (ComputeRingBackRefRepoints → PublishStagedRegion) owns those;
                    //  - aimed into a root this operation never replaces (a ring root, unless the caller widened
                    //    the scope) — the leaf they point at is still the staging tree's leaf at that cell.
                    //A leaf sitting inside a replaced cell lies in the root that cell narrowed, so one dictionary
                    //lookup finds every cell that could contain it — never a scan of the operation's whole cell set.
                    Vector3Int neighborRootIndex = RootIndexOfLeaf(neighbor);

                    if (!_ReplaceableRootIndices.Contains(neighborRootIndex) &&
                        !(narrowed                                                                                        &&
                          _Transaction.RebuiltCellsByRoot.TryGetValue(neighborRootIndex, out List<StagingTransaction.RebuiltCell> rootCells) &&
                          TouchesAnyReplaceableCell(neighbor.Bounds, rootCells)))
                        continue;

                    Leaf liveAtCell = StagedLeafAt(neighbor.Bounds.center);

                    //still the staging tree's live leaf at its cell → consistent edge, keep it.
                    if (ReferenceEquals(liveAtCell, neighbor))
                        continue;

                    //discarded clone: sever the stale edge both ways …
                    //…and both ends go through the transaction, which writes inline on a leaf this operation
                    //replaces and DEFERS to the publish's write lock on one shared with the live tree. That is the
                    //whole reason Phase 2 can proceed: this pass's write set is the graph's two-step neighbourhood
                    //of a cell, which no geometric clone predicate can bound, so it is rerouted rather than bounded.
                    EditFreeNeighbor(_Transaction, leaf,     neighbor, false);
                    EditFreeNeighbor(_Transaction, neighbor, leaf,     false);

                    //… and reconnect to whatever replaced it. The dropped cell may have been refined into several
                    //finer leaves; a single center probe reaches only the middle one and leaves the off-center
                    //replacements unlinked — the former "known corner", promoted to a full-face sweep when it
                    //produced real MISSING_FREE_NEIGHBORs (2026-07-13: a surviving coarse shell leaf vs refined
                    //ring-root neighbours across the z=0 root boundary after a pre-baked re-enable).
                    ReconnectAcrossDroppedCell(_Transaction, leaf, neighbor, StagedLeafAt);
                }
            }

            ConstructionProgress?.SetGraphRepairProgress(repairTotal, repairTotal);

            DevStepEnd("repair: verify refs");

            //The count of edits that would otherwise have landed in the live tree unlocked — non-zero by design now
            //that the footprint clone is narrowed (Phase 2). cumulative across the passes that ran before this
            //point (the grid's connection pass and the edge-connection pass defer through the same queue), not the
            //repair's own share; the acceptance oracle for all of them is the clone anatomy's
            //`shared leaves mutated == 0`.
            DevCountWork("repair: edits deferred to the publish (shared-leaf owners)", _Transaction?.DeferredSharedGraphEdits.Count ?? 0);

            //What the pass WALKED, beside what it changed: a large walk with no edits is the shape the parallelism
            //study found on a first load, and only these two numbers together can tell it from a repair that worked.
            DevCountWork("repair: staged free leaves walked", stagedFreeLeaves.Count);
        }

        //One free-neighbour edit, routed by who OWNS the leaf being written to. With no transaction (a caller that
        //has none) it is the plain write this pass always made.
        static void EditFreeNeighbor(StagingTransaction _Transaction, Leaf _Owner, Leaf _Neighbour, bool _Add)
        {
            if (_Transaction == null)
            {
                if (_Add)
                    _Owner.AddFreeNeighbor(_Neighbour);
                else
                    _Owner.RemoveFreeNeighbor(_Neighbour);

                return;
            }

            _Transaction.EditFreeNeighbor(_Owner, _Neighbour, _Add);
        }

        //Does the volume lie in, or against, a cell this operation replaced? Tolerance 0 asks "is this leaf itself
        //inside a replaced cell"; a positive tolerance asks "could a leaf in this volume hold a ref into one".
        static bool TouchesAnyReplaceableCell(Bounds _Bounds, List<StagingTransaction.RebuiltCell> _Cells)
        {
            foreach (StagingTransaction.RebuiltCell cell in _Cells)
            {
                if (cell.Bounds.Intersects(_Bounds))
                    return true;
            }

            return false;
        }

        //Reconnects _Survivor to every free leaf now occupying the dropped neighbour's cell across their shared face.
        //Sweeps the overlap rectangle of the two faces on the dropped cell's side: fixed min-bucket steps along one
        //face axis, adaptive advance (by the found leaf's span) along the other, so a coarse dropped cell refined to
        //min-bucket leaves costs bounded probes while nothing finer than min bucket can be skipped. AddFreeNeighbor
        //is set-idempotent, so re-finding the same replacement from several probe points is harmless.
        void ReconnectAcrossDroppedCell(StagingTransaction _Transaction, Leaf _Survivor, Leaf _Dropped, System.Func<Vector3, Leaf> _StagedLeafAt)
        {
            Bounds survivorBounds = _Survivor.Bounds;
            Bounds droppedBounds  = _Dropped.Bounds;

            //shared-face axis = dominant axis of the center offset; the face plane lies on the survivor's boundary
            Vector3 delta = droppedBounds.center - survivorBounds.center;

            int axis = Mathf.Abs(delta.x) >= Mathf.Abs(delta.y) && Mathf.Abs(delta.x) >= Mathf.Abs(delta.z) ? 0
                : Mathf.Abs(delta.y) >= Mathf.Abs(delta.z)                                                   ? 1
                                                                                                             : 2;

            int uAxis = (axis + 1) % 3;
            int vAxis = (axis + 2) % 3;

            float sign  = Mathf.Sign(delta[axis]);
            float inset = m_MinBucketSizeReal * 0.25f;

            //probe just across the survivor's face, on the dropped cell's side
            float probeDepth = survivorBounds.center[axis] + (survivorBounds.extents[axis] + inset) * sign;

            float uMin = Mathf.Max(survivorBounds.min[uAxis], droppedBounds.min[uAxis]);
            float uMax = Mathf.Min(survivorBounds.max[uAxis], droppedBounds.max[uAxis]);
            float vMin = Mathf.Max(survivorBounds.min[vAxis], droppedBounds.min[vAxis]);
            float vMax = Mathf.Min(survivorBounds.max[vAxis], droppedBounds.max[vAxis]);

            float step = m_MinBucketSizeReal;

            for (float u = uMin + inset; u < uMax; u += step)
            {
                float v = vMin + inset;

                while (v < vMax)
                {
                    Vector3 point = Vector3.zero;
                    point[axis]  = probeDepth;
                    point[uAxis] = u;
                    point[vAxis] = v;

                    Leaf  candidate = _StagedLeafAt(point);
                    float advance   = step;

                    if (candidate != null)
                    {
                        if (!candidate.Occupied && !ReferenceEquals(candidate, _Survivor) && AreLeavesAdjacent(_Survivor, candidate))
                        {
                            //same rerouting as the sever above: inline on a clone, deferred to the write lock on a
                            //leaf shared with the live tree
                            EditFreeNeighbor(_Transaction, _Survivor,  candidate, true);
                            EditFreeNeighbor(_Transaction, candidate, _Survivor,  true);
                        }

                        //skip the remainder of this leaf's span along v — nothing finer can hide inside it
                        advance = Mathf.Max(step, candidate.Bounds.max[vAxis] - v);
                    }

                    v += advance;
                }
            }
        }

        #endregion
    }
}
