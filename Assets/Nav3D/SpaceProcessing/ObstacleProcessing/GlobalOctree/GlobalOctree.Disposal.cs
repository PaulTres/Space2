using System;
using System.Threading;
using UnityEngine;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree
{
    //The epoch's death unlinks its world (2026-09-18): a
    //discarded object graph must be unlinked, because Unity's conservative collector pins single dead nodes and the
    //graph's linkage turns one pin into everything reachable from it. The per-operation teardown already followed
    //that rule; the largest discard there is — the whole live tree when its obstacle manager is torn down — did not,
    //and stayed resident with nothing referencing it (~750 MB per switch between two heavy worlds in the editor).
    //This unlinks the tree and the rest of what the octree owns.
    partial class GlobalOctree
    {
        #region Attributes

        int m_WorldReleased;

        #endregion

        #region Public methods

        //Idempotent: a teardown reaches it twice (Uninitialize, then OnDestroy's own Uninitialize).
        //It runs ON the OPERATIONS executor, behind whatever was scheduled: the manager has just cancelled every
        //operation's token, so they are winding down, and an exclusive scheduler makes "after them" structural rather than
        //a wait — nothing is unlinked under a build that is still reading it. Inside, the tree's write lock waits out the
        //searches still inside the read lock, the way a publish does, and on a worker, never on the thread tearing down
        internal void ReleaseWorld()
        {
            if (Interlocked.Exchange(ref m_WorldReleased, 1) != 0)
                return;

            m_OperationsExecutor.ScheduleAfterAll(
                    () =>
                    {
                        try
                        {
                            m_NavTree.SeverWorld();
                            m_GeometryRegistry.Clear();

                            //dropped, not cleared: a scratch kept at its high-water mark is sized to the world that just died
                            m_OperationScratch.End(true);

                            ReleaseBakedData();
                        }
                        catch (Exception exception)
                        {
                            Debug.LogException(exception);
                        }
                    }
                );
        }

        #endregion

        #region Service methods

        //the pre-baked image is an edition's content; the shared file only declares that it dies with the world
        partial void ReleaseBakedData();

        #endregion
    }
}
