using Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.Hierarchy;
using System;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree
{
    //The tree's region map (hierarchical pathfinding, roadmap A4): built only where the flat search has shown it is
    //expensive, kept only while it delivers plans, and otherwise never built at all — on an open world the flat
    //search is already cheap and the map is pure cost (P51). Armed by one flat search above ARM_EXPANSIONS; retired
    //when a build finds no doorway or a window of plan attempts delivers none; armed again only after RETIRE_PUBLISHES
    //publishes, since a world that did not change cannot have gained rooms. While armed it is built on a worker and replaced whole. A publish
    //marks the roots it rebuilt (NoteRebuild, under the write lock) and the next build repaints the field only inside
    //them, so a mover costs a few roots of painting and one transform. The old map serves while a build runs — a stale
    //plan costs a failed leg and a fall back to the flat search, never a wrong route, because every leg is searched on
    //the live tree. With no map yet, a search starts the first build and runs flat; nothing ever waits on a build but
    //the probes that ask to.
    internal sealed partial class NavTree
    {
        #region Constants

        //a flat search this expensive is where the heuristic's slack could be the cost the hierarchy removes: TUNNEL's
        //flat search spends ~155 000 expansions a request, CLUTTERED's ~5 000
        const int ARM_EXPANSIONS = 20_000;

        //the plan attempts judged at a time; a window that delivers no plan retires the map
        const int PLAN_WINDOW = 64;

        //publishes after a retirement before an expensive search may arm the map again
        const int RETIRE_PUBLISHES = 256;

        #endregion

        #region Attributes

        //armed: the map is wanted and kept built; m_RegionMapRetiredEpoch is the publish epoch of the last retirement
        volatile bool m_RegionMapArmed;
        int           m_RegionMapRetiredEpoch = int.MinValue / 2;

        //the current plan window: attempts, and plans delivered
        int m_PlanWindowAttempts;
        int m_PlanWindowDelivered;

        volatile RegionMap m_RegionMap;

        //what the next build owes, under the gate: the roots rebuilt since the last build started, or the whole world
        readonly HashSet<Vector3Int> m_RegionMapDirtyRoots   = new HashSet<Vector3Int>();
        readonly object              m_RegionMapGate         = new object();
        bool                         m_RegionMapWholeRebuild = true;

        int           m_RegionMapBuilding;
        volatile bool m_RegionMapDirty;
        volatile bool m_RegionMapSevered;

        //the builds so far, for the probes: how many, their total and their longest
        long   m_RegionMapBuilds;
        long   m_RegionMapRetirements;
        double m_RegionMapBuildMilliseconds;
        double m_RegionMapBuildMillisecondsMax;

        #endregion

        #region Properties

        internal RegionMap RegionMap => m_RegionMap;

        internal GlobalOctree Octree => m_Octree;

        internal float HierarchyThreshold => m_Octree.MaxAgentRadius * GlobalOctree.HierarchyCoreClearanceFactor;

        internal bool RegionMapBuilding => m_RegionMapBuilding != 0;

        internal bool RegionMapArmed => m_RegionMapArmed;

        internal (long Builds, double TotalMilliseconds, double MaxMilliseconds) RegionMapBuildStats
            => (Interlocked.Read(ref m_RegionMapBuilds), m_RegionMapBuildMilliseconds, m_RegionMapBuildMillisecondsMax);

        internal long RegionMapRetirements => Interlocked.Read(ref m_RegionMapRetirements);

        #endregion

        #region Public methods

        //after every publish (GlobalOctree.NotifyRegionChanged, on the operation's thread): the build the publish's
        //NoteRebuild left owed is started, publishes that land while one build runs coalesce into the next; with the
        //hierarchy off the map is dropped
        internal void ScheduleRegionMapRebuild()
        {
            if (!GlobalOctree.HierarchyEnabled)
            {
                m_RegionMap = null;

                return;
            }

            if (m_RegionMapArmed)
                StartBuildIfIdle();
        }

        //after every flat search that was not a hierarchy leg: one expensive enough arms the map
        internal void NoteFlatSearchCost(int _Expansions)
        {
            if (_Expansions < ARM_EXPANSIONS || m_RegionMapArmed || m_RegionMapSevered || !GlobalOctree.HierarchyEnabled)
                return;

            if (m_PublishEpoch - Volatile.Read(ref m_RegionMapRetiredEpoch) < RETIRE_PUBLISHES)
                return;

            Arm();
        }

        //after every plan attempt: a window of PLAN_WINDOW attempts that delivered nothing retires the map
        internal void NoteRegionPlan(bool _Delivered)
        {
            if (_Delivered)
                Interlocked.Increment(ref m_PlanWindowDelivered);

            if (Interlocked.Increment(ref m_PlanWindowAttempts) < PLAN_WINDOW)
                return;

            lock (m_RegionMapGate)
            {
                if (m_PlanWindowAttempts < PLAN_WINDOW)
                    return;

                bool earned = m_PlanWindowDelivered > 0;

                m_PlanWindowAttempts  = 0;
                m_PlanWindowDelivered = 0;

                if (!earned)
                    RetireUnderGate();
            }
        }

        //the map a search plans on: the current one — its replacement started here when a publish left one owed and
        //nothing else did (an edit-mode add notifies nobody) — or null while the first build, started here, runs
        internal RegionMap RegionMapForSearch()
        {
            if (m_RegionMapSevered || !m_RegionMapArmed)
                return null;

            RegionMap map = m_RegionMap;

            if (map == null)
                RequestWholeRebuild();

            if (m_RegionMapDirty)
                StartBuildIfIdle();

            return map;
        }

        //for the probes: the map that answers for every publish so far — armed here unless _Arm is false, since a probe
        //asking for the map measures the hierarchy, not its trigger — or null on timeout, when the build retired it
        //(no doorway), or when nothing armed it
        internal RegionMap WaitForRegionMap(int _TimeoutMilliseconds, bool _Arm = true)
        {
            DateTime deadline = DateTime.UtcNow.AddMilliseconds(_TimeoutMilliseconds);

            if (!m_RegionMapArmed && _Arm)
                Arm();

            RegionMapForSearch();

            while (DateTime.UtcNow < deadline)
            {
                if (!m_RegionMapArmed)
                    return null;

                RegionMap map = m_RegionMap;

                if (map != null && map.Epoch >= m_PublishEpoch && !m_RegionMapDirty && m_RegionMapBuilding == 0)
                    return map;

                Thread.Sleep(5);
            }

            return null;
        }

        #endregion

        #region Service methods

        //from NoteRebuild, under the write lock: the roots this publish rebuilt, owed to the next build
        void MarkRegionMapRoots(HashSet<Vector3Int> _RebuiltRootIndices)
        {
            if (!m_RegionMapArmed)
                return;

            lock (m_RegionMapGate)
            {
                m_RegionMapDirtyRoots.UnionWith(_RebuiltRootIndices);

                m_RegionMapDirty = true;
            }
        }

        void Arm()
        {
            lock (m_RegionMapGate)
            {
                if (m_RegionMapArmed)
                    return;

                m_PlanWindowAttempts  = 0;
                m_PlanWindowDelivered = 0;
                m_RegionMapArmed      = true;
            }

            RequestWholeRebuild();
            StartBuildIfIdle();
        }

        //under the gate: the map dropped and no longer rebuilt; a running build finishes and is discarded
        void RetireUnderGate()
        {
            m_RegionMapArmed        = false;
            m_RegionMap             = null;
            m_RegionMapDirty        = false;
            m_RegionMapRetiredEpoch = m_PublishEpoch;

            m_RegionMapDirtyRoots.Clear();

            Interlocked.Increment(ref m_RegionMapRetirements);
        }

        void RequestWholeRebuild()
        {
            lock (m_RegionMapGate)
            {
                m_RegionMapWholeRebuild = true;
                m_RegionMapDirty        = true;
            }
        }

        void StartBuildIfIdle()
        {
            if (m_RegionMapSevered || Interlocked.CompareExchange(ref m_RegionMapBuilding, 1, 0) != 0)
                return;

            Task.Run(BuildLoop);
        }

        void BuildLoop()
        {
            try
            {
                while (m_RegionMapDirty && !m_RegionMapSevered)
                {
                    bool         whole;
                    Vector3Int[] changed;

                    lock (m_RegionMapGate)
                    {
                        whole   = m_RegionMapWholeRebuild || m_RegionMap == null;
                        changed = new Vector3Int[m_RegionMapDirtyRoots.Count];

                        m_RegionMapDirtyRoots.CopyTo(changed);
                        m_RegionMapDirtyRoots.Clear();

                        m_RegionMapWholeRebuild = false;
                        m_RegionMapDirty        = false;
                    }

                    RegionMap built = RegionMapBuilder.Build(this, HierarchyThreshold, () => m_RegionMapSevered, whole ? null : m_RegionMap, whole ? null : changed);

                    if (m_RegionMapSevered)
                        return;

                    if (built == null)
                    {
                        //an empty world, or a build that gave up: the next publish builds whole
                        lock (m_RegionMapGate)
                            m_RegionMapWholeRebuild = true;

                        continue;
                    }

                    Interlocked.Increment(ref m_RegionMapBuilds);

                    m_RegionMapBuildMilliseconds   += built.BuildMilliseconds;
                    m_RegionMapBuildMillisecondsMax = Math.Max(m_RegionMapBuildMillisecondsMax, built.BuildMilliseconds);

                    lock (m_RegionMapGate)
                    {
                        //retired while it was built
                        if (!m_RegionMapArmed)
                            return;

                        //a map with no doorway plans nothing, whatever is asked of it
                        if (built.Doorways.Count == 0)
                        {
                            RetireUnderGate();

                            return;
                        }

                        m_RegionMap = built;
                    }
                }
            }
            catch (Exception exception)
            {
                Debug.LogException(exception);

                lock (m_RegionMapGate)
                    m_RegionMapWholeRebuild = true;
            }
            finally
            {
                Interlocked.Exchange(ref m_RegionMapBuilding, 0);

                //a publish that landed after the loop's last check and before the flag dropped
                if (m_RegionMapDirty && !m_RegionMapSevered)
                    StartBuildIfIdle();
            }
        }

        //the world's death (SeverWorld): no map, no build
        void SeverRegionMap()
        {
            m_RegionMapSevered = true;
            m_RegionMap        = null;
        }

        #endregion
    }
}
