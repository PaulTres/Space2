using Nav3D.API;
using System;
using System.Threading;
using UnityEngine;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.OperationsProgress
{
    class GraphConstructionProgress
    {
        #region Constants

        public static GraphConstructionProgress INITIAL   => new GraphConstructionProgress { Status = ConstructionStatus.WAITING_FOR_CONSTRUCTION };

        const    string PROCESSING_GENERAL_INFO            = "Processing..";
        const    string TREE_CONSTRUCTION_PREPARATION_INFO = "Preparing to build a tree..";
        readonly string TIRIANGLE_STORAGE_PROGRESS_INFO    = "{0} of {1} triangles registered.";
        readonly string TREE_CONSTRUCTION                  = "Octree construction: {0}";
        readonly string GRAPH_CONNECTIONS_BUILDING_INFO    = "Forming leaves connections. {0} of {1} layers processed.";
        readonly string GRAPH_REPAIR_INFO                  = "Repairing the navigation graph. {0} of {1} leaves verified.";
        const    string PUBLISHING_INFO                    = "Publishing the navigation space..";

        //the BANDS. Each reported phase owns one disjoint, ascending slice of [0, 1], so the operation advances
        //through the bar once. Before this every reporter wrote the raw fraction of its own phase straight into
        //Progress, so a build swept 0 -> 1 as many times as it had phases, and whatever ran after the last reporter
        //(the free-graph repair and the publish, which report here now) left the bar pinned at its top for the rest
        //of the operation.
        //the WIDTHS are measured, and against the build as it SHIPS today rather than the numbers the phase
        //attribution was first taken on: the 2026-09-08 work parallelised the open-face classification (1 478 ->
        //342 ms) and made the free-graph repair a no-op on a first load, so a Skyline add is ~4.3 s of which
        //ConstructRoots ~1.19 s, FormLeavesConnections ~1.84 s, the publish ~0.35 s, and the repair costs nothing
        //until something is actually being REPLACED.
        const float BAND_TRIANGLE_STORAGE_END  = 0.04f;
        const float BAND_TREE_CONSTRUCTION_END = 0.32f;
        const float BAND_CONNECTIONS_END       = 0.86f;
        const float BAND_GRAPH_REPAIR_END      = 0.93f;

        #endregion

        #region Nested types

        public enum ConstructionStatus
        {
            WAITING_FOR_CONSTRUCTION      = 0,
            FILLING_TRIANGLE_STORAGE      = 1,
            TREE_CONSTRUCTION_PREPARATION = 2,
            TREE_CONSTRUCTION             = 3,
            GRAPH_CONNECTIONS_BUILDING    = 4,
            FINISHED                      = 5,

            //The two phases that used to run unannounced, after the workers had already declared the construction
            //FINISHED: they report progress now, so they name themselves too. Appended rather than slotted in front
            //of FINISHED so no existing value moves; the status is a label here, and the durations in Result are
            //still cut by the workers' own SetStatus calls exactly where they always were.
            GRAPH_REPAIR                  = 6,
            PUBLISHING                    = 7
        }

        #endregion

        #region Attributes

        //long, not int: the octree-construction total is roots * 8^maxLayer for the whole global octree, which overflows
        //int for scene-spanning bakes (8^9 ≈ 134M, a couple dozen roots blows past int.MaxValue) — an overflowed total
        //wraps to garbage and the progress sticks near 0% no matter how much is actually built.
        long m_LeavesTotalCount;
        long m_LeavesCounterCurrent;

        int m_TrianglesCounterCurrent;
        int m_TrianglesCounterTotal;

        int m_LayersProcessedTotalCount;
        int m_LayersProcessedCounterCurrent;

        int m_RepairedLeavesTotalCount;
        int m_RepairedLeavesCounterCurrent;

        //Processing start time.
        DateTime m_Start;
        //Processing completion time.
        DateTime m_Finish;
        //Triangle storage building time.
        TimeSpan m_TriangleStorageBuildDuration;
        //Octree build preparations time;
        TimeSpan m_OctreeBulidPreparationsDuration;
        //Octree building time.
        TimeSpan m_OctreeBulidDuration;
        //Navigation graph building time by octree.
        TimeSpan m_GraphBuildDuration;

        CancellationTokenSource m_CTS = new CancellationTokenSource();

        #endregion

        #region Properties

        public ConstructionStatus     Status   { get; private set; }
        public float                  Progress { get; private set; }
        public ObstacleAdditionResult Result   { get; private set; }

        public CancellationToken CancellationToken => m_CTS.Token;

        #endregion

        #region Public methods

        public void SetStatus(ConstructionStatus _Status)
        {
            Status = _Status;

            switch (_Status)
            {
                case ConstructionStatus.FILLING_TRIANGLE_STORAGE:
                    m_Start = DateTime.UtcNow;
                    break;
                case ConstructionStatus.TREE_CONSTRUCTION_PREPARATION:
                    m_TriangleStorageBuildDuration = DateTime.UtcNow - m_Start;
                    break;
                case ConstructionStatus.TREE_CONSTRUCTION:
                    m_OctreeBulidPreparationsDuration = DateTime.UtcNow - m_Start - m_TriangleStorageBuildDuration;
                    break;
                case ConstructionStatus.GRAPH_CONNECTIONS_BUILDING:
                    m_OctreeBulidDuration = DateTime.UtcNow - m_Start - m_TriangleStorageBuildDuration - m_OctreeBulidPreparationsDuration;
                    break;
                case ConstructionStatus.FINISHED:
                    m_Finish             = DateTime.UtcNow;
                    m_GraphBuildDuration = m_Finish - m_Start - m_TriangleStorageBuildDuration - m_OctreeBulidPreparationsDuration - m_OctreeBulidDuration;
                    Result = new ObstacleAdditionResult(
                            m_Start,
                            m_Finish,
                            m_TriangleStorageBuildDuration,
                            m_OctreeBulidPreparationsDuration,
                            m_OctreeBulidDuration,
                            m_GraphBuildDuration
                        );
                    break;
            }
        }

        public string GetInfo()
        {
            switch (Status)
            {
                case ConstructionStatus.FILLING_TRIANGLE_STORAGE:
                    return string.Format(TIRIANGLE_STORAGE_PROGRESS_INFO, m_TrianglesCounterCurrent, m_TrianglesCounterTotal);
                case ConstructionStatus.TREE_CONSTRUCTION_PREPARATION:
                    return TREE_CONSTRUCTION_PREPARATION_INFO;
                case ConstructionStatus.TREE_CONSTRUCTION:
                    //the phase's own fraction, not Progress — Progress is banded over the whole operation now, so
                    //quoting it here would report the tree build as 6-28 % complete for its entire duration
                    return string.Format(TREE_CONSTRUCTION, string.Format("{0:P2}", TreeConstructionFraction()));
                case ConstructionStatus.GRAPH_CONNECTIONS_BUILDING:
                    return string.Format(GRAPH_CONNECTIONS_BUILDING_INFO, m_LayersProcessedCounterCurrent, m_LayersProcessedTotalCount);
                case ConstructionStatus.GRAPH_REPAIR:
                    return string.Format(GRAPH_REPAIR_INFO, m_RepairedLeavesCounterCurrent, m_RepairedLeavesTotalCount);
                case ConstructionStatus.PUBLISHING:
                    return PUBLISHING_INFO;
            }

            return PROCESSING_GENERAL_INFO;
        }

        public void SetTrianglesStorageProgress(int _CurrentCounter, int _TotalCounter)
        {
            m_TrianglesCounterCurrent = _CurrentCounter;
            m_TrianglesCounterTotal   = _TotalCounter;

            Report(Band(0f, BAND_TRIANGLE_STORAGE_END, m_TrianglesCounterTotal > 0 ? m_TrianglesCounterCurrent / (float) m_TrianglesCounterTotal : 0f));
        }

        public void SetLayersProcessignProgress(int _CurrentCounter, int _TotalCounter)
        {
            m_LayersProcessedCounterCurrent = _CurrentCounter;
            m_LayersProcessedTotalCount     = _TotalCounter;

            Status = ConstructionStatus.GRAPH_CONNECTIONS_BUILDING;

            Report(Band(BAND_TREE_CONSTRUCTION_END, BAND_CONNECTIONS_END, m_LayersProcessedCounterCurrent / (float) m_LayersProcessedTotalCount));
        }

        //Free-graph repair: the staged free leaves this pass has verified, of the set it gathered. Sets the status
        //directly rather than through SetStatus, which is the workers' cut line for the Result durations and must
        //stay where they put it — here the status is only the label GetInfo reads.
        public void SetGraphRepairProgress(int _CurrentCounter, int _TotalCounter)
        {
            m_RepairedLeavesCounterCurrent = _CurrentCounter;
            m_RepairedLeavesTotalCount     = _TotalCounter;

            Status = ConstructionStatus.GRAPH_REPAIR;

            Report(Band(BAND_CONNECTIONS_END, BAND_GRAPH_REPAIR_END, _TotalCounter > 0 ? _CurrentCounter / (float) _TotalCounter : 1f));
        }

        //Publish: no counter to divide, so its steps report their own share of the phase as they complete.
        public void SetPublishProgress(float _Fraction)
        {
            Status = ConstructionStatus.PUBLISHING;

            Report(Band(BAND_GRAPH_REPAIR_END, 1f, _Fraction));
        }

        public void SetTotalRootsCount(long _RootsCount)
        {
            m_LeavesTotalCount = _RootsCount;
        }

        public void AddTotalLeavesCount(int _AddValue)
        {
            Interlocked.Add(ref m_LeavesTotalCount, _AddValue);
            ReportTreeConstruction();
        }

        public void AddProcessedLeavesCount(int _AddValue)
        {
            Interlocked.Add(ref m_LeavesCounterCurrent, _AddValue);
            ReportTreeConstruction();
        }

        public void CancelConstruction()
        {
            m_CTS.Cancel();
        }

        #endregion

        #region Service methods

        void ReportTreeConstruction()
        {
            Report(Band(BAND_TRIANGLE_STORAGE_END, BAND_TREE_CONSTRUCTION_END, TreeConstructionFraction()));
        }

        float TreeConstructionFraction()
        {
            long total = Interlocked.Read(ref m_LeavesTotalCount);

            return total > 0 ? Interlocked.Read(ref m_LeavesCounterCurrent) / (float) total : 0f;
        }

        static float Band(float _Start, float _End, float _Fraction)
        {
            return _Start + (_End - _Start) * Mathf.Clamp01(_Fraction);
        }

        //monotone by construction. The counters behind the bands are not: the construction total GROWS while roots
        //are built (AddTotalLeavesCount), and RootBuilder resets it outright for its second entry point, so the raw
        //fraction dips. A bar that walks backwards reads as a stall or a restart, and this one is what a player
        //watches on a demo's splash card, so a lower value is simply not reported.
        //Unsynchronised on purpose: this runs per root inside the parallel construction, and the worst a lost race
        //can do is show a value one update stale.
        void Report(float _Value)
        {
            float value = Mathf.Clamp01(_Value);

            if (value > Progress)
                Progress = value;
        }

        #endregion
    }
}
