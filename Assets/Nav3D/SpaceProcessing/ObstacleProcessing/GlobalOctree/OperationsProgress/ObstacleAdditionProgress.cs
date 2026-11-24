using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.OperationsProgress;
using UnityEngine;

namespace Nav3D.API
{
    class ObstacleAdditionProgress : IObstacleAdditionProgress
    {
        #region Constants

        public static ObstacleAdditionProgress INITIAL => new ObstacleAdditionProgress { m_Status = ObstacleAdditionStatus.QUEUE_WAITING };
        const         string                   WAITING_FOR_BEGIN_INFO           = "Waiting for processing to begin..";
        const         string                   OBSTACLES_CLUSTERING_INFO        = "Obstacle clustering..";
        const         string                   OBSOLETE_OBSTACLES_CLEARING_INFO = "Clearing obsolete obstacles..";
        const         string                   FINISHED_INFO                    = "Obstacle addition finished!";
        const         string                   PROCESSING_INFO                  = "Processing";

        //The addition's own bands, over the octree operation the graph construction reports (see the band block in
        //GraphConstructionProgress): clustering owns the head, the construction owns everything up to the finish.
        const float CLUSTERIZATION_PROGRESS    = 0.02f;
        const float CONSTRUCTION_BAND_START    = 0.03f;
        const float CONSTRUCTION_BAND_END      = 0.99f;

        const    string RESULT_STATS_HEADER                             = "Obstacle addition stats:";
        const    string RESULT_STATS_IN_PROGRESS_INFO                   = "Obstacle adding still in progress. Please stand by.";
        readonly string RESULT_STATS_TRIANGLE_STORAGE_CONSTRUCTION_INFO = "Building a triangle storage takes: {0}";
        readonly string RESULT_STATS_OCTREE_PREPARATION_INFO            = "Octree construction preparations takes: {0}";
        readonly string RESULT_STATS_OCTREE_CONSTRUCTION_INFO           = "Octree construction takes: {0}";
        readonly string RESULT_STATS_GRAPH_CONSTRUCTION_INFO            = "Graph construction takes: {0}";
        readonly string RESULT_STATS_TOTAL_PROCESSING_INFO              = "Total processing takes: {0}";

        #endregion

        #region Nested types

        public enum ObstacleAdditionStatus
        {
            QUEUE_WAITING                = 0,
            OBSTACLES_CLUSTERIZATION     = 1,
            OBSTACLES_GRAPH_CONSTRUCTION = 2,
            REMOVING_OBSOLETE_OBSTACLES  = 3,
            FINISHED                     = 4
        }

        #endregion

        #region Attributes

        List<GraphConstructionProgress> m_Constructions = new List<GraphConstructionProgress>();

        //Obstacle addition operation may include the sequence of independent graph construction operations, so here we store just the current one
        GraphConstructionProgress m_CurrentConstructionProgress;
        ObstacleAdditionStatus    m_Status;

        CancellationTokenSource m_CTS = new CancellationTokenSource();

        //what SetProgress writes: an explicitly pushed floor, never below the band the current status gives
        float m_PushedProgress;

        #endregion

        #region Properties

        //the value A user POLLS.
        //It used to be a plain field that nothing in the addition path ever wrote: the only writer was SetFinished,
        //so every caller reading IObstacleAdditionProgress.Progress saw 0 for the whole build and 1 at the end —
        //a loading bar that does not move. The live value was reachable only through GetProgress(), which is not on
        //the public interface. It is computed here now, and GetProgress() is kept as its alias.
        public float Progress
        {
            get
            {
                switch (m_Status)
                {
                    case ObstacleAdditionStatus.OBSTACLES_CLUSTERIZATION:
                        return Mathf.Max(m_PushedProgress, CLUSTERIZATION_PROGRESS);
                    case ObstacleAdditionStatus.OBSTACLES_GRAPH_CONSTRUCTION:
                        return Mathf.Max(
                                m_PushedProgress,
                                CONSTRUCTION_BAND_START +
                                (CONSTRUCTION_BAND_END - CONSTRUCTION_BAND_START) * Mathf.Clamp01(m_CurrentConstructionProgress?.Progress ?? 0f)
                            );
                    case ObstacleAdditionStatus.REMOVING_OBSOLETE_OBSTACLES:
                        return Mathf.Max(m_PushedProgress, CONSTRUCTION_BAND_END);
                    case ObstacleAdditionStatus.FINISHED:
                        return 1f;
                    default:
                        return m_PushedProgress;
                }
            }
        }

        public bool  Finished => m_Status == ObstacleAdditionStatus.FINISHED;

        public ObstacleAdditionResult Result { get; private set; }

        public CancellationToken CancellationToken => m_CTS.Token;

        #endregion

        #region Public methods

        public string GetStatusInfo()
        {
            switch (m_Status)
            {
                case ObstacleAdditionStatus.QUEUE_WAITING:
                    return WAITING_FOR_BEGIN_INFO;
                case ObstacleAdditionStatus.OBSTACLES_CLUSTERIZATION:
                    return OBSTACLES_CLUSTERING_INFO;
                case ObstacleAdditionStatus.OBSTACLES_GRAPH_CONSTRUCTION:
                    return m_CurrentConstructionProgress?.GetInfo() ?? PROCESSING_INFO;
                case ObstacleAdditionStatus.REMOVING_OBSOLETE_OBSTACLES:
                    return OBSOLETE_OBSTACLES_CLEARING_INFO;
                case ObstacleAdditionStatus.FINISHED:
                    return FINISHED_INFO;
                default:
                    return PROCESSING_INFO;
            }
        }

        public string GetResultStats(bool _InMilliseconds = false)
        {
            if (m_Status != ObstacleAdditionStatus.FINISHED)
                return RESULT_STATS_IN_PROGRESS_INFO;

            StringBuilder stringBuilder = new StringBuilder();

            stringBuilder.AppendLine(RESULT_STATS_HEADER);
            stringBuilder.AppendLine(
                    string.Format(
                            RESULT_STATS_TRIANGLE_STORAGE_CONSTRUCTION_INFO,
                            _InMilliseconds ? Result.TriangleStorageBuildDuration.TotalMilliseconds.ToString() : Result.TriangleStorageBuildDuration.ToString()
                        )
                );
            stringBuilder.AppendLine(
                    string.Format(
                            RESULT_STATS_OCTREE_PREPARATION_INFO,
                            _InMilliseconds ? Result.OctreeBulidPreparationsDuration.TotalMilliseconds.ToString() : Result.OctreeBulidPreparationsDuration.ToString()
                        )
                );
            stringBuilder.AppendLine(
                    string.Format(
                            RESULT_STATS_OCTREE_CONSTRUCTION_INFO,
                            _InMilliseconds ? Result.OctreeBulidDuration.TotalMilliseconds.ToString() : Result.OctreeBulidDuration.ToString()
                        )
                );
            stringBuilder.AppendLine(
                    string.Format(
                            RESULT_STATS_GRAPH_CONSTRUCTION_INFO,
                            _InMilliseconds ? Result.GraphBuildDuration.TotalMilliseconds.ToString() : Result.GraphBuildDuration.ToString()
                        )
                );
            stringBuilder.AppendLine(
                    string.Format(
                            RESULT_STATS_TOTAL_PROCESSING_INFO,
                            _InMilliseconds ? Result.TotalProcessingDuration.TotalMilliseconds.ToString() : Result.TotalProcessingDuration.ToString()
                        )
                );

            return stringBuilder.ToString();
        }

        public void SetCurrentConstructionProgress(GraphConstructionProgress _ConstructionProgress)
        {
            m_CurrentConstructionProgress = _ConstructionProgress;
            m_Constructions.Add(_ConstructionProgress);
        }

        public void SetStatus(ObstacleAdditionStatus _Status)
        {
            m_Status = _Status;

            switch (_Status)
            {
                case ObstacleAdditionStatus.FINISHED:
                    Result = ObstacleAdditionResult.GetCombinedResult(m_Constructions.Select(_ConstructionProgress => _ConstructionProgress.Result).ToArray());
                    break;
            }
        }

        public float GetProgress()
        {
            return Progress;
        }

        public void SetProgress(float _Progress)
        {
            m_PushedProgress = Mathf.Clamp01(_Progress);
        }

        public void CancelAddition()
        {
            if (m_Status == ObstacleAdditionStatus.FINISHED)
                return;

            m_CTS.Cancel();
            m_CurrentConstructionProgress?.CancelConstruction();
        }

        public void SetFinished()
        {
            SetStatus(ObstacleAdditionStatus.FINISHED);
            m_PushedProgress = 1f;
        }

        #endregion
    }
}
