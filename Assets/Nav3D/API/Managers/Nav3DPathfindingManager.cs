using Nav3D.Pathfinding;

namespace Nav3D.API
{
    public static class Nav3DPathfindingManager
    {
        #region Properties

        /// <summary>
        /// The ceiling on concurrently executing pathfinding tasks. By default, Environment.ProcessorCount - 1;
        /// values below 1 are clamped to 1.
        /// With adaptive search concurrency off, this is exactly how many searches may run at once.
        /// With it on, the adaptive limit works underneath this ceiling and OWNS the live number, so the ceiling is
        /// sealed: a write is refused with a warning and changes nothing. Set "Max concurrent searches" on the
        /// Nav3DWorldSettings asset before initialization, or turn "Adaptive search concurrency" off, to drive it
        /// yourself.
        /// </summary>
        public static int MaxPathfindingTasks
        {
            get
            {
                Nav3DManager.CheckInitedHard();

                return PathfindingManager.Instance.PathFindingTasksMaxCount;
            }
            set
            {
                Nav3DManager.CheckInitedHard();

                PathfindingManager.Instance.PathFindingTasksMaxCount = value;
            }
        }

        /// <summary>
        /// The number of pathfinding tasks that are currently running
        /// </summary>
        public static int CurrentPathfindingTasksCount
        {
            get
            {
                if (!Nav3DManager.Inited)
                    throw new Nav3DManager.Nav3DManagerNotInitializedException();

                return PathfindingManager.Instance.PathFindingTasksOperatingCount;
            }
        }

        #endregion
    }
}