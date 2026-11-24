using Nav3D.API;
using UnityEngine;

namespace Nav3D.Demo
{
    //Watches the authored obstacles being read into the navigation space and measures how far along they are,
    //each weighted by its triangle count
    public class SkylineWorldLoader : MonoBehaviour
    {
        #region Serialized fields

        [SerializeField] Nav3DObstacleRuntime[] m_Obstacles;

        #endregion

        #region Attributes

        float[] m_ObstacleWeights;
        float   m_TotalWeight;

        float m_LoadStart;

        #endregion

        #region Properties

        public float TotalTriangles => m_TotalWeight;

        public float ElapsedSeconds => Time.realtimeSinceStartup - m_LoadStart;

        //a component whose progress object does not exist yet counts as not started: it will register,
        //and its absence must not read as completion
        public bool AllFinished
        {
            get
            {
                bool allFinished = true;

                foreach (Nav3DObstacleRuntime obstacle in m_Obstacles)
                {
                    IObstacleAdditionProgress progress = obstacle != null ? obstacle.AdditionProgress : null;

                    if (progress == null || !progress.Finished)
                        allFinished = false;
                }

                return allFinished;
            }
        }

        public float Fraction
        {
            get
            {
                if (m_ObstacleWeights == null || m_TotalWeight <= 0f)
                    return 0f;

                float sum = 0f;

                for (int i = 0; i < m_Obstacles.Length; i++)
                {
                    IObstacleAdditionProgress progress = m_Obstacles[i] != null ? m_Obstacles[i].AdditionProgress : null;

                    if (progress != null)
                        sum += m_ObstacleWeights[i] * (progress.Finished ? 1f : Mathf.Clamp01(progress.Progress));
                }

                return sum / m_TotalWeight;
            }
        }

        //quotes the engine's phase text rather than rendering the fraction back as a triangle count: the progress is weighted by
        //the operation's phases, and past halfway every triangle is in while repair and publish remain
        public string Status
        {
            get
            {
                foreach (Nav3DObstacleRuntime obstacle in m_Obstacles)
                {
                    IObstacleAdditionProgress progress = obstacle != null ? obstacle.AdditionProgress : null;

                    if (progress != null && !progress.Finished)
                        return progress.GetStatusInfo();
                }

                return string.Empty;
            }
        }

        #endregion

        #region Unity events

        void Start()
        {
            m_LoadStart = Time.realtimeSinceStartup;

            WeighObstacles();
        }

        #endregion

        #region Service methods

        //an obstacle's share of the bar is its triangles (the same meshes the component captures)
        void WeighObstacles()
        {
            m_ObstacleWeights = new float[m_Obstacles.Length];
            m_TotalWeight     = 0f;

            for (int i = 0; i < m_Obstacles.Length; i++)
            {
                float triangles = 0f;

                if (m_Obstacles[i] != null)
                {
                    foreach (MeshFilter filter in m_Obstacles[i].GetComponentsInChildren<MeshFilter>())
                    {
                        //decor under a nested obstacle component (the window bands) is not this obstacle's geometry:
                        //a nested component owns its subtree, so the octree never sees those triangles
                        if (filter.sharedMesh != null && filter.GetComponentInParent<Nav3DObstacle>() == m_Obstacles[i])
                            triangles += filter.sharedMesh.triangles.Length / 3f;
                    }
                }

                //an obstacle with no mesh still counts for something, or it could never register as done
                m_ObstacleWeights[i]  = Mathf.Max(triangles, 1f);
                m_TotalWeight        += m_ObstacleWeights[i];
            }
        }

        #endregion
    }
}
