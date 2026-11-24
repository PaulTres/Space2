using Nav3D.Agents;
using Nav3D.Common;
using System.Collections.Generic;
using System.Collections.Concurrent;
using System.Linq;
using System;
using UnityEngine;

//The namespace is LocalAvoidance's, the assembly is the agent layer's. This class is the frame driver for every
//agent-like body and is edition-neutral -- only the solvers beside which it used to live are Pro -- and the
//avoidance stage reads it from above, so it cannot live in the same assembly as the solvers without making the
//two mutually dependent. The name did not move because it is public.
namespace Nav3D.LocalAvoidance
{
    [DisallowMultipleComponent]
    public partial class AgentManager : MonoBehaviour
    {

        #region Nested types

        enum AgentOperationType
        {
            ADD,
            REMOVE
        }

        #endregion

        #region Attributes

        float m_DangerDistance    = float.MinValue;
        float m_DangerDistanceSqr = float.MinValue;

        readonly HashSet<Nav3DAgentBase>                               m_Agents           = new HashSet<Nav3DAgentBase>();
        readonly ConcurrentQueue<(Nav3DAgentBase, AgentOperationType)> m_AgentsOperations = new ConcurrentQueue<(Nav3DAgentBase, AgentOperationType)>();

        SpatialHashMap<IMovable> m_AgentsStorage;

        //live census of the distinct avoidance radii in play — see MaxRegisteredAvoidanceRadius
        readonly SortedList<float, int> m_RegisteredRadii = new SortedList<float, int>();

        #endregion

        #region Properties

        public static AgentManager Instance => Singleton<AgentManager>.Instance;
        public static bool         Doomed   { get; private set; } = false;

        //Read-only view of the registered agent bodies, for main-THREAD debug readers (Nav3DAgentPathDrawer).
        //The set itself is mutated only while this manager drains m_AgentsOperations in its own update, so a
        //reader on any other thread would be racing an enumeration — there is no lock here to make it safe.
        public IReadOnlyCollection<Nav3DAgentBase> RegisteredAgents => m_Agents;

        float DangerDistance
        {
            get => m_DangerDistance;
            set
            {
                //ReSharper disable once CompareOfFloatsByEqualityOperator
                if (m_DangerDistance == value)
                    return;

                m_DangerDistance    = value;
                m_DangerDistanceSqr = m_DangerDistance * m_DangerDistance;

                m_AgentsStorage.SetBucketSize(m_DangerDistance * 2);
            }
        }

        public float DangerDistanceSqr => m_DangerDistanceSqr;

        float BucketSize => m_DangerDistance * 2;

        public static DateTime Now { get; private set; }

        public float StorageBucketSize => m_AgentsStorage.BucketSize;

        #endregion

        #region Public methods

        public void Initialize(IEnumerable<IMovable> _Agents, float _MaxAgentRadius)
        {
            //also here, not only in Awake: edit mode runs no Awake, and the last play session's OnDestroy left it set
            Doomed = false;

            WorldMaxAgentRadius = _MaxAgentRadius;

            m_AgentsStorage = new SpatialHashMap<IMovable>(BucketSize, _Agents);
        }

        //world-settings-derived bound on any agent's radius — the ceiling an agent may be authored at
        public float WorldMaxAgentRadius { get; private set; }

        /// <summary>
        /// The largest avoidance radius among the movables actually registered right now — the neighbour-range
        /// term for "the other party could be this big". It replaced <see cref="WorldMaxAgentRadius"/> there
        /// because the world ceiling makes every agent pay for the biggest body the SETTINGS allow rather than
        /// the biggest one present: a uniform crowd in a world sized for one large body was querying a range
        /// several times wider than anything could ever need. Falls back to the world ceiling while nothing is
        /// registered. Unlike the spatial hash's bucket size this one SHRINKS again — the counts are kept per
        /// distinct radius, so a large body leaving takes its cost with it.
        /// </summary>
        public float MaxRegisteredAvoidanceRadius => m_RegisteredRadii.Count > 0 ? m_RegisteredRadii.Keys[m_RegisteredRadii.Count - 1] : WorldMaxAgentRadius;

        /// <summary>
        /// Appends every registered movable within _Range of _Position into _Results (list reused by caller).
        /// </summary>
        public void GetMovablesInRange(Vector3 _Position, float _Range, List<IMovable> _Results)
        {
            m_AgentsStorage.GetElementsInRange(_Position, _Range, _Results);
        }

        public void Uninitialize(bool _NeedDestroy = true)
        {
            if (!_NeedDestroy)
                return;

            UtilsCommon.SmartDestroyImmediateWithHolder(this);
        }

        public void RegisterAgent(Nav3DAgentBase _Agent)
        {
            m_AgentsOperations.Enqueue((_Agent, AgentOperationType.ADD));
        }

        public void UnregisterAgent(Nav3DAgentBase _Agent)
        {
            m_AgentsOperations.Enqueue((_Agent, AgentOperationType.REMOVE));
        }

        public void RegisterAgentMover(IMovable _Agent)
        {
            m_AgentsStorage.Insert(_Agent);

            AddRegisteredRadius(_Agent.GetRadius());

            UpdateCellSize((_Agent.GetRadius() + _Agent.GetMaxSpeed()) * 2f);
        }

        public void UnregisterAgentMover(IMovable _Agent)
        {
            m_AgentsStorage?.Remove(_Agent);

            RemoveRegisteredRadius(_Agent.GetRadius());
        }

        //Distinct radii with their live counts (agents share configs, so the map stays a handful of entries
        //however many agents there are). Sorted so the maximum is the last key — O(log distinct) per
        //registration, and the whole point is that it can go down again.
        void AddRegisteredRadius(float _Radius)
        {
            m_RegisteredRadii.TryGetValue(_Radius, out int count);

            m_RegisteredRadii[_Radius] = count + 1;
        }

        void RemoveRegisteredRadius(float _Radius)
        {
            if (!m_RegisteredRadii.TryGetValue(_Radius, out int count))
                return;

            if (count <= 1)
                m_RegisteredRadii.Remove(_Radius);
            else
                m_RegisteredRadii[_Radius] = count - 1;
        }

        /// <summary>
        /// Determines agent bucket, returns all agents from bucket.
        /// </summary>
        /// <param name="_Agent">Agent.</param>
        /// <returns>Agents in bucket.</returns>
        public HashSet<IMovable> GetAgentBucketAgents(IMovable _Agent)
        {
            return m_AgentsStorage.GetElementBucketElements(_Agent);
        }

        /// <summary>
        /// Returns all agents from bucket.
        /// </summary>
        /// <param name="_InsidePoint">The point inside target bucket.</param>
        /// <returns>Agents in bucket.</returns>
        public HashSet<IMovable> GetBucketMovables(Vector3 _InsidePoint)
        {
            return m_AgentsStorage.GetBucketElements(_InsidePoint);
        }

        public IEnumerable<IMovable> GetMovablesInBounds(Bounds _Bounds)
        {
            return m_AgentsStorage.GetMovablesInBounds(_Bounds);
        }

        public bool HasNeighbors(IMovable _Movable)
        {
            return m_AgentsStorage.HasNeighbors(_Movable);
        }

        public Bounds GetStaticObstaclesDangerBounds(IMovable _Movable)
        {
            return m_AgentsStorage.GetStaticObstaclesDangerBounds(_Movable);
        }

        public void SetMovablesInBoundsObstacleDirty(Bounds _Bounds)
        {
            m_AgentsStorage.SetMovablesInBoundsObstacleDirty(_Bounds);
        }

        #endregion

        #region Service methods

        void UpdateCellSize(float _Value)
        {
            if (DangerDistance < _Value)
                DangerDistance = _Value;
        }

        #endregion

        #region Unity methods

        void Awake()
        {
            Doomed = false;
        }

        void OnDestroy()
        {
            Doomed = true;

            Uninitialize(false);

            //Drop the static reference so m_AgentsStorage (the agent spatial hash) is no longer GC-rooted after
            //this destroy — see Singleton<T>.ClearInstance.
            Singleton<AgentManager>.ClearInstance(this);
        }

        void Update()
        {
            Now = DateTime.Now;
        }

        void FixedUpdate()
        {
            while (m_AgentsOperations.Any())
            {
                if (!m_AgentsOperations.TryDequeue(out (Nav3DAgentBase agent, AgentOperationType operation) agentOperation))
                    continue;

                if (agentOperation.operation == AgentOperationType.ADD)
                {
                    m_Agents.Add(agentOperation.agent);

                    continue;
                }

                m_Agents.Remove(agentOperation.agent);
            }

            //one profiler sample around the whole step: the Profiler window and the Skyrift endurance probe's
            //frame-cost arm read it against the streaming stages (a step that swells after an obstacle add is
            //the static-triangle refresh below, Nav3D.NearTrianglesRefresh, not the solver)
            UnityEngine.Profiling.Profiler.BeginSample("Nav3D.AgentsStep");

            //the step is one of the main-thread slices the adaptive search concurrency watches (Nav3DMainThreadCost)
            long stepBegin = Nav3DMainThreadCost.Begin();

            foreach (Nav3DAgentBase agent in m_Agents)
            {
                agent.DoBeforeFixedUpdate();
            }

            foreach (Nav3DAgentBase agent in m_Agents)
            {
                agent.DoFixedUpdate();
            }

            foreach (Nav3DAgentBase agent in m_Agents)
            {
                agent.DoAfterFixedUpdate();
            }

            Nav3DMainThreadCost.End(stepBegin);

            UnityEngine.Profiling.Profiler.EndSample();
        }

        #endregion

    }
}