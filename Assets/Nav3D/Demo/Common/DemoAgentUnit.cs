using Nav3D.API;
using UnityEngine;

namespace Nav3D.Demo
{
    //DEMO kit — a unit flown by a Nav3DAgent. The config on the prefab's agent is the one place its size lives: the body is
    //scaled to that radius when the unit wakes, so the meshes under it are authored at unit extent.
    public class DemoAgentUnit : DemoUnit
    {
        #region Serialized fields

        [SerializeField] Nav3DAgent m_Agent;

        [Tooltip("Scaled to the agent's diameter at Awake — the meshes under it are authored at unit extent, so the config's Radius is the one size to tune")]
        [SerializeField] Transform m_Body;

        #endregion

        #region Properties

        public Nav3DAgent Agent => m_Agent;

        public float Radius => m_Agent.Config.Radius;

        #endregion

        #region Unity events

        protected virtual void Awake()
        {
            FitBody(Radius);
        }

        #endregion

        #region Public methods

        //an instance's agent would fly a copy of its config; handed the asset itself, it follows an edit made in the inspector while
        //the scene plays, and so does every other agent spawned from the same prefab
        public static T Spawn<T>(T _Prefab, Vector3 _Position, Transform _Parent) where T : DemoAgentUnit
        {
            T unit = Instantiate(_Prefab, _Position, Quaternion.identity, _Parent);

            unit.Agent.SetConfig(_Prefab.Agent.Config);

            return unit;
        }

        //for a scene that draws the body at a radius other than the config's own
        public void FitBody(float _Radius)
        {
            m_Body.localScale = Vector3.one * (_Radius * 2f);
        }

        #endregion
    }
}
