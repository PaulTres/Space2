using Nav3D.API;

namespace Nav3D.Agents
{
    /// <summary>
    /// The CONSTANT motion mode: the config's desired speed, every tick, read live so a runtime Speed reassignment
    /// applies at once. It does not wrap ConstantSpeedProfile: the profile's rail-distance marching has no meaning
    /// for a re-plannable path.
    /// </summary>
    class ConstantSpeedController : ISpeedController
    {
        #region Attributes

        readonly Nav3DAgentConfig m_Config;

        #endregion

        #region Properties

        public bool NeedsPathContext => false;

        #endregion

        #region Constructors

        public ConstantSpeedController(Nav3DAgentConfig _Config)
        {
            m_Config = _Config;
        }

        #endregion

        #region Public methods

        public float Advance(float _DeltaTime, float _RemainingDistance, float _TotalPathLength)
        {
            return m_Config.Speed;
        }

        public void OnNewOrder(float _EntrySpeed, bool _LoopedOrder)
        {
            //stateless: nothing order-scoped to restart
        }

        #endregion
    }
}
