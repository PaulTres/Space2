using UnityEngine;

namespace Nav3D.API
{
    public partial class Nav3DObstacleRuntime : Nav3DObstacle
    {
        #region Constants

        const float AUTO_UPDATE_MIN_PERIOD = 1f;

        #endregion

        #region Serialized fields

        [SerializeField] bool  m_AutoUpdateIfChanged;
        [SerializeField] float m_AutoUpdateMinPeriod = AUTO_UPDATE_MIN_PERIOD;

        #endregion

        #region Attributes

        ObstacleAdditionProgress m_AdditionProgress;

        #endregion

        #region Properties

        public override IObstacleAdditionProgress AdditionProgress => m_AdditionProgress;

        public float AutoUpdateMinPeriod
        {
            get => m_AutoUpdateMinPeriod;
            set => m_AutoUpdateMinPeriod = Mathf.Max(0f, value);
        }

        #endregion

        #region Service methods

        partial void CheckEditorDoEnableGuard(ref bool _Skip);

        partial void EnableHook();

        partial void UpdateHook();

        internal override void DoEnable()
        {
            base.DoEnable();

            bool skip = false;
            CheckEditorDoEnableGuard(ref skip);
            if (skip)
                return;

            EnableHook();
        }

        partial void CheckEditorDoUpdateGuard(ref bool _Skip);

        internal override void DoUpdate()
        {
            bool skip = false;
            CheckEditorDoUpdateGuard(ref skip);

            if (skip)
                return;

            UpdateHook();
        }

        #endregion
    }
}
