using AudioSystem;
using UnityEngine;
using Utilities;
namespace Weapons
{
    public class Heatsink : MonoBehaviour, IResettable
    {
        [SerializeField] protected float overheatThreshold = 1.0f;
        [SerializeField] protected float heatLoss = .34f;
        [SerializeField] protected float heatLossWhileOverheated = .3f;
        [SerializeField] protected float heatLossCooldown = 2f;
        [SerializeField] protected SoundData overheatSound;
        public bool Overheated { get; protected set; }
        public float CurrentHeat { get; protected set; }
        public float OverheatPercentage
        {
            get
            {
                return Mathf.Clamp01(CurrentHeat / overheatThreshold);
            }
        }
        protected float lastShotTime;

        protected void OnEnable()
        {
            PerformReset();
            GlobalUpdater.RegisterFixedUpdate(PerformFixedUpdate);
        }

        protected void OnDisable()
        {
            GlobalUpdater.DeregisterFixedUpdate(PerformFixedUpdate);
        }

        public void PerformReset()
        {
            Overheated = false;
            lastShotTime = float.MinValue;
            CurrentHeat = 0;
        }

        public void PerformFixedUpdate(float dt)
        {
            if (Time.time - lastShotTime >= heatLossCooldown)
            {
                CurrentHeat = Mathf.Max(0, CurrentHeat - heatLoss * dt);
                if (CurrentHeat == 0) Overheated = false;
            }
        }

        public void IncreaseHeat(float heat)
        {
            CurrentHeat += heat;
            lastShotTime = Time.time;
            if (CurrentHeat >= overheatThreshold)
            {
                Overheated = true;
                overheatSound?.Play(transform.position);
            }
        }
    }
}