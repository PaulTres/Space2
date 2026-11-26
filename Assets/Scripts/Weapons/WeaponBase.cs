using Timers;
using UnityEngine;
using Utilities;
namespace Weapons
{
    public abstract class WeaponBase : MonoBehaviour, IResettable
    {
        #region Helpers
        Unit unit;
        /// <summary>
        /// The unit that owns this Weapons. Lazily initialized by searching for a Unit
        /// component in the root of the transform hierarchy.
        /// </summary>
        public Unit UnitProperty
        {
            get
            {
                if (unit == null)
                {
                    unit = transform.root.GetComponent<Unit>();
                }
                return unit;
            }
        }
        protected CountdownTimer shotTimer;
        protected float currentShotCooldown;
        protected float readinessValue = -1;
        public float ReadinessValue
        {
            get => readinessValue;
            set
            {
                if (readinessValue != value)
                {
                    readinessValue = value;
                    currentShotCooldown = Mathf.Lerp(@params.MinShotCooldown, @params.MaxShotCooldown, @params.GetCurveValue(readinessValue));
                    //if (transform.root.name.Contains("Shadow") && transform.name.Contains("PD")) Debugging.Log($"Curve value: {@params.GetCurveValue(readinessValue)}");
                }
            }
        }

        public BulletData BulletData => @params != null ? @params.BulletData : null;
        /// <summary>
        /// 0 - the weapon is fully reloaded <br/>
        /// 1 - the weapon just fired and has started reloading <br/>
        /// 0.x - the weapon is in the process of reloading, with 0.5 meaning it is halfway through the reload time
        /// </summary>
        public float ReloadStatus
        {
            get
            {
                if (shotTimer.IsRunning) return shotTimer.Progress;
                return 0;
            }
        }
        #endregion

        #region Parameters
        [SerializeField] protected WeaponData @params;
        [Tooltip("A separate bullet will be spawned with the position and rotation of each of these objects.")]
        [SerializeField] protected Transform[] shootPoints;
        [field: SerializeField] public Heatsink Heatsink { get; protected set; }
        #endregion

        #region Setup
        protected virtual void OnValidate()
        {
            if (shootPoints.IsEmpty()) shootPoints = new Transform[] { transform };
            //if (Heatsink == null)Heatsink=GetComponentInParent<Heatsink>();
        }

        protected virtual void Awake()
        {
            Debug.Assert(@params != null, $"{transform} has no Weapons parameters!");
            Debug.Assert(@params.MaxShotCooldown >= 0, $"{transform} has negative cooldown!");
            if (shootPoints.IsEmpty())
            {
                //Debugging.LogWarning($"{transform} has no shoot points!");
                shootPoints = new Transform[] { transform };
            }
        }

        protected virtual void OnEnable()
        {
            PerformReset();
        }

        /// <summary>
        /// Reset shot cooldown.
        /// </summary>
        public virtual void PerformReset()
        {
            shotTimer ??= new(@params.MaxShotCooldown);
            //currentShotCooldown = @params.MaxShotCooldown;
            ReadinessValue = 0;
            shotTimer.Reset(currentShotCooldown);
            if (!@params.StartLoaded) shotTimer.Start();
        }

        protected virtual void OnDisable()
        {
            shotTimer?.Stop();
        }

        protected virtual void OnDestroy()
        {
            shotTimer?.Dispose();
        }
        #endregion

        #region Functionality

        /// <summary>
        /// Increases weapon rate of fire.
        /// </summary>
        /// <param name="deltaTime"></param>
        public virtual void IncreaseReadiness(float deltaTime)
        {
            if (@params.RateOfFireRampUp > 0)
            {
                ReadinessValue = Mathf.Min(1, ReadinessValue + deltaTime * @params.RateOfFireRampUp);
            }
        }

        /// <summary>
        /// Decreases weapon rate of fire.
        /// </summary>
        /// <param name="deltaTime"></param>
        public virtual void DecreaseReadiness(float deltaTime)
        {
            if (@params.RateOfFireRampDown > 0)
            {
                ReadinessValue = Mathf.Max(0, ReadinessValue - deltaTime * @params.RateOfFireRampDown);
            }
        }

        /// <summary>
        /// Checks if the weapon can shoot by using the shot cooldown timer.
        /// </summary>
        /// <returns></returns>
        public virtual bool CanShoot()
        {
            return (shotTimer == null || !shotTimer.IsRunning) && (Heatsink == null || !Heatsink.Overheated);
        }

        #region Shooting
        public bool Shoot(Vector3 targetPosition, EntityBase target)
        {
            if (!CanShoot()) return false;
            shotTimer.Reset(currentShotCooldown);
            shotTimer.Start();
            Fire(targetPosition, target);
            return true;
        }

        /// <summary>
        /// Fire once for every shoot point.
        /// </summary>
        public virtual void Fire(Vector3 targetPosition, EntityBase target)
        {
            for (int i = 0; i < shootPoints.Length; i++)
            {
                if (shootPoints[i] == null || !shootPoints[i].gameObject.activeSelf) continue;
                shootPoints[i].LookAt(targetPosition);
                ProcessBullet(@params.GetInstance(shootPoints[i]), target);
            }
        }

        /// <summary>
        /// Assign the bullet to the Weapons's owner and increase the owner's signature. 
        /// Then activate the bullet.
        /// </summary>
        /// <param name="bullet">The bullet to be processed.</param>
        public Bullet ProcessBullet(Bullet bullet, EntityBase target)
        {
            //UnitProperty.IncreaseSignature(@params.SignatureIncrease);
            bullet.Owner = UnitProperty;
            if (Heatsink != null) Heatsink.IncreaseHeat(@params.HeatPerShot);
            PerformBulletInitialization(bullet, target);
            bullet.gameObject.SetActive(true);
            return bullet;
        }

        /// <summary>
        /// Initializes the bullet with extra information, such as its target.
        /// </summary>
        /// <param name="bullet"></param>
        /// <param name="target"></param>
        protected abstract void PerformBulletInitialization(Bullet bullet, EntityBase target);
        #endregion
        #endregion
    }
}