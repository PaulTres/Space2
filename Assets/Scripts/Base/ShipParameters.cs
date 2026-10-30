using Effects;
using UnityEngine;


/// <summary>
/// This class is used to store parameters about a ship/unit on the object.
/// </summary>
public class ShipParameters : MonoBehaviour
{
    public float MaxVelocityMagnitude = 30;
    public float VelocitySoftCap = 29;
    public float TickInterval = 0.2f;
    [Tooltip("Plays when the ship is spawned in.")]
    public PoolableEffect EffectOnSpawn;
    [Tooltip("On spawn, the unit will check for obstructions in a sphere of this radius.")]
    public float SpawnCheckRadius = 5;
    public float EffectScaleFactor = 1;
    [Header("Ship")]
    public float ForwardThrust = 15;
    public float StrafeThrust = 15;
    public float RotationSpeed = 50;
    [Tooltip("Turrets will engage enemy missiles within this distance of the ship.")]
    public float MissileTargetingRange = 50;
    [SerializeField] protected bool validateData = true;

    protected void OnValidate()
    {
        VelocitySoftCap = Mathf.Min(VelocitySoftCap, MaxVelocityMagnitude * .9f);
        if (validateData)
        {
            validateData = false;
            StrafeThrust = Mathf.Min(StrafeThrust, ForwardThrust / 2);
            Debug.Log("Validation performed on " + transform.root);
        }
    }
}
