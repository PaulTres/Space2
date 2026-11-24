using UnityEngine;

namespace Nav3D.Common
{
    /// <summary>
    /// Flight motion's speed sub-model only: thrust inertia plus a lateral-acceleration turn-slowdown cap.
    /// Steering (pursuit heading, banking, the carrot itself) is geometry-dependent and stays external to
    /// this class - it only tracks the scalar speed budget, fed the steering geometry's curvature each frame.
    /// Does not implement <see cref="ISpeedProfile"/>: it isn't arc-length/end-behavior driven like the
    /// on-rail profiles, it's curvature-driven.
    /// </summary>
    public sealed class FlightSpeedProfile
    {
        public float CurrentSpeed { get; private set; }

        public void Reset(float _InitialSpeed = 0f)
        {
            CurrentSpeed = _InitialSpeed;
        }

        /// <summary>
        /// _Curvature: pure-pursuit arc curvature toward the steering target (2*sin(turnAngle)/range; 0 when
        /// there's no turn). _ArrivalRemainingDistance: when set, additionally caps speed by the braking
        /// envelope toward that remaining distance (Stop end behavior, once the steering target rests at the
        /// path end).
        /// </summary>
        public float Advance(
                float  _DeltaTime,
                float  _MaxSpeed,
                float  _Acceleration,
                float  _MaxLateralAcceleration,
                float  _MaxDeviation,
                float  _Curvature,
                float? _ArrivalRemainingDistance
            )
        {
            //pure-pursuit arc curvature to the target caps the speed via the lateral acceleration budget;
            //MaxDeviation widens the allowed arc (swing-out through the corner), relaxing the braking
            float turnSpeedLimit = _Curvature > 1e-3f
                ? Mathf.Sqrt(_MaxLateralAcceleration * (1f / _Curvature + _MaxDeviation))
                : _MaxSpeed;

            float desiredSpeed = Mathf.Min(_MaxSpeed, turnSpeedLimit);

            if (_ArrivalRemainingDistance.HasValue)
                desiredSpeed = Mathf.Min(desiredSpeed, Mathf.Sqrt(2f * _Acceleration * Mathf.Max(_ArrivalRemainingDistance.Value, 0f)));

            CurrentSpeed = Mathf.MoveTowards(CurrentSpeed, desiredSpeed, _Acceleration * _DeltaTime);

            return CurrentSpeed;
        }
    }
}