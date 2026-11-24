using UnityEngine;

namespace Nav3D.Demo
{
    //DEMO kit — times obstacle operations for a HUD line and counts the additions still in flight.
    //Each operation carries its own Op, because OnObstacleAdded fires on the first add and on every auto re-add of a moved obstacle, and several
    //operations run at once: a start time and a label kept in shared fields are overwritten by whichever operation spoke last.
    public sealed class DemoObstacleOpMeter
    {
        #region Nested types

        public sealed class Op
        {
            //the addition has not reported yet and still holds a pending slot
            internal bool AddPending { get; set; }

            //0 = nothing armed: a completion that arrives now is an auto re-add nobody asked to time
            internal float StartedAt { get; set; }

            internal string Label { get; set; }
        }

        #endregion

        #region Properties

        public int Pending { get; private set; }

        //null until the first operation finishes
        public string LastLabel { get; private set; }

        public float LastMs { get; private set; }

        #endregion

        #region Public methods

        //an obstacle component is being attached: takes a pending slot and arms the timing
        public Op BeginAdd(string _Label)
        {
            Pending++;

            return new Op { AddPending = true, StartedAt = Time.realtimeSinceStartup, Label = _Label };
        }

        //the obstacle was moved or turned by hand: times the re-add that lands after it
        public void Arm(Op _Op, string _Label)
        {
            _Op.StartedAt = Time.realtimeSinceStartup;
            _Op.Label     = _Label;
        }

        //OnObstacleAdded. True when an armed operation finished, false for an auto re-add nobody timed
        public bool Finish(Op _Op)
        {
            Release(_Op);

            if (_Op.StartedAt <= 0f)
                return false;

            LastMs    = (Time.realtimeSinceStartup - _Op.StartedAt) * 1000f;
            LastLabel = _Op.Label;

            _Op.StartedAt = 0f;

            return true;
        }

        //the obstacle is gone before its addition reported: a cancelled addition never does, and its slot would stay taken
        public void Release(Op _Op)
        {
            if (!_Op.AddPending)
                return;

            _Op.AddPending = false;

            Pending = Mathf.Max(0, Pending - 1);
        }

        //an operation with no Op of its own (a removal): the start time and the label travel with the caller's callback
        public void Record(string _Label, float _StartedAt)
        {
            LastMs    = (Time.realtimeSinceStartup - _StartedAt) * 1000f;
            LastLabel = _Label;
        }

        #endregion
    }
}
