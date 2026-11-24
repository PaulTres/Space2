using UnityEngine;

namespace Nav3D.Common
{
    //shared end-of-range handling for profiles driven by a plain (distance, direction) pair
    //(ConstantSpeedProfile, AcceleratedSpeedProfile) - Eased tracks extra state (a normalized clock) and
    //resolves its own bounds instead
    static class RailBounds
    {
        //returns true once, when a Stop end behavior reaches its boundary
        public static bool Resolve(ref float _Distance, ref int _Direction, float _Length, MotionEndBehavior _EndBehavior)
        {
            switch (_EndBehavior)
            {
                case MotionEndBehavior.STOP:
                    if (_Distance >= _Length && _Direction > 0)
                    {
                        _Distance = _Length;
                        return true;
                    }

                    if (_Distance <= 0f && _Direction < 0)
                    {
                        _Distance = 0f;
                        return true;
                    }

                    break;

                case MotionEndBehavior.LOOP:
                    _Distance = Mathf.Repeat(_Distance, _Length);
                    break;

                case MotionEndBehavior.PING_PONG:
                    while (_Distance > _Length || _Distance < 0f)
                    {
                        _Distance  = _Distance > _Length ? 2f * _Length - _Distance : -_Distance;
                        _Direction = -_Direction;
                    }

                    break;
            }

            return false;
        }
    }
}