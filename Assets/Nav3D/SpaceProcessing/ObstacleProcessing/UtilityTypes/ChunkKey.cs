using System;
using UnityEngine;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.UtilityTypes
{
    //The address of a chunk of an obstacle's geometry: a cell of the grid aligned with the octree roots, at a level
    //below the root (0 = a root cell, 1 = a half, 2 = a quarter), or WHOLE for an obstacle small enough to be one
    //chunk.
    internal readonly struct ChunkKey : IEquatable<ChunkKey>
    {

        #region Constants

        public static ChunkKey Whole { get; } = new ChunkKey(Vector3Int.zero, -1);

        #endregion

        #region Properties

        public Vector3Int Index { get; }
        public int        Level { get; }

        public bool IsWhole => Level < 0;

        #endregion

        #region Constructors

        public ChunkKey(Vector3Int _Index, int _Level)
        {
            Index = _Index;
            Level = _Level;
        }

        #endregion

        #region Public methods

        public bool Equals(ChunkKey _Other)
        {
            return Level == _Other.Level && Index == _Other.Index;
        }

        public override bool Equals(object _Object)
        {
            return _Object is ChunkKey other && Equals(other);
        }

        public override int GetHashCode()
        {
            unchecked
            {
                int hash = Level;

                hash = hash * 486187739 + Index.x;
                hash = hash * 486187739 + Index.y;
                hash = hash * 486187739 + Index.z;

                return hash;
            }
        }

        public override string ToString()
        {
            return IsWhole ? "WHOLE" : $"L{Level}({Index.x},{Index.y},{Index.z})";
        }

        #endregion

    }
}
