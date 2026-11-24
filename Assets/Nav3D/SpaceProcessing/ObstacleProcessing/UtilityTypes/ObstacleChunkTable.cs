using System.Collections.Generic;
using UnityEngine;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.UtilityTypes
{
    //One chunk of an obstacle's registered geometry: its address, the fingerprint of its triangles, their bounds and
    //the parts (sub-clusters) they were split into.
    internal sealed class ObstacleChunk
    {

        #region Properties

        public ChunkKey Key { get; }

        //an order-independent combination of the triangles' hashes: the same triangles in any order give the same value
        public ulong Hash { get; }

        public int TriangleCount { get; }

        //the raw bounds of the chunk's triangles
        public Bounds Bounds { get; }

        public ClusterPart[] Parts { get; }

        #endregion

        #region Constructors

        public ObstacleChunk(ChunkKey _Key, ulong _Hash, int _TriangleCount, Bounds _Bounds, ClusterPart[] _Parts)
        {
            Key           = _Key;
            Hash          = _Hash;
            TriangleCount = _TriangleCount;
            Bounds        = _Bounds;
            Parts         = _Parts;
        }

        #endregion

    }

    //An obstacle's geometry as the registry keeps it between updates: its chunks by address. The next update diffs its
    //capture against this table and replaces only what changed.
    internal sealed class ObstacleChunkTable
    {

        #region Attributes

        readonly Dictionary<ChunkKey, ObstacleChunk> m_Chunks;

        #endregion

        #region Properties

        public IReadOnlyDictionary<ChunkKey, ObstacleChunk> Chunks => m_Chunks;

        public int TriangleCount { get; }

        //the union of the chunks' raw bounds
        public Bounds Bounds { get; }

        #endregion

        #region Constructors

        public ObstacleChunkTable(Dictionary<ChunkKey, ObstacleChunk> _Chunks)
        {
            m_Chunks = _Chunks;

            bool   first  = true;
            Bounds bounds = default;
            int    count  = 0;

            foreach (ObstacleChunk chunk in _Chunks.Values)
            {
                count += chunk.TriangleCount;

                if (first)
                {
                    bounds = chunk.Bounds;
                    first  = false;

                    continue;
                }

                bounds.Encapsulate(chunk.Bounds);
            }

            Bounds        = bounds;
            TriangleCount = count;
        }

        #endregion

    }
}
