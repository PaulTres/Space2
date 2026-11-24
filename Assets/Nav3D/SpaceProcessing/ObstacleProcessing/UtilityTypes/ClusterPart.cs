using Nav3D.Common;
using Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.SpatialStorages;
using System;
using UnityEngine;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.UtilityTypes
{
    //One sub-cluster of one chunk: a triangle set with its own BVH, shared by every cluster version that keeps it. A
    //cluster is a set of parts; an update that leaves a chunk's hash unchanged reuses its parts, so the BVH of
    //untouched geometry is never rebuilt.
    internal sealed class ClusterPart : IBoundable
    {

        #region Attributes

        readonly object m_StorageLock = new object();

        TriangleStorage m_Storage;

        #endregion

        #region Properties

        public Triangle[] Triangles { get; private set; }

        //the raw bounds of the part's triangles
        public Bounds Bounds { get; }

        public ChunkKey Chunk { get; }

        //built on first use, once, for the part's life
        public TriangleStorage Storage
        {
            get
            {
                TriangleStorage storage = m_Storage;

                if (storage != null)
                    return storage;

                lock (m_StorageLock)
                {
                    if (m_Storage != null)
                        return m_Storage;

                    System.Diagnostics.Stopwatch watch = System.Diagnostics.Stopwatch.StartNew();

                    m_Storage = new TriangleStorage(Triangles);

                    GlobalOctree.GlobalOctree.DevWorkSink?.Invoke("parts: BVH builds", 1);
                    GlobalOctree.GlobalOctree.DevWorkSink?.Invoke("parts: BVH build ms", watch.ElapsedMilliseconds);

                    return m_Storage;
                }
            }
        }

        public bool Released { get; private set; }

        #endregion

        #region Constructors

        public ClusterPart(Triangle[] _Triangles, Bounds _Bounds, ChunkKey _Chunk)
        {
            Triangles = _Triangles;
            Bounds    = _Bounds;
            Chunk     = _Chunk;
        }

        #endregion

        #region Public methods

        //Severs the triangles and the BVH once no live cluster holds the part (the P28 hub rule: a pinned dead part
        //must retain one object, not a chunk's geometry).
        public void ReleaseGeometry()
        {
            lock (m_StorageLock)
            {
                Triangles = Array.Empty<Triangle>();
                m_Storage = null;
                Released  = true;
            }
        }

        #endregion

    }
}
