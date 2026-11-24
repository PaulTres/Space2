using System;
using System.Collections.Generic;
using UnityEngine;

namespace Nav3D.Obstacles
{
    //Immutable per-operation snapshot of the registered resolution regions. Octree construction
    //(GlobalOctree.UpdateMinBucketSize / CheckMaxLayer) reads this frozen view on
    //worker threads instead of re-querying the live ResolutionRegionManager per node. Two reasons:
    //1) the whole build sees one consistent region set — different nodes can no longer observe a region set that a
    //   main-thread Register/Unregister changed mid-construction;
    //2) it removes worker-thread reads of the live manager's mutable storage.
    //The query predicates mirror the manager exactly: "crossing" = region.Bounds.Intersects(query),
    //"embracing" = region.Bounds.Embrace(query). Region count is small, so linear scans match the spatial-hash results.
    sealed class ResolutionRegionSnapshot
    {
        #region Nested types

        public readonly struct Entry
        {
            public readonly Guid   Id;
            public readonly Bounds Bounds;
            public readonly float  MinBucketSize;

            public Entry(Guid _Id, Bounds _Bounds, float _MinBucketSize)
            {
                Id            = _Id;
                Bounds        = _Bounds;
                MinBucketSize = _MinBucketSize;
            }
        }

        #endregion

        #region Attributes

        readonly Entry[] m_Entries;

        #endregion

        #region Properties

        public static ResolutionRegionSnapshot Empty { get; } = new ResolutionRegionSnapshot(System.Array.Empty<Entry>());

        public bool IsEmpty => m_Entries.Length == 0;

        #endregion

        #region Constructors

        public ResolutionRegionSnapshot(Entry[] _Entries)
        {
            m_Entries = _Entries ?? System.Array.Empty<Entry>();
        }

        #endregion

        #region Public methods

        //Min MinBucketSize across all regions (false when there are none).
        //Mirrors ResolutionRegionManager.TryGetCurrentMinBucketSize.
        public bool TryGetMinBucketSize(out float _MinBucketSize)
        {
            _MinBucketSize = float.MaxValue;

            if (m_Entries.Length == 0)
                return false;

            for (int i = 0; i < m_Entries.Length; i++)
            {
                if (m_Entries[i].MinBucketSize < _MinBucketSize)
                    _MinBucketSize = m_Entries[i].MinBucketSize;
            }

            return true;
        }

        //The entries whose bounds intersect _Bounds. Used at bake to capture the regions intersecting an obstacle
        //(GlobalOctree.WriteIntoBinary → per-obstacle RegionDescriptor[]).
        public List<Entry> GetCrossingEntries(Bounds _Bounds)
        {
            List<Entry> result = new List<Entry>();

            for (int i = 0; i < m_Entries.Length; i++)
            {
                if (m_Entries[i].Bounds.Intersects(_Bounds))
                    result.Add(m_Entries[i]);
            }

            return result;
        }

        //Any region whose bounds intersect _Bounds. Mirrors manager.HasCrossingRegions.
        public bool HasCrossingRegions(Bounds _Bounds)
        {
            for (int i = 0; i < m_Entries.Length; i++)
            {
                if (m_Entries[i].Bounds.Intersects(_Bounds))
                    return true;
            }

            return false;
        }

        //Finest (smallest) MinBucketSize among regions whose bounds intersect _Bounds (false when none cross). A node is
        //refined to this resolution when it merely TOUCHES a region — it need not be entirely inside one.
        public bool TryGetCrossingRegionsMinBucketSize(Bounds _Bounds, out float _MinBucketSize)
        {
            _MinBucketSize = float.MaxValue;

            bool any = false;

            for (int i = 0; i < m_Entries.Length; i++)
            {
                if (!m_Entries[i].Bounds.Intersects(_Bounds))
                    continue;

                if (m_Entries[i].MinBucketSize < _MinBucketSize)
                    _MinBucketSize = m_Entries[i].MinBucketSize;

                any = true;
            }

            return any;
        }

        //COARSEST MinBucketSize among the regions crossing _Bounds (false when none cross). The counterpart of the
        //method above, needed for a different reason: CheckMaxLayer CAPS subdivision as well as raising it, so a
        //region asking for less detail than the base makes occupied terminal leaves inside its volume as large as its
        //own bucket size. Anything bounding "how far past its geometry can a newly occupied leaf reach" must use the
        //coarsest crossing region, not the finest.
        public bool TryGetCrossingRegionsMaxBucketSize(Bounds _Bounds, out float _MaxBucketSize)
        {
            _MaxBucketSize = 0f;

            bool any = false;

            for (int i = 0; i < m_Entries.Length; i++)
            {
                if (!m_Entries[i].Bounds.Intersects(_Bounds))
                    continue;

                if (m_Entries[i].MinBucketSize > _MaxBucketSize)
                    _MaxBucketSize = m_Entries[i].MinBucketSize;

                any = true;
            }

            return any;
        }

        #endregion
    }
}
