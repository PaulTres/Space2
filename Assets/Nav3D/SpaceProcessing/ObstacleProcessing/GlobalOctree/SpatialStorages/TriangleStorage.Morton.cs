using System;
using Nav3D.Common;
using UnityEngine;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.SpatialStorages
{
    partial class TriangleStorage
    {

        #region Constants

        //Conservative large-mesh cutoff measured on synthetic and full-world builds.
        //Neither constant is a public setting.
        const int    MORTON_MIN_TRIANGLES       = 65536;
        const double MORTON_MAX_RELATIVE_EXTENT = 0.125;

        #endregion

        #region Nested types

        enum BvhBuildStrategy
        {
            SAH,
            MORTON
        }

        struct MortonPrimitive : IComparable<MortonPrimitive>
        {
            public uint    Key           { get; set; }
            public int     OriginalIndex { get; set; }
            public Vector3 Min           { get; set; }
            public Vector3 Max           { get; set; }

            public int CompareTo(MortonPrimitive _Other)
            {
                int comparison = Key.CompareTo(_Other.Key);
                return comparison != 0 ? comparison : OriginalIndex.CompareTo(_Other.OriginalIndex);
            }
        }

        #endregion

        #region Attributes

        MortonPrimitive[] m_MortonPrimitives;
        Triangle[]        m_MortonReorder;

        #endregion

        #region BVH Construction

        BvhBuildStrategy SelectBuildStrategy()
        {
            if (m_TriangleCount < MORTON_MIN_TRIANGLES)
                return BvhBuildStrategy.SAH;

            Vector3 min         = Vector3.one * float.MaxValue;
            Vector3 max         = -min;
            double  totalExtent = 0;

            for (int i = 0; i < m_TriangleCount; i++)
            {
                Bounds  bounds = m_Triangles[i].Bounds;
                Vector3 lo     = bounds.min;
                Vector3 hi     = bounds.max;
                min = Vector3.Min(min, lo);
                max = Vector3.Max(max, hi);
                Vector3 size = hi - lo;
                totalExtent += size.x + size.y + size.z;
            }

            Vector3 span        = max             - min;
            double  worldExtent = (double) span.x + span.y + span.z;

            //Wide primitives / coincident box centres are a poor signal for centroid ordering. Keep SAH conservatively.
            if (double.IsNaN(worldExtent) || double.IsInfinity(worldExtent) || worldExtent <= 0 ||
                double.IsNaN(totalExtent) || double.IsInfinity(totalExtent) ||
                totalExtent > worldExtent * m_TriangleCount * MORTON_MAX_RELATIVE_EXTENT)
                return BvhBuildStrategy.SAH;

            return BvhBuildStrategy.MORTON;
        }

        static uint SpreadMortonBits(uint _Bits)
        {
            _Bits &= 1023;
            _Bits =  (_Bits | (_Bits << 16)) & 0x030000FF;
            _Bits =  (_Bits | (_Bits << 8))  & 0x0300F00F;
            _Bits =  (_Bits | (_Bits << 4))  & 0x030C30C3;
            return (_Bits | (_Bits << 2)) & 0x09249249;
        }

        bool BuildMorton()
        {
            int count = m_TriangleCount;

            if (m_MortonPrimitives == null || m_MortonPrimitives.Length < count)
                m_MortonPrimitives = new MortonPrimitive[m_Triangles.Length];

            if (m_MortonReorder == null || m_MortonReorder.Length < count)
                m_MortonReorder = new Triangle[m_Triangles.Length];

            Vector3 min = Vector3.one * float.MaxValue;
            Vector3 max = -min;

            for (int i = 0; i < count; i++)
            {
                Bounds  bounds = m_Triangles[i].Bounds;
                Vector3 lo     = bounds.min;
                Vector3 hi     = bounds.max;
                m_MortonPrimitives[i] = new MortonPrimitive { OriginalIndex = i, Min = lo, Max = hi };
                min                   = Vector3.Min(min, lo);
                max                   = Vector3.Max(max, hi);
            }

            Vector3 extent = max - min;

            for (int axis = 0; axis < 3; axis++)
            {
                //Sorting is optional indexing only. Preserve the original SAH path for unsupported numeric ranges.
                if (float.IsNaN(min[axis])    || float.IsInfinity(min[axis]) ||
                    float.IsNaN(extent[axis]) || float.IsInfinity(extent[axis]))
                    return false;

                if (extent[axis] <= 0)
                    extent[axis] = 1;
            }

            for (int i = 0; i < count; i++)
            {
                ref MortonPrimitive primitive = ref m_MortonPrimitives[i];
                Vector3             center    = primitive.Min * 0.5f + primitive.Max * 0.5f;
                uint                x         = (uint) (Mathf.Clamp01((center.x - min.x) / extent.x) * 1023f);
                uint                y         = (uint) (Mathf.Clamp01((center.y - min.y) / extent.y) * 1023f);
                uint                z         = (uint) (Mathf.Clamp01((center.z - min.z) / extent.z) * 1023f);
                primitive.Key = SpreadMortonBits(x) | (SpreadMortonBits(y) << 1) | (SpreadMortonBits(z) << 2);
            }

            Array.Sort(m_MortonPrimitives, 0, count);

            try
            {
                for (int i = 0; i < count; i++)
                    m_MortonReorder[i] = m_Triangles[m_MortonPrimitives[i].OriginalIndex];

                Array.Copy(m_MortonReorder, m_Triangles, count);
            }
            finally
            {
                Array.Clear(m_MortonReorder, 0, count);
            }

            int leafCount = 1;

            while (leafCount < (count + 3) / 4)
                leafCount *= 2;

            int nodeCount = leafCount * 2 - 1;

            if (m_Nodes == null || m_Nodes.Length < nodeCount)
                m_Nodes = new BvhNode[nodeCount];

            for (int i = 0; i < leafCount; i++)
            {
                Vector3 lo     = Vector3.one * float.MaxValue;
                Vector3 hi     = -lo;
                int     start  = i * 4;
                int     length = Math.Max(0, Math.Min(4, count - start));

                for (int k = 0; k < length; k++)
                {
                    ref MortonPrimitive primitive = ref m_MortonPrimitives[start + k];
                    lo = Vector3.Min(lo, primitive.Min);
                    hi = Vector3.Max(hi, primitive.Max);
                }

                m_Nodes[leafCount - 1 + i] = new BvhNode
                {
                    BoundsMin = lo, BoundsMax = hi, StartOrLeft = start, CountOrRight = length
                };
            }

            for (int i = leafCount - 2; i >= 0; i--)
            {
                int         left       = i * 2 + 1;
                int         right      = left  + 1;
                ref BvhNode a          = ref m_Nodes[left];
                ref BvhNode b          = ref m_Nodes[right];
                bool        emptyLeft  = a.BoundsMin.x > a.BoundsMax.x;
                bool        emptyRight = b.BoundsMin.x > b.BoundsMax.x;

                if (emptyLeft && !emptyRight)
                    m_Nodes[i] = b;
                else if (emptyRight && !emptyLeft)
                    m_Nodes[i] = a;
                else
                    m_Nodes[i] = new BvhNode
                    {
                        BoundsMin   = Vector3.Min(a.BoundsMin, b.BoundsMin),
                        BoundsMax   = Vector3.Max(a.BoundsMax, b.BoundsMax),
                        StartOrLeft = left, CountOrRight = -right
                    };
            }

            m_NodeCount = nodeCount;
            return true;
        }

        #endregion

    }
}