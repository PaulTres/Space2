using Nav3D.Common;
using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.UtilityTypes
{
    public class ObstacleInfo : IBoundable
    {

        #region Attributes

        readonly Lazy<HashSet<Triangle>> m_Triangles;
        readonly Lazy<Bounds>            m_Bounds;

        #endregion

        #region Properties

        public int ID { get; }

        //Both may be deferred (the second constructor): the triangle set is then built on the first read — inside
        //the octree operation, on its worker — from mesh data the main thread only copied. Every main-thread
        //reader of an ObstacleInfo reads ID alone (the executor's enqueue), which is what makes the deferral safe.
        public Bounds Bounds => m_Bounds.Value;

        public HashSet<Triangle> Triangles => m_Triangles.Value;

        //Accepts the triangles whose closed shells stay hollow (the owning component asked for it); null = every closed
        //shell is solid. The clusters built from this obstacle carry it.
        public Predicate<Triangle> HollowTriangles { get; }

        public static Predicate<Triangle> AllTriangles { get; } = _ => true;

        //Where the geometry changed since the previous capture, when the caller knows it (RequestUpdate(Bounds)); null
        //means the engine derives the changed region itself. A hint only narrows the rebuild inside the chunks it
        //touches — a chunk that changed outside it widens it.
        public Bounds? ChangedVolumeHint { get; }

        #endregion

        #region Constructors

        public ObstacleInfo(int _ID, HashSet<Triangle> _Triangles, float _MaxAgentRadius, Predicate<Triangle> _HollowTriangles = null, Bounds? _ChangedVolumeHint = null)
        {
            ID                = _ID;
            HollowTriangles   = _HollowTriangles;
            ChangedVolumeHint = _ChangedVolumeHint;

            m_Triangles = new Lazy<HashSet<Triangle>>(() => _Triangles);
            m_Bounds    = new Lazy<Bounds>(() => ExtensionBounds.TrianglesBounds(_Triangles.ToArray()).Enlarge(_MaxAgentRadius * 2));
        }

        //The deferred form: _BuildTriangles runs once, on whichever thread first asks (thread-safe publication), so
        //a runtime obstacle's world-space triangles — the expensive half of its capture — are built off the frame.
        public ObstacleInfo(int _ID, Func<HashSet<Triangle>> _BuildTriangles, float _MaxAgentRadius, Predicate<Triangle> _HollowTriangles = null, Bounds? _ChangedVolumeHint = null)
        {
            ID                = _ID;
            HollowTriangles   = _HollowTriangles;
            ChangedVolumeHint = _ChangedVolumeHint;

            m_Triangles = new Lazy<HashSet<Triangle>>(_BuildTriangles, System.Threading.LazyThreadSafetyMode.ExecutionAndPublication);
            m_Bounds    = new Lazy<Bounds>(() => ExtensionBounds.TrianglesBounds(m_Triangles.Value.ToArray()).Enlarge(_MaxAgentRadius * 2));
        }

        #endregion

    }
}
