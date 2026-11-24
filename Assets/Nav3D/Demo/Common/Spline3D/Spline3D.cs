using System.Collections.Generic;
using UnityEngine;
using Nav3D.Common;

namespace Nav3D.Demo
{
    /// <summary>Centripetal Catmull-Rom spline through a Vector3 list or Transforms (a list, or the direct children); a moved source re-bakes it.
    /// World space throughout: this object's own transform does not affect the curve. Uniform mode spaces the samples by arc length.</summary>
    public partial class Spline3D : MonoBehaviour
    {
        #region Nested types

        public enum PointsSource
        {
            //m_Points, world space
            POINTS,
            //m_Transforms positions
            TRANSFORMS
        }

        #endregion

        #region Serialized fields

        [SerializeField] PointsSource    m_PointsSource = PointsSource.POINTS;
        [SerializeField] List<Vector3>   m_Points       = new List<Vector3>();
        [SerializeField] List<Transform> m_Transforms   = new List<Transform>();
        [SerializeField] bool m_UseChildTransforms;
        //uniform mode: max distance between consecutive sampled points
        [SerializeField] float m_DistanceStep = 0.25f;
        //non-uniform mode: curve parameter step per segment
        [SerializeField] float m_ParamStep = 0.05f;
        [SerializeField] bool  m_UniformDistribution = true;
        [SerializeField] bool  m_Loop;

        #endregion

        #region Attributes

        CatmullRomSpline m_Spline;
        Vector3[]        m_BakedPoints;
        //positions the current bake was built from, used to detect source transform movement
        Vector3[]        m_LastTransformPositions;
        bool             m_Dirty = true;

        #endregion

        #region Properties

        public PointsSource Source
        {
            get => m_PointsSource;
            set
            {
                m_PointsSource = value;
                SetDirty();
            }
        }

        public int PointCount => m_PointsSource == PointsSource.TRANSFORMS ? GetSourceTransforms().Count : m_Points.Count;

        public bool UseChildTransforms
        {
            get => m_UseChildTransforms;
            set
            {
                m_UseChildTransforms = value;
                SetDirty();
            }
        }

        public float TotalLength
        {
            get
            {
                RebuildIfDirty();
                return m_Spline.TotalLength;
            }
        }

        public bool Loop
        {
            get => m_Loop;
            set
            {
                m_Loop = value;
                SetDirty();
            }
        }

        public bool UniformDistribution
        {
            get => m_UniformDistribution;
            set
            {
                m_UniformDistribution = value;
                SetDirty();
            }
        }

        #endregion

        #region Public methods

        /// <summary>Sampled points; baked lazily, a new array per call.</summary>
        public Vector3[] GetSplinePoints()
        {
            RebuildIfDirty();

            return (Vector3[])m_BakedPoints.Clone();
        }

        /// <summary>Point at the arc-length distance from the start; wraps for loops, clamps otherwise.</summary>
        public Vector3 GetPointAtDistance(float _Distance)
        {
            RebuildIfDirty();

            return m_Spline.GetPointAtDistance(_Distance);
        }

        /// <summary>Normalized direction at the arc-length distance from the start.</summary>
        public Vector3 GetDirectionAtDistance(float _Distance)
        {
            RebuildIfDirty();

            return m_Spline.GetTangentAtDistance(_Distance);
        }

        /// <summary>Arc-length distance of the closest curve point; exact up to the sampling step.</summary>
        public float GetClosestPointDistance(Vector3 _Position)
        {
            RebuildIfDirty();

            if (m_BakedPoints.Length < 2)
                return 0f;

            int   bestIndex = 0;
            float bestSqr   = float.MaxValue;

            for (int i = 0; i < m_BakedPoints.Length; i++)
            {
                float sqr = (m_BakedPoints[i] - _Position).sqrMagnitude;

                if (sqr < bestSqr)
                {
                    bestSqr   = sqr;
                    bestIndex = i;
                }
            }

            return m_Spline.TotalLength * bestIndex / (m_BakedPoints.Length - 1);
        }

        public Vector3 GetPoint(int _Index)
        {
            if (m_PointsSource == PointsSource.TRANSFORMS)
            {
                Transform pointTransform = GetSourceTransforms()[_Index];
                return pointTransform != null ? pointTransform.position : Vector3.zero;
            }

            return m_Points[_Index];
        }

        /// <summary>Replaces the control points and switches the source to POINTS.</summary>
        public void SetPoints(IEnumerable<Vector3> _Positions)
        {
            m_Points.Clear();
            m_Points.AddRange(_Positions);

            m_PointsSource = PointsSource.POINTS;

            SetDirty();
        }

        /// <summary>Replaces the control points and switches the source to TRANSFORMS (manual list, UseChildTransforms off).</summary>
        public void SetTransforms(IEnumerable<Transform> _Transforms)
        {
            m_Transforms.Clear();
            m_Transforms.AddRange(_Transforms);

            m_PointsSource       = PointsSource.TRANSFORMS;
            m_UseChildTransforms = false;

            SetDirty();
        }

        //the individual mutators below operate on the Points source

        public void SetPoint(int _Index, Vector3 _Position)
        {
            m_Points[_Index] = _Position;
            SetDirty();
        }

        public void AddPoint(Vector3 _Position)
        {
            m_Points.Add(_Position);
            SetDirty();
        }

        public void RemovePoint(int _Index)
        {
            m_Points.RemoveAt(_Index);
            SetDirty();
        }

        public void SetDirty()
        {
            m_Dirty = true;
        }

        /// <summary>The direct children when UseChildTransforms is on, else the manual list.</summary>
        public IReadOnlyList<Transform> GetSourceTransforms()
        {
            if (!m_UseChildTransforms)
                return m_Transforms;

            List<Transform> children = new List<Transform>(transform.childCount);

            foreach (Transform child in transform)
                children.Add(child);

            return children;
        }

        #endregion

        #region Service methods

        void RebuildIfDirty()
        {
            if (m_PointsSource == PointsSource.TRANSFORMS)
                CheckTransformsMoved();

            if (!m_Dirty && m_Spline != null)
                return;

            IReadOnlyList<Vector3> controlPoints = GatherControlPoints();

            m_Spline = new CatmullRomSpline(controlPoints, m_Loop);

            m_BakedPoints = m_UniformDistribution
                ? m_Spline.GetUniformPoints(m_DistanceStep)
                : m_Spline.GetParameterSampledPoints(m_ParamStep);

            m_Dirty = false;
        }

        IReadOnlyList<Vector3> GatherControlPoints()
        {
            if (m_PointsSource != PointsSource.TRANSFORMS)
            {
                m_LastTransformPositions = null;

                return m_Points;
            }

            IReadOnlyList<Transform> sourceTransforms = GetSourceTransforms();

            List<Vector3> controlPoints = new List<Vector3>(sourceTransforms.Count);

            foreach (Transform pointTransform in sourceTransforms)
            {
                if (pointTransform != null)
                    controlPoints.Add(pointTransform.position);
            }

            m_LastTransformPositions = controlPoints.ToArray();

            return controlPoints;
        }

        //marks the spline dirty when any source transform has moved since the last bake
        void CheckTransformsMoved()
        {
            if (m_LastTransformPositions == null)
            {
                m_Dirty = true;
                return;
            }

            int index = 0;

            foreach (Transform pointTransform in GetSourceTransforms())
            {
                if (pointTransform == null)
                    continue;

                if (index >= m_LastTransformPositions.Length || m_LastTransformPositions[index] != pointTransform.position)
                {
                    m_Dirty = true;
                    return;
                }

                index++;
            }

            if (index != m_LastTransformPositions.Length)
                m_Dirty = true;
        }

        #endregion

        #region Unity events

        void OnValidate()
        {
            m_DistanceStep = Mathf.Max(m_DistanceStep, 0.01f);
            m_ParamStep    = Mathf.Clamp(m_ParamStep, 0.001f, 1f);

            SetDirty();
        }

        #endregion
    }
}
