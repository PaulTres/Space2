using System.Collections.Generic;
using System.Globalization;
using UnityEngine;

namespace Nav3D.Common
{
    //The surface a box, sphere or capsule collider stands for, as the vertex and index arrays plus the matrix an obstacle
    //capture reads a mesh as. A sphere and a capsule are polygons inscribed in the collider: their faces lie within 1 % of
    //the radius inside it, well under any octree cell.
    static class ColliderGeometry
    {
        #region Constants

        //around the axis, and from a pole to the equator: 15 degrees a step either way
        const int   SEGMENTS         = 24;
        const int   HEMISPHERE_RINGS = 6;
        const float MIN_HALF_LENGTH  = 1e-5f;

        #endregion

        #region Attributes

        static readonly Vector3[] s_BoxCorners =
        {
            new Vector3(-0.5f, -0.5f, -0.5f), new Vector3(0.5f, -0.5f, -0.5f), new Vector3(0.5f, 0.5f, -0.5f), new Vector3(-0.5f, 0.5f, -0.5f),
            new Vector3(-0.5f, -0.5f, 0.5f), new Vector3(0.5f, -0.5f, 0.5f), new Vector3(0.5f, 0.5f, 0.5f), new Vector3(-0.5f, 0.5f, 0.5f)
        };

        static readonly int[] s_BoxIndices = BuildBoxIndices();

        #endregion

        #region Public methods

        public static bool IsSupported(Collider _Collider)
        {
            return _Collider is BoxCollider || _Collider is SphereCollider || _Collider is CapsuleCollider || _Collider is MeshCollider || _Collider is TerrainCollider;
        }

        //box, sphere and capsule; a mesh or terrain collider is read through its mesh or height map instead
        public static bool TryGetPrimitive(Collider _Collider, out Matrix4x4 _LocalToWorld, out Vector3[] _Vertices, out int[] _Indices)
        {
            Transform transform = _Collider.transform;

            switch (_Collider)
            {
                case BoxCollider box:
                {
                    Vector3 size = Abs(box.size);

                    _Vertices = new Vector3[s_BoxCorners.Length];

                    for (int i = 0; i < s_BoxCorners.Length; i++)
                        _Vertices[i] = box.center + Vector3.Scale(s_BoxCorners[i], size);

                    _Indices      = s_BoxIndices;
                    _LocalToWorld = transform.localToWorldMatrix;

                    return true;
                }

                case SphereCollider sphere:
                {
                    Vector3 scale  = Abs(transform.lossyScale);
                    float   radius = sphere.radius * Mathf.Max(scale.x, scale.y, scale.z);

                    BuildCapsule(radius, 0f, out _Vertices, out _Indices);

                    _LocalToWorld = Matrix4x4.TRS(transform.TransformPoint(sphere.center), transform.rotation, Vector3.one);

                    return true;
                }

                case CapsuleCollider capsule:
                {
                    Vector3 scale = Abs(transform.lossyScale);

                    //Unity scales the radius by the larger of the two cross-axis scales and the height by the axis's own
                    (float axisScale, float crossScale, Quaternion axisRotation) = capsule.direction switch
                    {
                        0 => (scale.x, Mathf.Max(scale.y, scale.z), Quaternion.Euler(0f, 0f, -90f)),
                        2 => (scale.z, Mathf.Max(scale.x, scale.y), Quaternion.Euler(90f, 0f, 0f)),
                        _ => (scale.y, Mathf.Max(scale.x, scale.z), Quaternion.identity)
                    };

                    float radius     = capsule.radius * crossScale;
                    float halfLength = Mathf.Max(0f, capsule.height * axisScale * 0.5f - radius);

                    BuildCapsule(radius, halfLength, out _Vertices, out _Indices);

                    _LocalToWorld = Matrix4x4.TRS(transform.TransformPoint(capsule.center), transform.rotation * axisRotation, Vector3.one);

                    return true;
                }

                default:
                    _LocalToWorld = default;
                    _Vertices     = null;
                    _Indices      = null;

                    return false;
            }
        }

        //what the bake's imprint records of a collider besides its transform, so resizing one invalidates the bake
        public static string Describe(Collider _Collider)
        {
            switch (_Collider)
            {
                case BoxCollider box:
                    return $"box{box.center.ToStringExt()}{box.size.ToStringExt()}";

                case SphereCollider sphere:
                    return $"sphere{sphere.center.ToStringExt()}{sphere.radius.ToString(CultureInfo.InvariantCulture)}";

                case CapsuleCollider capsule:
                    return $"capsule{capsule.center.ToStringExt()}{capsule.radius.ToString(CultureInfo.InvariantCulture)}" +
                           $"|{capsule.height.ToString(CultureInfo.InvariantCulture)}|{capsule.direction}";

                case MeshCollider _:
                    return "mesh";

                default:
                    return "terrain";
            }
        }

        #endregion

        #region Service methods

        static Vector3 Abs(Vector3 _Vector)
        {
            return new Vector3(Mathf.Abs(_Vector.x), Mathf.Abs(_Vector.y), Mathf.Abs(_Vector.z));
        }

        static int[] BuildBoxIndices()
        {
            int[][] faces =
            {
                new[] { 0, 1, 2, 3 }, new[] { 4, 5, 6, 7 }, new[] { 0, 1, 5, 4 },
                new[] { 3, 2, 6, 7 }, new[] { 0, 3, 7, 4 }, new[] { 1, 2, 6, 5 }
            };

            List<int> indices = new List<int>(36);

            foreach (int[] face in faces)
            {
                AddOutward(indices, s_BoxCorners, face[0], face[1], face[2]);
                AddOutward(indices, s_BoxCorners, face[0], face[2], face[3]);
            }

            return indices.ToArray();
        }

        //A capsule along local Y centred on the origin; a half length of 0 is a sphere. Rings run from the top pole to the
        //bottom one, the equator doubled when there is a cylinder between the two hemispheres.
        static void BuildCapsule(float _Radius, float _HalfLength, out Vector3[] _Vertices, out int[] _Indices)
        {
            bool                          cylinder = _HalfLength > MIN_HALF_LENGTH;
            List<(float Y, float Radius)> rings    = new List<(float Y, float Radius)>();

            for (int ring = 1; ring <= HEMISPHERE_RINGS; ring++)
            {
                float angle = ring * Mathf.PI * 0.5f / HEMISPHERE_RINGS;

                rings.Add((_HalfLength + _Radius * Mathf.Cos(angle), _Radius * Mathf.Sin(angle)));
            }

            for (int ring = cylinder ? HEMISPHERE_RINGS : HEMISPHERE_RINGS - 1; ring >= 1; ring--)
            {
                float angle = ring * Mathf.PI * 0.5f / HEMISPHERE_RINGS;

                rings.Add((-_HalfLength - _Radius * Mathf.Cos(angle), _Radius * Mathf.Sin(angle)));
            }

            int bottomPole = 1 + rings.Count * SEGMENTS;

            _Vertices = new Vector3[bottomPole + 1];

            _Vertices[0]          = new Vector3(0f, _HalfLength + _Radius, 0f);
            _Vertices[bottomPole] = new Vector3(0f, -_HalfLength - _Radius, 0f);

            for (int ring = 0; ring < rings.Count; ring++)
            {
                for (int segment = 0; segment < SEGMENTS; segment++)
                {
                    float angle = segment * Mathf.PI * 2f / SEGMENTS;

                    _Vertices[RingVertex(ring, segment)] = new Vector3(rings[ring].Radius * Mathf.Cos(angle), rings[ring].Y, rings[ring].Radius * Mathf.Sin(angle));
                }
            }

            List<int> indices = new List<int>(SEGMENTS * 6 * rings.Count);

            for (int segment = 0; segment < SEGMENTS; segment++)
            {
                int next = (segment + 1) % SEGMENTS;

                AddOutward(indices, _Vertices, 0, RingVertex(0, segment), RingVertex(0, next));
                AddOutward(indices, _Vertices, bottomPole, RingVertex(rings.Count - 1, segment), RingVertex(rings.Count - 1, next));

                for (int ring = 0; ring < rings.Count - 1; ring++)
                {
                    AddOutward(indices, _Vertices, RingVertex(ring, segment), RingVertex(ring + 1, segment), RingVertex(ring + 1, next));
                    AddOutward(indices, _Vertices, RingVertex(ring, segment), RingVertex(ring + 1, next), RingVertex(ring, next));
                }
            }

            _Indices = indices.ToArray();
        }

        static int RingVertex(int _Ring, int _Segment)
        {
            return 1 + _Ring * SEGMENTS + _Segment;
        }

        //Unity's winding: the normal Cross(b - a, c - a) points out of the body. Every shape here is convex around the
        //origin, so a face points out exactly when its normal points away from the origin.
        static void AddOutward(List<int> _Indices, Vector3[] _Vertices, int _A, int _B, int _C)
        {
            Vector3 a = _Vertices[_A];
            Vector3 b = _Vertices[_B];
            Vector3 c = _Vertices[_C];

            if (Vector3.Dot(Vector3.Cross(b - a, c - a), a + b + c) < 0f)
                (_B, _C) = (_C, _B);

            _Indices.Add(_A);
            _Indices.Add(_B);
            _Indices.Add(_C);
        }

        #endregion
    }
}
