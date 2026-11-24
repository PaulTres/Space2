using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

namespace Nav3D.Common
{
    /// <summary>
    /// Builds the mesh of a capsule swept along a polyline: the volume a body of the given radius occupies along a route.
    /// One ring of vertices per path point, carried by parallel transport so the tube never twists at a bend, and a
    /// hemisphere on each capped end. Main thread only (it reuses static scratch lists).
    /// </summary>
    public static class PathCapsuleMesh
    {

        #region Constants

        public const int DEFAULT_SIDES = 20;

        //rings per hemispherical end cap, the equator counted
        public const int CAP_RINGS = 6;

        //closer path points than this are one ring: a zero-length segment has no direction, and its ring would fold into the next
        const float MIN_RING_SPACING = 0.001f;

        #endregion

        #region Attributes

        static readonly List<Vector3>                                                           s_Points = new List<Vector3>();
        static readonly List<(Vector3 Centre, Vector3 Normal, Vector3 Binormal, Vector3 Along)> s_Frames = new List<(Vector3, Vector3, Vector3, Vector3)>();

        static readonly List<(Vector3 Centre, Vector3 Normal, Vector3 Binormal, float Radius, Vector3 Axial, Vector3 Along)> s_Rings =
            new List<(Vector3, Vector3, Vector3, float, Vector3, Vector3)>();

        #endregion

        #region Public methods

        /// <summary>
        /// Fills <paramref name="_Reuse"/> (or a new mesh when it is null) with the capsule along <paramref name="_Points"/>.
        /// An open end (a cap switched off) ends on a plain ring, so two capsules can meet on one shared ring.
        /// Vertices are written in <paramref name="_Local"/>'s space when it is given, else in world space.
        /// Fewer than two distinct points yield an empty mesh.
        /// </summary>
        public static Mesh Build(
                IReadOnlyList<Vector3> _Points,
                float                  _Radius,
                Mesh                   _Reuse,
                int                    _Sides    = DEFAULT_SIDES,
                bool                   _CapStart = true,
                bool                   _CapEnd   = true,
                Transform              _Local    = null
            )
        {
            Mesh mesh = _Reuse != null ? _Reuse : new Mesh { name = nameof(PathCapsuleMesh) };

            mesh.Clear();

            int sides = Mathf.Max(3, _Sides);

            s_Points.Clear();

            foreach (Vector3 point in _Points)
            {
                if (s_Points.Count == 0 || (point - s_Points[s_Points.Count - 1]).sqrMagnitude > MIN_RING_SPACING * MIN_RING_SPACING)
                    s_Points.Add(point);
            }

            if (s_Points.Count < 2)
                return mesh;

            BuildFrames();
            BuildRings(_Radius, _CapStart, _CapEnd);

            (Vector3 Centre, Vector3 Normal, Vector3 Binormal, Vector3 Along) first = s_Frames[0];
            (Vector3 Centre, Vector3 Normal, Vector3 Binormal, Vector3 Along) last  = s_Frames[s_Frames.Count - 1];

            int       ringVertices = s_Rings.Count * sides;
            int       capCount     = (_CapStart ? 1 : 0) + (_CapEnd ? 1 : 0);
            Vector3[] vertices     = new Vector3[ringVertices + capCount];
            Vector3[] normals      = new Vector3[vertices.Length];
            //the route's direction at the vertex, w the share of the vertex that is tube rather than cap: what a shader needs to
            //tell where a point sits across the tube as seen from the camera, which the normal alone cannot once the tube is foreshortened
            Vector4[] tangents = new Vector4[vertices.Length];
            int[]     indices  = new int[(s_Rings.Count - 1) * sides * 6 + capCount * sides * 3];

            for (int r = 0; r < s_Rings.Count; r++)
            {
                float   axialShare  = s_Rings[r].Axial.magnitude;
                float   radialShare = Mathf.Sqrt(Mathf.Max(0f, 1f - axialShare * axialShare));
                Vector3 along       = ToLocalDirection(_Local, s_Rings[r].Along);
                Vector4 tangent     = new Vector4(along.x, along.y, along.z, radialShare);

                for (int side = 0; side < sides; side++)
                {
                    float   angle  = side              * Mathf.PI                               * 2f / sides;
                    Vector3 radial = s_Rings[r].Normal * Mathf.Cos(angle) + s_Rings[r].Binormal * Mathf.Sin(angle);

                    vertices[r * sides + side] = ToLocal(_Local, s_Rings[r].Centre + radial * s_Rings[r].Radius);
                    normals[r  * sides + side] = ToLocalDirection(_Local, radial            * radialShare + s_Rings[r].Axial);
                    tangents[r * sides + side] = tangent;
                }
            }

            int index = 0;

            for (int r = 0; r < s_Rings.Count - 1; r++)
            {
                for (int side = 0; side < sides; side++)
                {
                    int a = r * sides + side;
                    int b = r * sides + (side + 1) % sides;

                    indices[index++] = a;
                    indices[index++] = b;
                    indices[index++] = a + sides;
                    indices[index++] = b;
                    indices[index++] = b + sides;
                    indices[index++] = a + sides;
                }
            }

            int lastRing = (s_Rings.Count - 1) * sides;
            int pole     = ringVertices;

            if (_CapStart)
            {
                vertices[pole] = ToLocal(_Local, first.Centre - first.Along * _Radius);
                normals[pole]  = ToLocalDirection(_Local, -first.Along);
                tangents[pole] = ToLocalDirection(_Local, first.Along);

                for (int side = 0; side < sides; side++)
                {
                    indices[index++] = pole;
                    indices[index++] = (side + 1) % sides;
                    indices[index++] = side;
                }

                pole++;
            }

            if (_CapEnd)
            {
                vertices[pole] = ToLocal(_Local, last.Centre + last.Along * _Radius);
                normals[pole]  = ToLocalDirection(_Local, last.Along);
                tangents[pole] = ToLocalDirection(_Local, last.Along);

                for (int side = 0; side < sides; side++)
                {
                    indices[index++] = lastRing + side;
                    indices[index++] = lastRing + (side + 1) % sides;
                    indices[index++] = pole;
                }
            }

            mesh.indexFormat = vertices.Length > 65000 ? IndexFormat.UInt32 : IndexFormat.UInt16;
            mesh.vertices    = vertices;
            mesh.normals     = normals;
            mesh.tangents    = tangents;
            mesh.triangles   = indices;
            mesh.RecalculateBounds();

            return mesh;
        }

        #endregion

        #region Service methods

        //a frame per point: the along direction bisects the bend, the normal is the previous one projected onto the ring's plane
        static void BuildFrames()
        {
            s_Frames.Clear();

            Vector3 tangent = (s_Points[1] - s_Points[0]).normalized;
            Vector3 normal  = Vector3.Cross(tangent, Mathf.Abs(tangent.y) < 0.9f ? Vector3.up : Vector3.right).normalized;

            for (int i = 0; i < s_Points.Count; i++)
            {
                Vector3 next  = i                          < s_Points.Count - 1 ? (s_Points[i + 1] - s_Points[i]).normalized : tangent;
                Vector3 prev  = i                          > 0 ? (s_Points[i]                      - s_Points[i - 1]).normalized : next;
                Vector3 along = (prev + next).sqrMagnitude > 1e-6f ? (prev                         + next).normalized : tangent;

                normal = Vector3.ProjectOnPlane(normal, along);
                normal = normal.sqrMagnitude > 1e-6f ? normal.normalized : Vector3.Cross(along, Mathf.Abs(along.y) < 0.9f ? Vector3.up : Vector3.right).normalized;

                tangent = along;

                s_Frames.Add((s_Points[i], normal, Vector3.Cross(along, normal), along));
            }
        }

        //every ring in order along the capsule: the start hemisphere from next to its pole out to its equator, the path rings, the
        //end hemisphere from its equator in; each with its radius and the share of its normal that points along the axis
        static void BuildRings(float _Radius, bool _CapStart, bool _CapEnd)
        {
            s_Rings.Clear();

            (Vector3 Centre, Vector3 Normal, Vector3 Binormal, Vector3 Along) first = s_Frames[0];
            (Vector3 Centre, Vector3 Normal, Vector3 Binormal, Vector3 Along) last  = s_Frames[s_Frames.Count - 1];

            if (_CapStart)
            {
                for (int k = CAP_RINGS - 1; k >= 1; k--)
                {
                    float phi = k * Mathf.PI * 0.5f / CAP_RINGS;

                    s_Rings.Add(
                            (first.Centre - first.Along * (_Radius * Mathf.Sin(phi)), first.Normal, first.Binormal, _Radius * Mathf.Cos(phi), -first.Along * Mathf.Sin(phi),
                             first.Along)
                        );
                }
            }

            foreach ((Vector3 centre, Vector3 normal, Vector3 binormal, Vector3 along) in s_Frames)
                s_Rings.Add((centre, normal, binormal, _Radius, Vector3.zero, along));

            if (_CapEnd)
            {
                for (int k = 1; k < CAP_RINGS; k++)
                {
                    float phi = k * Mathf.PI * 0.5f / CAP_RINGS;

                    s_Rings.Add(
                            (last.Centre + last.Along * (_Radius * Mathf.Sin(phi)), last.Normal, last.Binormal, _Radius * Mathf.Cos(phi), last.Along * Mathf.Sin(phi), last.Along)
                        );
                }
            }
        }

        static Vector3 ToLocal(Transform _Local, Vector3 _World)
        {
            return _Local != null ? _Local.InverseTransformPoint(_World) : _World;
        }

        static Vector3 ToLocalDirection(Transform _Local, Vector3 _World)
        {
            return _Local != null ? _Local.InverseTransformDirection(_World) : _World;
        }

        #endregion

    }
}