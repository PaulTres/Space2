using Nav3D.Common;
using Nav3D.Common.Extensions;
using System.Collections.Generic;
using UnityEngine;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.UtilityTypes
{
    //Clusters triangles as their connected components over shared vertex positions (an exact weld), the components then
    //merged by the box distance the triangle-level clustering used. Each triangle is visited once against one
    //dictionary, where the box-distance clustering filed every face into ~25 hash cells and scanned ~98 per triangle;
    //measured 2026-10-09 on a Skyrift pavilion, 524 → 81 ms. The merge keeps
    //clusters a box-level rule: two triangles the triangle clustering chained (boxes within the distance) still chain
    //their components, and two components merge when their boxes come within it even where their triangles do not —
    //the part-box semantics of chunked clustering. Clusters scope work and never define
    //occupancy, so the construction is unaffected.
    internal static class WeldClustering
    {

        #region Constants

        //a set with more welded components than this (a mesh with no shared vertex at all) merges them by the
        //triangle-level clustering instead of pairwise
        const int COMPONENTS_PAIRWISE_MAX = 1024;

        #endregion

        #region Nested types

        //The reusable buffers of the weld: the vertex ids by exact position, the union-find over the triangles, the first
        //triangle at each vertex and the per-component lists. Cleared on the way out; not thread-safe.
        public sealed class Scratch
        {
            public Dictionary<Vector3, int> VertexIds   { get; } = new Dictionary<Vector3, int>(ShellAnalysis.ExactPositionComparer.Instance);
            public List<int>                VertexOwner { get; } = new List<int>();
            public List<List<int>>          Components  { get; } = new List<List<int>>();

            public int[] Parent { get; set; } = System.Array.Empty<int>();
        }

        #endregion

        #region Public methods

        //the groups of _Triangles: the welded components merged by box distance
        public static Triangle[][] Groups(IList<Triangle> _Triangles, float _MergeDistance, ExtensionIBoundable.ClusteringScratch _Scratch, Scratch _Weld = null)
        {
            return Components(_Triangles, _MergeDistance, _Scratch, _Weld ?? new Scratch());
        }

        #endregion

        #region Service methods

        //the connected components over shared vertex positions, then merged by the box distance — pairwise over a few
        //components, which for a mesh of welded faces is one
        static Triangle[][] Components(IList<Triangle> _Triangles, float _MergeDistance, ExtensionIBoundable.ClusteringScratch _Scratch, Scratch _Weld)
        {
            int count = _Triangles.Count;

            if (_Weld.Parent.Length < count)
                _Weld.Parent = new int[Mathf.Max(count, _Weld.Parent.Length * 2)];

            int[] parent = _Weld.Parent;

            for (int i = 0; i < count; i++)
                parent[i] = i;

            Dictionary<Vector3, int> vertexIds   = _Weld.VertexIds;
            List<int>                vertexOwner = _Weld.VertexOwner;

            vertexIds.Clear();
            vertexOwner.Clear();

            for (int i = 0; i < count; i++)
            {
                Triangle triangle = _Triangles[i];

                UnionAtVertex(triangle.V1, i, vertexIds, vertexOwner, parent);
                UnionAtVertex(triangle.V2, i, vertexIds, vertexOwner, parent);
                UnionAtVertex(triangle.V3, i, vertexIds, vertexOwner, parent);
            }

            List<List<int>> components = _Weld.Components;

            foreach (List<int> component in components)
                component.Clear();

            Dictionary<int, int> componentOfRoot = new Dictionary<int, int>();

            int used = 0;

            for (int i = 0; i < count; i++)
            {
                int root = Find(parent, i);

                if (!componentOfRoot.TryGetValue(root, out int componentIndex))
                {
                    componentIndex = used++;

                    componentOfRoot.Add(root, componentIndex);

                    if (components.Count <= componentIndex)
                        components.Add(new List<int>());
                }

                components[componentIndex].Add(i);
            }

            if (used == 1)
                return new[] { ToArray(_Triangles) };

            //too many components to pair (a mesh with no welded vertex at all): the first cut's clustering over the triangles
            if (used > COMPONENTS_PAIRWISE_MAX)
                return ExtensionIBoundable.ClusterByAABBDistance(_Triangles, _MergeDistance, _Scratch);

            Bounds[] boxes = new Bounds[used];

            for (int c = 0; c < used; c++)
            {
                List<int> component = components[c];

                Bounds box = _Triangles[component[0]].Bounds;

                for (int k = 1; k < component.Count; k++)
                    box.Encapsulate(_Triangles[component[k]].Bounds);

                boxes[c] = box;
            }

            int[] groupParent = new int[used];

            for (int c = 0; c < used; c++)
                groupParent[c] = c;

            float sqrGap = _MergeDistance * _MergeDistance;

            for (int a = 0; a < used; a++)
                for (int b = a + 1; b < used; b++)
                {
                    if (SqrBoxDistance(boxes[a], boxes[b]) > sqrGap)
                        continue;

                    int ra = Find(groupParent, a);
                    int rb = Find(groupParent, b);

                    if (ra != rb)
                        groupParent[ra] = rb;
                }

            Dictionary<int, List<int>> groups = new Dictionary<int, List<int>>();

            for (int c = 0; c < used; c++)
            {
                int root = Find(groupParent, c);

                if (!groups.TryGetValue(root, out List<int> members))
                {
                    members = new List<int>();
                    groups.Add(root, members);
                }

                members.AddRange(components[c]);
            }

            Triangle[][] result = new Triangle[groups.Count][];

            int g = 0;

            foreach (List<int> members in groups.Values)
            {
                Triangle[] triangles = new Triangle[members.Count];

                for (int k = 0; k < members.Count; k++)
                    triangles[k] = _Triangles[members[k]];

                result[g++] = triangles;
            }

            return result;
        }

        static void UnionAtVertex(Vector3 _Position, int _Triangle, Dictionary<Vector3, int> _VertexIds, List<int> _VertexOwner, int[] _Parent)
        {
            if (_VertexIds.TryGetValue(_Position, out int id))
            {
                int a = Find(_Parent, _Triangle);
                int b = Find(_Parent, _VertexOwner[id]);

                if (a != b)
                    _Parent[a] = b;

                return;
            }

            _VertexIds.Add(_Position, _VertexOwner.Count);
            _VertexOwner.Add(_Triangle);
        }

        static int Find(int[] _Parent, int _Index)
        {
            while (_Parent[_Index] != _Index)
            {
                _Parent[_Index] = _Parent[_Parent[_Index]];
                _Index          = _Parent[_Index];
            }

            return _Index;
        }

        static float SqrBoxDistance(Bounds _A, Bounds _B)
        {
            float dx = Mathf.Max(0f, Mathf.Max(_A.min.x - _B.max.x, _B.min.x - _A.max.x));
            float dy = Mathf.Max(0f, Mathf.Max(_A.min.y - _B.max.y, _B.min.y - _A.max.y));
            float dz = Mathf.Max(0f, Mathf.Max(_A.min.z - _B.max.z, _B.min.z - _A.max.z));

            return dx * dx + dy * dy + dz * dz;
        }

        static Triangle[] ToArray(IList<Triangle> _Triangles)
        {
            Triangle[] array = new Triangle[_Triangles.Count];

            _Triangles.CopyTo(array, 0);

            return array;
        }

        #endregion

    }
}