using System.Collections.Generic;
using System.IO;
using UnityEngine;

namespace Nav3D.Common.Serialization
{
    static partial class ExtensionBinaryWriter
    {

        #region Public methods

        public static void Write(this BinaryWriter _Writer, Vector3 _Vector)
        {
            _Writer.Write(_Vector.x);
            _Writer.Write(_Vector.y);
            _Writer.Write(_Vector.z);
        }
        
        public static void Write(this BinaryWriter _Writer, Vector3Int _Vector)
        {
            _Writer.Write(_Vector.x);
            _Writer.Write(_Vector.y);
            _Writer.Write(_Vector.z);
        }

        public static void Write(this BinaryWriter _Writer, Bounds _Bounds)
        {
            _Writer.Write(_Bounds.center);
            _Writer.Write(_Bounds.size);
        }

        public static void WriteTrianglePool(this BinaryWriter _Writer, Triangle[] _Pool)
        {
            List<Vector3>           vertices      = new List<Vector3>(_Pool.Length * 3);
            Dictionary<Vector3, int> vertexToIndex = new Dictionary<Vector3, int>(_Pool.Length * 3);

            int[] triangleVertexIndices = new int[_Pool.Length * 3];

            for (int t = 0; t < _Pool.Length; t++)
            {
                Triangle tri = _Pool[t];

                triangleVertexIndices[t * 3]     = GetOrAddVertex(tri.V1, vertices, vertexToIndex);
                triangleVertexIndices[t * 3 + 1] = GetOrAddVertex(tri.V2, vertices, vertexToIndex);
                triangleVertexIndices[t * 3 + 2] = GetOrAddVertex(tri.V3, vertices, vertexToIndex);
            }

            _Writer.Write(vertices.Count);
            foreach (Vector3 vertex in vertices)
            {
                _Writer.Write(vertex.x);
                _Writer.Write(vertex.y);
                _Writer.Write(vertex.z);
            }

            _Writer.Write(_Pool.Length);
            foreach (int vIndex in triangleVertexIndices)
            {
                _Writer.Write(vIndex);
            }
        }

        public static void WriteClusterTriangleIndices(this BinaryWriter _Writer, HashSet<Triangle> _Triangles, Dictionary<Triangle, int> _TriangleToPoolIndex)
        {
            _Writer.Write(_Triangles.Count);
            foreach (Triangle triangle in _Triangles)
            {
                _Writer.Write(_TriangleToPoolIndex[triangle]);
            }
        }

        #endregion

        #region Service methods

        static int GetOrAddVertex(Vector3 _Vertex, List<Vector3> _Vertices, Dictionary<Vector3, int> _VertexToIndex)
        {
            if (_VertexToIndex.TryGetValue(_Vertex, out int index))
                return index;

            int newIndex = _Vertices.Count;
            _Vertices.Add(_Vertex);
            _VertexToIndex.Add(_Vertex, newIndex);
            return newIndex;

        #endregion
        }
    }
}