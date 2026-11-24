using System.Collections.Generic;
using System.IO;
using UnityEngine;

namespace Nav3D.Common.Serialization
{
    static partial class ExtensionBinaryReader
    {

        #region Public methods

        public static Vector3 ReadVector3(this BinaryReader _Reader)
        {
            return new Vector3(_Reader.ReadSingle(), _Reader.ReadSingle(), _Reader.ReadSingle());
        }

        public static Vector3Int ReadVector3Int(this BinaryReader _Reader)
        {
            return new Vector3Int(_Reader.ReadInt32(), _Reader.ReadInt32(), _Reader.ReadInt32());
        }

        public static Bounds ReadBounds(this BinaryReader _Reader)
        {
            return new Bounds(_Reader.ReadVector3(), _Reader.ReadVector3());
        }

        public static Triangle[] ReadTrianglePool(this BinaryReader _Reader)
        {
            int           verticesCount = _Reader.ReadInt32();
            List<Vector3> vertices      = new List<Vector3>(verticesCount);

            for (int i = 0; i < verticesCount; i++)
            {
                vertices.Add(new Vector3(_Reader.ReadSingle(), _Reader.ReadSingle(), _Reader.ReadSingle()));
            }

            int        triangleCount = _Reader.ReadInt32();
            Triangle[] pool          = new Triangle[triangleCount];

            for (int i = 0; i < triangleCount; i++)
            {
                int i1 = _Reader.ReadInt32();
                int i2 = _Reader.ReadInt32();
                int i3 = _Reader.ReadInt32();
                pool[i] = new Triangle(vertices[i1], vertices[i2], vertices[i3]);
            }

            return pool;
        }

        public static HashSet<Triangle> ReadClusterTriangleIndices(this BinaryReader _Reader, Triangle[] _Pool)
        {
            int               count  = _Reader.ReadInt32();
            HashSet<Triangle> result = new HashSet<Triangle>(count);

            for (int i = 0; i < count; i++)
            {
                result.Add(_Pool[_Reader.ReadInt32()]);
            }

            return result;

        #endregion
        }
    }
}