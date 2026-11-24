using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Object = UnityEngine.Object;

namespace Nav3D.Common
{
    public static partial class UtilsCommon
    {
        #region Nested types

        class RandomSeedInvariant : IDisposable
        {
            #region Attributes

            UnityEngine.Random.State m_RandomStateBuffer;

            #endregion

            #region Construction

            public RandomSeedInvariant()
            {
                m_RandomStateBuffer = UnityEngine.Random.state;
            }

            #endregion

            #region IDisposable methods

            public void Dispose() => UnityEngine.Random.state = m_RandomStateBuffer;

            #endregion
        }

        #endregion

        #region Properties

        public static IDisposable RandomSeedPermanence => new RandomSeedInvariant();

        #endregion

        #region Public methods

        public static Color[] GetNDistancedColors(int _N)
        {
            Color[] result = new Color[_N];

            float step = (360f / _N) / 360f;
            float hue  = 0;

            for (int i = 0; i < _N; i++)
            {
                result[i] = Color.HSVToRGB(hue, 1f, 1f);

                hue += step;
            }

            return result;
        }

        static partial void SmartDestroyEditor(Object _Object, ref bool _Handled);

        //teardown-path destroy: immediate in play mode too. Object.Destroy defers to end of frame, which is one
        //frame too late for a dispose/re-init pair living in the same frame (a scene switch): the next epoch's
        //Singleton<T> resolve still FINDS the dying component, initializes it, and the deferred OnDestroy then
        //clears the statics (Doomed, ClearInstance, the dispatcher's main-thread capture) out from under the
        //freshly initialized epoch. Reserved for the Nav3D
        //lifecycle teardown (Nav3DManager.Dispose3DNav → the managers' Uninitialize); ordinary gameplay
        //destruction stays SmartDestroy
        public static void SmartDestroyImmediate(Object _Object)
        {
            if (_Object == null)
                return;

            Object.DestroyImmediate(_Object);
        }

        //lifecycle-singleton teardown: destroy the component immediately (the rule above) together with its
        //holder GameObject. Singleton<T> creates the holder with DontDestroyOnLoad, so destroying only the
        //component left an empty immortal GameObject behind on every dispose/init cycle — one husk set per
        //scene reload, found via a demo R-reset. The holder goes only when the component is the
        //single thing it carries besides the Transform; a manager placed by hand on an object that carries
        //anything else loses just the component, as before. DestroyImmediate on the GameObject runs the
        //component's OnDestroy synchronously, so the invariant (statics cleared before the successor can
        //resolve) is preserved unchanged.
        public static void SmartDestroyImmediateWithHolder(Component _Component)
        {
            if (_Component == null)
                return;

            GameObject holder = _Component.gameObject;

            if (holder.GetComponents<Component>().Length == 2)
                Object.DestroyImmediate(holder);
            else
                Object.DestroyImmediate(_Component);
        }

        public static void SmartDestroy(Object _Object)
        {
            if (_Object == null)
            {
                return;
            }

            bool handled = false;
            SmartDestroyEditor(_Object, ref handled);
            if (!handled)
            {
                Object.Destroy(_Object);
            }
        }

        public static string GetPointsString(IEnumerable<Vector3> _Points)
        {
            return $"[{string.Join(", ", _Points.Select(_Target => _Target.ToStringExt()))}]";
        }

        //Trims subsequences of equal points
        //[{0.1, 0.1, 0.1}, {0.1, 0.1, 0.1}, {0.6, 0.6, 0.6}, {0.1, 0.1, 0.1}] -> [{0.1, 0.1, 0.1}, {0.6, 0.6, 0.6}, {0.1, 0.1, 0.1}]
        public static Vector3[] TrimEqualPoints(Vector3[] _Points)
        {
            return _Points == null ? null : TrimEqualPointIndices(_Points).Select(_Index => _Points[_Index]).ToArray();
        }

        //The indices TrimEqualPoints keeps: the first point of every run of equal consecutive points.
        public static int[] TrimEqualPointIndices(Vector3[] _Points)
        {
            return Enumerable.Range(0, _Points.Length).Where(_Index => _Index == 0 || _Points[_Index - 1] != _Points[_Index]).ToArray();
        }

        //Enumerates point starting from the closest one. If loop is true, then add beginning of the sequence to the end of ordered set
        public static Vector3[] ReorderStartingFromClosest(Vector3[] _Points, Vector3 _Position, bool _Loop)
        {
            return OrderStartingFromClosest(_Points, _Position, _Loop).Select(_Index => _Points[_Index]).ToArray();
        }

        //The indices ReorderStartingFromClosest visits, in its order.
        public static int[] OrderStartingFromClosest(Vector3[] _Points, Vector3 _Position, bool _Loop)
        {
            int   indexOfClosest = -1;
            float minSqrDistance = float.MaxValue;

            for (int i = 0; i < _Points.Length; i++)
            {
                float sqrDistance = (_Position - _Points[i]).sqrMagnitude;

                if (!(sqrDistance < minSqrDistance))
                    continue;

                minSqrDistance = sqrDistance;
                indexOfClosest = i;
            }

            List<int> result = new List<int>(_Points.Length);

            for (int i = indexOfClosest; i < _Points.Length; i++)
                result.Add(i);

            if (_Loop)
            {
                for (int i = 0; i < indexOfClosest; i++)
                    result.Add(i);
            }

            return result.ToArray();
        }

        //The thread-free form: the mesh's arrays and the transform's matrix were read on the main thread (the only
        //Unity calls the capture needs), the world-space triangles are built here on any thread.
        public static HashSet<Triangle> GetMeshTransformTriangles(Matrix4x4 _LocalToWorld, Vector3[] _Vertices, int[] _Indices)
        {
            int               trianglesCount = _Indices.Length / 3;
            HashSet<Triangle> triangles      = new HashSet<Triangle>(trianglesCount);

            //a mirroring transform (negative determinant) turns a triangle's winding around; Unity flips culling for it,
            //and so must the capture, or the triangle's normal — which decides a body's inside and local avoidance's
            //side — points into the body
            bool mirrored = _LocalToWorld.determinant < 0f;

            for (int i = 0; i < trianglesCount; i++)
            {
                int index = i * 3;

                Vector3 a = _LocalToWorld.MultiplyPoint3x4(_Vertices[_Indices[index]]);
                Vector3 b = _LocalToWorld.MultiplyPoint3x4(_Vertices[_Indices[index + 1]]);
                Vector3 c = _LocalToWorld.MultiplyPoint3x4(_Vertices[_Indices[index + 2]]);

                Triangle triangle = mirrored ? new Triangle(a, c, b) : new Triangle(a, b, c);

                triangles.Add(triangle);
            }

            return triangles;
        }

        public static HashSet<Triangle> GetMeshTransformTriangles(Transform _Transform, Mesh _Mesh)
        {
            int[]     meshTriangles = _Mesh.triangles;
            Vector3[] meshVertices  = _Mesh.vertices;

            Quaternion rotation = _Transform.rotation;
            Vector3    position = _Transform.position;
            Vector3    scale    = _Transform.lossyScale;

            //same winding rule as the matrix overload: an odd number of negative scale axes mirrors
            bool mirrored = scale.x * scale.y * scale.z < 0f;

            int               trianglesCount = meshTriangles.Length / 3;
            HashSet<Triangle> triangles      = new HashSet<Triangle>(trianglesCount);

            for (int i = 0; i < trianglesCount; i++)
            {
                int index = i * 3;

                Vector3 p0 = meshVertices[meshTriangles[index]];
                Vector3 p1 = meshVertices[meshTriangles[index + 1]];
                Vector3 p2 = meshVertices[meshTriangles[index + 2]];

                //We should apply transformations following next order:
                //1) scale
                //2) rotate
                //3) translate
                p0.Scale(scale);
                p0 = rotation * p0 + position;
                p1.Scale(scale);
                p1 = rotation * p1 + position;
                p2.Scale(scale);
                p2 = rotation * p2 + position;

                //triangle in global space
                Triangle triangle = mirrored ? new Triangle(p0, p2, p1) : new Triangle(p0, p1, p2);

                triangles.Add(triangle);
            }

            return triangles;
        }
        
        public static HashSet<Triangle> GetTerrainTransformTriangles(Transform _Transform, TerrainData _TerrainData)
        {
            //    Z
            //    ^
            //    |
            //  z —   p2 *————*p3
            //    |      |\   |
            //    |      | \  |
            //    |      |  \ |
            //    | 	 |   \|
            //z-1—   p0 *————*p1 
            //    |
            //    O——————|————|————> X
            //		    x-1   x 
            HashSet<Triangle> result = new HashSet<Triangle>();
            
            int      heightMapResolution        = _TerrainData.heightmapResolution;
            float[,] heightMap                  = _TerrainData.GetHeights(0, 0, heightMapResolution, heightMapResolution);
            float    xRealSize                  = _TerrainData.size.x;
            float    yRealSize                  = _TerrainData.size.y;
            float    zRealSize                  = _TerrainData.size.z;
            float    inverseHeightMapResolution = 1f / heightMapResolution;

            Vector3 position = _Transform.position;
            
            for (int x = 1; x < heightMapResolution; x++)
            {
                int xPre = x - 1;
                int zPre = 0;

                Vector3 p0 = position + new Vector3(
                        xRealSize             * xPre * inverseHeightMapResolution,
                        heightMap[zPre, xPre] * yRealSize,
                        zRealSize             * zPre * inverseHeightMapResolution
                    );

                Vector3 p1 = position + new Vector3(
                        xRealSize          * x * inverseHeightMapResolution,
                        heightMap[zPre, x] * yRealSize,
                        zRealSize          * zPre * inverseHeightMapResolution
                    );

                for (int z = 0; z < heightMapResolution; z++)
                {
                    Vector3 p2 = position + new Vector3(
                            xRealSize          * xPre * inverseHeightMapResolution,
                            heightMap[z, xPre] * yRealSize,
                            zRealSize          * z * inverseHeightMapResolution
                        );

                    Vector3 p3 = position + new Vector3(
                            xRealSize       * x * inverseHeightMapResolution,
                            heightMap[z, x] * yRealSize,
                            zRealSize       * z * inverseHeightMapResolution
                        );

                    result.Add(new Triangle(p0, p2, p1));
                    result.Add(new Triangle(p1, p2, p3));

                    zPre++;
                    p0 = p2;
                    p1 = p3;
                }
            }

            return result;
        }

        #endregion
    }
}
