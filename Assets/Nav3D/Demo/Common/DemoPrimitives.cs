using System.Collections.Generic;
using UnityEngine;

namespace Nav3D.Demo
{
    //DEMO kit — primitive-shaped bodies without a collider. GameObject.CreatePrimitive builds one with every body, and destroying
    //it in the same frame still puts a static actor through the physics scene (measured on Skyrift: a broadphase add + remove
    //per body and physics steps of 40–170 ms); here the mesh is taken once per shape and shared
    public static class DemoPrimitives
    {
        #region Attributes

        //built-in meshes are never unloaded, so the cache survives scene loads
        static readonly Dictionary<PrimitiveType, Mesh> s_Meshes = new Dictionary<PrimitiveType, Mesh>();

        #endregion

        #region Public methods

        public static GameObject CreateBody(PrimitiveType _Shape, string _Name)
        {
            GameObject body = new GameObject(_Name);

            body.AddComponent<MeshFilter>().sharedMesh = Mesh(_Shape);
            body.AddComponent<MeshRenderer>();

            return body;
        }

        public static Mesh Mesh(PrimitiveType _Shape)
        {
            if (s_Meshes.TryGetValue(_Shape, out Mesh mesh) && mesh != null)
                return mesh;

            //the one primitive per shape per session that carries a collider; destroyed at the end of this frame, its built-in mesh kept
            GameObject primitive = GameObject.CreatePrimitive(_Shape);

            mesh = primitive.GetComponent<MeshFilter>().sharedMesh;

            Object.Destroy(primitive);

            s_Meshes[_Shape] = mesh;

            return mesh;
        }

        #endregion
    }
}
