using UnityEngine;

namespace Nav3D.Demo
{
    //A countdown dial whose sector closes as the clock runs out. The sector's geometry is the animation, so it is the one part
    //built at runtime; place, radii and the two state materials stay authored. Needs clearance
    //of its outer radius behind it: a billboarded dial swings its rim toward the eye in any direction.
    [RequireComponent(typeof(MeshFilter))]
    public class DemoRadialTimer : MonoBehaviour
    {
        #region Serialized fields

        [SerializeField]                 float m_InnerRadius = 0.55f;
        [SerializeField]                 float m_OuterRadius = 1.6f;
        [SerializeField, Range(12, 180)] int   m_Segments    = 90;

        #endregion

        #region Attributes

        Mesh      m_Mesh;
        Vector3[] m_Vertices;

        float m_Remaining = 1f;

        #endregion

        #region Unity events

        void Awake()
        {
            BuildDial();
        }

        void OnDestroy()
        {
            //the mesh belongs to this instance, not to the scene's assets
            if (m_Mesh != null)
                Destroy(m_Mesh);
        }

        #endregion

        #region Public methods

        //1 = the full circle, 0 = nothing left; the same segments always span the current sweep, so it collapses smoothly to nothing
        public void SetRemaining(float _Remaining01)
        {
            float remaining = Mathf.Clamp01(_Remaining01);

            if (Mathf.Approximately(remaining, m_Remaining))
                return;

            m_Remaining = remaining;

            WriteSector(remaining);

            m_Mesh.vertices = m_Vertices;
        }

        #endregion

        #region Service methods

        void BuildDial()
        {
            int sidePoints = (m_Segments + 1) * 2;   //an inner and an outer point per step of the sweep

            m_Vertices = new Vector3[sidePoints * 2];   //the same points twice: one face each way

            Vector3[] normals   = new Vector3[m_Vertices.Length];
            int[]     triangles = new int[m_Segments * 12];

            for (int point = 0; point < sidePoints; point++)
            {
                normals[point]              = Vector3.back;
                normals[sidePoints + point] = Vector3.forward;
            }

            for (int segment = 0; segment < m_Segments; segment++)
            {
                int inner     = segment * 2;
                int outer     = inner + 1;
                int nextInner = inner + 2;
                int nextOuter = inner + 3;

                int triangle = segment * 12;

                //wound so the normal is the dial's local -Z
                triangles[triangle]     = inner;
                triangles[triangle + 1] = outer;
                triangles[triangle + 2] = nextOuter;
                triangles[triangle + 3] = inner;
                triangles[triangle + 4] = nextOuter;
                triangles[triangle + 5] = nextInner;

                //the same quad reversed on the back-facing copy of the points
                triangles[triangle + 6]  = sidePoints + inner;
                triangles[triangle + 7]  = sidePoints + nextOuter;
                triangles[triangle + 8]  = sidePoints + outer;
                triangles[triangle + 9]  = sidePoints + inner;
                triangles[triangle + 10] = sidePoints + nextInner;
                triangles[triangle + 11] = sidePoints + nextOuter;
            }

            WriteSector(m_Remaining);

            m_Mesh = new Mesh { name = $"{name}Sector" };

            m_Mesh.MarkDynamic();

            m_Mesh.vertices  = m_Vertices;
            m_Mesh.normals   = normals;
            m_Mesh.triangles = triangles;

            //fixed bounds: recomputed as the sector closes they would shrink onto the wedge and let the dial cull itself off screen
            m_Mesh.bounds = new Bounds(Vector3.zero, new Vector3(m_OuterRadius * 2f, m_OuterRadius * 2f, 0f));

            GetComponent<MeshFilter>().sharedMesh = m_Mesh;
        }

        void WriteSector(float _Remaining01)
        {
            int   sidePoints = (m_Segments + 1) * 2;
            float sweep      = 2f * Mathf.PI * _Remaining01;

            //what is left ends at twelve o'clock and the leading edge walks clockwise toward it, the way a clock hand does
            float start = 2f * Mathf.PI - sweep;

            for (int step = 0; step <= m_Segments; step++)
            {
                float angle = start + sweep * step / m_Segments;

                //twelve o'clock is +Y, the sweep runs toward +X: clockwise for the eye the dial faces
                Vector3 direction = new Vector3(Mathf.Sin(angle), Mathf.Cos(angle), 0f);

                int inner = step * 2;

                m_Vertices[inner]     = direction * m_InnerRadius;
                m_Vertices[inner + 1] = direction * m_OuterRadius;

                m_Vertices[sidePoints + inner]     = m_Vertices[inner];
                m_Vertices[sidePoints + inner + 1] = m_Vertices[inner + 1];
            }
        }

        #endregion
    }
}
