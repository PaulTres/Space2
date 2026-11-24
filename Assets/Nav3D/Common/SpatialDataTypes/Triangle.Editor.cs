#if UNITY_EDITOR
using UnityEngine;

namespace Nav3D.Common
{
    public partial class Triangle
    {
        public void Draw(bool _DrawNormal = false)
        {
            Gizmos.DrawLine(V1, V2);
            Gizmos.DrawLine(V1, V3);
            Gizmos.DrawLine(V2, V3);

            if (_DrawNormal)
            {
                float sizeFactor = Mathf.Max(
                        Vector3.Distance(V1, V2),
                        Vector3.Distance(V1, V3),
                        Vector3.Distance(V2, V3)
                    );

                Gizmos.DrawLine(Center, Center + Normal.normalized * sizeFactor);
            }
        }
    }
}
#endif
