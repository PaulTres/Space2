#if UNITY_EDITOR

using Nav3D.Common.Debug;
using UnityEditor;
using UnityEngine;

namespace Nav3D.Pathfinding
{
    public partial class Path
    {
        // capsule-body rendering seam — implemented in Path.Volumetric.Editor.cs; no-op when stripped.
        partial void VolumetricDrawBodyHook();

        public void Draw()
        {
            if (m_Request == null || TrajectoryFinal == null)
                return;

            VolumetricDrawBodyHook();

            const float TARGET_RADIUS = 0.1f;

            using (UtilsGizmos.ColorPermanence)
            {
                Gizmos.color = IsValid ? Color.cyan : Color.red;

                for (int i = 1; i < TrajectoryFinal.Length; i++)
                {
                    Gizmos.DrawLine(TrajectoryFinal[i - 1], TrajectoryFinal[i]);
                }

                GUIStyle style = new GUIStyle
                {
                    normal =
                    {
                        textColor = Gizmos.color
                    }
                };

                for (int i = 0; i < m_Request.Points.Length; i++)
                {
                    Vector3 target = m_Request.Points[i];
                    Gizmos.DrawWireSphere(target, TARGET_RADIUS);
                    Handles.Label(target, $"[{i}]", style);
                }
            }
        }
    }
}

#endif
