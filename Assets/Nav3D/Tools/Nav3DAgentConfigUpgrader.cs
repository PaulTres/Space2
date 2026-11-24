#if UNITY_EDITOR

using System.Collections.Generic;
using Nav3D.API;
using UnityEditor;
using UnityEngine;

namespace Nav3D.Tools
{
    /// <summary>
    /// Upgrades Nav3DAgentConfig assets of former versions to the current config format (redesign V2
    /// deliverable; successor of the Dev-only Nav3DProjectUpdater's config repairs):
    ///   v0 -> v1: MotionNavigationType (COMBINED/GLOBAL) -> LocalAvoidanceEnabled flag,
    ///   v1 -> v2: Speed/MaxSpeed units-per-tick -> units-per-SECOND, ORCATau ticks -> seconds
    ///             (literal, behavior-preserving conversion at the project's fixed timestep),
    ///             plus MaxSpeed derivation when a legacy asset holds MaxSpeed = 0,
    ///   v2 -> v3: the final ease curve became chronological (keys reversed, behavior-preserving),
    ///   v3 -> v4: the seconds-valued avoidance horizon (ORCATau / ORCATauObstacle) was removed — the horizon
    ///             is derived per encounter now, and AvoidanceLookahead
    ///             replaces it. Nothing converts: re-saving is what drops the dead keys from the asset.
    ///   v4 -> v5: MaxAgentDegreesRotationPerTick -> MaxAgentDegreesRotationPerSecond, divided by the
    ///             project's fixed timestep (zero remains zero; fresh default is 250 degrees/second).
    /// The editor reads old fields by their actual names (text and binary assets) and records the source
    /// version on load. Nav3DAgentConfigAssetMigration persists the converted values after import/domain reload
    /// and before a build; this window runs the same pass manually. Already migrated assets are untouched.
    /// No FormerlySerializedAs mapping is needed; the rotation compatibility source is editor-only.
    /// </summary>
    public class Nav3DAgentConfigUpgrader : EditorWindow
    {

        #region Constants

        const string NAME_MENU   = "Tools/Nav3D/Upgrade Agent Configs";
        const string NAME_WINDOW = "Agent Config Upgrader";

        #endregion

        #region Attributes

        readonly List<string> m_ReportLines = new List<string>();

        Vector2 m_ScrollPosition;

        #endregion

        #region Public methods

        [MenuItem(NAME_MENU)]
        public static void ShowWindow()
        {
            Nav3DAgentConfigUpgrader window = GetWindow<Nav3DAgentConfigUpgrader>();

            window.titleContent = new GUIContent(NAME_WINDOW);
            window.Show();
        }

        #endregion

        #region Unity methods

        void OnGUI()
        {
            EditorGUILayout.HelpBox(
                    "Upgrades all Nav3DAgentConfig assets in the project to the current config format:\n"      +
                    "• navigation type (Combined/Global) → 'Local avoidance enabled' flag\n"                   +
                    "• Speed / Max speed: units-per-tick → units-per-SECOND\n"                                 +
                    "• Max rotation: degrees-per-tick → degrees-per-SECOND\n"                                  +
                    $"Conversion uses the project's fixed timestep ({Time.fixedDeltaTime:F4}s) and preserves " +
                    "the agents' existing real-time behavior. Safe to run repeatedly.",
                    MessageType.Info
                );

            if (GUILayout.Button("Upgrade all agent configs"))
                UpgradeAllConfigs();

            if (m_ReportLines.Count == 0)
                return;

            m_ScrollPosition = EditorGUILayout.BeginScrollView(m_ScrollPosition);

            foreach (string reportLine in m_ReportLines)
                EditorGUILayout.LabelField(reportLine, EditorStyles.miniLabel);

            EditorGUILayout.EndScrollView();
        }

        #endregion

        #region Service methods

        void UpgradeAllConfigs()
        {
            m_ReportLines.Clear();
            Nav3DAgentConfigAssetMigration.UpgradeProject(m_ReportLines);
            Debug.Log($"[{nameof(Nav3DAgentConfigUpgrader)}]:\n" + string.Join("\n", m_ReportLines));
        }

        #endregion

    }
}

#endif