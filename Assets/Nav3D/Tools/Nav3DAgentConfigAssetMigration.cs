using System;
using System.Collections.Generic;
using Nav3D.API;
using UnityEditor;
using UnityEditor.Build;
using UnityEditor.Build.Reporting;
using UnityEngine;

namespace Nav3D.Tools
{
    //Native serialization reads the editor-only legacy field, including binary/sub-assets.
    //This pass persists only configs that actually migrated; no global SaveAssets or YAML rewriting.
    [InitializeOnLoad]
    public sealed class Nav3DAgentConfigAssetMigration : AssetPostprocessor, IPreprocessBuildWithReport
    {
        static bool s_Scheduled;
        static bool s_Running;
        public int  callbackOrder => -1000;

        static Nav3DAgentConfigAssetMigration()
        {
            EditorApplication.playModeStateChanged += _State =>
            {
                if (_State == PlayModeStateChange.EnteredEditMode)
                    Schedule();
            };

            Schedule();
        }

        static void OnPostprocessAllAssets(string[] _Imported, string[] _Deleted, string[] _Moved, string[] _MovedFrom, bool _DidDomainReload)
        {
            if (s_Running)
                return;

            if (_DidDomainReload || _Imported.Length > 0 || _Moved.Length > 0)
                Schedule();
        }

        static void Schedule()
        {
            if (s_Scheduled)
                return;

            s_Scheduled                 =  true;
            EditorApplication.delayCall += RunScheduled;
        }

        static void RunScheduled()
        {
            s_Scheduled = false;

            if (EditorApplication.isPlayingOrWillChangePlaymode)
                return; //the edit-mode transition schedules the pending pass again

            if (EditorApplication.isCompiling || EditorApplication.isUpdating)
            {
                Schedule();
                return;
            }

            try
            {
                int count = UpgradeProject();

                if (count > 0)
                    Debug.Log($"[Nav3D] Automatically migrated {count} agent config asset(s) to v{Nav3DAgentConfig.ConfigVersionCurrent}.");
            }
            catch (Exception exception)
            {
                Debug.LogError("[Nav3D] Agent config asset migration failed. Use Nav3D/Tools/Upgrade Agent Configs after fixing asset write access. " + exception.Message);
            }
        }

        public static int UpgradeProject(List<string> _Report = null)
        {
            if (s_Running)
                throw new InvalidOperationException("Agent config migration is already running.");

            s_Running = true;
            int upgraded = 0;

            try
            {
                foreach (string guid in AssetDatabase.FindAssets("t:Nav3DAgentConfig"))
                {
                    string path = AssetDatabase.GUIDToAssetPath(guid);

                    foreach (UnityEngine.Object asset in AssetDatabase.LoadAllAssetsAtPath(path))
                    {
                        if (!(asset is Nav3DAgentConfig config))
                            continue;

                        config.MigrateLegacyFieldsIfNeeded();
                        int sourceVersion = config.AssetMigrationSourceVersion;

                        if (sourceVersion < 0)
                            continue;

                        //Package-cache assets are immutable; silently losing an old rate in a build is unacceptable.
                        if (!path.StartsWith("Assets/", StringComparison.Ordinal) || !AssetDatabase.IsOpenForEdit(path))
                            throw new InvalidOperationException($"Cannot persist {path}. Copy the legacy config to writable Assets or make it editable first.");

                        config.MarkAssetMigrationPersisted();

                        try
                        {
                            EditorUtility.SetDirty(config);
                            AssetDatabase.SaveAssetIfDirty(config);

                            if (EditorUtility.IsDirty(config))
                                throw new InvalidOperationException($"Unity did not save {path}.");
                        }
                        catch
                        {
                            config.SetAssetMigrationSourceVersion(sourceVersion);
                            throw;
                        }

                        _Report?.Add(
                                ($"{path} ({config.name}): v{sourceVersion} → v{Nav3DAgentConfig.ConfigVersionCurrent}, rotation " +
                                 $"{config.MaxAgentDegreesRotationPerSecond:F3} degrees/s")
                            );

                        upgraded++;
                    }
                }
            }
            finally
            {
                s_Running = false;
            }

            _Report?.Add($"{upgraded} config asset(s) upgraded; current assets unchanged.");
            return upgraded;
        }

        public void OnPreprocessBuild(BuildReport _Report)
        {
            try
            {
                UpgradeProject();
            }
            catch (Exception exception)
            {
                throw new BuildFailedException("Nav3D agent configs must be migrated before building: " + exception.Message);
            }
        }
    }
}