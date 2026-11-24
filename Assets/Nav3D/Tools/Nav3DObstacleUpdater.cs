#if UNITY_EDITOR

using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;
using UnityEditor;

namespace Nav3D.Tools
{
    public static class Nav3DObstacleUpdater
    {
        #region Constants

        const string NAME_MENU               = "Tools/Nav3D/Update Nav3DObstacles (1.6.1 -> 2.0+)";
        const string OBSOLETE_OBSTACLE_GUID  = "7eae62bb6cdc05a4eaa8345045fe4cee";
        const string OBSTACLE_PRE_BAKED_GUID = "d1964ad550c34dd7a45189151b7ae5b9";
        const string OBSTACLE_RUNTIME_GUID   = "3801bdcc7fbe4332986dc3ad4f36a368";
        
        #endregion
        
        #region Attributes

        static readonly Regex s_MonoBehGUIDRegex         = new Regex(@"guid: ([0-9a-f]{32})");
        static readonly Regex s_ObstacleDataSourceRegex  = new Regex(@"m_ObstacleDataSource: ([0-9])");
        static readonly Regex s_AutoUpdateIfChangedRegex = new Regex(@"m_AutoUpdateIfChanged: ([0-9])");
        static readonly Regex s_AutoUpdateMinPeriodRegex = new Regex(@"m_AutoUpdateMinPeriod: ([0-9])");
        
        #endregion
        
        #region Service methods
        
        [MenuItem(NAME_MENU)]
        static void ReplaceNav3DObstacles()
        {
            ProcessScenes();

            ProcessPrefabs();
        }

        static void ProcessScenes()
        {
            foreach (string scenePath in AssetDatabase.FindAssets("t:Scene").Select(AssetDatabase.GUIDToAssetPath))
            {
                UpgradeObstacleOccurrences(scenePath);
            }
        }

        static void ProcessPrefabs()
        {
            foreach (string prefabPath in AssetDatabase.FindAssets("t:Prefab").Select(AssetDatabase.GUIDToAssetPath))
            {
                UpgradeObstacleOccurrences(prefabPath);
            }
        }

        static void UpgradeObstacleOccurrences(string _AssetPath)
        {
            bool isMonoBehHunk         = false;
            bool isObstacleMonoBehHunk = false;
            
            StringBuilder curMonoBehContent = new StringBuilder();

            Dictionary<string, string> monoBehForReplacement = new Dictionary<string, string>();
            
            foreach (string line in File.ReadLines(_AssetPath))
            {
                if (isMonoBehHunk)
                {
                    //record line
                    curMonoBehContent.AppendLine(line);

                    //check script type if Nav3DObstacle then set isObstacleMonoBehHunk = true
                    if (line.Contains("m_Script") && s_MonoBehGUIDRegex.IsMatch(line))
                    {
                        if (s_MonoBehGUIDRegex.Match(line).Groups[1].Value == OBSOLETE_OBSTACLE_GUID)
                        {
                            isObstacleMonoBehHunk = true;
                        }
                        else
                        {
                            isMonoBehHunk = false;
                            curMonoBehContent.Clear();
                        }
                    }

                    //we
                    if (line.Substring(0, 3) == "---")
                    {
                        if (isObstacleMonoBehHunk)
                        {
                            string obstacleMonoBehString = curMonoBehContent.ToString();

                            monoBehForReplacement.Add(obstacleMonoBehString, UpdateObstacleInHunk(obstacleMonoBehString));
                        }

                        isMonoBehHunk         = false;
                        isObstacleMonoBehHunk = false;
                        curMonoBehContent.Clear();
                    }
                }
                else if (line == "MonoBehaviour:")
                {
                    isMonoBehHunk = true;
                    curMonoBehContent.AppendLine(line);
                }
            }

            string file = File.ReadAllText(_AssetPath);

            foreach (KeyValuePair<string, string> replacementData in monoBehForReplacement)
            {
                file = file.Replace(replacementData.Key, replacementData.Value);
            }

            File.WriteAllText(_AssetPath, file);
        }

        static string UpdateObstacleInHunk(string _Obstacle)
        {
            List<string> lines = _Obstacle.Split('\n').ToList();

            string dataSourceString = lines!.FirstOrDefault(_Line => s_ObstacleDataSourceRegex.IsMatch(_Line));
            string dataSource       = s_ObstacleDataSourceRegex.Match(dataSourceString!).Groups[1].Value;

            lines.Remove(dataSourceString);

            int monoBehGUIDLineIndex = lines.FindIndex(_Line => s_MonoBehGUIDRegex.IsMatch(_Line));
            lines[monoBehGUIDLineIndex] = lines[monoBehGUIDLineIndex].Replace(OBSOLETE_OBSTACLE_GUID, dataSource == "0" ? OBSTACLE_RUNTIME_GUID : OBSTACLE_PRE_BAKED_GUID);
            
            //if Pre-baked obstacle type then remove auto-update children options
            if (dataSourceString == "1")
            {
                lines.Remove(lines.FirstOrDefault(_Line => s_AutoUpdateIfChangedRegex.IsMatch(_Line)));
                lines.Remove(lines.FirstOrDefault(_Line => s_AutoUpdateMinPeriodRegex.IsMatch(_Line)));
            }

            return string.Join("\n", lines);
        }
        
        #endregion
    }
}

#endif