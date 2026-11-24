using Nav3D.Common;
using UnityEditor;
using UnityEngine;

namespace Nav3D.API.Editor
{
    //The inspector twin of Nav3DPath.ReportRadiusAboveWorldBound: the world settings are looked up through the
    //open scene's Nav3DInitializer, so a config asset can be judged only while such a scene is open.
    public static class AgentRadiusWorldCheck
    {

        #region Constants

        const string WARNING = "Radius {0:0.##} exceeds Max agent radius {1:0.##} of {2}, so paths keep only {1:0.##} clear of obstacles. " +
                               "Raise Max agent radius or reduce the radius.";

        const double LOOKUP_PERIOD_SECONDS = 1d;

        #endregion

        #region Attributes

        static Nav3DInitializer s_Initializer;
        static double           s_LookupTime = -1d;

        #endregion

        #region Public methods

        public static void DrawHelpBox(Nav3DAgentConfig _Config)
        {
            if (_Config != null)
                DrawHelpBox(_Config.PathfindingRadius);
        }

        //the same judgement for a request that names its radius directly (Nav3DPathTester)
        public static void DrawHelpBox(float _PathfindingRadius)
        {
            if (_PathfindingRadius <= 0f)
                return;

            Nav3DInitializer initializer = FindInitializer();

            if (initializer == null || _PathfindingRadius <= initializer.MaxAgentRadius)
                return;

            string settingsName = initializer.Settings != null ? initializer.Settings.name : nameof(Nav3DInitializer);

            EditorGUILayout.HelpBox(string.Format(WARNING, _PathfindingRadius, initializer.MaxAgentRadius, settingsName), MessageType.Warning);
        }

        #endregion

        #region Service methods

        //a scene scan per repaint is too much for an inspector; once a second is not
        static Nav3DInitializer FindInitializer()
        {
            double now = EditorApplication.timeSinceStartup;

            if (s_Initializer == null || now - s_LookupTime > LOOKUP_PERIOD_SECONDS)
            {
                s_Initializer = UtilsSceneObjects.FindAny<Nav3DInitializer>();
                s_LookupTime  = now;
            }

            return s_Initializer;
        }

        #endregion

    }
}