using System;
using System.Collections.Generic;
using System.Linq;
using UnityEditor;
using UnityEditor.PackageManager;
using UnityEngine;
using UnityEngine.Rendering;
using PackageInfo = UnityEditor.PackageManager.PackageInfo;

#if NAV3D_SRP_CORE

using UnityEditor.Rendering;

#endif

#if NAV3D_URP

using UnityEditor.Rendering.Universal;
using UnityEngine.Rendering.Universal;

#endif

#if NAV3D_HDRP

using UnityEngine.Rendering.HighDefinition;

#endif

namespace Nav3D.Tools
{
    //Converts editable demo materials after import and from the menu after a pipeline switch.
    sealed class Nav3DDemoMaterialConverter : AssetPostprocessor
    {
        #if NAV3D_HDRP

        #region Nested types

        //The authored demo palette uses Standard colours, scalar surface values and base-colour textures.
        sealed class DemoHdrpUpgrader : MaterialUpgrader
        {
            #region Constructors

            public DemoHdrpUpgrader()
            {
                RenameShader("Standard", "HDRP/Lit");
                RenameColor("_Color", "_BaseColor");
                RenameTexture("_MainTex", "_BaseColorMap");
                RenameFloat("_Glossiness", "_Smoothness");
            }

            #endregion

            #region Public methods

            public override void Convert(Material _Source, Material _Destination)
            {
                base.Convert(_Source, _Destination);
                _Destination.SetFloat("_Metallic", Mathf.Pow(_Source.GetFloat("_Metallic"), 2.2f));
                HDMaterial.SetUseEmissiveIntensity(_Destination, false);
                _Destination.SetColor("_EmissiveColor", _Source.IsKeywordEnabled("_EMISSION") ? _Source.GetColor("_EmissionColor") : Color.black);
                HDMaterial.SetSurfaceType(_Destination, _Source.GetFloat("_Mode") == 3f);
                HDMaterial.SetRenderingPass(_Destination, HDMaterial.RenderingPass.Default);
                HDMaterial.ValidateMaterial(_Destination);
            }

            #endregion
        }

        #endregion

        #endif

        #region Constants

        const string MENU_ITEM     = "Tools/Nav3D/Convert Demo Materials To Active Render Pipeline";
        const string DEMO_PATH     = "/Nav3D/Demo/";
        const string LOG_TAG       = "[Nav3D]";

        #endregion

        #region Attributes

        static bool s_ConversionScheduled;

        #endregion

        #region Public methods

        [MenuItem(MENU_ITEM)]
        public static void ConvertFromMenu()
        {
            Convert(_FromMenu: true);
        }

        #endregion

        #region Service methods

        //The package import delivers the materials as Built-in assets; converting right after it is what spares the
        //customer the magenta scenes. The conversion re-imports what it changed, which arrives here again and finds
        //nothing left to convert.
        static void OnPostprocessAllAssets(string[] _Imported, string[] _Deleted, string[] _Moved, string[] _MovedFrom, bool _DidDomainReload)
        {
            //A sample imports scripts as well as materials; its assembly reload can discard an earlier delayCall.
            if (s_ConversionScheduled || (!_DidDomainReload
                && !_Imported.Any(_Path => IsEditableDemoPath(_Path) && _Path.EndsWith(".mat", StringComparison.Ordinal))))
                return;

            s_ConversionScheduled = true;
            EditorApplication.delayCall += ConvertAfterImport;
        }

        static void ConvertAfterImport()
        {
            if (EditorApplication.isCompiling || EditorApplication.isUpdating)
            {
                EditorApplication.delayCall += ConvertAfterImport;
                return;
            }
            s_ConversionScheduled = false;
            Convert(_FromMenu: false);
        }

        static bool IsEditableDemoPath(string _Path)
        {
            if (_Path.StartsWith("Assets/", StringComparison.Ordinal))
                return _Path.Contains(DEMO_PATH)
                    || (_Path.StartsWith("Assets/Samples/Nav 3D Demos/", StringComparison.Ordinal) && _Path.Contains("/Demo/"));
            if (!_Path.StartsWith("Packages/studio.softbrix.nav3d.demos/Demo/", StringComparison.Ordinal)
                && !_Path.StartsWith("Packages/studio.softbrix.nav3d-lite.demos/Demo/", StringComparison.Ordinal))
                return false;
            PackageInfo package = PackageInfo.FindForAssetPath(_Path);
            return package != null && package.source == PackageSource.Embedded;
        }

        static void Convert(bool _FromMenu)
        {
            RenderPipelineAsset pipeline = GraphicsSettings.currentRenderPipeline;

            if (pipeline == null)
            {
                if (_FromMenu)
                    Debug.Log($"{LOG_TAG} The project runs the Built-in render pipeline; the demo materials are authored for it and need no conversion.");

                return;
            }

            #if NAV3D_SRP_CORE

            List<MaterialUpgrader> upgraders = UpgradersFor(pipeline);

            if (upgraders == null)
            {
                Debug.LogWarning($"{LOG_TAG} No material converter is known for {pipeline.GetType().Name}; use the pipeline's material converter on the demo folder.");

                return;
            }

            int converted = 0;

            foreach (string path in AssetDatabase.GetAllAssetPaths().Where(_Path => _Path.EndsWith(".mat", StringComparison.Ordinal)))
            {

                if (!IsEditableDemoPath(path))
                    continue;

                Material material = AssetDatabase.LoadAssetAtPath<Material>(path);
                Shader   before = material.shader;

                MaterialUpgrader.Upgrade(material, upgraders, MaterialUpgrader.UpgradeFlags.None);

                if (material.shader == before)
                    continue;

                EditorUtility.SetDirty(material);

                converted++;
            }

            if (converted > 0)
                AssetDatabase.SaveAssets();

            if (converted > 0 || _FromMenu)
                Debug.Log($"{LOG_TAG} {converted} demo material(s) converted to {pipeline.GetType().Name}.");

            #else

            Debug.LogWarning($"{LOG_TAG} The project runs {pipeline.GetType().Name} without its render-pipeline package; the demo materials cannot be converted.");

            #endif

        }

        #if NAV3D_SRP_CORE

        static List<MaterialUpgrader> UpgradersFor(RenderPipelineAsset _Pipeline)
        {

            #if NAV3D_URP

            if (_Pipeline is UniversalRenderPipelineAsset)
                return new List<MaterialUpgrader> { new StandardUpgrader("Standard"), new StandardUpgrader("Standard (Specular setup)") };

            #endif

            #if NAV3D_HDRP

            if (_Pipeline is HDRenderPipelineAsset)
                return new List<MaterialUpgrader> { new DemoHdrpUpgrader() };

            #endif

            return null;
        }

        #endif

        #endregion

    }
}
