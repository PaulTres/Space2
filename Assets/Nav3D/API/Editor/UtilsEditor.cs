using System;
using UnityEngine;
using UnityEditor;

namespace Nav3D.API.Editor
{
    public static class UtilsEditor
    {
        #region Attributes

        static Texture s_CheckTexture;

        static readonly VolumetricOptimality[] s_OptimalityOrder =
        {
            VolumetricOptimality.FAST,
            VolumetricOptimality.QUALITY,
            VolumetricOptimality.HIGH_QUALITY
        };

        static readonly GUIContent[] s_OptimalityNames = Array.ConvertAll(s_OptimalityOrder, _Optimality => new GUIContent(_Optimality.ToString()));

        #endregion

        #region Properties

        static Texture CheckTexture => s_CheckTexture ??= (Texture)AssetDatabase.LoadAssetAtPath(AssetDatabase.GUIDToAssetPath("af79b0ff5863a4b4db304d2cef5b5dc2"), typeof(Texture));

        public static GUIStyle BoldAndWrappedStyle
        {
            get
            {
                GUIStyle style = EditorStyles.wordWrappedLabel;

                style.fontStyle = FontStyle.Bold;
                style.alignment = TextAnchor.MiddleLeft;

                return style;
            }
        }

        public static GUIStyle WrappedStyle
        {
            get
            {
                GUIStyle style = EditorStyles.wordWrappedLabel;

                style.fontStyle = FontStyle.Normal;
                style.alignment = TextAnchor.MiddleLeft;

                return style;
            }
        }

        #endregion

        #region Public methods

        public static void GetCheckIcon(float _Size = 32)
        {
            Rect rect = GUILayoutUtility.GetRect(_Size, _Size);
            GUI.DrawTexture(rect, CheckTexture, ScaleMode.ScaleToFit);
        }

        public static void SeparatorHorizontal(int _Thickness = 1)
        {
            EditorGUILayout.Space();

            Rect rect = EditorGUI.IndentedRect(EditorGUILayout.GetControlRect(false, _Thickness));
            
            rect.height = _Thickness;

            EditorGUI.DrawRect(rect, new Color(0.5f, 0.5f, 0.5f, 1));

            EditorGUILayout.Space();
        }

        //Unity's enum popup lists members by value, and QUALITY stays 0 to stay the default of every serialized config; this one
        //lists them in order of quality and cost.
        public static void OptimalityField(SerializedProperty _Property, GUIContent _Label)
        {
            Rect rect = EditorGUILayout.GetControlRect();

            GUIContent label = EditorGUI.BeginProperty(rect, _Label, _Property);

            EditorGUI.BeginChangeCheck();

            int picked = EditorGUI.Popup(rect, label, Array.IndexOf(s_OptimalityOrder, (VolumetricOptimality) _Property.intValue), s_OptimalityNames);

            if (EditorGUI.EndChangeCheck())
                _Property.intValue = (int) s_OptimalityOrder[picked];

            EditorGUI.EndProperty();
        }

        #endregion
    }
}
