using System.IO;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UI;

namespace Nav3D.Demo
{
    //Esc-toggled overlay listing every scene in the build settings; a row loads it. Runtime-built, no prefab.
    public class DemoSceneMenu : MonoBehaviour
    {

        #region Constants

        const int    CANVAS_SORT_ORDER = 20;
        const float  ROW_HEIGHT        = 42f;
        const string BUILTIN_FONT      = "LegacyRuntime.ttf";

        #endregion

        #region Attributes

        GameObject m_Root;

        #endregion

        #region Properties

        //a scene that locks the hardware cursor gives it back for the rows
        public bool IsOpen => m_Root != null && m_Root.activeSelf;

        #endregion

        #region Unity events

        void Awake()
        {
            BuildMenu();

            m_Root.SetActive(false);
        }

        void Update()
        {
            if (DemoInput.GetKeyDown(KeyCode.Escape))
                m_Root.SetActive(!m_Root.activeSelf);
        }

        #endregion

        #region Service methods

        void BuildMenu()
        {
            Canvas canvas = gameObject.AddComponent<Canvas>();

            canvas.renderMode   = RenderMode.ScreenSpaceOverlay;
            canvas.sortingOrder = CANVAS_SORT_ORDER;

            CanvasScaler scaler = gameObject.AddComponent<CanvasScaler>();

            scaler.uiScaleMode         = CanvasScaler.ScaleMode.ScaleWithScreenSize;
            scaler.referenceResolution = new Vector2(1920f, 1080f);

            gameObject.AddComponent<GraphicRaycaster>();

            m_Root = new GameObject("MenuRoot");

            m_Root.transform.SetParent(transform, false);

            RectTransform rootRect = m_Root.AddComponent<RectTransform>();

            rootRect.anchorMin = Vector2.zero;
            rootRect.anchorMax = Vector2.one;
            rootRect.offsetMin = Vector2.zero;
            rootRect.offsetMax = Vector2.zero;

            Image backdrop = m_Root.AddComponent<Image>();

            backdrop.color = new Color(0f, 0f, 0f, 0.85f);

            GameObject listGo = new GameObject("SceneList");

            listGo.transform.SetParent(m_Root.transform, false);

            RectTransform listRect = listGo.AddComponent<RectTransform>();

            listRect.anchorMin        = new Vector2(0.5f, 0.5f);
            listRect.anchorMax        = new Vector2(0.5f, 0.5f);
            listRect.pivot            = new Vector2(0.5f, 0.5f);
            listRect.anchoredPosition = Vector2.zero;
            listRect.sizeDelta        = new Vector2(600f, 800f);

            VerticalLayoutGroup layout = listGo.AddComponent<VerticalLayoutGroup>();

            layout.spacing               = 6f;
            layout.childAlignment        = TextAnchor.MiddleCenter;
            layout.childControlWidth     = true;
            layout.childControlHeight    = false;
            layout.childForceExpandWidth = true;

            ContentSizeFitter fitter = listGo.AddComponent<ContentSizeFitter>();

            fitter.verticalFit = ContentSizeFitter.FitMode.PreferredSize;

            CreateHeader(listGo.transform);

            int sceneCount   = SceneManager.sceneCountInBuildSettings;
            int currentIndex = SceneManager.GetActiveScene().buildIndex;

            for (int i = 0; i < sceneCount; i++)
            {
                string scenePath = SceneUtility.GetScenePathByBuildIndex(i);
                string sceneName = Path.GetFileNameWithoutExtension(scenePath);

                CreateSceneRow(listGo.transform, i, i == currentIndex ? $"> {sceneName} <" : sceneName);
            }
        }

        void CreateHeader(Transform _Parent)
        {
            GameObject headerGo = new GameObject("Header");

            headerGo.transform.SetParent(_Parent, false);

            RectTransform rect = headerGo.AddComponent<RectTransform>();

            rect.sizeDelta = new Vector2(600f, ROW_HEIGHT * 1.5f);

            Text text = headerGo.AddComponent<Text>();

            text.font      = Resources.GetBuiltinResource<Font>(BUILTIN_FONT);
            text.fontSize  = 30;
            text.fontStyle = FontStyle.Bold;
            text.alignment = TextAnchor.MiddleCenter;
            text.color     = Color.white;
            text.text      = "Nav3D Demos  (Esc to close)";
        }

        void CreateSceneRow(Transform _Parent, int _BuildIndex, string _Label)
        {
            GameObject rowGo = new GameObject($"Scene_{_BuildIndex}");

            rowGo.transform.SetParent(_Parent, false);

            RectTransform rect = rowGo.AddComponent<RectTransform>();

            rect.sizeDelta = new Vector2(600f, ROW_HEIGHT);

            Image background = rowGo.AddComponent<Image>();

            background.color = new Color(1f, 1f, 1f, 0.08f);

            Button button = rowGo.AddComponent<Button>();

            int buildIndex = _BuildIndex;

            button.onClick.AddListener(() =>
                    {
                        Time.timeScale = 1f;

                        SceneManager.LoadScene(buildIndex);
                    }
                );

            GameObject labelGo = new GameObject("Label");

            labelGo.transform.SetParent(rowGo.transform, false);

            RectTransform labelRect = labelGo.AddComponent<RectTransform>();

            labelRect.anchorMin = Vector2.zero;
            labelRect.anchorMax = Vector2.one;
            labelRect.offsetMin = Vector2.zero;
            labelRect.offsetMax = Vector2.zero;

            Text label = labelGo.AddComponent<Text>();

            label.font      = Resources.GetBuiltinResource<Font>(BUILTIN_FONT);
            label.fontSize  = 22;
            label.alignment = TextAnchor.MiddleCenter;
            label.color     = Color.white;
            label.text      = _Label;
        }

        #endregion

    }
}