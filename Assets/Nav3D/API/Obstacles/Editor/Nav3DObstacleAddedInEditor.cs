using UnityEditor;

namespace Nav3D.API.Editor
{
    //Gives an obstacle added through the Editor's UI the first geometry source by priority. Only the UI raises
    //componentWasAdded, so an obstacle a script adds, and one saved before the setting existed, keep reading meshes and
    //terrains: a scene builder, a test rig or a customer's spawner gets the geometry it always got.
    static class Nav3DObstacleAddedInEditor
    {
        #region Service methods

        [InitializeOnLoadMethod]
        static void Subscribe()
        {
            ObjectFactory.componentWasAdded += _Component =>
            {
                if (!(_Component is Nav3DObstacle))
                    return;

                SerializedObject serialized = new SerializedObject(_Component);

                serialized.FindProperty("m_GeometrySources").intValue = (int) ObstacleGeometrySources.AUTO;
                serialized.ApplyModifiedPropertiesWithoutUndo();
            };
        }

        #endregion
    }
}
