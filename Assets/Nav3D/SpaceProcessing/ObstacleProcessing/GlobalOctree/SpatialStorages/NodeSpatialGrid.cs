using Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.Elements;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.SpatialStorages
{
    class NodeSpatialGrid
    {
        #region Attributes

        readonly int                                          m_LayersCount;
        readonly List<ConcurrentDictionary<Vector3Int, Node>> m_Layers;

        #endregion

        #region Constructors

        public NodeSpatialGrid(int _LayersCount, IEnumerable<Node> _InitialNodes = null)
        {
            m_LayersCount = _LayersCount;
            m_Layers      = new List<ConcurrentDictionary<Vector3Int, Node>>(m_LayersCount);

            for (int i = 0; i < m_LayersCount; i++)
                m_Layers.Add(new ConcurrentDictionary<Vector3Int, Node>());

            if (_InitialNodes != null)
                foreach (Node node in _InitialNodes)
                    AddNode(node);
        }

        #endregion

        #region Public methods

        public void AddNode(Node _Node)
        {
            m_Layers[_Node.GridLayer][_Node.Index] = _Node;
        }

        public void RemoveNode(int _Layer, Vector3Int _Index)
        {
            m_Layers[_Layer].TryRemove(_Index, out _);
        }

        public bool TryGetNode(int _GridLayer, Vector3Int _Index, out Node _Node)
        {
            if (_GridLayer > m_Layers.Count - 1)
            {
                _Node = null;
                return false;
            }

            return m_Layers[_GridLayer].TryGetValue(_Index, out _Node);
        }

        public Node[] GetAllAtLevel(int _GridLevel)
        {
            return m_Layers[_GridLevel].Values.ToArray();
        }

        //P28: a finished staging grid is cleared at transaction Dispose so a conservatively pinned grid object
        //cannot go on retaining the nodes it staged. Never call
        //on the live roots grid.
        public void Clear()
        {
            foreach (ConcurrentDictionary<Vector3Int, Node> layer in m_Layers)
                layer.Clear();
        }

        #endregion
    }
}
