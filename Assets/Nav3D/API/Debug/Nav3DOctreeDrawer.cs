#if UNITY_EDITOR || NAV3D_DEBUG_VISUALIZATION

using Nav3D.Common;
using Nav3D.Obstacles;
using Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree;
using System;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;
using UnityEngine.Rendering;

#if UNITY_EDITOR

using UnityEditor;

#endif

namespace Nav3D.API
{
    //What the octree drawer renders: everything the octree holds, or a zone of interest (a radius around a
    //Transform and/or an explicit world Bounds), filtered when the instance list is built.
    public enum OctreeDrawScope
    {
        WHOLE_WORLD = 0,
        ZONE        = 1,
    }

    //How a population of cells is drawn: translucent filled cubes, their twelve edges as lines, or both.
    public enum OctreeCellStyle
    {
        FILLED               = 0,
        WIREFRAME            = 1,
        FILLED_AND_WIREFRAME = 2,
    }

    /// <summary>
    /// Runtime (play-mode) renderer of the octree's occupied leaves, of the free space as the navigation boxes, and of
    /// the navigation graph between them; the octree's own free leaves are an advanced view.
    /// Compiled in the editor, and in a player build only under the NAV3D_DEBUG_VISUALIZATION scripting define.
    /// </summary>
    [DisallowMultipleComponent]
    public class Nav3DOctreeDrawer : MonoBehaviour
    {
        #region Constants

        //Graphics.DrawMeshInstanced hard cap per call
        const int BATCH_CAPACITY = 1023;

        //loaded from Resources so a player build ships it
        const string SHADER_RESOURCE = "Nav3DOctreeDrawer";

        //leaf-identity resolution: finer than any bucket the octree can build, coarse enough that the same leaf
        //rebuilt by a later operation lands on the same key
        const float KEY_QUANT = 0.001f;

        //edge of one spatial bucket of the render loop: big enough that a bucket holds many cells and the per-bucket
        //frustum test pays for itself, small enough that accepting one does not drag half the world in with it
        const float BUCKET_SIZE = 32f;

        //per-cell cube scale: inset against z-fighting with the geometry the cells wrap
        const float CUBE_INSET = 0.97f;

        //edge-node marker size as a fraction of its leaf
        const float EDGE_NODE_MARKER_SCALE = 0.2f;

        //how far a scoped graph snapshot reaches past the zone, as a fraction of the zone's size: the zone can move
        //that far before the held graph stops covering it and a new snapshot is taken
        const float GRAPH_SCOPE_MARGIN = 0.5f;

        //a one-pixel edge at a cube's fill alpha all but vanishes, so a wireframe cell is drawn at this multiple of its
        //population's alpha
        const float WIREFRAME_ALPHA_SCALE = 4f;

        const string LINES_KEYWORD = "NAV3D_OCCUPANCY_LINES";

        const int LAYER_COUNT = 5;

        #endregion

        #region Nested types

        //The fixed order the populations blend in, one render queue each. Every draw call is a transparent object of its own,
        //which Unity sorts back to front by the centre of its bounds, so without a queue per population the crust's batches
        //and the free boxes' traded places as the camera moved and every overlap of the two flickered.
        enum DrawLayer
        {
            INTERIOR   = 0,
            OCCUPIED   = 1,
            FREE       = 2,
            EDGE_NODES = 3,
            GRAPH      = 4,
        }

        //Identity of one leaf across two snapshots: its place and size quantized to KEY_QUANT, plus its kind. An
        //untouched leaf produces the same key in both snapshots, so a key the new snapshot holds and the old one did
        //not is a leaf the operation actually created, split or flipped — which is what the pulse marks.
        readonly struct LeafKey : IEquatable<LeafKey>
        {
            readonly int  m_X;
            readonly int  m_Y;
            readonly int  m_Z;
            readonly int  m_Size;
            readonly bool m_Occupied;

            public LeafKey(Bounds _Bounds, bool _Occupied)
            {
                m_X        = Mathf.RoundToInt(_Bounds.center.x / KEY_QUANT);
                m_Y        = Mathf.RoundToInt(_Bounds.center.y / KEY_QUANT);
                m_Z        = Mathf.RoundToInt(_Bounds.center.z / KEY_QUANT);
                m_Size     = Mathf.RoundToInt(_Bounds.size.x   / KEY_QUANT);
                m_Occupied = _Occupied;
            }

            public bool Equals(LeafKey _Other)
            {
                return m_X == _Other.m_X && m_Y == _Other.m_Y && m_Z == _Other.m_Z && m_Size == _Other.m_Size && m_Occupied == _Other.m_Occupied;
            }

            public override bool Equals(object _Other) => _Other is LeafKey other && Equals(other);

            public override int GetHashCode()
            {
                int hash = m_X;

                hash = hash * 397 ^ m_Y;
                hash = hash * 397 ^ m_Z;
                hash = hash * 397 ^ m_Size;

                return hash * 397 ^ (m_Occupied ? 1 : 0);
            }
        }

        //One drawable population of cells, split into spatial BUCKETS of BUCKET_SIZE: the render loop tests a
        //bucket's own bounds against the frustum first and skips every cell in it at once, which is what keeps the
        //per-frame cost proportional to what the camera looks at rather than to what the world holds.
        sealed class CellInstances
        {
            //One bucket's cells. Buckets are pooled and refilled rather than rebuilt, so a rebuild allocates only
            //while the world is growing.
            public sealed class Bucket
            {
                public List<Matrix4x4> Matrices    { get; } = new List<Matrix4x4>();
                public List<Bounds>    CellBounds  { get; } = new List<Bounds>();
                public List<float>     PulseStarts { get; } = new List<float>();

                public Bounds Bounds { get; private set; }

                public int Count => Matrices.Count;

                public void Clear()
                {
                    Matrices.Clear();
                    CellBounds.Clear();
                    PulseStarts.Clear();
                }

                public void Add(Matrix4x4 _Matrix, Bounds _Bounds, float _PulseStart)
                {
                    if (Matrices.Count == 0)
                        Bounds = _Bounds;
                    else
                    {
                        Bounds bounds = Bounds;

                        bounds.Encapsulate(_Bounds);

                        Bounds = bounds;
                    }

                    Matrices.Add(_Matrix);
                    CellBounds.Add(_Bounds);
                    PulseStarts.Add(_PulseStart);
                }
            }

            readonly Dictionary<Vector3Int, Bucket> m_Buckets = new Dictionary<Vector3Int, Bucket>();
            readonly List<Bucket>                   m_Pool    = new List<Bucket>();

            public Dictionary<Vector3Int, Bucket>.ValueCollection Buckets => m_Buckets.Values;

            public int Count { get; private set; }

            public void Clear()
            {
                foreach (Bucket bucket in m_Buckets.Values)
                {
                    bucket.Clear();

                    m_Pool.Add(bucket);
                }

                m_Buckets.Clear();

                Count = 0;
            }

            public void Add(Matrix4x4 _Matrix, Bounds _Bounds, float _PulseStart)
            {
                Vector3Int key = new Vector3Int(Mathf.FloorToInt(_Bounds.center.x / BUCKET_SIZE),
                                                Mathf.FloorToInt(_Bounds.center.y / BUCKET_SIZE),
                                                Mathf.FloorToInt(_Bounds.center.z / BUCKET_SIZE));

                if (!m_Buckets.TryGetValue(key, out Bucket bucket))
                {
                    bucket = TakeBucket();

                    m_Buckets.Add(key, bucket);
                }

                bucket.Add(_Matrix, _Bounds, _PulseStart);

                Count++;
            }

            Bucket TakeBucket()
            {
                if (m_Pool.Count == 0)
                    return new Bucket();

                Bucket bucket = m_Pool[m_Pool.Count - 1];

                m_Pool.RemoveAt(m_Pool.Count - 1);

                return bucket;
            }
        }

        //Line geometry — wireframe cells or graph edges — in spatial buckets of BUCKET_SIZE, one mesh per bucket. The
        //meshes are rebuilt only when the instance lists are, so a quiet frame costs one draw call per visible bucket.
        sealed class LineBuckets
        {
            public sealed class Bucket
            {
                public List<Vector3> Vertices { get; } = new List<Vector3>();
                public List<Vector2> Pulse    { get; } = new List<Vector2>();
                public List<int>     Indices  { get; } = new List<int>();

                public Mesh   Mesh      { get; set; }
                public Bounds Bounds    { get; set; }
                public int    ItemCount { get; set; }

                public void Clear()
                {
                    Vertices.Clear();
                    Pulse.Clear();
                    Indices.Clear();

                    ItemCount = 0;
                }
            }

            //wireframe cube corners as offsets in half-extents, and the twelve edges between them
            static readonly Vector3[] s_CubeCorners =
            {
                new Vector3(-1f, -1f, -1f), new Vector3(1f, -1f, -1f), new Vector3(1f, -1f, 1f), new Vector3(-1f, -1f, 1f),
                new Vector3(-1f, 1f, -1f), new Vector3(1f, 1f, -1f), new Vector3(1f, 1f, 1f), new Vector3(-1f, 1f, 1f),
            };

            static readonly int[] s_CubeEdges = { 0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7 };

            readonly Dictionary<Vector3Int, Bucket> m_Buckets = new Dictionary<Vector3Int, Bucket>();
            readonly List<Bucket>                   m_Pool    = new List<Bucket>();

            public Dictionary<Vector3Int, Bucket>.ValueCollection Buckets => m_Buckets.Values;

            public int Count { get; private set; }

            public int VertexCount { get; private set; }

            public void Clear()
            {
                foreach (Bucket bucket in m_Buckets.Values)
                {
                    bucket.Clear();

                    m_Pool.Add(bucket);
                }

                m_Buckets.Clear();

                Count       = 0;
                VertexCount = 0;
            }

            public void AddWireCell(Bounds _Bounds, Vector2 _Pulse)
            {
                Bucket  bucket = TakeBucket(_Bounds.center, _Bounds);
                int     first  = bucket.Vertices.Count;
                Vector3 half   = _Bounds.extents * CUBE_INSET;

                foreach (Vector3 corner in s_CubeCorners)
                {
                    bucket.Vertices.Add(_Bounds.center + Vector3.Scale(corner, half));
                    bucket.Pulse.Add(_Pulse);
                }

                foreach (int index in s_CubeEdges)
                    bucket.Indices.Add(first + index);

                bucket.ItemCount++;
                Count++;
                VertexCount += s_CubeCorners.Length;
            }

            //one graph edge: leaf centre -> contact point -> neighbour centre
            public void AddEdge(Vector3 _From, Vector3 _Contact, Vector3 _To)
            {
                Bounds bounds = new Bounds(_From, Vector3.zero);

                bounds.Encapsulate(_Contact);
                bounds.Encapsulate(_To);

                Bucket bucket = TakeBucket(_Contact, bounds);
                int    first  = bucket.Vertices.Count;

                bucket.Vertices.Add(_From);
                bucket.Vertices.Add(_Contact);
                bucket.Vertices.Add(_To);

                for (int i = 0; i < 3; i++)
                    bucket.Pulse.Add(Vector2.zero);

                bucket.Indices.Add(first);
                bucket.Indices.Add(first + 1);
                bucket.Indices.Add(first + 1);
                bucket.Indices.Add(first + 2);

                bucket.ItemCount++;
                Count++;
                VertexCount += 3;
            }

            public void Upload()
            {
                foreach (Bucket bucket in m_Buckets.Values)
                {
                    if (bucket.Mesh == null)
                        bucket.Mesh = new Mesh { name = "Nav3DOctreeOccupancyLines", indexFormat = IndexFormat.UInt32, hideFlags = HideFlags.HideAndDontSave };
                    else
                        bucket.Mesh.Clear();

                    bucket.Mesh.SetVertices(bucket.Vertices);
                    bucket.Mesh.SetUVs(0, bucket.Pulse);
                    bucket.Mesh.SetIndices(bucket.Indices, MeshTopology.Lines, 0, false);
                    bucket.Mesh.bounds = bucket.Bounds;
                }

                //a pooled bucket keeps its mesh for reuse; its old contents must not linger in memory
                foreach (Bucket bucket in m_Pool)
                {
                    if (bucket.Mesh != null)
                        bucket.Mesh.Clear();
                }
            }

            public void Destroy()
            {
                Clear();

                foreach (Bucket bucket in m_Pool)
                {
                    if (bucket.Mesh != null)
                        UnityEngine.Object.Destroy(bucket.Mesh);
                }

                m_Pool.Clear();
            }

            Bucket TakeBucket(Vector3 _Anchor, Bounds _ItemBounds)
            {
                Vector3Int key = new Vector3Int(Mathf.FloorToInt(_Anchor.x / BUCKET_SIZE),
                                                Mathf.FloorToInt(_Anchor.y / BUCKET_SIZE),
                                                Mathf.FloorToInt(_Anchor.z / BUCKET_SIZE));

                if (m_Buckets.TryGetValue(key, out Bucket bucket))
                {
                    Bounds bounds = bucket.Bounds;

                    bounds.Encapsulate(_ItemBounds);

                    bucket.Bounds = bounds;

                    return bucket;
                }

                if (m_Pool.Count > 0)
                {
                    bucket = m_Pool[m_Pool.Count - 1];

                    m_Pool.RemoveAt(m_Pool.Count - 1);
                }
                else
                    bucket = new Bucket();

                bucket.Bounds = _ItemBounds;

                m_Buckets.Add(key, bucket);

                return bucket;
            }
        }

        //A finished worker snapshot on its way to the main thread: the copied-out leaves and graph, or the failure.
        sealed class SnapshotDelivery
        {
            public SnapshotDelivery(List<NavLeafInfo> _Leaves, NavGraphSnapshot _Graph, NavBoxSnapshot _Boxes, float _DurationMs, int _Epoch, bool _IncludesFreeLeaves, Exception _Error)
            {
                Leaves             = _Leaves;
                Graph              = _Graph;
                Boxes              = _Boxes;
                DurationMs         = _DurationMs;
                Epoch              = _Epoch;
                IncludesFreeLeaves = _IncludesFreeLeaves;
                Error              = _Error;
            }

            public List<NavLeafInfo> Leaves             { get; }
            public NavGraphSnapshot  Graph              { get; }
            public NavBoxSnapshot    Boxes              { get; }
            public float             DurationMs         { get; }
            public int               Epoch              { get; }
            public bool              IncludesFreeLeaves { get; }
            public Exception         Error              { get; }
        }

        #endregion

        #region Serialized fields

        [Header("Display modes")]
        //Master switch, off by default: on it costs frame time (the more leaves the world holds, the more), off it
        //costs nothing.
        [SerializeField, Tooltip(
                 "Master switch: while inactive the drawer neither snapshots nor draws — F3 in demo scenes, or set " +
                 "Active from code."
             )]
        bool m_Active;

        [SerializeField, Tooltip("Steady boundary shell: occupied cells drawn continuously while enabled.")]
        bool m_DrawSteadyShell = true;

        //OFF by default: a body's inside is a solid block of cells that would hide the crust drawn around it.
        [SerializeField, Tooltip(
                 "Interior: the cells enclosed by a closed obstacle — solid, never navigable — drawn in Interior Cell Color " +
                 "and taken out of the occupied shell, which then shows only the cells the geometry crosses."
             )]
        bool m_DrawInterior;

        [SerializeField, Tooltip(
                 "Interior X-ray: draw the interior cells through the scene geometry in front of them, so the solid insides show " +
                 "without hiding the obstacles' renderers."
             )]
        bool m_InteriorXRay = true;

        //OFF by default: the free population is the larger one in an open world, and a viewer who has not asked for
        //it wants the obstacle crust.
        [SerializeField, Tooltip(
                 "Free space: the navigable volume as the free boxes of the navigation graph — each an axis-aligned box " +
                 "clear of every obstacle, merged from the octree's free cells."
             )]
        bool m_DrawFreeCells;

        [SerializeField, Tooltip("Update pulse: the cells an octree operation actually rebuilt light up and decay.")]
        bool m_DrawUpdatePulse = true;

        [Header("Navigation graph")]
        //OFF by default: the edges outnumber the free cells several times over. Best used with ZONE scope, which also
        //limits what the snapshot copies.
        [SerializeField, Tooltip(
                 "Navigation graph: the connections between free boxes, each drawn from a box's centre through the centre " +
                 "of the face it shares with its neighbour."
             )]
        bool m_DrawGraph;

        [Header("Scope")]
        [SerializeField]
        OctreeDrawScope m_Scope = OctreeDrawScope.WHOLE_WORLD;

        [SerializeField, Tooltip("ZONE scope: draw cells within Zone radius of this transform.")]
        Transform m_ZoneCenter;

        [SerializeField]
        float m_ZoneRadius = 10f;

        [SerializeField, Tooltip("ZONE scope: additionally draw cells intersecting Zone bounds.")]
        bool m_UseZoneBounds;

        [SerializeField]
        Bounds m_ZoneBounds = new Bounds(Vector3.zero, Vector3.one * 20f);

        [Header("Style")]
        [SerializeField, Tooltip("Draw only occupied cells with at least one free-or-absent face neighbour — the boundary crust. Off = full volume including interiors.")]
        bool m_BoundaryOnly = true;

        [SerializeField]
        Color m_CellColor = new Color(0.9f, 0.3f, 0.25f, 0.08f);

        [SerializeField]
        Color m_FreeCellColor = new Color(0.35f, 0.85f, 0.6f, 0.06f);

        [SerializeField, Tooltip("Colour of the interior cells (Draw Interior); they are drawn in Occupied Cell Style.")]
        Color m_InteriorCellColor = new Color(0.55f, 0.55f, 0.55f, 0.12f);

        [SerializeField]
        float m_PulseSeconds = 1.2f;

        [SerializeField, ColorUsage(false), Tooltip("Colour the update pulse flashes in, for occupied and free cells alike; its peak alpha comes from Pulse Alpha Multiplier.")]
        Color m_PulseColor = new Color(0.9f, 0.3f, 0.25f);

        [SerializeField, Tooltip("How occupied cells are drawn: filled cubes, their edges as lines (at four times the cell alpha), or both.")]
        OctreeCellStyle m_OccupiedCellStyle = OctreeCellStyle.FILLED;

        [SerializeField, Tooltip("How free cells are drawn: filled cubes, their edges as lines (at four times the cell alpha), or both.")]
        OctreeCellStyle m_FreeCellStyle = OctreeCellStyle.FILLED;

        [SerializeField]
        Color m_GraphColor = new Color(0.55f, 0.95f, 1f, 0.35f);

        [SerializeField, Tooltip("Pulse peak alpha as a multiple of the steady alpha.")]
        float m_PulseAlphaMultiplier = 3.5f;

        [SerializeField, Tooltip("Fresnel edge-brightening power; 0 disables the term.")]
        float m_FresnelPower = 2f;

        [Header("Budgets")]
        [SerializeField, Tooltip("Most cells drawn per population and per style; nearer buckets are drawn first for the wireframe.")]
        int m_MaxInstances = 200000;

        [SerializeField, Tooltip("Most graph edges drawn; nearer buckets are drawn first.")]
        int m_MaxGraphEdges = 200000;

        [SerializeField, Tooltip("Skip cells outside the main camera frustum.")]
        bool m_FrustumCull = true;

        [SerializeField, Tooltip("Skip cells farther than this from the main camera; 0 = unlimited. Lines are skipped by bucket.")]
        float m_MaxDrawDistance;

        [SerializeField, Tooltip("Skip filled cells whose size/distance ratio to the main camera falls below this; 0 = off.")]
        float m_MinProjectedSize;

        [Header("Diagnostics")]
        [SerializeField, Tooltip("Log every snapshot's leaf counts and duration.")]
        bool m_LogSnapshotStats;

        //the advanced (debug) group — the custom inspector draws these in their own foldout, below everything else
        [SerializeField, Tooltip(
                 "Advanced (debug): draw the octree's own free leaves and the connections between them in place of the free " +
                 "boxes and theirs — the cells the boxes are merged from."
             )]
        bool m_DrawOctreeLeaves;

        [SerializeField, Tooltip(
                 "Octree leaves: draw only the free leaves with an occupied neighbour — the navigable skin over the geometry. " +
                 "Off = every free leaf in scope, which fills open space."
             )]
        bool m_DrawFreeCellsTouchingObstaclesOnly = true;

        [SerializeField, Tooltip(
                 "Octree leaves: a marker on every free leaf with a face open to empty space — where a route may leave the " +
                 "leaves for a straight line toward its goal. Prefer ZONE scope on large worlds."
             )]
        bool m_DrawEdgeNodes;

        [SerializeField]
        Color m_EdgeNodeColor = new Color(1f, 0.85f, 0.35f, 0.6f);

        #endregion

        #region Attributes

        static readonly int s_CellColorID    = Shader.PropertyToID("_CellColor");
        static readonly int s_PulseColorID   = Shader.PropertyToID("_PulseColor");
        static readonly int s_PulseAlphaID   = Shader.PropertyToID("_PulseAlpha");
        static readonly int s_PulseSecondsID = Shader.PropertyToID("_PulseSeconds");
        static readonly int s_PulseNowID     = Shader.PropertyToID("_PulseNow");
        static readonly int s_FresnelPowerID = Shader.PropertyToID("_FresnelPower");
        static readonly int s_ZTestID        = Shader.PropertyToID("_ZTest");
        static readonly int s_PulseDataID    = Shader.PropertyToID("_PulseData");

        ObstacleManager m_SubscribedManager;

        //bumped whenever the subscribed world changes; a worker snapshot carries the epoch it was started under and
        //an apply with a stale epoch is dropped, so a snapshot of a torn-down world can never populate the new one
        int m_WorldEpoch;

        bool m_SnapshotRequested;
        bool m_SnapshotInFlight;

        //a finished worker snapshot waiting for the main thread. Handed over here and not through ThreadDispatcher:
        //that queue drains in Update, which the paused editor never runs, so a snapshot ordered while paused would
        //sit in it until Play resumed — and the drawer is switched on in pause precisely to inspect a frozen world
        SnapshotDelivery m_PendingSnapshot;

        //the full-world copied-out leaf set of the last applied snapshot (replaced whole on apply, never mutated)
        List<NavLeafInfo> m_LeafData = new List<NavLeafInfo>();

        //volumes reported by OnRegionChanged while their snapshot is pending; they bound the leaf diff done on apply
        //(a leaf outside every reported volume cannot have changed) and are dropped once that diff has run
        readonly List<Bounds> m_PendingPulseVolumes = new List<Bounds>();

        //when each changed leaf started pulsing; entries expire after m_PulseSeconds
        readonly Dictionary<LeafKey, float> m_LeafPulseStarts = new Dictionary<LeafKey, float>();

        readonly HashSet<LeafKey> m_PreviousLeafKeys = new HashSet<LeafKey>();
        readonly List<LeafKey>    m_ExpiredPulseKeys = new List<LeafKey>();

        //whether the snapshot now held carries free leaves: when that flag changes, every free leaf reads as new, so
        //the next snapshot pulses nothing
        bool m_SnapshotHoldsFreeLeaves;

        //the leaf graph of the last applied snapshot; null when it was taken without one
        NavGraphSnapshot m_GraphData;

        //the box graph of the last applied snapshot — the free boxes and their portals; null when it was taken without one
        NavBoxSnapshot m_BoxData;

        //the whole-world graph over budget has been reported for this world
        bool m_GraphBudgetWarned;

        bool m_HasSnapshot;

        bool m_InstancesDirty;

        //the instance arrays the render loop walks — m_LeafData post scope/boundary filters, one set per kind
        readonly CellInstances m_OccupiedInstances = new CellInstances();
        readonly CellInstances m_FreeInstances     = new CellInstances();
        readonly CellInstances m_InteriorInstances = new CellInstances();
        readonly CellInstances m_EdgeNodeInstances = new CellInstances();

        readonly LineBuckets m_OccupiedWire = new LineBuckets();
        readonly LineBuckets m_FreeWire     = new LineBuckets();
        readonly LineBuckets m_InteriorWire = new LineBuckets();
        readonly LineBuckets m_GraphLines   = new LineBuckets();

        //the visible line buckets of one layer, nearest first; reused every frame
        readonly List<(float SqrDistance, LineBuckets.Bucket Bucket)> m_VisibleLineBuckets = new List<(float SqrDistance, LineBuckets.Bucket Bucket)>();

        static readonly Comparison<(float SqrDistance, LineBuckets.Bucket Bucket)> s_NearestFirst = (_A, _B) => _A.SqrDistance.CompareTo(_B.SqrDistance);

        Vector3 m_AppliedZoneCenter;

        //one cell and one line material per DrawLayer, indexed by it
        readonly Material[] m_CellMaterials = new Material[LAYER_COUNT];
        readonly Material[] m_LineMaterials = new Material[LAYER_COUNT];

        Mesh                  m_CubeMesh;
        Mesh                  m_MarkerMesh;
        MaterialPropertyBlock m_MPB;
        MaterialPropertyBlock m_LineMPB;
        bool                  m_ShaderMissingLogged;

#if UNITY_EDITOR

        //while the editor is paused the drawer ticks from EditorApplication.update; a tick that changed what is
        //drawn asks the views to repaint, since nothing else re-renders a paused Game view
        bool m_PausedTickHooked;
        bool m_RepaintWanted;

        //a pulse was fading on the previous paused tick: the views repaint until one tick after the last one expired
        bool m_PulsingWhilePaused;

#endif

        //whether m_BatchPulseData currently holds all zeros, so the quiet-world upload can be skipped
        bool m_BatchPulseDataZeroed;

        readonly Plane[]     m_FrustumPlanes  = new Plane[6];
        readonly Matrix4x4[] m_BatchMatrices  = new Matrix4x4[BATCH_CAPACITY];
        readonly Vector4[]   m_BatchPulseData = new Vector4[BATCH_CAPACITY];

        #endregion

        #region Properties

        public static Nav3DOctreeDrawer Instance => Singleton<Nav3DOctreeDrawer>.Instance;

        //The drawer if one already exists, without resolving or creating it. A caller that runs during teardown
        //must use this: resolving from an OnDestroy after the singleton has been destroyed creates a fresh
        //GameObject while the scene is closing, which Unity reports as "objects were not cleaned up".
        public static Nav3DOctreeDrawer InstanceIfResolved => Singleton<Nav3DOctreeDrawer>.InstanceIfResolved;

        //the master switch: an inactive drawer costs one boolean per frame. Activation orders a fresh
        //snapshot, so the picture is current the moment it appears.
        public bool Active
        {
            get => m_Active;
            set
            {
                if (m_Active == value)
                    return;

                m_Active = value;

                if (m_Active)
                    m_SnapshotRequested = true;
            }
        }

        public bool DrawSteadyShell
        {
            get => m_DrawSteadyShell;
            set => m_DrawSteadyShell = value;
        }

        public bool DrawInterior
        {
            get => m_DrawInterior;
            set
            {
                m_DrawInterior   = value;
                m_InstancesDirty = true;
            }
        }

        public bool InteriorXRay
        {
            get => m_InteriorXRay;
            set => m_InteriorXRay = value;
        }

        public bool DrawUpdatePulse
        {
            get => m_DrawUpdatePulse;
            set => m_DrawUpdatePulse = value;
        }

        //Turning free cells ON orders a fresh snapshot: the walk only copies them out when asked, so the cached
        //snapshot taken while this was off holds none of them.
        public bool DrawFreeCells
        {
            get => m_DrawFreeCells;
            set
            {
                if (m_DrawFreeCells == value)
                    return;

                m_DrawFreeCells = value;

                m_InstancesDirty    = true;
                m_SnapshotRequested = true;
            }
        }

        //Advanced (debug): the octree's own free leaves and their connections in place of the free boxes and theirs. The
        //two views come from different copies, so switching orders a snapshot.
        public bool DrawOctreeLeaves
        {
            get => m_DrawOctreeLeaves;
            set
            {
                if (m_DrawOctreeLeaves == value)
                    return;

                m_DrawOctreeLeaves  = value;
                m_InstancesDirty    = true;
                m_SnapshotRequested = true;
            }
        }

        //The graph and the edge-node markers come from the same snapshot pass; turning either on orders a snapshot
        //when the one held was taken without a graph.
        public bool DrawGraph
        {
            get => m_DrawGraph;
            set
            {
                m_DrawGraph      = value;
                m_InstancesDirty = true;
            }
        }

        public bool DrawEdgeNodes
        {
            get => m_DrawEdgeNodes;
            set
            {
                m_DrawEdgeNodes  = value;
                m_InstancesDirty = true;
            }
        }

        public OctreeCellStyle OccupiedCellStyle
        {
            get => m_OccupiedCellStyle;
            set
            {
                m_OccupiedCellStyle = value;
                m_InstancesDirty    = true;
            }
        }

        public OctreeCellStyle FreeCellStyle
        {
            get => m_FreeCellStyle;
            set
            {
                m_FreeCellStyle  = value;
                m_InstancesDirty = true;
            }
        }

        public Color GraphColor
        {
            get => m_GraphColor;
            set => m_GraphColor = value;
        }

        public Color EdgeNodeColor
        {
            get => m_EdgeNodeColor;
            set => m_EdgeNodeColor = value;
        }

        public int MaxGraphEdges
        {
            get => m_MaxGraphEdges;
            set => m_MaxGraphEdges = value;
        }

        public bool DrawFreeCellsTouchingObstaclesOnly
        {
            get => m_DrawFreeCellsTouchingObstaclesOnly;
            set
            {
                m_DrawFreeCellsTouchingObstaclesOnly = value;
                m_InstancesDirty                     = true;
            }
        }

        public Color CellColor
        {
            get => m_CellColor;
            set => m_CellColor = value;
        }

        public Color FreeCellColor
        {
            get => m_FreeCellColor;
            set => m_FreeCellColor = value;
        }

        public Color InteriorCellColor
        {
            get => m_InteriorCellColor;
            set => m_InteriorCellColor = value;
        }

        public Color PulseColor
        {
            get => m_PulseColor;
            set => m_PulseColor = value;
        }

        public bool BoundaryOnly
        {
            get => m_BoundaryOnly;
            set
            {
                m_BoundaryOnly   = value;
                m_InstancesDirty = true;
            }
        }

        public OctreeDrawScope Scope
        {
            get => m_Scope;
            set
            {
                m_Scope          = value;
                m_InstancesDirty = true;
            }
        }

        public Transform ZoneCenter
        {
            get => m_ZoneCenter;
            set
            {
                m_ZoneCenter     = value;
                m_InstancesDirty = true;
            }
        }

        public float ZoneRadius
        {
            get => m_ZoneRadius;
            set
            {
                m_ZoneRadius     = value;
                m_InstancesDirty = true;
            }
        }

        public bool UseZoneBounds
        {
            get => m_UseZoneBounds;
            set
            {
                m_UseZoneBounds  = value;
                m_InstancesDirty = true;
            }
        }

        public Bounds ZoneBounds
        {
            get => m_ZoneBounds;
            set
            {
                m_ZoneBounds     = value;
                m_InstancesDirty = true;
            }
        }

        //last applied snapshot's measurements (the design's measure-at-each-step numbers)
        public float LastSnapshotMs            { get; private set; }
        public int   LastSnapshotLeafCount     { get; private set; }
        public int   LastSnapshotBoundaryCount { get; private set; }
        public int   LastSnapshotFreeLeafCount { get; private set; }
        public int   LastSnapshotGraphEdgeCount { get; private set; }

        public int InstanceCount     => m_OccupiedInstances.Count + m_FreeInstances.Count + m_InteriorInstances.Count;
        public int FreeInstanceCount => m_FreeInstances.Count;
        public int GraphEdgeCount    => m_GraphLines.Count;

        //dev surface for the graph gate and the cost probe
        internal NavGraphSnapshot  GraphSnapshot       => m_GraphData;
        internal NavBoxSnapshot    BoxSnapshot         => m_BoxData;
        internal List<NavLeafInfo> LeafSnapshot        => m_LeafData;
        internal bool              SnapshotPending     => m_SnapshotRequested || m_SnapshotInFlight;
        internal int               GraphLineVertices   => m_GraphLines.VertexCount;
        internal int               WireframeCellCount  => m_OccupiedWire.Count + m_FreeWire.Count + m_InteriorWire.Count;
        internal float             LastDrawMs          { get; private set; }
        internal float             LastRebuildMs       { get; private set; }

        //The pulse's clock is real time, not game time: in the paused editor game time stands still, and a pulse stamped just
        //before the pause, or by a snapshot delivered during it, stayed lit for as long as the pause lasted.
        static float PulseNow => Time.realtimeSinceStartup;

        bool WantsLeafGraph  => m_DrawOctreeLeaves && (m_DrawGraph || m_DrawEdgeNodes);
        bool WantsFreeLeaves => m_DrawOctreeLeaves && m_DrawFreeCells;
        bool WantsBoxes      => !m_DrawOctreeLeaves && (m_DrawFreeCells || m_DrawGraph);

        #endregion

        #region Public methods

        /// <summary>
        /// Forces a fresh occupancy snapshot on the next frame (the drawer re-snapshots automatically on every
        /// octree change; this is for manual refresh only).
        /// </summary>
        public void RequestSnapshot()
        {
            m_SnapshotRequested = true;
        }

        #endregion

        #region Unity methods

        void OnEnable()
        {
            //the cells are submitted per camera render, not per LateUpdate: a paused editor still repaints its
            //views, and a submission made inside the camera's own pre-cull lands in that very render — which is what
            //lets the drawer be switched on in pause and show the world at once. Built-in pipeline and SRP hooks
            //both; only the one the project runs ever fires
            Camera.onPreCull                         += OnCameraPreCull;
            RenderPipelineManager.beginCameraRendering += OnBeginCameraRendering;

#if UNITY_EDITOR

            EditorApplication.pauseStateChanged += OnPauseStateChanged;

            if (EditorApplication.isPaused)
                HookPausedTick(true);

#endif

        }

        void LateUpdate()
        {
            Tick();
        }

        void OnValidate()
        {
            //inspector tweaks mid-play re-apply the snapshot-independent filters (scope, boundary-only, inset)
            m_InstancesDirty = true;
        }

        void OnDisable()
        {
            Camera.onPreCull                         -= OnCameraPreCull;
            RenderPipelineManager.beginCameraRendering -= OnBeginCameraRendering;

#if UNITY_EDITOR

            EditorApplication.pauseStateChanged -= OnPauseStateChanged;

            HookPausedTick(false);

#endif

            DropWorldSubscription();
        }

        void OnDestroy()
        {
            DropWorldSubscription();

            foreach (Material material in m_CellMaterials)
            {
                if (material != null)
                    Destroy(material);
            }

            foreach (Material material in m_LineMaterials)
            {
                if (material != null)
                    Destroy(material);
            }

            if (m_CubeMesh != null)
                Destroy(m_CubeMesh);

            if (m_MarkerMesh != null)
                Destroy(m_MarkerMesh);

            m_OccupiedWire.Destroy();
            m_FreeWire.Destroy();
            m_InteriorWire.Destroy();
            m_GraphLines.Destroy();

            Singleton<Nav3DOctreeDrawer>.ClearInstance(this);
        }

        #endregion

        #region Service methods

        //One step of the drawer's own work — the world subscription, the snapshot hand-off, the instance rebuild.
        //Runs from LateUpdate while playing and from the editor's update loop while paused; drawing is not here
        void Tick()
        {
            SyncWorldSubscription();

            DeliverPendingSnapshot();

            //dormant: keep the world subscription current (activation must find a live manager), do
            //no other work — the request flag latches, so flipping Active on snapshots immediately
            if (!m_Active)
                return;

            if (m_SnapshotRequested && !m_SnapshotInFlight && m_SubscribedManager != null)
                StartSnapshot();

            CheckZoneMovement();

            CheckFreeCellCoverage();

            CheckGraphCoverage();

            PruneExpiredPulses();

            if (m_InstancesDirty)
                RebuildInstances();
        }

        void OnCameraPreCull(Camera _Camera)
        {
            DrawForCamera(_Camera);
        }

        void OnBeginCameraRendering(ScriptableRenderContext _Context, Camera _Camera)
        {
            DrawForCamera(_Camera);
        }

        void DrawForCamera(Camera _Camera)
        {
            if (!m_Active || !isActiveAndEnabled)
                return;

            //the views a person looks at; not reflection probes, previews or thumbnail captures
            if (_Camera.cameraType != CameraType.Game && _Camera.cameraType != CameraType.SceneView)
                return;

            System.Diagnostics.Stopwatch watch = System.Diagnostics.Stopwatch.StartNew();

            Draw(_Camera);

            LastDrawMs = (float) watch.Elapsed.TotalMilliseconds;
        }

#if UNITY_EDITOR

        void OnPauseStateChanged(PauseState _State)
        {
            HookPausedTick(_State == PauseState.Paused);
        }

        void HookPausedTick(bool _Hook)
        {
            if (m_PausedTickHooked == _Hook)
                return;

            m_PausedTickHooked = _Hook;

            if (_Hook)
                EditorApplication.update += PausedTick;
            else
                EditorApplication.update -= PausedTick;
        }

        //the paused editor runs no player loop, so the drawer's step runs from the editor's own loop instead, and a
        //step that changed the picture repaints the views — a paused Game view re-renders only when asked
        void PausedTick()
        {
            if (!Application.isPlaying || this == null)
                return;

            m_RepaintWanted = false;

            Tick();

            //a paused view re-renders only when asked, so a fading pulse would stop mid-fade on screen
            bool pulsing = m_Active && m_DrawUpdatePulse && m_LeafPulseStarts.Count > 0;

            if (pulsing || m_PulsingWhilePaused)
                m_RepaintWanted = true;

            m_PulsingWhilePaused = pulsing;

            if (m_RepaintWanted)
                UnityEditorInternal.InternalEditorUtility.RepaintAllViews();
        }

#endif

        //Tracks the live world: subscribes to the current ObstacleManager's region-changed signal and requests the
        //initial snapshot; drops everything (and invalidates in-flight snapshots via the epoch) when the world goes
        //away or is replaced by a scene switch.
        void SyncWorldSubscription()
        {
            ObstacleManager manager = Nav3DManager.Inited && !ObstacleManager.Doomed ? ObstacleManager.InstanceIfResolved : null;

            //InstanceIfResolved may hand back a destroyed Unity object; normalize it to a real null before the
            //reference-identity check below
            if (manager == null)
                manager = null;

            if (ReferenceEquals(manager, m_SubscribedManager))
                return;

            DropWorldSubscription();

            m_SubscribedManager = manager;

            if (m_SubscribedManager != null)
            {
                m_SubscribedManager.OnRegionChanged += OnRegionChanged;

                m_SnapshotRequested = true;
            }
        }

        void DropWorldSubscription()
        {
            if (m_SubscribedManager != null)
                m_SubscribedManager.OnRegionChanged -= OnRegionChanged;

            m_SubscribedManager = null;

            m_WorldEpoch++;

            m_SnapshotRequested = false;
            m_SnapshotInFlight  = false;

            m_LeafData.Clear();
            m_GraphData         = null;
            m_GraphBudgetWarned = false;
            m_PendingPulseVolumes.Clear();
            m_LeafPulseStarts.Clear();

            m_HasSnapshot = false;

            m_InstancesDirty = true;
        }

        //Main-thread (the octree dispatches post-publish notifications through ThreadDispatcher).
        void OnRegionChanged(IReadOnlyList<Bounds> _ChangedVolumes)
        {
            m_SnapshotRequested = true;

            //a dormant drawer must not bank pulse volumes: in a streamed world the operations never
            //stop, and the pending list would grow for as long as nobody presses F3
            if (!m_Active || !m_DrawUpdatePulse)
                return;

            foreach (Bounds volume in _ChangedVolumes)
                m_PendingPulseVolumes.Add(volume);
        }

        void StartSnapshot()
        {
            m_SnapshotRequested = false;
            m_SnapshotInFlight  = true;

            //captured on the main thread; the worker below only calls pure C# on it — a worker must never resolve
            //the manager itself
            ObstacleManager  manager     = m_SubscribedManager;
            int              epoch       = m_WorldEpoch;
            bool             includeFree = WantsFreeLeaves;
            NavGraphSnapshot graph       = WantsLeafGraph ? new NavGraphSnapshot(DesiredGraphScope()) : null;
            NavBoxSnapshot   boxes       = WantsBoxes ? new NavBoxSnapshot(null) : null;

            Task.Run(() =>
                    {
                        SnapshotDelivery delivery;

                        try
                        {
                            List<NavLeafInfo> leaves = new List<NavLeafInfo>();

                            System.Diagnostics.Stopwatch watch = System.Diagnostics.Stopwatch.StartNew();

                            manager.SnapshotLeaves(leaves, includeFree, graph);

                            if (boxes != null)
                                manager.SnapshotBoxes(boxes);

                            watch.Stop();

                            delivery = new SnapshotDelivery(leaves, graph, boxes, (float) watch.Elapsed.TotalMilliseconds, epoch, includeFree, null);
                        }
                        catch (Exception exception)
                        {
                            delivery = new SnapshotDelivery(null, null, null, 0f, epoch, includeFree, exception);
                        }

                        //the main thread's Tick picks it up; a delivery nobody collected before the next one is
                        //superseded by it, which is the same result the dispatcher's queue would have reached
                        Interlocked.Exchange(ref m_PendingSnapshot, delivery);
                    }
                );
        }

        //Main thread. Applies the snapshot a worker finished since the last tick, if any.
        void DeliverPendingSnapshot()
        {
            SnapshotDelivery delivery = Interlocked.Exchange(ref m_PendingSnapshot, null);

            if (delivery == null)
                return;

            if (delivery.Error != null)
            {
                if (delivery.Epoch == m_WorldEpoch)
                    m_SnapshotInFlight = false;

                UnityEngine.Debug.LogError($"{nameof(Nav3DOctreeDrawer)}: leaf snapshot failed: {delivery.Error}");

                return;
            }

            ApplySnapshot(delivery.Leaves, delivery.Graph, delivery.Boxes, delivery.DurationMs, delivery.Epoch, delivery.IncludesFreeLeaves);
        }

        void ApplySnapshot(List<NavLeafInfo> _Leaves, NavGraphSnapshot _Graph, NavBoxSnapshot _Boxes, float _DurationMs, int _Epoch, bool _IncludesFreeLeaves)
        {
            //a snapshot of a world that has since been torn down or replaced
            if (_Epoch != m_WorldEpoch)
                return;

            m_SnapshotInFlight = false;

            //the boxes first: the leaves' diff consumes the reported volumes both are bounded by
            MarkChangedBoxes(_Boxes);
            MarkChangedLeaves(_Leaves, _IncludesFreeLeaves);

            m_LeafData                = _Leaves;
            m_GraphData               = _Graph;
            m_BoxData                 = _Boxes;
            m_SnapshotHoldsFreeLeaves = _IncludesFreeLeaves;
            m_HasSnapshot             = true;

            int boundaryCount = 0;
            int freeCount     = 0;

            foreach (NavLeafInfo leaf in _Leaves)
            {
                if (!leaf.Occupied)
                {
                    freeCount++;

                    continue;
                }

                if (leaf.IsBoundary)
                    boundaryCount++;
            }

            LastSnapshotMs            = _DurationMs;
            LastSnapshotLeafCount     = _Leaves.Count - freeCount;
            LastSnapshotBoundaryCount = boundaryCount;
            LastSnapshotFreeLeafCount = freeCount;
            LastSnapshotGraphEdgeCount = _Graph?.Edges.Count ?? _Boxes?.Portals.Count ?? 0;

            m_InstancesDirty = true;

            if (_Graph != null && _Graph.Scope == null && _Graph.Edges.Count > m_MaxGraphEdges && !m_GraphBudgetWarned)
            {
                m_GraphBudgetWarned = true;

                UnityEngine.Debug.LogWarning(
                        $"{nameof(Nav3DOctreeDrawer)}: the navigation graph holds {_Graph.Edges.Count} edges, more than " +
                        $"{nameof(MaxGraphEdges)} ({m_MaxGraphEdges}), and the whole world is copied out again after every obstacle " +
                        $"change. Set {nameof(Scope)} to {nameof(OctreeDrawScope.ZONE)}: only the octree roots around the zone are copied then."
                    );
            }

            if (m_LogSnapshotStats)
                UnityEngine.Debug.Log($"{nameof(Nav3DOctreeDrawer)}: snapshot {_Leaves.Count - freeCount} occupied leaves " +
                                      $"({boundaryCount} boundary) + {freeCount} free + {_Boxes?.Boxes.Count ?? 0} boxes + {LastSnapshotGraphEdgeCount} graph edges in {_DurationMs:F2} ms.");
        }

        //A moving zone anchor re-applies the zone filter from the cached snapshot — no tree walk involved.
        void CheckZoneMovement()
        {
            if (m_Scope != OctreeDrawScope.ZONE || m_ZoneCenter == null)
                return;

            float threshold = Mathf.Max(m_ZoneRadius * 0.1f, 0.01f);

            if ((m_ZoneCenter.position - m_AppliedZoneCenter).sqrMagnitude > threshold * threshold)
                m_InstancesDirty = true;
        }

        //The held snapshot must carry free leaves while they are to be drawn. Checked against the state rather than
        //ordered by the setter alone: the inspector writes the serialized switch without passing through the property.
        void CheckFreeCellCoverage()
        {
            if (!m_HasSnapshot || m_SnapshotRequested || m_SnapshotInFlight || m_SubscribedManager == null)
                return;

            if ((WantsFreeLeaves && !m_SnapshotHoldsFreeLeaves) || (WantsBoxes && m_BoxData == null))
                m_SnapshotRequested = true;
        }

        //The held graph must cover what is to be drawn: taken at all, taken whole for WHOLE_WORLD, and for ZONE taken
        //around a region that still contains the zone.
        void CheckGraphCoverage()
        {
            if (!WantsLeafGraph || m_SnapshotRequested || m_SnapshotInFlight || m_SubscribedManager == null)
                return;

            if (m_GraphData == null)
            {
                m_SnapshotRequested = true;

                return;
            }

            if (m_GraphData.Scope == null)
                return;

            if (m_Scope == OctreeDrawScope.WHOLE_WORLD)
            {
                m_SnapshotRequested = true;

                return;
            }

            Bounds held = m_GraphData.Scope.Value;
            Bounds zone = ZoneExtent();

            if (!held.Contains(zone.min) || !held.Contains(zone.max))
                m_SnapshotRequested = true;
        }

        //null = the whole world
        Bounds? DesiredGraphScope()
        {
            if (m_Scope == OctreeDrawScope.WHOLE_WORLD)
                return null;

            Bounds zone = ZoneExtent();

            zone.Expand(zone.size * GRAPH_SCOPE_MARGIN * 2f);

            return zone;
        }

        //the box around everything PassesZone can accept; an unconfigured zone is an empty box
        Bounds ZoneExtent()
        {
            bool   any    = false;
            Bounds extent = new Bounds();

            if (m_ZoneCenter != null)
            {
                extent = new Bounds(m_ZoneCenter.position, Vector3.one * (m_ZoneRadius * 2f));
                any    = true;
            }

            if (m_UseZoneBounds)
            {
                if (any)
                    extent.Encapsulate(m_ZoneBounds);
                else
                    extent = m_ZoneBounds;
            }

            return extent;
        }

        //Pulses the leaves this snapshot actually gained, not the volumes the operation reported. A reported volume is
        //the region the octree re-examined; what it rebuilt inside it can be a handful of leaves, so lighting the
        //volume would show a block rebuild where the engine did surgery. The new snapshot is therefore diffed against
        //the one it replaces, inside the reported volumes only, and a leaf whose key the previous snapshot did not
        //hold is what lights up.
        void MarkChangedLeaves(List<NavLeafInfo> _Leaves, bool _IncludesFreeLeaves)
        {
            //no baseline to diff against, or a snapshot whose leaf population changed for a reason other than an
            //operation (the free-cell switch): this one pulses nothing
            bool diffable = m_HasSnapshot && _IncludesFreeLeaves == m_SnapshotHoldsFreeLeaves;

            if (m_PendingPulseVolumes.Count > 0 && diffable && m_DrawUpdatePulse)
            {
                m_PreviousLeafKeys.Clear();

                foreach (NavLeafInfo leaf in m_LeafData)
                {
                    if (IntersectsPendingVolume(leaf.Bounds))
                        m_PreviousLeafKeys.Add(new LeafKey(leaf.Bounds, leaf.Occupied));
                }

                float now = PulseNow;

                foreach (NavLeafInfo leaf in _Leaves)
                {
                    if (!IntersectsPendingVolume(leaf.Bounds))
                        continue;

                    LeafKey key = new LeafKey(leaf.Bounds, leaf.Occupied);

                    if (m_PreviousLeafKeys.Contains(key))
                        continue;

                    m_LeafPulseStarts[key] = now;
                }

                m_PreviousLeafKeys.Clear();
            }

            m_PendingPulseVolumes.Clear();
        }

        //The same diff over the boxes: a box the new snapshot holds and the previous one did not, inside a reported volume,
        //is where the operation changed the free space. Called before MarkChangedLeaves, which drops the volumes.
        void MarkChangedBoxes(NavBoxSnapshot _Boxes)
        {
            if (m_PendingPulseVolumes.Count == 0 || m_BoxData == null || _Boxes == null || !m_DrawUpdatePulse)
                return;

            m_PreviousLeafKeys.Clear();

            foreach (Bounds box in m_BoxData.Boxes)
            {
                if (IntersectsPendingVolume(box))
                    m_PreviousLeafKeys.Add(new LeafKey(box, false));
            }

            float now = PulseNow;

            foreach (Bounds box in _Boxes.Boxes)
            {
                if (!IntersectsPendingVolume(box))
                    continue;

                LeafKey key = new LeafKey(box, false);

                if (!m_PreviousLeafKeys.Contains(key))
                    m_LeafPulseStarts[key] = now;
            }

            m_PreviousLeafKeys.Clear();
        }

        bool IntersectsPendingVolume(Bounds _LeafBounds)
        {
            foreach (Bounds volume in m_PendingPulseVolumes)
            {
                if (volume.Intersects(_LeafBounds))
                    return true;
            }

            return false;
        }

        void PruneExpiredPulses()
        {
            if (m_LeafPulseStarts.Count == 0)
                return;

            float now = PulseNow;

            foreach (KeyValuePair<LeafKey, float> pulse in m_LeafPulseStarts)
            {
                if (now - pulse.Value > m_PulseSeconds)
                    m_ExpiredPulseKeys.Add(pulse.Key);
            }

            foreach (LeafKey key in m_ExpiredPulseKeys)
                m_LeafPulseStarts.Remove(key);

            m_ExpiredPulseKeys.Clear();
        }

        void RebuildInstances()
        {
            m_InstancesDirty = false;

            System.Diagnostics.Stopwatch watch = System.Diagnostics.Stopwatch.StartNew();

            m_OccupiedInstances.Clear();
            m_FreeInstances.Clear();
            m_InteriorInstances.Clear();
            m_EdgeNodeInstances.Clear();

            m_OccupiedWire.Clear();
            m_FreeWire.Clear();
            m_InteriorWire.Clear();
            m_GraphLines.Clear();

            m_AppliedZoneCenter = m_ZoneCenter != null ? m_ZoneCenter.position : Vector3.zero;

            foreach (NavLeafInfo leaf in m_LeafData)
            {
                CellInstances   filled;
                LineBuckets     wire;
                OctreeCellStyle style;

                if (leaf.Interior && m_DrawInterior)
                {
                    //the whole inside, whatever Boundary Only says: the crust filter is about the occupied shell
                    filled = m_InteriorInstances;
                    wire   = m_InteriorWire;
                    style  = m_OccupiedCellStyle;
                }
                else if (leaf.Occupied)
                {
                    if (m_BoundaryOnly && !leaf.IsBoundary)
                        continue;

                    filled = m_OccupiedInstances;
                    wire   = m_OccupiedWire;
                    style  = m_OccupiedCellStyle;
                }
                else
                {
                    //a snapshot taken while free leaves were wanted survives the switch being turned off: filter here too
                    if (!WantsFreeLeaves)
                        continue;

                    if (m_DrawFreeCellsTouchingObstaclesOnly && !leaf.TouchesOccupied)
                        continue;

                    filled = m_FreeInstances;
                    wire   = m_FreeWire;
                    style  = m_FreeCellStyle;
                }

                Bounds bounds = leaf.Bounds;

                if (!PassesZone(bounds))
                    continue;

                AddCell(filled, wire, style, bounds, FindPulseStart(leaf.Bounds, leaf.Occupied));
            }

            if (!m_DrawOctreeLeaves && m_BoxData != null)
            {
                if (m_DrawFreeCells)
                {
                    foreach (Bounds box in m_BoxData.Boxes)
                    {
                        if (PassesZone(box))
                            AddCell(m_FreeInstances, m_FreeWire, m_FreeCellStyle, box, FindPulseStart(box, false));
                    }
                }

                if (m_DrawGraph)
                {
                    foreach (NavGraphEdge portal in m_BoxData.Portals)
                    {
                        Bounds bounds = new Bounds(portal.From, Vector3.zero);

                        bounds.Encapsulate(portal.To);

                        if (PassesZone(bounds))
                            m_GraphLines.AddEdge(portal.From, portal.Contact, portal.To);
                    }
                }
            }

            //the leaf graph of a snapshot taken while it was wanted survives the switch being turned off: filter here too
            if (m_GraphData != null && m_DrawOctreeLeaves)
            {
                if (m_DrawGraph)
                {
                    foreach (NavGraphEdge edge in m_GraphData.Edges)
                    {
                        Bounds bounds = new Bounds(edge.From, Vector3.zero);

                        bounds.Encapsulate(edge.To);

                        if (PassesZone(bounds))
                            m_GraphLines.AddEdge(edge.From, edge.Contact, edge.To);
                    }
                }

                if (m_DrawEdgeNodes)
                {
                    foreach (Bounds node in m_GraphData.EdgeNodes)
                    {
                        if (PassesZone(node))
                            m_EdgeNodeInstances.Add(Matrix4x4.TRS(node.center, Quaternion.identity, node.size * EDGE_NODE_MARKER_SCALE), node, float.NegativeInfinity);
                    }
                }
            }

            m_OccupiedWire.Upload();
            m_FreeWire.Upload();
            m_InteriorWire.Upload();
            m_GraphLines.Upload();

            LastRebuildMs = (float) watch.Elapsed.TotalMilliseconds;

#if UNITY_EDITOR

            m_RepaintWanted = true;

#endif

        }

        bool PassesZone(Bounds _LeafBounds)
        {
            if (m_Scope == OctreeDrawScope.WHOLE_WORLD)
                return true;

            if (m_ZoneCenter != null && _LeafBounds.SqrDistance(m_AppliedZoneCenter) <= m_ZoneRadius * m_ZoneRadius)
                return true;

            if (m_UseZoneBounds && m_ZoneBounds.Intersects(_LeafBounds))
                return true;

            //ZONE scope with nothing configured draws nothing — an empty zone, not a silent fallback to everything
            return false;
        }

        float FindPulseStart(Bounds _Bounds, bool _Occupied)
        {
            return m_LeafPulseStarts.TryGetValue(new LeafKey(_Bounds, _Occupied), out float start) ? start : float.NegativeInfinity;
        }

        static void AddCell(CellInstances _Filled, LineBuckets _Wire, OctreeCellStyle _Style, Bounds _Bounds, float _PulseStart)
        {
            if (_Style != OctreeCellStyle.WIREFRAME)
                _Filled.Add(Matrix4x4.TRS(_Bounds.center, Quaternion.identity, _Bounds.size * CUBE_INSET), _Bounds, _PulseStart);

            if (_Style != OctreeCellStyle.FILLED)
                _Wire.AddWireCell(_Bounds, float.IsNegativeInfinity(_PulseStart) ? Vector2.zero : new Vector2(_PulseStart, 1f));
        }

        void Draw(Camera _Camera)
        {
            bool drawCells = m_DrawSteadyShell || m_DrawUpdatePulse;

            if (!drawCells && !m_DrawGraph && !m_DrawEdgeNodes)
                return;

            if (EnsureMaterial(DrawLayer.OCCUPIED, false) == null)
                return;

            float interiorZTest = (float) (m_InteriorXRay ? CompareFunction.Always : CompareFunction.LessEqual);

            EnsureMaterial(DrawLayer.INTERIOR, false).SetFloat(s_ZTestID, interiorZTest);
            EnsureMaterial(DrawLayer.INTERIOR, true).SetFloat(s_ZTestID, interiorZTest);

            if (m_CubeMesh == null)
                m_CubeMesh = BuildUnitCube();

            bool cull = m_FrustumCull;

            if (cull)
                GeometryUtility.CalculateFrustumPlanes(_Camera, m_FrustumPlanes);

            //nothing is pulsing (the steady state of a quiet world): the pulse data is a constant zero
            bool anyPulse = m_DrawUpdatePulse && m_LeafPulseStarts.Count > 0;

            if (drawCells)
            {
                if (m_DrawInterior)
                    DrawSet(_Camera, EnsureMaterial(DrawLayer.INTERIOR, false), m_InteriorInstances, m_InteriorCellColor, m_CubeMesh, true, cull, anyPulse);

                DrawSet(_Camera, EnsureMaterial(DrawLayer.OCCUPIED, false), m_OccupiedInstances, m_CellColor, m_CubeMesh, true, cull, anyPulse);

                if (m_DrawFreeCells)
                    DrawSet(_Camera, EnsureMaterial(DrawLayer.FREE, false), m_FreeInstances, m_FreeCellColor, m_CubeMesh, true, cull, anyPulse);
            }

            if (m_DrawEdgeNodes && m_EdgeNodeInstances.Count > 0)
            {
                if (m_MarkerMesh == null)
                    m_MarkerMesh = BuildOctahedron();

                DrawSet(_Camera, EnsureMaterial(DrawLayer.EDGE_NODES, false), m_EdgeNodeInstances, m_EdgeNodeColor, m_MarkerMesh, false, cull, false);
            }

            bool drawWire  = drawCells && (m_DrawSteadyShell || anyPulse) && m_OccupiedWire.Count + m_FreeWire.Count + m_InteriorWire.Count > 0;
            bool drawGraph = m_DrawGraph && m_GraphLines.Count > 0;

            if (!drawWire && !drawGraph)
                return;

            if (drawWire)
            {
                if (m_DrawInterior)
                    DrawLines(_Camera, EnsureMaterial(DrawLayer.INTERIOR, true), m_InteriorWire, m_InteriorCellColor, true, m_MaxInstances, cull);

                DrawLines(_Camera, EnsureMaterial(DrawLayer.OCCUPIED, true), m_OccupiedWire, m_CellColor, true, m_MaxInstances, cull);

                if (m_DrawFreeCells)
                    DrawLines(_Camera, EnsureMaterial(DrawLayer.FREE, true), m_FreeWire, m_FreeCellColor, true, m_MaxInstances, cull);
            }

            if (drawGraph)
                DrawLines(_Camera, EnsureMaterial(DrawLayer.GRAPH, true), m_GraphLines, m_GraphColor, false, m_MaxGraphEdges, cull);
        }

        //One population, one colour. The instance budget, the frustum cull and the distance filters apply per set,
        //so a crowded crust can never leave the free cells undrawn (or the other way round). _FollowsShell: the cells
        //obey the steady-shell switch; the edge-node markers do not.
        void DrawSet(Camera _Camera, Material _Material, CellInstances _Instances, Color _Color, Mesh _Mesh, bool _FollowsShell, bool _Cull, bool _AnyPulse)
        {
            if (_Instances.Count == 0)
                return;

            m_MPB ??= new MaterialPropertyBlock();

            Color cellColor = _Color;

            //shell off: steady alpha 0, so pulsed cells decay to nothing
            bool steady = !_FollowsShell || m_DrawSteadyShell;

            if (!steady)
                cellColor.a = 0f;

            m_MPB.SetColor(s_CellColorID, cellColor);
            m_MPB.SetColor(s_PulseColorID, m_PulseColor);
            m_MPB.SetFloat(s_PulseAlphaID, _Color.a * m_PulseAlphaMultiplier);
            m_MPB.SetFloat(s_PulseSecondsID, m_PulseSeconds);
            m_MPB.SetFloat(s_PulseNowID, PulseNow);
            m_MPB.SetFloat(s_FresnelPowerID, m_FresnelPower);

            Camera renderCamera = _Camera;

            Vector3 cameraPosition = renderCamera.transform.position;

            float now = PulseNow;

            //the per-instance pulse data is a constant zero while nothing pulses, so it is zeroed and uploaded once here
            //instead of 1023 vectors per batch
            if (!_AnyPulse)
            {
                if (!m_BatchPulseDataZeroed)
                {
                    Array.Clear(m_BatchPulseData, 0, m_BatchPulseData.Length);

                    m_BatchPulseDataZeroed = true;
                }

                m_MPB.SetVectorArray(s_PulseDataID, m_BatchPulseData);
            }
            else
                m_BatchPulseDataZeroed = false;

            int batchCount = 0;
            int drawnCount = 0;

            foreach (CellInstances.Bucket bucket in _Instances.Buckets)
            {
                if (bucket.Count == 0)
                    continue;

                //one test for every cell the bucket holds
                if (_Cull && !GeometryUtility.TestPlanesAABB(m_FrustumPlanes, bucket.Bounds))
                    continue;

                List<Matrix4x4> matrices    = bucket.Matrices;
                List<Bounds>    cellBounds  = bucket.CellBounds;
                List<float>     pulseStarts = bucket.PulseStarts;

                for (int i = 0; i < matrices.Count; i++)
                {
                    if (drawnCount >= m_MaxInstances)
                        break;

                    float pulseStart  = pulseStarts[i];
                    bool  pulseActive = _AnyPulse && now - pulseStart <= m_PulseSeconds;

                    if (!steady && !pulseActive)
                        continue;

                    Bounds bounds = cellBounds[i];

                    if (_Cull && !GeometryUtility.TestPlanesAABB(m_FrustumPlanes, bounds))
                        continue;

                    if (m_MaxDrawDistance > 0f || m_MinProjectedSize > 0f)
                    {
                        float distance = Vector3.Distance(bounds.center, cameraPosition);

                        if (m_MaxDrawDistance > 0f && distance > m_MaxDrawDistance)
                            continue;

                        if (m_MinProjectedSize > 0f && distance > 0.001f && bounds.size.x / distance < m_MinProjectedSize)
                            continue;
                    }

                    m_BatchMatrices[batchCount] = matrices[i];

                    if (_AnyPulse)
                        m_BatchPulseData[batchCount] = new Vector4(pulseActive ? pulseStart : 0f, pulseActive ? 1f : 0f, 0f, 0f);

                    batchCount++;
                    drawnCount++;

                    if (batchCount == BATCH_CAPACITY)
                    {
                        FlushBatch(_Camera, _Material, _Mesh, batchCount, _AnyPulse);

                        batchCount = 0;
                    }
                }

                if (drawnCount >= m_MaxInstances)
                    break;
            }

            if (batchCount > 0)
                FlushBatch(_Camera, _Material, _Mesh, batchCount, _AnyPulse);
        }

        //One line layer: the buckets in view, nearest first, until the layer's budget is spent. A line bucket is drawn
        //whole or not at all, so the distance cutoff and the budget work per bucket. The wire cells carry their pulse
        //in the vertices; graph edges do not pulse.
        void DrawLines(Camera _Camera, Material _Material, LineBuckets _Lines, Color _Color, bool _FollowsShell, int _Budget, bool _Cull)
        {
            m_LineMPB ??= new MaterialPropertyBlock();

            Color color = _Color;

            if (_FollowsShell)
                color.a = Mathf.Min(1f, color.a * WIREFRAME_ALPHA_SCALE);

            float pulseAlpha = Mathf.Min(1f, color.a * m_PulseAlphaMultiplier);

            if (_FollowsShell && !m_DrawSteadyShell)
                color.a = 0f;

            m_LineMPB.SetColor(s_CellColorID, color);
            m_LineMPB.SetColor(s_PulseColorID, m_PulseColor);
            m_LineMPB.SetFloat(s_PulseAlphaID, pulseAlpha);
            m_LineMPB.SetFloat(s_PulseSecondsID, m_PulseSeconds);
            m_LineMPB.SetFloat(s_PulseNowID, PulseNow);

            Vector3 cameraPosition = _Camera.transform.position;
            float   maxSqrDistance = m_MaxDrawDistance > 0f ? m_MaxDrawDistance * m_MaxDrawDistance : float.PositiveInfinity;

            m_VisibleLineBuckets.Clear();

            foreach (LineBuckets.Bucket bucket in _Lines.Buckets)
            {
                if (bucket.ItemCount == 0)
                    continue;

                if (_Cull && !GeometryUtility.TestPlanesAABB(m_FrustumPlanes, bucket.Bounds))
                    continue;

                float sqrDistance = bucket.Bounds.SqrDistance(cameraPosition);

                if (sqrDistance > maxSqrDistance)
                    continue;

                m_VisibleLineBuckets.Add((sqrDistance, bucket));
            }

            m_VisibleLineBuckets.Sort(s_NearestFirst);

            int drawn = 0;

            foreach ((float _, LineBuckets.Bucket bucket) in m_VisibleLineBuckets)
            {
                if (drawn + bucket.ItemCount > _Budget)
                    break;

                drawn += bucket.ItemCount;

                Graphics.DrawMesh(bucket.Mesh, Matrix4x4.identity, _Material, 0, _Camera, 0, m_LineMPB, ShadowCastingMode.Off, false);
            }
        }

        void FlushBatch(Camera _Camera, Material _Material, Mesh _Mesh, int _Count, bool _UploadPulseData)
        {
            if (_UploadPulseData)
                m_MPB.SetVectorArray(s_PulseDataID, m_BatchPulseData);

            //DrawMeshInstanced copies the property block's contents at call time, so the arrays are safe to rewrite
            //for the next batch
            Graphics.DrawMeshInstanced(_Mesh, 0, _Material, m_BatchMatrices, _Count, m_MPB, ShadowCastingMode.Off, false, 0, _Camera);
        }

        //The cells and the lines share one shader; the lines are its keyword variant (vertex pulse, no fresnel).
        Material EnsureMaterial(DrawLayer _Layer, bool _Lines)
        {
            Material[] materials = _Lines ? m_LineMaterials : m_CellMaterials;

            if (materials[(int) _Layer] != null)
                return materials[(int) _Layer];

            Shader shader = Resources.Load<Shader>(SHADER_RESOURCE);

            if (shader == null)
            {
                if (!m_ShaderMissingLogged)
                {
                    m_ShaderMissingLogged = true;

                    UnityEngine.Debug.LogError($"{nameof(Nav3DOctreeDrawer)}: shader resource '{SHADER_RESOURCE}' is missing.");
                }

                return null;
            }

            Material material = new Material(shader)
            {
                enableInstancing = !_Lines,
                hideFlags        = HideFlags.HideAndDontSave,
                renderQueue      = (int) RenderQueue.Transparent + (int) _Layer
            };

            if (_Lines)
                material.EnableKeyword(LINES_KEYWORD);

            materials[(int) _Layer] = material;

            return material;
        }

        //edge-node marker: an octahedron with flat face normals, so the fresnel term outlines it like a cube
        static Mesh BuildOctahedron()
        {
            Vector3[] tips  = { Vector3.right, Vector3.forward, Vector3.left, Vector3.back };
            Vector3[] poles = { Vector3.up, Vector3.down };

            Vector3[] vertices  = new Vector3[24];
            Vector3[] normals   = new Vector3[24];
            int[]     triangles = new int[24];

            int vertex = 0;

            for (int side = 0; side < 4; side++)
            {
                foreach (Vector3 poleDirection in poles)
                {
                    Vector3 pole   = poleDirection * 0.5f;
                    Vector3 first  = tips[side] * 0.5f;
                    Vector3 second = tips[(side + 1) % 4] * 0.5f;

                    //front faces: cross(v1 - v0, v2 - v0) points outward, the convention BuildUnitCube's winding follows
                    if (Vector3.Dot(Vector3.Cross(first - pole, second - pole), pole + first + second) < 0f)
                        (first, second) = (second, first);

                    Vector3 normal = Vector3.Cross(first - pole, second - pole).normalized;

                    vertices[vertex]     = pole;
                    vertices[vertex + 1] = first;
                    vertices[vertex + 2] = second;

                    for (int i = 0; i < 3; i++)
                    {
                        normals[vertex + i]   = normal;
                        triangles[vertex + i] = vertex + i;
                    }

                    vertex += 3;
                }
            }

            return new Mesh
            {
                name      = "Nav3DOctreeOccupancyMarker",
                vertices  = vertices,
                normals   = normals,
                triangles = triangles,
                hideFlags = HideFlags.HideAndDontSave
            };
        }

        static Mesh BuildUnitCube()
        {
            Vector3[] faceNormals = { Vector3.right, Vector3.left, Vector3.up, Vector3.down, Vector3.forward, Vector3.back };

            Vector3[] vertices  = new Vector3[24];
            Vector3[] normals   = new Vector3[24];
            int[]     triangles = new int[36];

            for (int face = 0; face < 6; face++)
            {
                Vector3 normal   = faceNormals[face];
                Vector3 tangentA = new Vector3(normal.y, normal.z, normal.x);
                Vector3 tangentB = Vector3.Cross(normal, tangentA);

                int vertex = face * 4;

                vertices[vertex]     = (normal + tangentA + tangentB) * 0.5f;
                vertices[vertex + 1] = (normal + tangentA - tangentB) * 0.5f;
                vertices[vertex + 2] = (normal - tangentA - tangentB) * 0.5f;
                vertices[vertex + 3] = (normal - tangentA + tangentB) * 0.5f;

                for (int i = 0; i < 4; i++)
                    normals[vertex + i] = normal;

                int triangle = face * 6;

                //winding matched to Unity's clockwise front faces for the (normal, tangentA, tangentB) frame above
                triangles[triangle]     = vertex;
                triangles[triangle + 1] = vertex + 2;
                triangles[triangle + 2] = vertex + 1;
                triangles[triangle + 3] = vertex;
                triangles[triangle + 4] = vertex + 3;
                triangles[triangle + 5] = vertex + 2;
            }

            return new Mesh
            {
                name      = "Nav3DOctreeOccupancyCube",
                vertices  = vertices,
                normals   = normals,
                triangles = triangles,
                hideFlags = HideFlags.HideAndDontSave
            };
        }

        #endregion
    }
}

#endif
