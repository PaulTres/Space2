using System.Collections.Concurrent;
using System.Collections.Generic;
using UnityEngine;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.Hierarchy
{
    //The coarse structure of the navigable space: regions (rooms = clearance cores with their skins, passages = what
    //joins two or more rooms, isolated pieces) over the field, the doorways between a passage and a room as the coarse
    //graph's entrances, and the lazily filled cache of routes between two doorways of one region. Immutable once built
    //but for the cache; a publish schedules a replacement (NavTree.Hierarchy.cs), so validity is the map's as a whole.
    internal sealed class RegionMap
    {
        #region Nested types

        public sealed class Region
        {
            public int           Id        { get; }
            public bool          IsPassage { get; }
            public int           Cells     { get; set; }
            public List<Doorway> Doorways  { get; } = new List<Doorway>();

            public Region(int _Id, bool _IsPassage)
            {
                Id        = _Id;
                IsPassage = _IsPassage;
            }
        }

        //an entrance of the coarse graph: where two regions meet — a passage and a room, or two rooms whose skins
        //touch across a slot — with a point on either side (a navigable cell centre in each region, so a search
        //restricted to one region can arrive at it). RegionA < RegionB by id.
        public sealed class Doorway
        {
            public int     Id        { get; }
            public int     RegionA   { get; }
            public int     RegionB   { get; }
            public Vector3 PointInA  { get; }
            public Vector3 PointInB  { get; }
            public float   Clearance { get; }

            //the clearance at the crossing itself — the smaller of its two points' — which is what a body must fit
            //through; Clearance above is the sheet's largest, the plan's measure of how wide the opening is
            public float CrossingClearance { get; }

            //how far the contact sheet reaches from the crossing point: the detour a route pays for crossing HERE rather
            //than anywhere on the sheet, which is what bounds a plan's suboptimality (a room-to-room contact across a
            //slot is a sheet as tall as the room; a tunnel's mouth is its cross-section)
            public float Extent { get; }

            public Doorway(int _Id, int _RegionA, int _RegionB, Vector3 _PointInA, Vector3 _PointInB, float _Clearance, float _CrossingClearance, float _Extent)
            {
                Id                = _Id;
                RegionA           = _RegionA;
                RegionB           = _RegionB;
                PointInA          = _PointInA;
                PointInB          = _PointInB;
                Clearance         = _Clearance;
                CrossingClearance = _CrossingClearance;
                Extent            = _Extent;
            }

            public int OtherRegion(int _Region) => _Region == RegionA ? RegionB : RegionA;

            public Vector3 PointIn(int _Region) => _Region == RegionA ? PointInA : PointInB;
        }

        //a route between two doorways inside one region, at one radius (0 = the point lane): null waypoints = none
        //exists at the epoch it was searched at
        public sealed class CachedRoute
        {
            public List<Vector3> Waypoints { get; }
            public float         Length    { get; }
            public int           Epoch     { get; }

            public CachedRoute(List<Vector3> _Waypoints, float _Length, int _Epoch)
            {
                Waypoints = _Waypoints;
                Length    = _Length;
                Epoch     = _Epoch;
            }
        }

        #endregion

        #region Attributes

        readonly ConcurrentDictionary<(long Pair, float Radius), CachedRoute> m_Routes = new ConcurrentDictionary<(long, float), CachedRoute>();

        //region pairs whose plan the detour guard refused: the flat search answers them without planning again
        readonly ConcurrentDictionary<long, bool> m_RefusedPairs = new ConcurrentDictionary<long, bool>();

        #endregion

        #region Properties

        public RegionField   Field    { get; }
        public List<Region>  Regions  { get; } //index = id; [0] is null (no region)
        public List<Doorway> Doorways { get; } = new List<Doorway>();
        public int           Epoch    { get; }
        public float         Threshold { get; }

        //how the map was built, for the probes: whole, or the previous field repainted only inside the rebuilt roots;
        //the field's share of the build (the holds on the tree) against the whole of it
        public bool   Incremental           { get; set; }
        public int    RepaintedCells        { get; set; }
        public double FieldMilliseconds     { get; set; }
        public double HoldWaitMilliseconds  { get; set; }
        public double TransformMilliseconds { get; set; }
        public double BuildMilliseconds     { get; set; }

        public int RouteCount => m_Routes.Count;

        #endregion

        #region Constructors

        public RegionMap(RegionField _Field, List<Region> _Regions, int _Epoch, float _Threshold)
        {
            Field     = _Field;
            Regions   = _Regions;
            Epoch     = _Epoch;
            Threshold = _Threshold;
        }

        #endregion

        #region Public methods

        //the region of a point, 0 when it lies in no navigable cell (solid, or outside the field)
        public int RegionOfPoint(Vector3 _Point)
        {
            int index = Field.IndexOfPoint(_Point);

            return index < 0 ? 0 : Field.RegionOf[index];
        }

        //The region a LEAF belongs to, by its centre: its own cell, else the nearest navigable cell among the 26 around
        //it. A free leaf against a wall shares its cell with the crust and that cell is solid; the cell one step off
        //the wall is the room's, and that is what the leaf is part of. 0 when nothing navigable is that close.
        public int RegionOfLeafCentre(Vector3 _Centre)
        {
            int index = Field.IndexOfPoint(_Centre);

            if (index < 0)
                return 0;

            int region = Field.RegionOf[index];

            if (region != 0)
                return region;

            Vector3Int cell    = Field.CellOf(index);
            float      bestSqr = float.PositiveInfinity;

            for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
            for (int dz = -1; dz <= 1; dz++)
            {
                if (dx == 0 && dy == 0 && dz == 0 || !Field.Inside(cell.x + dx, cell.y + dy, cell.z + dz))
                    continue;

                int   near = Field.IndexOf(cell.x + dx, cell.y + dy, cell.z + dz);
                int   id   = Field.RegionOf[near];

                if (id == 0)
                    continue;

                float sqr = (Field.CentreOf(near) - _Centre).sqrMagnitude;

                if (sqr < bestSqr)
                {
                    bestSqr = sqr;
                    region  = id;
                }
            }

            return region;
        }

        public bool IsPairRefused(int _RegionA, int _RegionB) => m_RefusedPairs.ContainsKey(PairKey(_RegionA, _RegionB));

        public void RefusePair(int _RegionA, int _RegionB) => m_RefusedPairs[PairKey(_RegionA, _RegionB)] = true;

        //whether two regions share a doorway
        public bool AreAdjacent(int _RegionA, int _RegionB)
        {
            foreach (Doorway doorway in Regions[_RegionA].Doorways)
            {
                if (doorway.OtherRegion(_RegionA) == _RegionB)
                    return true;
            }

            return false;
        }

        public bool TryGetRoute(int _Region, Doorway _A, Doorway _B, float _Radius, out CachedRoute _Route)
        {
            return m_Routes.TryGetValue((RouteKey(_Region, _A, _B), _Radius), out _Route);
        }

        public void StoreRoute(int _Region, Doorway _A, Doorway _B, float _Radius, CachedRoute _Route)
        {
            m_Routes[(RouteKey(_Region, _A, _B), _Radius)] = _Route;
        }

        #endregion

        #region Service methods

        static long PairKey(int _RegionA, int _RegionB) => ((long) Mathf.Min(_RegionA, _RegionB) << 32) | (uint) Mathf.Max(_RegionA, _RegionB);

        //two regions joined twice (a loop) hold the same pair in both, so the region is in the key
        static long RouteKey(int _Region, Doorway _A, Doorway _B)
        {
            int low  = Mathf.Min(_A.Id, _B.Id);
            int high = Mathf.Max(_A.Id, _B.Id);

            return ((long) _Region << 40) | ((long) low << 20) | (uint) high;
        }

        #endregion
    }
}
