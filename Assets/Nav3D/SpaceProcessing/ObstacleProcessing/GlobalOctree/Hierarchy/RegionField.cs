using Nav3D.Common;
using System;
using UnityEngine;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.Hierarchy
{
    //The fine grid the region map is derived from: per cell whether it is open (no sample inside it in an occupied
    //leaf), its euclidean clearance to the nearest non-open cell, and the region it was assigned to. The cell is at
    //most half the narrowest passage that must read as a passage (MaxAgentRadius by default), because a coarser field
    //cannot see it at all — an 8 u cell over an 8 u tunnel reads the tunnel as one solid-ish cell.
    internal sealed class RegionField
    {
        #region Properties

        public float      Cell     { get; }
        public Vector3Int Origin   { get; }
        public Vector3Int Dims     { get; }
        public bool[]     Open     { get; }
        public float[]    Distance { get; }
        public int[]      RegionOf { get; }

        public int Count => Open.Length;

        #endregion

        #region Constructors

        public RegionField(float _Cell, Vector3Int _Origin, Vector3Int _Dims)
        {
            Cell     = _Cell;
            Origin   = _Origin;
            Dims     = _Dims;
            Open     = new bool[_Dims.x * _Dims.y * _Dims.z];
            Distance = new float[Open.Length];
            RegionOf = new int[Open.Length];
        }

        #endregion

        #region Public methods

        public int IndexOf(int _X, int _Y, int _Z) => (_X * Dims.y + _Y) * Dims.z + _Z;

        public Vector3Int CellOf(int _Index)
        {
            int z = _Index % Dims.z;
            int y = _Index / Dims.z % Dims.y;
            int x = _Index / (Dims.z * Dims.y);

            return new Vector3Int(x, y, z);
        }

        public bool Inside(int _X, int _Y, int _Z) => _X >= 0 && _Y >= 0 && _Z >= 0 && _X < Dims.x && _Y < Dims.y && _Z < Dims.z;

        public Vector3 CentreOf(int _Index)
        {
            Vector3Int cell = CellOf(_Index);

            return new Vector3((Origin.x + cell.x + 0.5f) * Cell, (Origin.y + cell.y + 0.5f) * Cell, (Origin.z + cell.z + 0.5f) * Cell);
        }

        //-1 outside the grid
        public int IndexOfPoint(Vector3 _Point)
        {
            Vector3Int cell = UtilsSpatialGrid.GetBucketIndex(_Point, Cell) - Origin;

            return Inside(cell.x, cell.y, cell.z) ? IndexOf(cell.x, cell.y, cell.z) : -1;
        }

        //Exact euclidean distance (world units) from every open cell's centre to the nearest non-open cell, the
        //grid's outside counting as solid: Felzenszwalb and Huttenlocher's squared transform, one pass per axis.
        public void ComputeDistances()
        {
            float[] distance = Distance;
            int     nx       = Dims.x, ny = Dims.y, nz = Dims.z;
            float   infinity = (float) (nx + ny + nz) * (nx + ny + nz);

            for (int index = 0; index < distance.Length; index++)
                distance[index] = Open[index] ? infinity : 0f;

            //a border cell's centre is half a cell from the outside, which the passes cannot see
            for (int x = 0; x < nx; x++)
            for (int y = 0; y < ny; y++)
            for (int z = 0; z < nz; z++)
            {
                if (x == 0 || y == 0 || z == 0 || x == nx - 1 || y == ny - 1 || z == nz - 1)
                {
                    int index = IndexOf(x, y, z);

                    distance[index] = Mathf.Min(distance[index], 0.25f);
                }
            }

            int     longest = Math.Max(nx, Math.Max(ny, nz));
            float[] f       = new float[longest];
            float[] d       = new float[longest];
            int[]   v       = new int[longest];
            float[] bounds  = new float[longest + 1];

            void Pass(int _Count, Func<int, int> _IndexOf)
            {
                for (int i = 0; i < _Count; i++)
                    f[i] = distance[_IndexOf(i)];

                int k = 0;

                v[0]      = 0;
                bounds[0] = float.NegativeInfinity;
                bounds[1] = float.PositiveInfinity;

                for (int q = 1; q < _Count; q++)
                {
                    float s;

                    while (true)
                    {
                        int p = v[k];

                        s = (f[q] + q * q - (f[p] + p * p)) / (2f * q - 2f * p);

                        if (s > bounds[k])
                            break;

                        k--;
                    }

                    k++;

                    v[k]          = q;
                    bounds[k]     = s;
                    bounds[k + 1] = float.PositiveInfinity;
                }

                k = 0;

                for (int q = 0; q < _Count; q++)
                {
                    while (bounds[k + 1] < q)
                        k++;

                    int p = v[k];

                    d[q] = (q - p) * (q - p) + f[p];
                }

                for (int i = 0; i < _Count; i++)
                    distance[_IndexOf(i)] = d[i];
            }

            for (int y = 0; y < ny; y++)
            for (int z = 0; z < nz; z++)
            {
                int yy = y, zz = z;

                Pass(nx, _X => IndexOf(_X, yy, zz));
            }

            for (int x = 0; x < nx; x++)
            for (int z = 0; z < nz; z++)
            {
                int xx = x, zz = z;

                Pass(ny, _Y => IndexOf(xx, _Y, zz));
            }

            for (int x = 0; x < nx; x++)
            for (int y = 0; y < ny; y++)
            {
                int xx = x, yy = y;

                Pass(nz, _Z => IndexOf(xx, yy, _Z));
            }

            for (int index = 0; index < distance.Length; index++)
                distance[index] = Mathf.Sqrt(distance[index]) * Cell;
        }

        #endregion
    }
}
