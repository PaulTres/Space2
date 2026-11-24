using System;
using System.Collections.Generic;

namespace Nav3D.SpaceProcessing.ObstacleProcessing.GlobalOctree.Elements
{
    //A fork's eight child slots addressed by ForkChildOctIndex, replacing the Dictionary every fork carried: the map
    //was the bulk of a fork's 762 B against a bare leaf's 144 B, and every descent hashed an enum key in
    //pathfinding's hot loop; an octant is a number from 0 to 7, so a lookup is one array read
    //. A view: the slots live in the Fork, this struct carries
    //the reference and keeps the dictionary's member names, and writing goes through Fork.SetChild. Enumerating while
    //removing is safe, since the enumerator walks slot indices. Not thread-safe and need not be: written by
    //construction on one thread and by the publish under the write lock.
    readonly struct ForkChildren
    {
        #region Nested types

        //Yields the present children as key/value pairs in slot order, which for these flags is ascending flag
        //order — the order the serialization's child-ID list relies on.
        public struct Enumerator
        {
            readonly Node[] m_Slots;

            int m_Slot;

            public Enumerator(Node[] _Slots)
            {
                m_Slots = _Slots;
                m_Slot  = -1;
            }

            public KeyValuePair<ForkChildOctIndex, Node> Current => new KeyValuePair<ForkChildOctIndex, Node>(OctIndexOf(m_Slot), m_Slots[m_Slot]);

            public bool MoveNext()
            {
                while (++m_Slot < SLOTS_COUNT)
                {
                    if (m_Slots[m_Slot] != null)
                        return true;
                }

                return false;
            }
        }

        public readonly struct ValuesCollection
        {
            readonly Node[] m_Slots;

            public ValuesCollection(Node[] _Slots)
            {
                m_Slots = _Slots;
            }

            public ValuesEnumerator GetEnumerator()
            {
                return new ValuesEnumerator(m_Slots);
            }
        }

        public struct ValuesEnumerator
        {
            readonly Node[] m_Slots;

            int m_Slot;

            public ValuesEnumerator(Node[] _Slots)
            {
                m_Slots = _Slots;
                m_Slot  = -1;
            }

            public Node Current => m_Slots[m_Slot];

            public bool MoveNext()
            {
                while (++m_Slot < SLOTS_COUNT)
                {
                    if (m_Slots[m_Slot] != null)
                        return true;
                }

                return false;
            }
        }

        #endregion
        #region Constants

        public const int SLOTS_COUNT = 8;

        #endregion

        #region Attributes (static)

        //ForkChildOctIndex is a bit FLAG (1, 2, 4 … 128), not an ordinal, so the slot is the flag's bit position.
        //A 256-entry table makes that a single array read; every value that is not one of the eight flags maps to
        //INVALID_SLOT and is rejected loudly rather than silently landing in slot 0.
        const byte INVALID_SLOT = 255;

        static readonly byte[] s_SlotByFlag = BuildSlotByFlag();

        static readonly ForkChildOctIndex[] s_FlagBySlot =
        {
            ForkChildOctIndex.I000,
            ForkChildOctIndex.I001,
            ForkChildOctIndex.I010,
            ForkChildOctIndex.I011,
            ForkChildOctIndex.I100,
            ForkChildOctIndex.I101,
            ForkChildOctIndex.I110,
            ForkChildOctIndex.I111
        };

        #endregion

        #region Attributes

        readonly Node[] m_Slots;

        #endregion

        #region Properties

        //Present children — the count of non-empty slots. Callers use it to detect a childless fork.
        public int Count
        {
            get
            {
                int count = 0;

                for (int slot = 0; slot < SLOTS_COUNT; slot++)
                {
                    if (m_Slots[slot] != null)
                        count++;
                }

                return count;
            }
        }

        public ValuesCollection Values => new ValuesCollection(m_Slots);

        #endregion

        #region Constructors

        public ForkChildren(Node[] _Slots)
        {
            m_Slots = _Slots;
        }

        #endregion

        #region Public methods

        //The slot a flag addresses, and the flag a slot carries — public because the construction and prune passes
        //that used to hold ForkChildOctIndex keys need to move between the two representations.
        public static int SlotOf(ForkChildOctIndex _OctIndex)
        {
            byte slot = s_SlotByFlag[(byte) _OctIndex];

            if (slot == INVALID_SLOT)
                throw new ArgumentOutOfRangeException(nameof(_OctIndex), _OctIndex, "Not a single-octant ForkChildOctIndex flag.");

            return slot;
        }

        public static ForkChildOctIndex OctIndexOf(int _Slot)
        {
            return s_FlagBySlot[_Slot];
        }

        public bool TryGetValue(ForkChildOctIndex _OctIndex, out Node _Child)
        {
            _Child = m_Slots[SlotOf(_OctIndex)];

            return _Child != null;
        }

        public bool Remove(ForkChildOctIndex _OctIndex)
        {
            int slot = SlotOf(_OctIndex);

            if (m_Slots[slot] == null)
                return false;

            m_Slots[slot] = null;

            return true;
        }

        public void Clear()
        {
            Array.Clear(m_Slots, 0, SLOTS_COUNT);
        }

        //Replaces the `Any(_MapEl => _MapEl.Value is Leaf)` the prune passes used to run — same question, no
        //closure, no delegate, no boxed enumerator (that LINQ shape was itself a measured per-fork allocation,
        //see Fork.ProduceChildren's note).
        public bool AnyLeafChild()
        {
            for (int slot = 0; slot < SLOTS_COUNT; slot++)
            {
                if (m_Slots[slot] is Leaf)
                    return true;
            }

            return false;
        }

        public Enumerator GetEnumerator()
        {
            return new Enumerator(m_Slots);
        }

        #endregion

        #region Service methods

        static byte[] BuildSlotByFlag()
        {
            byte[] table = new byte[256];

            for (int index = 0; index < table.Length; index++)
                table[index] = INVALID_SLOT;

            table[(byte) ForkChildOctIndex.I000] = 0;
            table[(byte) ForkChildOctIndex.I001] = 1;
            table[(byte) ForkChildOctIndex.I010] = 2;
            table[(byte) ForkChildOctIndex.I011] = 3;
            table[(byte) ForkChildOctIndex.I100] = 4;
            table[(byte) ForkChildOctIndex.I101] = 5;
            table[(byte) ForkChildOctIndex.I110] = 6;
            table[(byte) ForkChildOctIndex.I111] = 7;

            return table;
        }

        #endregion

    }
}
