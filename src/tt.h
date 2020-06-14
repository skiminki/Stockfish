/*
  Stockfish, a UCI chess playing engine derived from Glaurung 2.1
  Copyright (C) 2004-2008 Tord Romstad (Glaurung author)
  Copyright (C) 2008-2015 Marco Costalba, Joona Kiiski, Tord Romstad
  Copyright (C) 2015-2020 Marco Costalba, Joona Kiiski, Gary Linscott, Tord Romstad

  Stockfish is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Stockfish is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef TT_H_INCLUDED
#define TT_H_INCLUDED

#include "misc.h"
#include "types.h"

/// TTEntry struct is the 10 bytes transposition table entry, defined as below:
///
/// move       16 bit
/// value      16 bit
/// eval value 16 bit
/// generation  5 bit
/// pv node     1 bit
/// bound type  2 bit
/// depth       8 bit

struct TTEntry {

  Move  move()  const { return move16; }
  Value value() const { return value16; }
  Value eval()  const { return eval16; }
  Depth depth() const { return m_depth; }
  bool is_pv()  const { return m_is_pv; }
  Bound bound() const { return m_bound; }
  void save(Key k, Value v, bool pv, Bound b, Depth d, Move m, Value ev);

private:
  friend class TranspositionTable;

  Move     move16;
  Value    value16;
  Value    eval16;
  bool     m_is_pv;
  Depth    m_depth;
  Bound    m_bound;

  uint64_t ttePos; // encodes cluster index and entry index
};

inline uint64_t zeroHighBits(uint64_t value, unsigned startShift)
{
    return _bzhi_u64(value, startShift);
}

template <unsigned shift, unsigned numBits, bool top = (shift + numBits == 64U) >
struct ExtractBitFieldImpl
{
    static constexpr unsigned leadingBits = 64U - shift - numBits;
    static constexpr inline uint64_t extract(uint64_t value)
    {
        return (value << leadingBits) >> (leadingBits + shift);
    }
};

template <unsigned numBits>
struct ExtractBitFieldImpl<0U, numBits, false>
{
    static inline uint64_t extract(uint64_t value)
    {
        return zeroHighBits(value, numBits);
    }
};

template <unsigned numBits>
struct ExtractBitFieldImpl<0U, numBits, true>
{
    static_assert(numBits == 64U, "Sanity check");
    static inline uint64_t extract(uint64_t value)
    {
        return value;
    }
};

template <unsigned shift, unsigned numBits>
struct ExtractBitFieldImpl<shift, numBits, true>
{
    static constexpr unsigned leadingBits = 64U - shift - numBits;
    static_assert(leadingBits == 0, "Sanity check");
    static constexpr inline uint64_t extract(uint64_t value)
    {
        return value >> shift;
    }
};

template <unsigned shift, unsigned numBits>
inline uint64_t extractBitField(uint64_t value)
{
    return ExtractBitFieldImpl<shift, numBits>::extract(value);
}

/// A TranspositionTable is an array of Cluster, of size clusterCount. Each
/// cluster consists of ClusterSize number of TTEntry. Each non-empty TTEntry
/// contains information on exactly one position. The size of a Cluster should
/// divide the size of a cache line for best performance,
/// as the cacheline is prefetched when possible.

class TranspositionTable {

  // fields in hash key
  static constexpr unsigned int EntryKeyBits = 21;
  static constexpr unsigned int EntryKeyShift = 64 - EntryKeyBits;
  static constexpr uint64_t EntryKeyBitMask = (uint64_t(1) << EntryKeyBits) - 1;

  static constexpr unsigned int ExtraEntryKeyBits = 3;
  static constexpr unsigned int ExtraEntryKeyShift = EntryKeyShift - ExtraEntryKeyBits;

  static constexpr unsigned int FirstTermClusterKeyBits = 32;
  static constexpr unsigned int FirstTermClusterKeyShift = 0;

  static constexpr unsigned int SecondTermClusterKeyBits = 16;
  static constexpr unsigned int SecondTermClusterKeyShift = 32;

  static constexpr unsigned int ClusterSize = 3;
  static constexpr int ClustersPerSuperCluster = 256;

  static uint32_t encodeMove(Move m);
  static Move decodeMove(uint32_t m);

  struct TTPackedEntry {
      uint64_t packedField;

      static constexpr unsigned movePos      = 0;
      static constexpr unsigned extraHashPos = 13;
      static constexpr unsigned valuePos     = 16;
      static constexpr unsigned evalPos      = 32;
      static constexpr unsigned depthPos     = 48;
      static constexpr unsigned boundPos     = 56;
      static constexpr unsigned pvPos        = 58;
      static constexpr unsigned genPos       = 59;

      uint32_t extractGen() const
      {
          return extractBitField<genPos, 5U>(packedField);
      }

      bool extractPv() const
      {
          return extractBitField<pvPos, 1U>(packedField);
      }

      Bound extractBound() const
      {
          return static_cast<Bound>(extractBitField<boundPos, 2U>(packedField));
      }

      Depth extractDepth() const
      {
          return static_cast<Depth>(uint8_t(packedField >> depthPos));
      }

      bool isOccupied() const
      {
          // note: move field is never zero for occupied entries
          return extractBitField<movePos, 13U>(packedField) != 0U;
      }

      Move extractMove() const
      {
          return decodeMove(extractBitField<movePos, 13U>(packedField));
      }

      uint32_t extractExtraHash() const
      {
          return extractBitField<extraHashPos, 3U>(packedField);
      }

      Value extractValue() const
      {
          return static_cast<Value>(int16_t(packedField >> valuePos));
      }

      Value extractEval() const
      {
          return static_cast<Value>(int16_t(packedField >> evalPos));
      }

      void store(Key k, Value v, bool pv, Bound b, Depth d, Move m, Value ev, uint8_t gen)
      {
          packedField =
                (uint64_t(uint16_t(v)) << valuePos)
              | (uint64_t(pv)    << pvPos)
              | (uint64_t(b) << boundPos)
              | (uint64_t(uint8_t(d)) << depthPos)
              | (uint64_t(encodeMove(m)) << movePos)
              | (extractBitField<ExtraEntryKeyShift, ExtraEntryKeyBits>(k) << extraHashPos)
              | (uint64_t(uint16_t(ev)) << evalPos)
              | (uint64_t(gen) << genPos);

          assert(v == extractValue());
          assert(pv == extractPv());
          assert(b == extractBound());
          assert(d == extractDepth());
          assert(m == extractMove());
          assert(ev == extractEval());
          assert(gen == extractGen());
      }

      TTPackedEntry storeGenOnly(uint32_t gen) const
      {
          static_assert(genPos + 5U == 64, "Sanity check");
          return TTPackedEntry { extractBitField<0U, genPos>(packedField) | uint64_t(gen) << genPos };
      }
  };
  static_assert(sizeof(TTPackedEntry) == 8, "Sanity check");

  struct Cluster {
    uint64_t entryKeys; // 3 * 21-bit keys = 63 bits
    TTPackedEntry entry[ClusterSize];
  };

  static_assert(sizeof(Cluster) == 32, "Unexpected Cluster size");
  static_assert(EntryKeyBits * ClusterSize <= sizeof(EntryKeyBitMask) * 8, "Too many entry key bits");

public:
 ~TranspositionTable() { aligned_ttmem_free(mem); }
  void new_search() { generation8 = (generation8 + 1) & 0x1FU; } // 5 bits reserved for generation
  bool probe(const Key key, TTEntry &tt) const;
  int hashfull() const;
  void resize(size_t mbSize);
  void clear();

  void prefetchCluster(const Key key) const {
    prefetch(getCluster(key));
  }

private:

  size_t getClusterIndex(const Key key) const {

    // The index is computed from
    // Idx = (K48 * SCC) / 2^40, with K48 the 48 lowest bits swizzled.

    const uint64_t firstTerm =  uint32_t(key >> FirstTermClusterKeyShift) * uint64_t(superClusterCount);
    static_assert(FirstTermClusterKeyBits == 32, "Sanity check");

    const uint64_t secondTerm = (uint16_t(key >> SecondTermClusterKeyShift) * uint64_t(superClusterCount)) >> 16;
    static_assert(SecondTermClusterKeyBits == 16, "Sanity check");

    return (firstTerm + secondTerm) >> 24;
  }

  Cluster* getCluster(const Key key) const {

      return &table[getClusterIndex(key)];
  }

  static Cluster& getClusterForEntry(const TTEntry &entry, unsigned int &entryIndex);

  friend struct TTEntry;

  size_t superClusterCount;
  Cluster* table;
  void* mem;
  uint32_t generation8; // Size must be not bigger than TTEntry::genBound8
};

extern TranspositionTable TT;

#endif // #ifndef TT_H_INCLUDED
