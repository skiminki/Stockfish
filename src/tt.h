/*
  Stockfish, a UCI chess playing engine derived from Glaurung 2.1
  Copyright (C) 2004-2020 The Stockfish developers (see AUTHORS file)

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

/// TTEntry struct is the 8 bytes transposition table entry, defined as below:
///
/// depth       8 bit
/// move       13 bit
/// reserved    3 bit
/// value      16 bit
/// eval value 16 bit
/// pv node     1 bit
/// bound type  2 bit
/// generation  5 bit

struct TTEntryPacked {

  uint16_t move13pv1bound2;
  int16_t value16;
  int16_t eval16;
};

struct TTCluster;

struct TTEntry {

  void save(Key k, Value v, bool pv, Bound b, Depth d, Move m, Value ev);

  Move  move()  const { return decodeTTMove(m_ttePacked->move13pv1bound2 & 0x1FFFU); }
  Value value() const { return static_cast<Value>(m_ttePacked->value16); }
  Value eval()  const { return static_cast<Value>(m_ttePacked->eval16); }
  Depth depth() const { return m_depth; }
  bool  is_pv() const { return static_cast<bool>(m_ttePacked->move13pv1bound2 & 0x2000U); }
  Bound bound() const { return static_cast<Bound>(m_ttePacked->move13pv1bound2 >> 14); }

private:
  friend class TranspositionTable;

  void load(TTCluster *cluster, unsigned i);
  void reset(TTCluster *cluster, unsigned i);
  static Move decodeTTMove(uint16_t encodedMove);
  static uint16_t encodeTTMove(Move m);

  TTCluster *m_cluster;
  TTEntryPacked *m_ttePacked;
  Depth m_depth;
};

struct TTCluster {
  static constexpr unsigned int ClusterSize = 3;
  static constexpr unsigned int KeyBits = 16;
  uint32_t keys24depths8[ClusterSize];   // 24 key bits per entry + 8 depth bits
  uint16_t gens5;                        // packed bitfield

  TTEntryPacked entry[ClusterSize];

  uint8_t  getTteDepth(unsigned int slot) const { return keys24depths8[slot] >> 24; }
  uint32_t getTteKey(unsigned int slot)   const { return keys24depths8[slot] & 0xFFFFFF; }

  void storeTteKeyDepth(unsigned int slot, uint32_t key24, uint32_t depth8) {
      keys24depths8[slot] = (depth8 << 24) + key24;
  }

  uint8_t getGen(unsigned int slot) const { return (gens5 >> (slot * 5)) & 0x1F; }
  void storeGen(unsigned int slot, uint8_t newGen5) {
    gens5 = (gens5 & ~(0x1FU << (slot * 5)))
          + (newGen5 << (slot * 5));
  }
};

static_assert(sizeof(TTCluster) == 32, "Unexpected Cluster size");


/// A TranspositionTable is an array of Cluster, of size clusterCount. Each
/// cluster consists of ClusterSize number of TTEntry. Each non-empty TTEntry
/// contains information on exactly one position. The size of a Cluster should
/// divide the size of a cache line for best performance, as the cacheline is
/// prefetched when possible.

class TranspositionTable {
  using Cluster = TTCluster;
  static constexpr unsigned int ClusterSize = Cluster::ClusterSize;

public:

 ~TranspositionTable() { freeMem(); }
  void new_search() { generation5 = (generation5 + 1) & 0x1FU; } // 5 bits, encoded with bound (2 bits) and pv (1 bit)
  bool probe(const Key key, TTEntry &entry) const;
  int hashfull() const;

  void resizeIfChanged(); // trigger resize if options changed
  void clear();           // clear if hash is dirty
  void markDirty() { state = State::Dirty; } // mark the hash dirty

  TTEntryPacked* first_entry(const Key key) const {
    return &table[mul_hi64(key, clusterCount)].entry[0];
  }

  enum class State {
      Clear,
      ClearNonResident,
      Dirty,
  };

private:

  friend struct TTEntry;

  size_t clusterCount;
  Cluster* table;
  void* mem = nullptr;
  State state = State::Clear;
  size_t memSize = 0; // in bytes
  size_t memPageSize = 0; // in bytes

  // Current TT config values -- used to trigger resize in resizeIfChanged()
  size_t memMb = 0;
  size_t numThreads = 0;

  uint8_t generation5; // Must be within 5 bits

  void allocMem(size_t mbSize); // sets clusterCount, mem, memSize, table
  void freeMem();
};

extern TranspositionTable TT;

#endif // #ifndef TT_H_INCLUDED
