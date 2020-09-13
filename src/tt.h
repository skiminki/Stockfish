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

/// TTEntry struct is the 10 bytes transposition table entry, defined as below:
///
/// key        16 bit
/// depth       8 bit
/// generation  5 bit
/// pv node     1 bit
/// bound type  2 bit
/// move       16 bit
/// value      16 bit
/// eval value 16 bit

struct TTEntryPacked {

  uint16_t key16;
  uint8_t  depth8;
  uint8_t  genBound8;
  uint16_t move16;
  int16_t  value16;
  int16_t  eval16;
};

struct TTEntry {

  void save(Key k, Value v, bool pv, Bound b, Depth d, Move m, Value ev);

  Move  move()  const { return m_move; }
  Value value() const { return m_value; }
  Value eval()  const { return m_eval; }
  Depth depth() const { return m_depth; }
  bool is_pv()  const { return m_pv; }
  Bound bound() const { return m_bound; }

private:
  friend class TranspositionTable;

  void load(TTEntryPacked *e, size_t clusterIndex, uint8_t slotIndex);

  Move m_move;
  Value m_value;
  Value m_eval;
  bool m_pv;
  Bound m_bound;
  Depth m_depth;

  size_t m_clusterIndex;
  uint8_t m_slotIndex;
};


/// A TranspositionTable is an array of Cluster, of size clusterCount. Each
/// cluster consists of ClusterSize number of TTEntry. Each non-empty TTEntry
/// contains information on exactly one position. The size of a Cluster should
/// divide the size of a cache line for best performance, as the cacheline is
/// prefetched when possible.

class TranspositionTable {

  static constexpr int ClusterSize = 3;

  struct Cluster {
    TTEntryPacked entry[ClusterSize];
    char padding[2]; // Pad to 32 bytes
  };

  static_assert(sizeof(Cluster) == 32, "Unexpected Cluster size");

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
