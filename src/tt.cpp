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

#include <cstring>   // For std::memset
#include <iostream>
#include <thread>

#include "bitboard.h"
#include "misc.h"
#include "thread.h"
#include "tt.h"
#include "uci.h"

TranspositionTable TT; // Our global transposition table

namespace {


}

TranspositionTable::Cluster& TranspositionTable::getClusterForEntry(
    const TTEntry &entry, unsigned int &entryIndex) {

  entryIndex = entry.ttePos & 3;
  return TT.table[entry.ttePos >> 2];
}

/// TTEntry::save populates the TTEntry with a new node's data, possibly
/// overwriting an old position. Update is not atomic and can be racy.

void TTEntry::save(Key k, Value v, bool pv, Bound b, Depth d, Move m, Value ev) {

  unsigned int entryIndex = 0;
  TranspositionTable::Cluster &cluster = TranspositionTable::getClusterForEntry(*this, entryIndex);
  TranspositionTable::TTPackedEntry tteOut;


  // The first entry offset is 8
  const unsigned int keyShift = entryIndex * TranspositionTable::EntryKeyBits;

  uint64_t shiftedKey =
      ((k >> TranspositionTable::EntryKeyShift) & TranspositionTable::EntryKeyBitMask) << keyShift;
  const uint64_t shiftedKeyMask = TranspositionTable::EntryKeyBitMask << keyShift;

  // Overwrite less valuable entries
  if (   b == BOUND_EXACT
      || d > m_depth - 4
      || (shiftedKey != (cluster.entryKeys & shiftedKeyMask))
      )
  {
      assert(d >= DEPTH_OFFSET);

      cluster.entryKeys = (cluster.entryKeys & ~shiftedKeyMask) | shiftedKey;
      tteOut.store(v, pv, b, d - DEPTH_OFFSET, m ? m : move16, ev, TT.generation8);
      cluster.entry[entryIndex] = tteOut;
  }
  else if (m && m != move16) {
      // Preserve any existing move for the same position
      tteOut.store(value16, m_is_pv, m_bound, m_depth - DEPTH_OFFSET, m, eval16, TT.generation8);
      cluster.entry[entryIndex] = tteOut;
  }
}


/// TranspositionTable::resize() sets the size of the transposition table,
/// measured in megabytes. Transposition table consists of a power of 2 number
/// of clusters and each cluster consists of ClusterSize number of TTEntry.

void TranspositionTable::resize(size_t mbSize) {

  Threads.main()->wait_for_search_finished();

  aligned_ttmem_free(mem);

  superClusterCount = mbSize * 1024 * 1024 / (sizeof(Cluster) * ClustersPerSuperCluster);

  table = static_cast<Cluster*>(
      aligned_ttmem_alloc(superClusterCount * ClustersPerSuperCluster * sizeof(Cluster), mem));
  if (!mem)
  {
      std::cerr << "Failed to allocate " << mbSize
                << "MB for transposition table." << std::endl;
      exit(EXIT_FAILURE);
  }

  clear();
}


/// TranspositionTable::clear() initializes the entire transposition table to zero,
//  in a multi-threaded way.

void TranspositionTable::clear() {

  std::vector<std::thread> threads;

  for (size_t idx = 0; idx < Options["Threads"]; ++idx)
  {
      threads.emplace_back([this, idx]() {

          const size_t clusterCount = superClusterCount * ClustersPerSuperCluster;

          // Thread binding gives faster search on systems with a first-touch policy
          if (Options["Threads"] > 8)
              WinProcGroup::bindThisThread(idx);

          // Each thread will zero its part of the hash table
          const size_t stride = size_t(clusterCount / Options["Threads"]),
                       start  = size_t(stride * idx),
                       len    = idx != Options["Threads"] - 1 ?
                                stride : clusterCount - start;

          std::memset(&table[start], 0, len * sizeof(Cluster));
      });
  }

  for (std::thread& th : threads)
      th.join();
}

/// TranspositionTable::probe() looks up the current position in the transposition
/// table. It returns true and a pointer to the TTEntry if the position is found.
/// Otherwise, it returns false and a pointer to an empty or least valuable TTEntry
/// to be replaced later. The replace value of an entry is calculated as its depth
/// minus 8 times its relative age. TTEntry t1 is considered more valuable than
/// TTEntry t2 if its replace value is greater than that of t2.

bool TranspositionTable::probe(const Key key, TTEntry &tteOut) const {

  const size_t clusterIndex = getClusterIndex(key);
  Cluster& cluster = table[clusterIndex];
  uint64_t clusterEntryKeys = cluster.entryKeys;
  const uint32_t probeKey = extractBitField<EntryKeyShift, EntryKeyBits>(key);

  for (unsigned int i = 0; i < ClusterSize; ++i)
  {
      const uint32_t entryKey = clusterEntryKeys & EntryKeyBitMask;
      if (!entryKey || entryKey == probeKey)
      {
          TTPackedEntry tteIn = cluster.entry[i];

          cluster.entry[i] = tteIn.storeGenOnly(generation8); // writeback

          tteOut.m_is_pv = tteIn.extractPv();
          tteOut.m_bound = tteIn.extractBound();
          tteOut.m_depth = tteIn.extractDepth() + DEPTH_OFFSET;
          tteOut.move16 = tteIn.extractMove();
          tteOut.value16 = tteIn.extractValue();
          tteOut.eval16 = tteIn.extractEval();
          tteOut.ttePos = (clusterIndex << 2) | i;

          return static_cast<bool>(entryKey);
      }

      clusterEntryKeys >>= EntryKeyBits;
  }

  // Find an entry to be replaced according to the replacement strategy
  unsigned int replaceIndex = 0;
  for (unsigned int i = 1; i < ClusterSize; ++i) {
      TTPackedEntry& replace = cluster.entry[replaceIndex];
      TTPackedEntry& candidate = cluster.entry[i];

      // Due to our packed storage format for generation and its cyclic
      // nature we add 263 (256 is the modulus plus 7 to keep the unrelated
      // lowest three bits from affecting the result) to calculate the entry
      // age correctly even after generation8 overflows into the next cycle.
      /*
      if (  replace.depth8 - ((generation8 - replace.gen) & 0x1F) * 8
          > candidate.depth8 - ((generation8 - candidate.gen) & 0x1F) * 8)
          replaceIndex = i;
      */
      int32_t depthDiff = int32_t(replace.extractDepth() - candidate.extractDepth());
      int32_t genDiff1  = int32_t((generation8 - replace.extractGen()) & 0x1F);
      int32_t genDiff2  = int32_t((generation8 - candidate.extractGen()) & 0x1F);
      if (  depthDiff > (genDiff1 - genDiff2) * 8)
          replaceIndex = i;
  }

  tteOut.move16 = Move { };
  tteOut.value16 = Value { };
  tteOut.eval16 =  Value { };
  tteOut.m_depth = 0;
  tteOut.m_bound = Bound { };
  tteOut.m_is_pv = false;

  tteOut.ttePos = (clusterIndex << 2) | replaceIndex;
  return false;
}


/// TranspositionTable::hashfull() returns an approximation of the hashtable
/// occupation during a search. The hash is x permill full, as per UCI protocol.

int TranspositionTable::hashfull() const {

  int cnt = 0;
  for (size_t i = 0; i < 1000; ++i)
      for (unsigned int j = 0; j < ClusterSize; ++j)
          cnt += (table[i].entry[j].extractGen() == generation8);

  return cnt / ClusterSize;
}
