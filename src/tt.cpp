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

#include <cstring>   // For std::memset
#include <iostream>
#include <thread>

#include "bitboard.h"
#include "misc.h"
#include "thread.h"
#include "tt.h"
#include "uci.h"

namespace {

inline int ageTteDepthByGen(uint8_t depth8, uint8_t curGen5, uint8_t prevGen5)
{
  constexpr uint8_t genDepthPenalty = 8; // every gen means depth reduction by 8
  const uint8_t aging = ((curGen5 - prevGen5) & 0x1FU) * genDepthPenalty;

  return int(depth8) - int(aging);
}

inline uint32_t entryKeyFromZobrist(Key k)
{
  return k & ((uint32_t(1) << TTCluster::KeyBits) - 1);
}

}

TranspositionTable TT; // Our global transposition table

Move TTEntry::decodeTTMove(uint16_t encodedMove)
{
  if (encodedMove & 0x1000U)
  {
      // special move type, need to figure out which one by from source rank
      encodedMove &= 0xFFFU; // 0xFFF = 07777

      static constexpr uint16_t sourceRankToOrBits[] = {
          CASTLING,           // RANK_1
          PROMOTION,          // RANK_2
          NORMAL,             // RANK_3
          ENPASSANT,          // RANK_4
          ENPASSANT,          // RANK_5
          NORMAL,             // RANK_6
          PROMOTION | 00070U, // RANK_7 + force destination rank 8
          CASTLING,           // RANK_8
      };

      static constexpr uint16_t sourceRankToAndBits[] = {
          07777U,             // RANK_1
          07707U,             // RANK_2 + force destination rank 1
          07777U,             // RANK_3
          07777U,             // RANK_4
          07777U,             // RANK_5
          07777U,             // RANK_6
          07777U,             // RANK_7
          07777U,             // RANK_8
      };

      static constexpr uint16_t sourceRankToPromofilter[] = {
          00000U,             // RANK_1
          00070U,             // RANK_2 promotion
          00000U,             // RANK_3
          00000U,             // RANK_4
          00000U,             // RANK_5
          00000U,             // RANK_6
          00070U,             // RANK_7 promotion
          00000U,             // RANK_8
      };

      const uint16_t sourceRank  = encodedMove >> 9;
      const uint16_t orBits      = sourceRankToOrBits[sourceRank];
      const uint16_t andBits     = sourceRankToAndBits[sourceRank];
      const uint16_t promoFilter = sourceRankToPromofilter[sourceRank];

      return Move((orBits | (encodedMove & andBits)) + ((encodedMove & promoFilter) << 9));
  }
  else
      return Move(encodedMove);
}

uint16_t TTEntry::encodeTTMove(Move m)
{
  static constexpr uint32_t moveTypeToAndBits[] = {
      007777U,             // NORMAL
      007707U,             // PROMOTION, reset destination rank
      007777U,             // ENPASSANT
      007777U,             // CASTLING
  };

  static constexpr uint32_t moveTypeToPromofilter[] = {
      000000U,             // NORMAL
      030000U,             // PROMOTION, promotion is in bits 12-13, these need to be shifted to destination rank (bits 3-5)
      000000U,             // ENPASSANT
      000000U,             // CASTLING
  };

  const uint32_t moveTypeBits = m >> 14;
  const uint32_t orBits       = moveTypeBits ? 010000U : 00000U;   // set special for other than normal move types
  const uint32_t andBits      = moveTypeToAndBits[moveTypeBits];
  const uint32_t promofilter  = moveTypeToPromofilter[moveTypeBits];

  uint32_t encodedMove = (m & andBits) + orBits + ((m & promofilter) >> 9);

#if !defined(NDEBUG)
  if (decodeTTMove(encodedMove) != m)
  {
      fprintf(stderr, "Move: 0x%04x  Encoded: 0x%04x  Decoded: 0x%04x From=%c%u To=%c%u\n",
              m, encodedMove, decodeTTMove(encodedMove), file_of(from_sq(m))+'A', 1 + rank_of(from_sq(m)), 'A'+file_of(to_sq(m)), 1+rank_of(to_sq(m)));
      assert(decodeTTMove(encodedMove) == m);
  }
  // range check
  assert(encodedMove < (1U << 13));
#endif

  return encodedMove;
}


void TTEntry::load(TTCluster *cluster, unsigned i)
{
  m_cluster = cluster;
  m_ttePacked = &cluster->entry[i];
  m_depth = cluster->getTteDepth(i) + DEPTH_OFFSET;
}

void TTEntry::reset(TTCluster *cluster, unsigned i)
{
  m_cluster = cluster;
  m_ttePacked = &cluster->entry[i];
  m_depth = DEPTH_NONE;
}

/// TTEntry::save() populates the TTEntry with a new node's data, possibly
/// overwriting an old position. Update is not atomic and can be racy.
void TTEntry::save(Key fullKey, Value v, bool pv, Bound b, Depth d, Move m, Value ev) {

  TranspositionTable::Cluster& cluster = *m_cluster;
  const unsigned slot = m_ttePacked - cluster.entry;

  const uint32_t k = entryKeyFromZobrist(fullKey);
  const uint32_t tteKey = cluster.getTteKey(slot);

  // Preserve any existing move for the same position
  if (m || (k != tteKey))
  {
      m_ttePacked->move13pv1bound2 &= ~uint16_t(0x1FFF);
      m_ttePacked->move13pv1bound2 += encodeTTMove(m);

      assert(move() == m);
  }

  // Overwrite less valuable entries (cheapest checks first)
  if (b == BOUND_EXACT
      || k != tteKey
      || d - DEPTH_OFFSET > cluster.getTteDepth(slot) - 4)
  {
      assert(d > DEPTH_OFFSET);
      assert(d < 256 + DEPTH_OFFSET);

      cluster.storeTteKeyDepth(slot, k, d - DEPTH_OFFSET);
      cluster.storeGen(slot, TT.generation5);

      m_ttePacked->move13pv1bound2 &= 0x1FFF;
      m_ttePacked->move13pv1bound2 += (uint16_t(pv) << 13) + (uint16_t(b) << 14);

      m_ttePacked->value16   = (int16_t)v;
      m_ttePacked->eval16    = (int16_t)ev;
  }
}


/// TranspositionTable::resize() sets the size of the transposition table,
/// measured in megabytes. Transposition table consists of a power of 2 number
/// of clusters and each cluster consists of ClusterSize number of TTEntry.

void TranspositionTable::resizeIfChanged() {

  const size_t optMemMb = Options["Hash"];
  const size_t optNumThreads = Options["Threads"];
  const size_t optHashPageSize = Options["Hash Page Size"];

  if (optMemMb != memMb || optNumThreads != numThreads || optHashPageSize != memPageSize)
  {
      bool clean = false;

      Threads.main()->wait_for_search_finished();
      freeMem();

      memMb = optMemMb;
      numThreads = optNumThreads;
      memPageSize = optHashPageSize;

      clusterCount = memMb * 1024 * 1024 / sizeof(Cluster);

      // check that pageSize is power of 2
      if (memPageSize & (memPageSize - 1))
      {
          std::cerr << "Hash page size must be power of 2" << std::endl;
          exit(EXIT_FAILURE);
      }

      table = static_cast<Cluster*>(aligned_ttmem_alloc(clusterCount * sizeof(Cluster), mem, memPageSize, clean));
      if (!mem)
      {
          std::cerr << "Failed to allocate " << memMb
                    << "MB for transposition table";

          if (memPageSize)
              std::cerr << " with hash page size " << memPageSize;

          std::cerr << "." << std::endl;
          exit(EXIT_FAILURE);
      }

      // after malloc, the state is dirty. But after paged alloc, all we need is
      // to make the pages resident
      state = (clean ? State::ClearNonResident : State::Dirty);
      clear();
  }
}

void TranspositionTable::freeMem() {
    if (mem)
    {
        aligned_ttmem_free(mem, clusterCount * sizeof(Cluster), memPageSize);
        mem = nullptr;
    }
}


/// TranspositionTable::clear() initializes the entire transposition table to zero,
//  in a multi-threaded way.

uint64_t roundUpPages(uint64_t value, uint64_t pageSize)
{
    if (pageSize == 0)
        return value;
    else
        return (value + pageSize - 1) &~ (pageSize - 1);
}

void TranspositionTable::clear() {

  if (state == State::Clear)
      return; // don't clear, hash already clean

  std::vector<std::thread> threads;

  const unsigned int optNumThreads = Options["Threads"];
  const unsigned int numClearThreads = Options["Hash Clear Threads"] ? static_cast<unsigned int>(Options["Hash Clear Threads"]) : optNumThreads;
  const uint64_t hashSize = clusterCount * sizeof(Cluster); // to avoid overflow in (hashSize * idx) on 32-bit archs

  for (size_t idx = 0; idx < numClearThreads; ++idx)
  {
      const uint64_t startOffset = roundUpPages(hashSize * idx / numClearThreads, memPageSize);
      const uint64_t endOffset =   roundUpPages(hashSize * (idx + 1) / numClearThreads, memPageSize);

      if (startOffset < endOffset)
      {
          threads.emplace_back([this, idx, optNumThreads, numClearThreads, startOffset, endOffset]() {

              // Thread binding gives faster search on systems with a first-touch policy
              if (optNumThreads > 8)
                  WinProcGroup::bindThisThread(idx * optNumThreads / numClearThreads);

              if (state == State::Dirty)
              {
                  std::memset(reinterpret_cast<uint8_t *>(table) + startOffset, 0, endOffset - startOffset);
              }
              else
              {
                  // ClearNonResident = we need to just touch every page
                  for (uint64_t i = startOffset; i < endOffset; i += memPageSize)
                  {
                      reinterpret_cast<uint8_t *>(table)[i] = 0;
                  }
              }
          });
      }
  }

  for (std::thread& th : threads)
      th.join();

  state = State::Clear;
}


/// TranspositionTable::probe() looks up the current position in the transposition
/// table. It returns true and a pointer to the TTEntry if the position is found.
/// Otherwise, it returns false and a pointer to an empty or least valuable TTEntry
/// to be replaced later. The replace value of an entry is calculated as its depth
/// minus 8 times its relative age. TTEntry t1 is considered more valuable than
/// TTEntry t2 if its replace value is greater than that of t2.
bool TranspositionTable::probe(const Key fullKey, TTEntry &entry) const {

  const size_t clusterIndex = mul_hi64(fullKey, clusterCount);
  Cluster& cluster = TT.table[clusterIndex];
  const uint32_t key = entryKeyFromZobrist(fullKey);

  for (unsigned int i = 0; i < ClusterSize; ++i)
  {
      if (key == cluster.getTteKey(i) || cluster.getTteDepth(i) == 0)
      {
          cluster.storeGen(i, generation5); // Refresh gen

          entry.load(&cluster, i);
          return cluster.getTteDepth(i);
      }
  }

  // Find an entry to be replaced according to the replacement strategy
  unsigned int replaceIndex = 0;
  int replaceTteDepth =
      ageTteDepthByGen(cluster.getTteDepth(0), generation5, cluster.getGen(0));

  for (unsigned int i = 1; i < ClusterSize; ++i)
  {
      const int tteDepth = ageTteDepthByGen(cluster.getTteDepth(i), generation5, cluster.getGen(i));
      if (replaceTteDepth > tteDepth)
      {
          replaceIndex = i;
          replaceTteDepth = tteDepth;
      }
  }

  entry.reset(&cluster, replaceIndex);
  return false;
}


/// TranspositionTable::hashfull() returns an approximation of the hashtable
/// occupation during a search. The hash is x permill full, as per UCI protocol.

int TranspositionTable::hashfull() const {

  int cnt = 0;
  for (int i = 0; i < 1000; ++i)
      for (unsigned int j = 0; j < ClusterSize; ++j)
          cnt += table[i].getTteDepth(j) && table[i].getGen(j) == generation5;

  return cnt / ClusterSize;
}
