#pragma once
/*
Copyright 2025 by the VCC Project Contributors.
This file is part of VCC (Virtual Color Computer).

    VCC (Virtual Color Computer) is free software: you can redistribute it
    and/or modify it under the terms of the GNU General Public License as
    published by the Free Software Foundation, either version 3 of the
    License, or (at your option) any later version.

    VCC (Virtual Color Computer) is distributed in the hope that it will be
    useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with VCC (Virtual Color Computer). If not, see
    <http://www.gnu.org/licenses/>.
*/

// BlockCache: Tracks basic blocks of 6809/6309 code for batch execution.
//
// A "block" is a straight-line sequence of instructions ending at a control
// flow change (branch, jump, call, return, SWI, SYNC, CWAI) or a maximum
// instruction count.
//
// Blocks are discovered dynamically: the first execution records the block,
// subsequent executions use the cached instruction count and cycle cost to
// execute the block in a tight loop without per-instruction interrupt checks.
//
// Write invalidation: A 64KB reverse map tracks which block (if any) covers
// each logical address. When memory is written, the reverse map provides O(1)
// lookup to find and invalidate the affected block. This handles self-modifying
// code, code loading, and other dynamic code scenarios.
//
// MMU changes invalidate the entire cache since logical-to-physical mappings
// have changed.

#include <cassert>
#include <cstdint>
#include <cstring>
#include "DecodedInst.h"
#include "BlockDecoder.h"

struct CachedBlock
{
    uint16_t start_pc;      // logical PC where this block starts
    uint16_t end_pc;        // PC after the last instruction in the block
    uint8_t  num_insns;     // number of instructions in the block
    uint8_t  total_cycles;  // total cycle cost of the block
    uint32_t generation;    // generation when this block was cached

    // Pre-decoded instructions for this block. Filled by DecodeBlock()
    // when the block is first cached. Handlers are called with a pointer
    // to the corresponding DecodedInst, allowing incremental conversion
    // from memory-reading to pre-decoded operand access.
    DecodedInst insns[12];  // MAX_BLOCK_INSNS — can't forward-ref constexpr
};

struct BlockCacheStats
{
    uint64_t block_hits;        // instructions executed via cached blocks
    uint64_t block_insns;       // total instructions in those blocks
    uint64_t single_steps;      // instructions executed one-at-a-time
    uint64_t blocks_recorded;   // new blocks committed to cache
    uint64_t record_cancels;    // in-progress recordings abandoned
    uint64_t rejected_blocks;   // decoded recordings rejected before caching
    uint64_t invalidations;     // individual block invalidations (write)
    uint64_t bulk_invalidations;// full cache invalidations (MMU change)
};

class BlockCache
{
public:
    // Cache size must be power of 2 for fast masking.
    static constexpr int CACHE_SIZE = 4096;
    static constexpr int CACHE_MASK = CACHE_SIZE - 1;

    // Maximum instructions per block. Keeps interrupt latency bounded.
    // At ~5 cycles/instruction average, 12 instructions = ~60 cycles = ~33us.
    static constexpr int MAX_BLOCK_INSNS = 12;

    // Sentinel value meaning "no block covers this address".
    static constexpr uint16_t NO_BLOCK = 0xFFFF;

    BlockCache() { Clear(); }

    void Clear()
    {
        memset(blocks_, 0, sizeof(blocks_));
        memset(reverseMap_, 0xFF, sizeof(reverseMap_));  // fill with NO_BLOCK
        memset(pageBitmap_, 0, sizeof(pageBitmap_));
        generation_ = 1;  // start at 1 so 0-initialized blocks are invalid
    }

    // Look up a block by its starting PC. Returns nullptr if not cached.
    const CachedBlock* Lookup(uint16_t pc) const
    {
        const CachedBlock& b = blocks_[pc & CACHE_MASK];
        if (b.generation == generation_ && b.start_pc == pc)
            return &b;
        return nullptr;
    }

    // Check if a memory write invalidates a cached block.
    // Called from MemWrite8 on every RAM write. Must be VERY fast.
    // The coarse bitmap (32 bytes, fits in L1) rejects 99.99% of writes
    // before touching the 128KB reverse map.
    void InvalidateIfCached(uint16_t address)
    {
        // Fast reject: check coarse bitmap (one bit per 256-byte page).
        // This 32-byte array stays in L1 cache. Almost all writes bail here.
        if (!(pageBitmap_[address >> 11] & (1 << ((address >> 8) & 7))))
            return;

        // Slow path: a block exists on this page. Use reverse map.
        uint16_t block_pc = reverseMap_[address];
        if (block_pc == NO_BLOCK)
            return;

        CachedBlock& b = blocks_[block_pc & CACHE_MASK];
        if (b.generation == generation_ && b.start_pc == block_pc)
        {
            ClearReverseMap(b.start_pc, b.end_pc);
            b.generation = 0;  // invalidate
            stats_.invalidations++;
        }
        else
        {
            // Stale reverse map entry — block was already evicted or generation changed.
            reverseMap_[address] = NO_BLOCK;
        }
    }

    // Begin recording a new block at the given PC.
    void BeginRecord(uint16_t pc)
    {
        recording_ = true;
        rec_start_pc_ = pc;
        rec_insn_count_ = 0;
        rec_cycle_start_ = 0;  // caller sets this
    }

    // Called after each instruction during recording.
    // Returns true if the block should continue, false if it should end.
    bool RecordInstruction(uint16_t next_pc, int cycle_counter)
    {
        if (!recording_) return false;

        rec_insn_count_++;

        if (rec_insn_count_ >= MAX_BLOCK_INSNS)
        {
            // Max size reached, end the block here
            FinishRecord(next_pc, cycle_counter);
            return false;
        }
        return true;
    }

    // End recording due to a block terminator (branch, jump, etc.).
    void EndRecord(uint16_t next_pc, int cycle_counter)
    {
        if (recording_)
            FinishRecord(next_pc, cycle_counter);
    }

    // Cancel recording (e.g., if an interrupt fires mid-block).
    void CancelRecord()
    {
        if (recording_)
            stats_.record_cancels++;
        recording_ = false;
    }

    bool IsRecording() const { return recording_; }

    void SetCycleStart(int cycles) { rec_cycle_start_ = cycles; }

    // Stats: call from the execution loop
    void RecordBlockHit(int num_insns) { stats_.block_hits++; stats_.block_insns += num_insns; }
    void RecordSingleStep() { stats_.single_steps++; }

    // Return accumulated stats and reset counters.
    BlockCacheStats GetAndResetStats()
    {
        BlockCacheStats s = stats_;
        memset(&stats_, 0, sizeof(stats_));
        return s;
    }

    // Invalidate a cache entry by PC.
    void Invalidate(uint16_t pc)
    {
        CachedBlock& b = blocks_[pc & CACHE_MASK];
        if (b.generation == generation_ && b.start_pc == pc)
        {
            ClearReverseMap(b.start_pc, b.end_pc);
            b.generation = 0;
            stats_.invalidations++;
        }
    }

    // Invalidate all entries (e.g., on MMU change).
    // O(1): just bump the generation counter. All existing blocks become
    // stale instantly. Reverse map and bitmap entries self-clean lazily.
    void InvalidateAll()
    {
        generation_++;
        stats_.bulk_invalidations++;
        // On the rare wraparound, do a full clear
        if (generation_ == 0)
            Clear();
    }

private:
    CachedBlock blocks_[CACHE_SIZE];

    // Reverse map: for each address in the 64KB logical space, stores the
    // start_pc of the cached block covering that address, or NO_BLOCK.
    // This gives O(1) lookup on memory writes.
    uint16_t reverseMap_[65536];

    // Coarse page bitmap: one bit per 256-byte page (32 bytes total).
    // Indexed as pageBitmap_[address >> 11] & (1 << ((address >> 8) & 7)).
    // Fits in a single L1 cache line for fast rejection of data-page writes.
    uint8_t pageBitmap_[32];

    // Generation counter for O(1) bulk invalidation.
    // Blocks are valid only if their generation matches this value.
    uint32_t generation_ = 1;

    // Recording state
    bool recording_ = false;
    uint16_t rec_start_pc_ = 0;
    int rec_insn_count_ = 0;
    int rec_cycle_start_ = 0;

    // Performance counters (reset by GetAndResetStats)
    BlockCacheStats stats_ = {};

    // Set a bit in the page bitmap for the given address.
    void SetPageBit(uint16_t address)
    {
        pageBitmap_[address >> 11] |= (1 << ((address >> 8) & 7));
    }

    // Clear page bitmap bits for a block's range.
    // Note: we could leave stale bits (safe, just causes unnecessary reverse
    // map lookups). But clearing is cheap for small blocks.
    void ClearPageBitmap(uint16_t start_pc, uint16_t end_pc)
    {
        // Only clear if no other block covers the same page.
        // For simplicity, just clear — the bitmap will be rebuilt as
        // blocks are re-cached. A stale '1' bit is safe (causes a reverse
        // map check that returns NO_BLOCK).
        // Actually, don't clear — stale bits are safe and avoiding the
        // scan is faster. The bitmap gets reset on InvalidateAll().
        (void)start_pc;
        (void)end_pc;
    }

    // Mark the reverse map for a block's address range.
    void SetReverseMap(uint16_t start_pc, uint16_t end_pc)
    {
        for (uint16_t a = start_pc; a != end_pc; a++)
            reverseMap_[a] = start_pc;
    }

    // Clear the reverse map for a block's address range.
    void ClearReverseMap(uint16_t start_pc, uint16_t end_pc)
    {
        for (uint16_t a = start_pc; a != end_pc; a++)
        {
            if (reverseMap_[a] == start_pc)
                reverseMap_[a] = NO_BLOCK;
        }
    }

    bool ValidateDecodedBlock(uint16_t start_pc, int num_insns,
                              const DecodedInst* insns, uint16_t decode_end_pc) const
    {
        if (num_insns <= 0 || num_insns > MAX_BLOCK_INSNS)
            return false;

        uint16_t pc = start_pc;
        for (int i = 0; i < num_insns; i++)
        {
            if (insns[i].handler == nullptr || insns[i].length == 0)
                return false;
            pc = (uint16_t)(pc + insns[i].length);
        }

        return pc == decode_end_pc;
    }

    void FinishRecord(uint16_t end_pc, int cycle_counter)
    {
        recording_ = false;
        int total_cycles = cycle_counter - rec_cycle_start_;
        (void)end_pc;

        // Only cache if we got useful data
        if (rec_insn_count_ > 0 && total_cycles > 0 && total_cycles < 256)
        {
            // Pre-decode instructions for the block execution path.
            DecodedInst decoded[MAX_BLOCK_INSNS];
            uint16_t decode_end_pc = DecodeBlock(rec_start_pc_, rec_insn_count_, decoded);
            if (!ValidateDecodedBlock(rec_start_pc_, rec_insn_count_, decoded, decode_end_pc))
            {
                stats_.rejected_blocks++;
                return;
            }

            CachedBlock& old = blocks_[rec_start_pc_ & CACHE_MASK];

            // If replacing a valid block, clear its reverse map first.
            if (old.generation == generation_)
                ClearReverseMap(old.start_pc, old.end_pc);

            old.start_pc = rec_start_pc_;
            old.end_pc = decode_end_pc;
            old.num_insns = (uint8_t)rec_insn_count_;
            old.total_cycles = (uint8_t)total_cycles;
            old.generation = generation_;
            memcpy(old.insns, decoded, sizeof(decoded));

            SetReverseMap(rec_start_pc_, decode_end_pc);
            stats_.blocks_recorded++;

            // Mark page bitmap for all pages this block spans
            for (uint16_t a = rec_start_pc_; a < decode_end_pc; a += 256)
                SetPageBit(a);
            SetPageBit((uint16_t)(decode_end_pc - 1));  // catch last page if block spans boundary
        }
    }
};
