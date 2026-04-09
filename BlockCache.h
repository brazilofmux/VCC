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
// The cache is direct-mapped by PC (logical address). MMU changes or writes
// to code pages should invalidate affected entries.

#include <cstdint>
#include <cstring>

struct CachedBlock
{
    uint16_t start_pc;      // logical PC where this block starts
    uint16_t end_pc;        // PC after the last instruction in the block
    uint8_t  num_insns;     // number of instructions in the block
    uint8_t  total_cycles;  // total cycle cost of the block
    bool     valid;         // is this entry populated?
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

    BlockCache() { Clear(); }

    void Clear()
    {
        memset(blocks_, 0, sizeof(blocks_));
    }

    // Look up a block by its starting PC. Returns nullptr if not cached.
    const CachedBlock* Lookup(uint16_t pc) const
    {
        const CachedBlock& b = blocks_[pc & CACHE_MASK];
        if (b.valid && b.start_pc == pc)
            return &b;
        return nullptr;
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
        recording_ = false;
    }

    bool IsRecording() const { return recording_; }

    void SetCycleStart(int cycles) { rec_cycle_start_ = cycles; }

    // Invalidate a cache entry by PC.
    void Invalidate(uint16_t pc)
    {
        CachedBlock& b = blocks_[pc & CACHE_MASK];
        if (b.valid && b.start_pc == pc)
            b.valid = false;
    }

    // Invalidate all entries (e.g., on MMU change).
    void InvalidateAll()
    {
        for (int i = 0; i < CACHE_SIZE; i++)
            blocks_[i].valid = false;
    }

private:
    CachedBlock blocks_[CACHE_SIZE];

    // Recording state
    bool recording_ = false;
    uint16_t rec_start_pc_ = 0;
    int rec_insn_count_ = 0;
    int rec_cycle_start_ = 0;

    void FinishRecord(uint16_t end_pc, int cycle_counter)
    {
        recording_ = false;
        int total_cycles = cycle_counter - rec_cycle_start_;

        // Only cache if we got useful data
        if (rec_insn_count_ > 0 && total_cycles > 0 && total_cycles < 256)
        {
            CachedBlock& b = blocks_[rec_start_pc_ & CACHE_MASK];
            b.start_pc = rec_start_pc_;
            b.end_pc = end_pc;
            b.num_insns = (uint8_t)rec_insn_count_;
            b.total_cycles = (uint8_t)total_cycles;
            b.valid = true;
        }
    }
};
