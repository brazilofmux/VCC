#pragma once
/*
Copyright 2026 by the VCC Project Contributors.
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

// BlockJit: level-1 call-handler trampoline emitter for the block cache.
//
// Each CachedBlock optionally gets a native x86 thunk that replaces the
// per-instruction dispatch loop. The thunk is a straight sequence of
// __cdecl handler calls, one per instruction in the block, with PC_REG
// pre-set to the post-instruction value as a baked immediate. Handlers
// themselves are unchanged - they still take const DecodedInst* and
// update CycleCounter from inside.
//
// The block model already handles all the hard correctness problems
// (re-entry mid-instruction, banking, self-modifying code, interrupt
// boundaries between blocks). JIT is purely additive: when a block has
// a native_entry, the dispatch loop calls it; otherwise it falls back
// to the existing interpreter loop. There's no flag day.
//
// Scope at level-1: ROM blocks only (those inserted via the
// pre-population path). Runtime-recorded blocks fall through to the
// interpreter so we don't have to deal with arena churn yet.

#include <cstdint>

struct CachedBlock;

namespace BlockJit
{
    using NativeEntry = void (*)(void);

    // Set up the code arena and remember where PC_REG lives so the
    // emitter can bake its address into mov instructions. Must be
    // called once at CPU init, before any EmitBlock call. Calling
    // again is idempotent and resets the arena.
    void Init(uint16_t* pc_reg_addr);

    // Reset the bump allocator. After this, every previously-emitted
    // thunk is invalid - the caller is responsible for clearing all
    // CachedBlock::native_entry pointers that referenced the arena
    // BEFORE calling Reset (otherwise the dispatch loop will jump into
    // freshly-overwritten arena memory).
    void Reset();

    // Emit a level-1 thunk for the given (already-inserted) cache slot.
    // The thunk bakes &slot.insns[i] as immediate operands, so the slot
    // must have stable storage - you cannot pass a stack-local
    // CachedBlock here. Returns nullptr if the arena is exhausted; the
    // caller should leave native_entry as nullptr in that case so the
    // block falls back to the interpreter dispatch loop.
    NativeEntry EmitBlock(const CachedBlock& slot);

    // Diagnostics.
    struct Stats
    {
        size_t  arena_size;
        size_t  arena_used;
        uint32_t blocks_emitted;
        uint32_t emit_failures;
    };
    Stats GetStats();
}
