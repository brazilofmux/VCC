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
#include "DecodedInst.h"

struct CachedBlock;

namespace BlockJit
{
    using NativeEntry = void (*)(void);

    // CPU state addresses the level-2 inline emitters need to bake
    // into mov/add instructions. Everything here is a static in
    // hd6309.cpp; the JIT needs the bare addresses, not the macros.
    struct CpuAddrs
    {
        uint16_t* pc;             // PC_REG
        uint8_t*  a;              // A_REG
        uint8_t*  b;              // B_REG
        uint16_t* d;              // D_REG (overlays A:B as a 16-bit value)
        uint16_t* x;              // X_REG
        uint16_t* y;              // Y_REG
        uint16_t* u;              // U_REG
        uint16_t* s;              // S_REG
        uint8_t*  cc;             // base of cc[8] - cc[C/V/Z/N] are bytes 0..3
        int*      cycle_counter;  // CycleCounter
        // Runtime cycle-cost bytes: NatEmuCyclesNN gets updated by the
        // mode-switch path when the CPU flips between 6809 emulation
        // mode and 6309 native mode. Inlined handlers that use these
        // costs read the live byte at runtime via movzx + add.
        uint8_t*  nat_cycles_21;  // NatEmuCycles21 (used by CLRA/CLRB and others)
        uint8_t*  nat_cycles_54;  // NatEmuCycles54 (used by LDY/LDS #imm)
    };

    // Handler addresses the level-2 emitter knows how to inline. The
    // JIT compares each DecodedInst's handler pointer against these and
    // emits a precomputed body when it matches; otherwise it falls
    // through to the level-1 call-handler trampoline. Adding a new
    // inlineable handler means adding a slot here, populating it from
    // hd6309.cpp at Init time, and teaching EmitBlock to recognize and
    // emit it.
    struct InlineableHandlers
    {
        InstHandler lda_m;        // LDA #imm   (op 0x86, +2 cycles - constant)
        InstHandler ldb_m;        // LDB #imm   (op 0xC6, +2 cycles - constant)
        InstHandler ldd_m;        // LDD #imm   (op 0xCC, +3 cycles - constant)
        InstHandler ldx_m;        // LDX #imm   (op 0x8E, +3 cycles - constant)
        InstHandler ldu_m;        // LDU #imm   (op 0xCE, +3 cycles - constant)
        InstHandler lds_i;        // LDS #imm   (op 0x10CE, +4 cycles - constant)
        InstHandler ldy_m;        // LDY #imm   (op 0x108E, +NatEmuCycles54 runtime)
        InstHandler clra_i;       // CLRA       (op 0x4F, +NatEmuCycles21 runtime)
        InstHandler clrb_i;       // CLRB       (op 0x5F, +NatEmuCycles21 runtime)
    };

    // Set up the code arena and remember the addresses the emitter
    // bakes into instructions. Must be called once at CPU init, before
    // any EmitBlock call. Calling again is idempotent and resets the
    // arena and stats.
    void Init(const CpuAddrs& addrs, const InlineableHandlers& handlers);

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
        uint32_t insns_called;       // emitted as a __cdecl handler call
        uint32_t insns_inlined;      // emitted as a level-2 inline body
        uint32_t pc_writes_emitted;  // emit count of "mov [PC], imm16"
        uint32_t pc_writes_skipped;  // PC writes elided thanks to inlining
    };
    Stats GetStats();
}
