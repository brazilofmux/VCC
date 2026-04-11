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
        uint16_t* dp;             // dp.Reg (16-bit; high byte is the DP page,
                                  //         low byte zero. LDA/LDB/STA/STB
                                  //         direct-page handlers compute the
                                  //         effective address as dp.Reg | off)
        uint8_t*  cc;             // base of cc[8] - cc[C/V/Z/N] are bytes 0..3
        int*      cycle_counter;  // CycleCounter
        // Runtime cycle-cost bytes: NatEmuCyclesNN gets updated by the
        // mode-switch path when the CPU flips between 6809 emulation
        // mode and 6309 native mode. Inlined handlers that use these
        // costs read the live byte at runtime via movzx + add.
        uint8_t*  nat_cycles_21;  // NatEmuCycles21 (CLRA/CLRB, most inherent ops)
        uint8_t*  nat_cycles_31;  // NatEmuCycles31 (ABX)
        uint8_t*  nat_cycles_43;  // NatEmuCycles43 (used by *_D handlers)
        uint8_t*  nat_cycles_54;  // NatEmuCycles54 (used by *_E handlers and LDY/LDS #imm)
        // Memory access entrypoints used by inlined 8-bit load/store
        // handlers. The emitter bakes call rel32 targets against these
        // addresses so inline bodies can talk to the MMU without
        // falling back through the interpreter handler. Both use the
        // standard (__cdecl) calling convention to match the handler
        // signatures in tcc1014mmu.cpp.
        unsigned char (*mem_read8)(unsigned short);
        void (*mem_write8)(unsigned char, unsigned short);
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
        // 8-bit memory loads/stores. These inline bodies still call
        // MemRead8/MemWrite8 to honor MMU paging and port I/O, but they
        // eliminate the handler dispatch overhead, bake the operand as
        // an immediate, and (because they do not read PC) let the
        // emitter skip the per-insn PC flush.
        InstHandler lda_d;        // LDA <dp    (op 0x96, +NatEmuCycles43 runtime)
        InstHandler ldb_d;        // LDB <dp    (op 0xD6, +NatEmuCycles43 runtime)
        InstHandler sta_d;        // STA <dp    (op 0x97, +NatEmuCycles43 runtime)
        InstHandler stb_d;        // STB <dp    (op 0xD7, +NatEmuCycles43 runtime)
        InstHandler lda_e;        // LDA ext    (op 0xB6, +NatEmuCycles54 runtime)
        InstHandler ldb_e;        // LDB ext    (op 0xF6, +NatEmuCycles54 runtime)
        InstHandler sta_e;        // STA ext    (op 0xB7, +NatEmuCycles54 runtime)
        InstHandler stb_e;        // STB ext    (op 0xF7, +NatEmuCycles54 runtime)
        // Inherent (accumulator) ops. These have small bodies and map
        // cleanly onto x86 flag-setting instructions plus setcc
        // sequences - the first use of runtime-CC emission, which is
        // the pattern we will reuse for compares and eventually lazy-
        // CC. All use NatEmuCycles21 except ABX which uses
        // NatEmuCycles31.
        InstHandler tsta_i;       // TSTA (0x4D)
        InstHandler tstb_i;       // TSTB (0x5D)
        InstHandler inca_i;       // INCA (0x4C)
        InstHandler incb_i;       // INCB (0x5C)
        InstHandler deca_i;       // DECA (0x4A)
        InstHandler decb_i;       // DECB (0x5A)
        InstHandler coma_i;       // COMA (0x43)
        InstHandler comb_i;       // COMB (0x53)
        InstHandler nega_i;       // NEGA (0x40)
        InstHandler negb_i;       // NEGB (0x50)
        InstHandler lsra_i;       // LSRA (0x44)
        InstHandler asra_i;       // ASRA (0x47)
        InstHandler asla_i;       // ASLA / LSLA (0x48)
        InstHandler abx_i;        // ABX  (0x3A)
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
        // cc[] write counters - updated as inlined ops emit. "Requested"
        // is what the op would write if DSE were off; "elided" is the
        // subset that the liveness pass proved dead. emitted = requested
        // - elided. Ratio elided/requested tells us how effective the
        // dead-store elimination pass actually is in practice.
        uint32_t cc_writes_requested;
        uint32_t cc_writes_elided;
    };
    Stats GetStats();
}
