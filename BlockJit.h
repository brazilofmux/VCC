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

#include <cstddef>
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
        // Base of the contiguous Hd6309State struct that every field
        // pointer below points into. Backends that pin a base register
        // (arm64) derive each field's offset as (field - base); the
        // x86-32 backend bakes the absolute addresses and ignores this.
        void*     base;
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
        // Appended for the indexed-mode inline family (arm64); the
        // x86-32 backend ignores everything from here down.
        uint8_t*  nat_cycles_32;  // indexed ,R++ / ,--R modes
        uint8_t*  nat_cycles_53;  // indexed n16,PCR mode
        uint8_t*  nat_cycles_65;  // TST indexed
        unsigned short (*mem_read16)(unsigned short);
        void (*mem_write16)(unsigned short, unsigned short);

        // Block-linking (chain stub) context. The arm64 backend emits
        // one shared stub that lets a thunk tail-jump straight into the
        // next block's thunk when the dispatcher would have done nothing
        // but ceremony: it rechecks the cycle budget, pending interrupt/
        // sync/halt state, and the cache slot (tag + generation +
        // native_entry) before jumping - so a cleared native_entry or a
        // generation bump severs every link with no patching. A null
        // chain_slot_base disables linking entirely (x86 backend, or
        // cores that don't populate it).
        int*           cycle_for;        // &cpu_state.ChainCycleFor (budget)
        // Maintained deliverability byte: nonzero when the dispatcher
        // would act right now (deliverable interrupt per the CC masks,
        // SYNC wait, or halted-insn). Masked-but-asserted lines do NOT
        // set it. One byte load answers "any reason to leave the chain".
        unsigned char* chain_break;      // &cpu_state.ChainBreak
        const uint32_t* generation_mirror; // &cpu_state.ChainGeneration
        void*          chain_slot_base;  // &blockCache.blocks_[0]
        uint32_t       chain_slot_size;  // sizeof(CachedBlock)
        uint64_t*      chain_runs;       // stub transitions (VCC_JIT_STATS)
    };

    // Budget overshoot allowance, shared by the dispatcher's fits-check
    // and the chain stub's. A block may start while (total_cycles -
    // slack) still fits the remaining slice budget; CPUCycle's drift
    // accounting absorbs the overshoot exactly (the same mechanism that
    // always absorbed last-instruction overshoot), so average timing is
    // unchanged and event jitter stays under half a scanline. Without
    // slack, large blocks (superblock traces especially) fail the check
    // near every slice end and the tail runs through the interpreter.
    inline constexpr int kBudgetSlack = 48;

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
        // Branch handlers (terminators). The arm64 backend inlines
        // these as the block's final instruction: both successor PCs
        // are emit-time constants, so a branch becomes a few cc-byte
        // loads and a PC store with no handler call. Appended after
        // the original fields so the x86-32 backend (which doesn't
        // know them and leaves them on the call path) is unaffected.
        InstHandler bra_r;        // BRA  (0x20) always
        InstHandler brn_r;        // BRN  (0x21) never
        InstHandler bhi_r;        // BHI  (0x22) !(C|Z)
        InstHandler bls_r;        // BLS  (0x23) C|Z
        InstHandler bhs_r;        // BHS  (0x24) !C
        InstHandler blo_r;        // BLO  (0x25) C
        InstHandler bne_r;        // BNE  (0x26) !Z
        InstHandler beq_r;        // BEQ  (0x27) Z
        InstHandler bvc_r;        // BVC  (0x28) !V
        InstHandler bvs_r;        // BVS  (0x29) V
        InstHandler bpl_r;        // BPL  (0x2A) !N
        InstHandler bmi_r;        // BMI  (0x2B) N
        InstHandler bge_r;        // BGE  (0x2C) !(N^V)
        InstHandler blt_r;        // BLT  (0x2D) N^V
        InstHandler bgt_r;        // BGT  (0x2E) !(Z|(N^V))
        InstHandler ble_r;        // BLE  (0x2F) Z|(N^V)
        InstHandler lbra_r;       // LBRA (0x16) always, NatEmuCycles54
        InstHandler lbne_r;       // LBNE (0x1026) !Z, 5+1
        InstHandler lbeq_r;       // LBEQ (0x1027) Z, 5+1

        // Immediate ALU family (also appended; x86-32 backend leaves
        // them on the call path).
        InstHandler anda_m;       // ANDA # (0x84)  Z,N from result, V=0
        InstHandler andb_m;       // ANDB # (0xC4)
        InstHandler ora_m;        // ORA  # (0x8A)
        InstHandler orb_m;        // ORB  # (0xCA)
        InstHandler eora_m;       // EORA # (0x88)
        InstHandler eorb_m;       // EORB # (0xC8)
        InstHandler bita_m;       // BITA # (0x85)  like AND, no writeback
        InstHandler bitb_m;       // BITB # (0xC5)
        InstHandler cmpa_m;       // CMPA # (0x81)  C,V,Z,N; no writeback
        InstHandler cmpb_m;       // CMPB # (0xC1)
        InstHandler suba_m;       // SUBA # (0x80)  C,V,Z,N + A
        InstHandler subb_m;       // SUBB # (0xC0)
        InstHandler adda_m;       // ADDA # (0x8B)  C,H,V,Z,N + A
        InstHandler addb_m;       // ADDB # (0xCB)

        // Indexed-mode family (arm64 backend; ~28% of executed
        // instructions under OS-9). The EA mode/register/offset are
        // pre-decoded into DecodedInst, so the emitter resolves the
        // addressing mode at emit time and inlines the direct modes.
        InstHandler leax_x;       // LEAX (0x30)  X=EA, Z
        InstHandler leay_x;       // LEAY (0x31)  Y=EA, Z
        InstHandler leau_x;       // LEAU (0x33)  U=EA, no flags
        InstHandler leas_x;       // LEAS (0x32)  S=EA, no flags
        InstHandler lda_x;        // LDA  (0xA6)
        InstHandler ldb_x;        // LDB  (0xE6)
        InstHandler sta_x;        // STA  (0xA7)
        InstHandler stb_x;        // STB  (0xE7)
        InstHandler tst_x;        // TST  (0x6D)
        InstHandler jmp_x;        // JMP  (0x6E)  terminator
        InstHandler ldx_x;        // LDX  (0xAE)
        InstHandler ldu_x;        // LDU  (0xEE)
        InstHandler ldd_x;        // LDD  (0xEC)
        InstHandler std_x;        // STD  (0xED)
        InstHandler stx_x;        // STX  (0xAF)

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

    // Registerized block state: arm64 thunks keep CycleCounter in w21
    // (with the dispatch budget in w22) across whole chains, so they
    // must be entered through this runner, which syncs the registers
    // with cpu_state on the way in and out. Null when the backend does
    // not registerize (x86-32, null backend) - call native_entry
    // directly in that case. Re-query after Reset: the runner is
    // re-emitted with the arena.
    using ThunkRunner = void (*)(NativeEntry);
    ThunkRunner GetThunkRunner();

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

    // Diagnostic: whether the most recent EmitBlock call produced a
    // side-effect-free thunk (no MemRead8/MemWrite8, no handler calls).
    // The null and x86-32 backends report false.
    bool EmitBlockWasPure();
}
