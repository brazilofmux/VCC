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

// AArch64 backend for the Level-1 call-handler trampoline emitter
// (see BlockJit.h for the contract and BlockJit.cpp for the x86-32
// original). Same shape as the x86 backend: each CachedBlock gets a
// native thunk that pre-sets PC and calls the instruction handlers in
// sequence, with the block model handling every hard correctness
// problem. Level-2 inlining (with the cc[] liveness DSE) lands here
// incrementally; until an op is inlined it goes through the call path.
//
// Thunk shape (AAPCS64):
//   stp  x29, x30, [sp, #-32]!      ; frame - the thunk makes calls
//   stp  x19, x20, [sp, #16]
//   mov  x29, sp
//   x19 = &cpu_state                ; every field is [x19, #imm]
//   x20 = &slot.insns[0]            ; operands are x20 + i*sizeof
//   per instruction:
//     movz w1, #post_insn_pc
//     strh w1, [x19, #off_pc]
//     add  x0, x20, #(i * sizeof(DecodedInst))
//     x16 = handler; blr x16        ; BL's ±128MB can't reach a MAP_JIT
//                                   ; arena reliably, so BLR through x16
//   ldp  x19, x20, [sp, #16]
//   ldp  x29, x30, [sp], #32
//   ret
//
// macOS arm64 W^X: the arena is MAP_JIT and emission is bracketed with
// pthread_jit_write_protect_np; the icache is invalidated per block
// (mandatory on ARM - the icache is not coherent with data writes).

#include "BlockJit.h"
#include "BlockCache.h"
#include "emit_a64.h"
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <sys/mman.h>
#ifdef __APPLE__
#include <pthread.h>
#endif

namespace BlockJit
{

// Same 16 MB arena budget as the x86 backend. arm64 thunks are ~1.5x
// the byte size (fixed 4-byte instructions), still comfortably inside.
static constexpr size_t kArenaSize = 16 * 1024 * 1024;

static uint8_t*           g_arena_base   = nullptr;
static size_t             g_arena_used   = 0;
static CpuAddrs           g_addrs        {};
static InlineableHandlers g_inlines      {};
static bool               g_disabled     = false;
static bool               g_no_inline    = false;
static int                g_inline_max   = 999;
static uint32_t           g_blocks_emitted = 0;
static uint32_t           g_emit_failures  = 0;
static uint32_t           g_insns_called   = 0;
static uint32_t           g_insns_inlined  = 0;
static uint32_t           g_pc_writes_emitted = 0;
static uint32_t           g_pc_writes_skipped = 0;
static uint32_t           g_cc_writes_requested = 0;
static uint32_t           g_cc_writes_elided    = 0;
static bool               g_emit_had_side_effects = false;   // diag: see EmitBlockWasPure

// Block linking: one shared chain stub per arena epoch (see
// EnsureChainInfra below). Null means "not emitted yet"; g_no_link
// disables linking entirely (env kill switch or a verify mode).
// g_thunk_runner is the registerized-state entry trampoline - thunks
// keep CycleCounter in w21, so they may ONLY be entered through it.
static uint8_t*           g_chain_stub = nullptr;
static uint8_t*           g_thunk_runner = nullptr;
static bool               g_no_link    = false;

// Field offsets from CpuAddrs.base, computed once at Init. Every one
// fits the unsigned-immediate load/store forms with room to spare.
static uint32_t g_off_pc  = 0;
static uint32_t g_off_a   = 0;
static uint32_t g_off_b   = 0;
static uint32_t g_off_d   = 0;
static uint32_t g_off_x   = 0;
static uint32_t g_off_y   = 0;
static uint32_t g_off_u   = 0;
static uint32_t g_off_s   = 0;
static uint32_t g_off_dp  = 0;
static uint32_t g_off_cc  = 0;   // cc[C]=+0, cc[V]=+1, cc[Z]=+2, cc[N]=+3
static uint32_t g_off_cyc = 0;
static uint32_t g_off_nat21 = 0;
static uint32_t g_off_nat31 = 0;
static uint32_t g_off_nat43 = 0;
static uint32_t g_off_nat54 = 0;
static uint32_t g_off_nat32 = 0;
static uint32_t g_off_nat53 = 0;
static uint32_t g_off_nat65 = 0;
static uint32_t g_off_nat64 = 0;
static uint32_t g_off_scratch16 = 0;
static uint32_t g_off_nat76 = 0;
// RAM fast path (bank tables + watch bitmap); see EmitMemRead8FP.
static bool g_fastmem = false;
static int64_t  g_off_pending = -1;   // packed pending quadword in cpu_state

// ---------- cc[] bit ordering (matches BlockJit.cpp / hd6309.cpp) ----------

static constexpr uint8_t CC_BIT_C = 1u << 0;
static constexpr uint8_t CC_BIT_V = 1u << 1;
static constexpr uint8_t CC_BIT_Z = 1u << 2;
static constexpr uint8_t CC_BIT_N = 1u << 3;
static constexpr uint8_t CC_ALL   = CC_BIT_C | CC_BIT_V | CC_BIT_Z | CC_BIT_N;
static constexpr uint8_t CC_UNKNOWN = 0xFF;

static inline void jit_writable_begin()
{
#ifdef __APPLE__
    pthread_jit_write_protect_np(0);
#endif
}

static inline void jit_writable_end()
{
#ifdef __APPLE__
    pthread_jit_write_protect_np(1);
#endif
}

void Init(const CpuAddrs& addrs, const InlineableHandlers& handlers)
{
    g_addrs = addrs;
    g_inlines = handlers;

    // Kill switch for A/B runs and emitter debugging: with the JIT
    // refused, every block falls back to the threaded interpreter.
    g_disabled = std::getenv("VCC_NO_JIT") != nullptr;
    g_no_inline = std::getenv("VCC_NO_INLINE") != nullptr;
    if (const char* im = std::getenv("VCC_INLINE_MAX")) g_inline_max = std::atoi(im);
    // Linking must be off under the differential verifiers: they run
    // ONE block from a snapshot and compare, and a chained thunk would
    // run its successors too before returning.
    g_no_link = std::getenv("VCC_NO_LINK") != nullptr ||
                std::getenv("VCC_VERIFY_PC") != nullptr ||
                std::getenv("VCC_VERIFY_PURE") != nullptr;

    if (g_arena_base == nullptr && !g_disabled)
    {
        int flags = MAP_PRIVATE | MAP_ANONYMOUS;
#ifdef MAP_JIT
        flags |= MAP_JIT;
#endif
        void* mem = mmap(nullptr, kArenaSize, PROT_READ | PROT_WRITE | PROT_EXEC,
                         flags, -1, 0);
        g_arena_base = (mem == MAP_FAILED) ? nullptr : (uint8_t*)mem;
    }

    if (g_addrs.base != nullptr && g_addrs.pc != nullptr)
    {
        const uint8_t* base = (const uint8_t*)g_addrs.base;
        g_off_pc    = (uint32_t)((uint8_t*)g_addrs.pc            - base);
        g_off_a     = (uint32_t)((uint8_t*)g_addrs.a             - base);
        g_off_b     = (uint32_t)((uint8_t*)g_addrs.b             - base);
        g_off_d     = (uint32_t)((uint8_t*)g_addrs.d             - base);
        g_off_x     = (uint32_t)((uint8_t*)g_addrs.x             - base);
        g_off_y     = (uint32_t)((uint8_t*)g_addrs.y             - base);
        g_off_u     = (uint32_t)((uint8_t*)g_addrs.u             - base);
        g_off_s     = (uint32_t)((uint8_t*)g_addrs.s             - base);
        g_off_dp    = (uint32_t)((uint8_t*)g_addrs.dp            - base);
        g_off_cc    = (uint32_t)((uint8_t*)g_addrs.cc            - base);
        g_off_cyc   = (uint32_t)((uint8_t*)g_addrs.cycle_counter - base);
        g_off_nat21 = (uint32_t)((uint8_t*)g_addrs.nat_cycles_21 - base);
        g_off_nat31 = (uint32_t)((uint8_t*)g_addrs.nat_cycles_31 - base);
        g_off_nat43 = (uint32_t)((uint8_t*)g_addrs.nat_cycles_43 - base);
        g_off_nat54 = (uint32_t)((uint8_t*)g_addrs.nat_cycles_54 - base);
        g_off_nat32 = (uint32_t)((uint8_t*)g_addrs.nat_cycles_32 - base);
        g_off_nat53 = (uint32_t)((uint8_t*)g_addrs.nat_cycles_53 - base);
        g_off_nat65 = (uint32_t)((uint8_t*)g_addrs.nat_cycles_65 - base);
        g_off_nat64 = (uint32_t)((uint8_t*)g_addrs.nat_cycles_64 - base);
        g_off_nat76 = (uint32_t)((uint8_t*)g_addrs.nat_cycles_76 - base);
        g_off_scratch16 = (uint32_t)((uint8_t*)g_addrs.jit_scratch16 - base);
        g_fastmem = g_addrs.fastmem_read_banks != nullptr &&
                    g_addrs.fastmem_write_banks != nullptr &&
                    g_addrs.fastmem_watch_bitmap != nullptr &&
                    std::getenv("VCC_NO_FASTMEM") == nullptr;
        g_off_pending = (g_addrs.chain_break != nullptr)
            ? (int64_t)((uint8_t*)g_addrs.chain_break - base) : -1;
        if (g_off_pending < 0 || g_off_pending >= 4096)
            g_off_pending = -1;
    }

    g_arena_used = 0;
    g_blocks_emitted = 0;
    g_emit_failures = 0;
    g_insns_called = 0;
    g_insns_inlined = 0;
    g_pc_writes_emitted = 0;
    g_pc_writes_skipped = 0;
    g_cc_writes_requested = 0;
    g_cc_writes_elided    = 0;
    g_chain_stub = nullptr;
    g_thunk_runner = nullptr;
}

void Reset()
{
    g_arena_used = 0;
    g_blocks_emitted = 0;
    g_emit_failures = 0;
    g_insns_called = 0;
    g_insns_inlined = 0;
    g_pc_writes_emitted = 0;
    g_pc_writes_skipped = 0;
    g_cc_writes_requested = 0;
    g_cc_writes_elided    = 0;
    g_chain_stub = nullptr;
    g_thunk_runner = nullptr;
}

// ---------- size budget (bytes of arm64 code) ----------

// Prologue: 3 frame insns + two imm64 materializations (up to 4 each).
static constexpr size_t kPrologueBytes = (3 + 4 + 4) * 4;
// Per instruction, worst case: a 16-bit arithmetic op with an indexed
// operand and every flag live runs the EA computation, an inlined
// two-byte RAM fast path with its slow-path fallback, and the manual
// flag math - about 60 instructions. 80 is a safe ceiling; the called
// path is still 8.
static constexpr size_t kMaxBytesPerInsn = 80 * 4;
// Epilogue: 2 ldp + ret, plus a final PC flush (movz+strh) when the
// block ends on an inlined op.
static constexpr size_t kEpilogueBytes = (2 + 1 + 2) * 4;

// ---------- small emit helpers ----------

// Registerized block state: CycleCounter lives in w21 for the entire
// thunk chain (the dispatch budget rides along in w22 for the chain
// stub's compares). The emitted runner (EnsureChainInfra) loads both
// on entry and spills w21 on exit; called handlers see the memory
// copy, so their call sites spill/reload around the BLR. Both are
// callee-saved, so mem_read/mem_write calls preserve them for free.

// CycleCounter += constant  (1 insn, was 3)
static void EmitCyclesConst(emit_t& e, uint32_t n)
{
    emit_add_w32_imm(&e, A64_W21, A64_W21, n);
}

// CycleCounter += NatEmuCyclesNN (live byte)  (2 insns, was 4)
static void EmitCyclesRuntime(emit_t& e, uint32_t nat_offset)
{
    emit_ldrb_imm(&e, A64_W1, A64_W19, nat_offset);
    emit_add_w32(&e, A64_W21, A64_W21, A64_W1);
}

// cc[idx] = 0 or 1  (1-2 insns)
static void EmitCcConst(emit_t& e, uint32_t idx, unsigned value)
{
    if (value == 0)
    {
        emit_strb_imm(&e, A64_WZR, A64_W19, g_off_cc + idx);
    }
    else
    {
        emit_movz_w32(&e, A64_W2, 1, 0);
        emit_strb_imm(&e, A64_W2, A64_W19, g_off_cc + idx);
    }
}


// AND-immediate with a register fallback when the value has no logical-
// immediate encoding. Never ignore emit_and_w32_imm's return value: a
// silently-skipped instruction is exactly the failure mode that hid the
// LSRA carry bug (see the UB note in emit_a64.h's encoder).
static void EmitAndImm(emit_t& e, a64_reg_t rd, a64_reg_t rn, uint32_t imm)
{
    if (!emit_and_w32_imm(&e, rd, rn, imm))
    {
        emit_movz_w32(&e, A64_W17, (uint16_t)imm, 0);
        emit_and_w32(&e, rd, rn, A64_W17);
    }
}

// OR/EOR-immediate with register fallback, same rationale as EmitAndImm.
static void EmitOrrImm(emit_t& e, a64_reg_t rd, a64_reg_t rn, uint32_t imm)
{
    if (!emit_orr_w32_imm(&e, rd, rn, imm))
    {
        emit_movz_w32(&e, A64_W17, (uint16_t)imm, 0);
        emit_orr_w32(&e, rd, rn, A64_W17);
    }
}

static void EmitEorImm(emit_t& e, a64_reg_t rd, a64_reg_t rn, uint32_t imm)
{
    if (!emit_eor_w32_imm(&e, rd, rn, imm))
    {
        emit_movz_w32(&e, A64_W17, (uint16_t)imm, 0);
        emit_eor_w32(&e, rd, rn, A64_W17);
    }
}

// Update cc[Z]/cc[N]/cc[V] from an 8-bit value in Wv (already masked
// to 0..255). Z: compare/cset; N: bit 7 shifted down; V: constant 0 -
// matching the interpreter's rules for loads/stores/TST.
static void EmitFlagsZNVFromReg(emit_t& e, a64_reg_t v, uint8_t mask)
{
    if (mask & CC_BIT_Z)
    {
        emit_cmp_w32_imm(&e, v, 0);
        emit_cset_w32(&e, A64_W2, A64_COND_EQ);
        emit_strb_imm(&e, A64_W2, A64_W19, g_off_cc + 2);
    }
    if (mask & CC_BIT_N)
    {
        emit_lsr_w32_imm(&e, A64_W2, v, 7);
        emit_strb_imm(&e, A64_W2, A64_W19, g_off_cc + 3);
    }
    if (mask & CC_BIT_V)
    {
        emit_strb_imm(&e, A64_WZR, A64_W19, g_off_cc + 1);
    }
}

// ---------- inline emitters ----------
// Each mirrors its interpreter handler exactly (and the x86-32 inline
// body in BlockJit.cpp); the flag_mask gates which cc[] stores are
// emitted, per the backward liveness pass below.

// LDA/LDB #imm - flags are compile-time constants of the immediate.
static void EmitInlineLd8Imm(emit_t& e, const DecodedInst& insn,
                             uint32_t reg_off, uint8_t mask)
{
    const uint8_t imm = (uint8_t)(insn.operand & 0xFF);
    emit_movz_w32(&e, A64_W1, imm, 0);
    emit_strb_imm(&e, A64_W1, A64_W19, reg_off);
    if (mask & CC_BIT_N) EmitCcConst(e, 3, (imm & 0x80) ? 1 : 0);
    if (mask & CC_BIT_Z) EmitCcConst(e, 2, (imm == 0) ? 1 : 0);
    if (mask & CC_BIT_V) EmitCcConst(e, 1, 0);
    EmitCyclesConst(e, 2);
}

// LDD/LDX/LDU #imm16 (constant cycles) and LDY/LDS (runtime cycles).
static void EmitInlineLd16Imm(emit_t& e, const DecodedInst& insn,
                              uint32_t reg_off, int const_cycles,
                              uint32_t nat_offset, uint8_t mask)
{
    const uint16_t imm = insn.operand;
    emit_movz_w32(&e, A64_W1, imm, 0);
    emit_strh_imm(&e, A64_W1, A64_W19, reg_off);
    if (mask & CC_BIT_N) EmitCcConst(e, 3, (imm & 0x8000) ? 1 : 0);
    if (mask & CC_BIT_Z) EmitCcConst(e, 2, (imm == 0) ? 1 : 0);
    if (mask & CC_BIT_V) EmitCcConst(e, 1, 0);
    if (const_cycles >= 0)
        EmitCyclesConst(e, (uint32_t)const_cycles);
    else
        EmitCyclesRuntime(e, nat_offset);
}

// CLRA/CLRB: register = 0, CC = fixed pattern C=0 V=0 Z=1 N=0.
static void EmitInlineClr8(emit_t& e, uint32_t reg_off, uint8_t mask)
{
    emit_strb_imm(&e, A64_WZR, A64_W19, reg_off);
    if (mask & CC_BIT_C) EmitCcConst(e, 0, 0);
    if (mask & CC_BIT_V) EmitCcConst(e, 1, 0);
    if (mask & CC_BIT_Z) EmitCcConst(e, 2, 1);
    if (mask & CC_BIT_N) EmitCcConst(e, 3, 0);
    EmitCyclesRuntime(e, g_off_nat21);
}

// ---------- guest memory access (RAM fast path) ----------
//
// Same design as the x86-64 backend: the MMU maintains 8 bank pointers
// (address >> 13); a null bank (I/O, ROM-as-write, cart space, the
// $FE00+ tail) falls back to the C helper. Stores additionally run
// InvalidateIfCached's fast-reject watch bitmap - a set bit means "a
// cached block may live in this 256-byte page" and the write takes the
// full MemWrite8 path so the real invalidation machinery runs. Bank
// pointers load indirectly at run time, so MMU rebuilds propagate to
// existing thunks with no re-emission.
//
// Register contract: W0 = data in (writes) / result out (reads),
// W1 = address (writes), or W0 = address in (reads). Scratch: W2-W4,
// X10-X12, X16. The pinned set (x19-x24) and W1/W5+ survive the fast
// path; a slow-path helper call clobbers all caller-saved registers.
// Slow paths sync w21 (CycleCounter) to cpu_state first so on-demand
// observers (scanline bursts, coco3.cpp) see exact guest time at every
// I/O access - the helpers never modify CycleCounter, so no reload.

static uint32_t HoleCbzX(emit_t& e, a64_reg_t r)
{
    const uint32_t at = e.offset;
    emit_cbz_x64(&e, r, 0);
    return at;
}
static uint32_t HoleCbnzW(emit_t& e, a64_reg_t r)
{
    const uint32_t at = e.offset;
    emit_cbnz_w32(&e, r, 0);
    return at;
}
static uint32_t HoleB(emit_t& e)
{
    const uint32_t at = e.offset;
    emit_b(&e, 0);
    return at;
}
static void PatchCbzX(emit_t& e, uint32_t at, a64_reg_t r)
{
    emit_t p { e.buf + at, 0, 4 };
    emit_cbz_x64(&p, r, (int32_t)(e.offset - at));
}
static void PatchCbnzW(emit_t& e, uint32_t at, a64_reg_t r)
{
    emit_t p { e.buf + at, 0, 4 };
    emit_cbnz_w32(&p, r, (int32_t)(e.offset - at));
}
static void PatchB(emit_t& e, uint32_t at)
{
    emit_t p { e.buf + at, 0, 4 };
    emit_b(&p, (int32_t)(e.offset - at));
}

// Watch-bitmap test for the page holding the address in `addr_reg`:
// leaves bit 0 of W2 = watched. Clobbers W2, W4; X12 must hold the
// bitmap base.
static void EmitWatchBit(emit_t& e, a64_reg_t addr_reg)
{
    emit_lsr_w32_imm(&e, A64_W2, addr_reg, 11);
    emit_ldrb_reg_uxtw(&e, A64_W2, A64_W12, A64_W2);
    emit_lsr_w32_imm(&e, A64_W4, addr_reg, 8);
    EmitAndImm(e, A64_W4, A64_W4, 7);
    emit_lsrv_w32(&e, A64_W2, A64_W2, A64_W4);
    emit_ubfx_w32(&e, A64_W2, A64_W2, 0, 1);
}

// W0 = MemRead8(W0). Fast: bank load + ldrb. Slow: helper call.
static void EmitMemRead8FP(emit_t& e)
{
    if (!g_fastmem)
    {
        emit_mov_x64_imm64(&e, A64_W16, (uint64_t)(uintptr_t)g_addrs.mem_read8);
        emit_blr(&e, A64_W16);
        return;
    }
    emit_lsr_w32_imm(&e, A64_W2, A64_W0, 13);
    emit_mov_x64_imm64(&e, A64_W10, (uint64_t)(uintptr_t)g_addrs.fastmem_read_banks);
    emit_ldr_x64_reg_lsl3(&e, A64_W10, A64_W10, A64_W2);
    const uint32_t to_slow = HoleCbzX(e, A64_W10);
    EmitAndImm(e, A64_W2, A64_W0, 0x1FFF);
    emit_ldrb_reg_uxtw(&e, A64_W0, A64_W10, A64_W2);
    const uint32_t to_done = HoleB(e);
    PatchCbzX(e, to_slow, A64_W10);
    emit_str_w32_imm(&e, A64_W21, A64_W19, g_off_cyc);
    emit_mov_x64_imm64(&e, A64_W16, (uint64_t)(uintptr_t)g_addrs.mem_read8);
    emit_blr(&e, A64_W16);
    PatchB(e, to_done);
}

// MemWrite8(W0, W1): data in W0's low byte, address in W1.
static void EmitMemWrite8FP(emit_t& e)
{
    if (!g_fastmem)
    {
        emit_mov_x64_imm64(&e, A64_W16, (uint64_t)(uintptr_t)g_addrs.mem_write8);
        emit_blr(&e, A64_W16);
        return;
    }
    uint32_t to_slow[2];
    emit_lsr_w32_imm(&e, A64_W2, A64_W1, 13);
    emit_mov_x64_imm64(&e, A64_W10, (uint64_t)(uintptr_t)g_addrs.fastmem_write_banks);
    emit_ldr_x64_reg_lsl3(&e, A64_W10, A64_W10, A64_W2);
    to_slow[0] = HoleCbzX(e, A64_W10);
    emit_mov_x64_imm64(&e, A64_W12, (uint64_t)(uintptr_t)g_addrs.fastmem_watch_bitmap);
    EmitWatchBit(e, A64_W1);
    to_slow[1] = HoleCbnzW(e, A64_W2);
    EmitAndImm(e, A64_W2, A64_W1, 0x1FFF);
    emit_strb_reg_uxtw(&e, A64_W0, A64_W10, A64_W2);
    const uint32_t to_done = HoleB(e);
    PatchCbzX(e, to_slow[0], A64_W10);
    PatchCbnzW(e, to_slow[1], A64_W2);
    emit_str_w32_imm(&e, A64_W21, A64_W19, g_off_cyc);
    emit_mov_x64_imm64(&e, A64_W16, (uint64_t)(uintptr_t)g_addrs.mem_write8);
    emit_blr(&e, A64_W16);
    PatchB(e, to_done);
}

// W0 = MemRead16(W0): big-endian pair, each byte's bank checked
// independently (a page-crossing pair is two lookups, exactly like the
// helper's two MemRead8 calls). Any miss falls back to the helper for
// the WHOLE pair - W0 is still the untouched address at both test
// points, so no state needs saving for the fallback.
static void EmitMemRead16FP(emit_t& e)
{
    if (!g_fastmem)
    {
        emit_mov_x64_imm64(&e, A64_W16, (uint64_t)(uintptr_t)g_addrs.mem_read16);
        emit_blr(&e, A64_W16);
        EmitAndImm(e, A64_W0, A64_W0, 0xFFFF);
        return;
    }
    uint32_t to_slow[2];
    emit_lsr_w32_imm(&e, A64_W2, A64_W0, 13);
    emit_mov_x64_imm64(&e, A64_W10, (uint64_t)(uintptr_t)g_addrs.fastmem_read_banks);
    emit_ldr_x64_reg_lsl3(&e, A64_W11, A64_W10, A64_W2);
    to_slow[0] = HoleCbzX(e, A64_W11);
    emit_add_w32_imm(&e, A64_W3, A64_W0, 1);
    EmitAndImm(e, A64_W3, A64_W3, 0xFFFF);
    emit_lsr_w32_imm(&e, A64_W2, A64_W3, 13);
    emit_ldr_x64_reg_lsl3(&e, A64_W10, A64_W10, A64_W2);
    to_slow[1] = HoleCbzX(e, A64_W10);
    EmitAndImm(e, A64_W2, A64_W0, 0x1FFF);
    emit_ldrb_reg_uxtw(&e, A64_W4, A64_W11, A64_W2);   // hi
    EmitAndImm(e, A64_W2, A64_W3, 0x1FFF);
    emit_ldrb_reg_uxtw(&e, A64_W0, A64_W10, A64_W2);   // lo
    emit_orr_w32_lsl(&e, A64_W0, A64_W0, A64_W4, 8);
    const uint32_t to_done = HoleB(e);
    PatchCbzX(e, to_slow[0], A64_W11);
    PatchCbzX(e, to_slow[1], A64_W10);
    emit_str_w32_imm(&e, A64_W21, A64_W19, g_off_cyc);
    emit_mov_x64_imm64(&e, A64_W16, (uint64_t)(uintptr_t)g_addrs.mem_read16);
    emit_blr(&e, A64_W16);
    EmitAndImm(e, A64_W0, A64_W0, 0xFFFF);
    PatchB(e, to_done);
}

// MemWrite16(W0, W1): 16-bit value in W0, address in W1. Both bytes'
// banks and watch bits are tested before either store so the fallback
// (whole-pair helper call, hi then lo like MemWrite16) never sees a
// half-committed pair. W0/W1 are preserved up to the commit point.
static void EmitMemWrite16FP(emit_t& e)
{
    if (!g_fastmem)
    {
        emit_mov_x64_imm64(&e, A64_W16, (uint64_t)(uintptr_t)g_addrs.mem_write16);
        emit_blr(&e, A64_W16);
        return;
    }
    uint32_t to_slow[4];
    emit_lsr_w32_imm(&e, A64_W2, A64_W1, 13);
    emit_mov_x64_imm64(&e, A64_W10, (uint64_t)(uintptr_t)g_addrs.fastmem_write_banks);
    emit_ldr_x64_reg_lsl3(&e, A64_W11, A64_W10, A64_W2);
    to_slow[0] = HoleCbzX(e, A64_W11);
    emit_add_w32_imm(&e, A64_W3, A64_W1, 1);
    EmitAndImm(e, A64_W3, A64_W3, 0xFFFF);
    emit_lsr_w32_imm(&e, A64_W2, A64_W3, 13);
    emit_ldr_x64_reg_lsl3(&e, A64_W10, A64_W10, A64_W2);
    to_slow[1] = HoleCbzX(e, A64_W10);
    emit_mov_x64_imm64(&e, A64_W12, (uint64_t)(uintptr_t)g_addrs.fastmem_watch_bitmap);
    EmitWatchBit(e, A64_W1);
    to_slow[2] = HoleCbnzW(e, A64_W2);
    EmitWatchBit(e, A64_W3);
    to_slow[3] = HoleCbnzW(e, A64_W2);
    emit_lsr_w32_imm(&e, A64_W4, A64_W0, 8);
    EmitAndImm(e, A64_W2, A64_W1, 0x1FFF);
    emit_strb_reg_uxtw(&e, A64_W4, A64_W11, A64_W2);   // hi at addr
    EmitAndImm(e, A64_W2, A64_W3, 0x1FFF);
    emit_strb_reg_uxtw(&e, A64_W0, A64_W10, A64_W2);   // lo at addr+1
    const uint32_t to_done = HoleB(e);
    PatchCbzX(e, to_slow[0], A64_W11);
    PatchCbzX(e, to_slow[1], A64_W10);
    PatchCbnzW(e, to_slow[2], A64_W2);
    PatchCbnzW(e, to_slow[3], A64_W2);
    emit_str_w32_imm(&e, A64_W21, A64_W19, g_off_cyc);
    emit_mov_x64_imm64(&e, A64_W16, (uint64_t)(uintptr_t)g_addrs.mem_write16);
    emit_blr(&e, A64_W16);
    PatchB(e, to_done);
}

// LDA/LDB <dp and ext: effective address, MemRead8 call, register
// store, Z/N/V from the loaded byte. The call clobbers caller-saved
// registers; x19/x20 survive.
static void EmitInlineLd8Mem(emit_t& e, const DecodedInst& insn,
                             uint32_t reg_off, bool direct_page,
                             uint8_t mask)
{
    if (direct_page)
    {
        const uint8_t offset = (uint8_t)(insn.operand & 0xFF);
        emit_ldrh_imm(&e, A64_W0, A64_W19, g_off_dp);
        if (offset != 0)
        {
            // dp.Reg's low byte is guaranteed zero, so OR is the add.
            if (!emit_orr_w32_imm(&e, A64_W0, A64_W0, offset))
            {
                emit_movz_w32(&e, A64_W1, offset, 0);
                emit_orr_w32(&e, A64_W0, A64_W0, A64_W1);
            }
        }
    }
    else
    {
        emit_movz_w32(&e, A64_W0, insn.operand, 0);
    }
    g_emit_had_side_effects = true;
    EmitMemRead8FP(e);
    // The return value contract only guarantees the low byte.
    EmitAndImm(e, A64_W0, A64_W0, 0xFF);
    emit_strb_imm(&e, A64_W0, A64_W19, reg_off);
    EmitFlagsZNVFromReg(e, A64_W0, mask);
    EmitCyclesRuntime(e, direct_page ? g_off_nat43 : g_off_nat54);
}

// STA/STB <dp and ext: flags from the register value (not memory),
// then MemWrite8(data, address).
static void EmitInlineSt8Mem(emit_t& e, const DecodedInst& insn,
                             uint32_t reg_off, bool direct_page,
                             uint8_t mask)
{
    emit_ldrb_imm(&e, A64_W0, A64_W19, reg_off);
    EmitFlagsZNVFromReg(e, A64_W0, mask);
    if (direct_page)
    {
        const uint8_t offset = (uint8_t)(insn.operand & 0xFF);
        emit_ldrh_imm(&e, A64_W1, A64_W19, g_off_dp);
        if (offset != 0)
        {
            if (!emit_orr_w32_imm(&e, A64_W1, A64_W1, offset))
            {
                emit_movz_w32(&e, A64_W2, offset, 0);
                emit_orr_w32(&e, A64_W1, A64_W1, A64_W2);
            }
        }
    }
    else
    {
        emit_movz_w32(&e, A64_W1, insn.operand, 0);
    }
    g_emit_had_side_effects = true;
    EmitMemWrite8FP(e);
    EmitCyclesRuntime(e, direct_page ? g_off_nat43 : g_off_nat54);
}

// TSTA/TSTB: Z/N from the register, V=0, C untouched.
static void EmitInlineTst8(emit_t& e, uint32_t reg_off, uint8_t mask)
{
    if (mask & (CC_BIT_Z | CC_BIT_N))
    {
        emit_ldrb_imm(&e, A64_W1, A64_W19, reg_off);
        EmitFlagsZNVFromReg(e, A64_W1, (uint8_t)(mask & (CC_BIT_Z | CC_BIT_N)));
    }
    if (mask & CC_BIT_V)
        EmitCcConst(e, 1, 0);
    EmitCyclesRuntime(e, g_off_nat21);
}

// INCA/INCB: V is set exactly on the 0x7F->0x80 wrap; C untouched.
// DECA/DECB: V on 0x80->0x7F. Both share this shape.
static void EmitInlineIncDec8(emit_t& e, uint32_t reg_off, bool is_inc,
                              uint8_t mask)
{
    emit_ldrb_imm(&e, A64_W1, A64_W19, reg_off);
    if (is_inc)
        emit_add_w32_imm(&e, A64_W1, A64_W1, 1);
    else
        emit_sub_w32_imm(&e, A64_W1, A64_W1, 1);
    EmitAndImm(e, A64_W1, A64_W1, 0xFF);
    emit_strb_imm(&e, A64_W1, A64_W19, reg_off);
    if (mask & CC_BIT_Z)
    {
        emit_cmp_w32_imm(&e, A64_W1, 0);
        emit_cset_w32(&e, A64_W2, A64_COND_EQ);
        emit_strb_imm(&e, A64_W2, A64_W19, g_off_cc + 2);
    }
    if (mask & CC_BIT_N)
    {
        emit_lsr_w32_imm(&e, A64_W2, A64_W1, 7);
        emit_strb_imm(&e, A64_W2, A64_W19, g_off_cc + 3);
    }
    if (mask & CC_BIT_V)
    {
        emit_cmp_w32_imm(&e, A64_W1, is_inc ? 0x80 : 0x7F);
        emit_cset_w32(&e, A64_W2, A64_COND_EQ);
        emit_strb_imm(&e, A64_W2, A64_W19, g_off_cc + 1);
    }
    EmitCyclesRuntime(e, g_off_nat21);
}

// COMA/COMB: ones-complement; Z/N from result, V=0, C=1.
static void EmitInlineCom8(emit_t& e, uint32_t reg_off, uint8_t mask)
{
    emit_ldrb_imm(&e, A64_W1, A64_W19, reg_off);
    emit_mvn_w32(&e, A64_W1, A64_W1);
    EmitAndImm(e, A64_W1, A64_W1, 0xFF);
    emit_strb_imm(&e, A64_W1, A64_W19, reg_off);
    EmitFlagsZNVFromReg(e, A64_W1, (uint8_t)(mask & (CC_BIT_Z | CC_BIT_N | CC_BIT_V)));
    if (mask & CC_BIT_C)
        EmitCcConst(e, 0, 1);
    EmitCyclesRuntime(e, g_off_nat21);
}

// NEGA/NEGB: C = (input != 0), V = (input == 0x80), Z/N from result.
static void EmitInlineNeg8(emit_t& e, uint32_t reg_off, uint8_t mask)
{
    emit_ldrb_imm(&e, A64_W0, A64_W19, reg_off);
    emit_neg_w32(&e, A64_W1, A64_W0);
    EmitAndImm(e, A64_W1, A64_W1, 0xFF);
    emit_strb_imm(&e, A64_W1, A64_W19, reg_off);
    if (mask & CC_BIT_C)
    {
        emit_cmp_w32_imm(&e, A64_W0, 0);
        emit_cset_w32(&e, A64_W2, A64_COND_NE);
        emit_strb_imm(&e, A64_W2, A64_W19, g_off_cc + 0);
    }
    if (mask & CC_BIT_V)
    {
        emit_cmp_w32_imm(&e, A64_W0, 0x80);
        emit_cset_w32(&e, A64_W2, A64_COND_EQ);
        emit_strb_imm(&e, A64_W2, A64_W19, g_off_cc + 1);
    }
    if (mask & CC_BIT_Z)
    {
        emit_cmp_w32_imm(&e, A64_W1, 0);
        emit_cset_w32(&e, A64_W2, A64_COND_EQ);
        emit_strb_imm(&e, A64_W2, A64_W19, g_off_cc + 2);
    }
    if (mask & CC_BIT_N)
    {
        emit_lsr_w32_imm(&e, A64_W2, A64_W1, 7);
        emit_strb_imm(&e, A64_W2, A64_W19, g_off_cc + 3);
    }
    EmitCyclesRuntime(e, g_off_nat21);
}

// LSRA: C = old bit 0, Z from result, N = 0. V untouched.
static void EmitInlineLsr8(emit_t& e, uint32_t reg_off, uint8_t mask)
{
    emit_ldrb_imm(&e, A64_W0, A64_W19, reg_off);
    emit_lsr_w32_imm(&e, A64_W1, A64_W0, 1);
    emit_strb_imm(&e, A64_W1, A64_W19, reg_off);
    if (mask & CC_BIT_C)
    {
        EmitAndImm(e, A64_W2, A64_W0, 1);
        emit_strb_imm(&e, A64_W2, A64_W19, g_off_cc + 0);
    }
    if (mask & CC_BIT_Z)
    {
        emit_cmp_w32_imm(&e, A64_W1, 0);
        emit_cset_w32(&e, A64_W2, A64_COND_EQ);
        emit_strb_imm(&e, A64_W2, A64_W19, g_off_cc + 2);
    }
    if (mask & CC_BIT_N)
        EmitCcConst(e, 3, 0);
    EmitCyclesRuntime(e, g_off_nat21);
}

// ASRA: sign bit propagates; C = old bit 0, Z/N from result. V untouched.
static void EmitInlineAsr8(emit_t& e, uint32_t reg_off, uint8_t mask)
{
    emit_ldrb_imm(&e, A64_W0, A64_W19, reg_off);
    emit_lsr_w32_imm(&e, A64_W1, A64_W0, 1);
    EmitAndImm(e, A64_W2, A64_W0, 0x80);
    emit_orr_w32(&e, A64_W1, A64_W1, A64_W2);
    emit_strb_imm(&e, A64_W1, A64_W19, reg_off);
    if (mask & CC_BIT_C)
    {
        EmitAndImm(e, A64_W2, A64_W0, 1);
        emit_strb_imm(&e, A64_W2, A64_W19, g_off_cc + 0);
    }
    if (mask & CC_BIT_Z)
    {
        emit_cmp_w32_imm(&e, A64_W1, 0);
        emit_cset_w32(&e, A64_W2, A64_COND_EQ);
        emit_strb_imm(&e, A64_W2, A64_W19, g_off_cc + 2);
    }
    if (mask & CC_BIT_N)
    {
        emit_lsr_w32_imm(&e, A64_W2, A64_W1, 7);
        emit_strb_imm(&e, A64_W2, A64_W19, g_off_cc + 3);
    }
    EmitCyclesRuntime(e, g_off_nat21);
}

// ASLA/LSLA: C = old bit 7, V = old bit 7 ^ old bit 6, Z/N from result.
static void EmitInlineAsl8(emit_t& e, uint32_t reg_off, uint8_t mask)
{
    emit_ldrb_imm(&e, A64_W0, A64_W19, reg_off);
    emit_lsl_w32_imm(&e, A64_W1, A64_W0, 1);
    EmitAndImm(e, A64_W1, A64_W1, 0xFF);
    emit_strb_imm(&e, A64_W1, A64_W19, reg_off);
    if (mask & CC_BIT_C)
    {
        emit_lsr_w32_imm(&e, A64_W2, A64_W0, 7);
        emit_strb_imm(&e, A64_W2, A64_W19, g_off_cc + 0);
    }
    if (mask & CC_BIT_V)
    {
        emit_lsr_w32_imm(&e, A64_W2, A64_W0, 7);
        emit_lsr_w32_imm(&e, A64_W3, A64_W0, 6);
        emit_eor_w32(&e, A64_W2, A64_W2, A64_W3);
        EmitAndImm(e, A64_W2, A64_W2, 1);
        emit_strb_imm(&e, A64_W2, A64_W19, g_off_cc + 1);
    }
    if (mask & CC_BIT_Z)
    {
        emit_cmp_w32_imm(&e, A64_W1, 0);
        emit_cset_w32(&e, A64_W2, A64_COND_EQ);
        emit_strb_imm(&e, A64_W2, A64_W19, g_off_cc + 2);
    }
    if (mask & CC_BIT_N)
    {
        emit_lsr_w32_imm(&e, A64_W2, A64_W1, 7);
        emit_strb_imm(&e, A64_W2, A64_W19, g_off_cc + 3);
    }
    EmitCyclesRuntime(e, g_off_nat21);
}

// ABX: X += B, no flags.
static void EmitInlineAbx(emit_t& e)
{
    emit_ldrb_imm(&e, A64_W1, A64_W19, g_off_b);
    emit_ldrh_imm(&e, A64_W2, A64_W19, g_off_x);
    emit_add_w32(&e, A64_W2, A64_W2, A64_W1);
    emit_strh_imm(&e, A64_W2, A64_W19, g_off_x);
    EmitCyclesRuntime(e, g_off_nat31);
}

// ANDA/ANDB/ORA/ORB/EORA/EORB #imm (writeback) and BITA/BITB #imm
// (flags only): result in W1, Z/N from it, V=0, +2 cycles.
enum class LogicOp : uint8_t { And, Or, Eor };

static void EmitInlineLogic8Imm(emit_t& e, const DecodedInst& insn,
                                uint32_t reg_off, LogicOp op,
                                bool writeback, uint8_t mask)
{
    const uint8_t imm = (uint8_t)(insn.operand & 0xFF);
    emit_ldrb_imm(&e, A64_W0, A64_W19, reg_off);
    switch (op)
    {
    case LogicOp::And: EmitAndImm(e, A64_W1, A64_W0, imm); break;
    case LogicOp::Or:  EmitOrrImm(e, A64_W1, A64_W0, imm); break;
    case LogicOp::Eor: EmitEorImm(e, A64_W1, A64_W0, imm); break;
    }
    if (writeback)
        emit_strb_imm(&e, A64_W1, A64_W19, reg_off);
    EmitFlagsZNVFromReg(e, A64_W1, mask);
    EmitCyclesConst(e, 2);
}

// CMPA/CMPB (no writeback), SUBA/SUBB, ADDA/ADDB #imm. Mirrors the
// handlers: C = borrow (sub/cmp: old < imm; add: bit 8 of the wide
// sum), V = C ^ bit7(imm ^ result ^ old), Z/N from the result byte.
// ADD also writes the untracked half-carry cc[H] (index 5)
// unconditionally, exactly as the handler does.
static void EmitInlineArith8Imm(emit_t& e, const DecodedInst& insn,
                                uint32_t reg_off, bool is_add,
                                bool writeback, uint8_t mask)
{
    const uint8_t imm = (uint8_t)(insn.operand & 0xFF);
    emit_ldrb_imm(&e, A64_W0, A64_W19, reg_off);          // old value
    if (is_add)
        emit_add_w32_imm(&e, A64_W1, A64_W0, imm);        // wide sum
    else
        emit_sub_w32_imm(&e, A64_W1, A64_W0, imm);        // wide diff
    EmitAndImm(e, A64_W3, A64_W1, 0xFF);                  // result byte
    if (writeback)
        emit_strb_imm(&e, A64_W3, A64_W19, reg_off);

    // C into W2 (also feeds V).
    if (is_add)
    {
        emit_lsr_w32_imm(&e, A64_W2, A64_W1, 8);          // sum bit 8
    }
    else
    {
        emit_cmp_w32_imm(&e, A64_W0, imm);
        emit_cset_w32(&e, A64_W2, A64_COND_LO);           // borrow
    }
    if (mask & CC_BIT_C)
        emit_strb_imm(&e, A64_W2, A64_W19, g_off_cc + 0);

    if (mask & CC_BIT_V)
    {
        emit_eor_w32(&e, A64_W4, A64_W3, A64_W0);         // result ^ old
        if (imm & 0x80)
            EmitEorImm(e, A64_W4, A64_W4, 0x80);          // ^ imm bit 7
        emit_lsr_w32_imm(&e, A64_W4, A64_W4, 7);
        EmitAndImm(e, A64_W4, A64_W4, 1);
        emit_eor_w32(&e, A64_W4, A64_W4, A64_W2);         // ^ C
        emit_strb_imm(&e, A64_W4, A64_W19, g_off_cc + 1);
    }

    if (is_add)
    {
        // H = bit 4 of (old ^ imm ^ result); untracked by the DSE
        // mask, written unconditionally like the handler.
        emit_eor_w32(&e, A64_W4, A64_W0, A64_W3);
        if (imm & 0x10)
            EmitEorImm(e, A64_W4, A64_W4, 0x10);
        emit_lsr_w32_imm(&e, A64_W4, A64_W4, 4);
        EmitAndImm(e, A64_W4, A64_W4, 1);
        emit_strb_imm(&e, A64_W4, A64_W19, g_off_cc + 5);
    }

    if (mask & CC_BIT_Z)
    {
        emit_cmp_w32_imm(&e, A64_W3, 0);
        emit_cset_w32(&e, A64_W2, A64_COND_EQ);
        emit_strb_imm(&e, A64_W2, A64_W19, g_off_cc + 2);
    }
    if (mask & CC_BIT_N)
    {
        emit_lsr_w32_imm(&e, A64_W2, A64_W3, 7);
        emit_strb_imm(&e, A64_W2, A64_W19, g_off_cc + 3);
    }
    EmitCyclesConst(e, 2);
}

// ---------- indexed effective addresses ----------
// The postbyte was resolved at decode time into ea_info (mode +
// register) and operand, so the addressing mode is an emit-time
// constant. The direct modes inline; indirect and 6309 W/E/F/D modes
// fall back to the call path. Cycle adds mirror the EA_Fn_* helpers
// in hd6309.cpp exactly.

#include "EAModes.h"

static bool EAModeSupported(uint8_t ea_info)
{
    switch (EA_MODE(ea_info))
    {
    case EA_POST_INC1: case EA_POST_INC2:
    case EA_PRE_DEC1:  case EA_PRE_DEC2:
    case EA_NO_OFFSET: case EA_5BIT:
    case EA_OFFSET_8:  case EA_OFFSET_16:
    case EA_PC_8:      case EA_PC_16:
    case EA_OFFSET_A:  case EA_OFFSET_B:
        return true;
    default:
        return false;
    }
}

static uint32_t EARegOffset(uint8_t ea_info)
{
    switch (EA_REG(ea_info))
    {
    case 0:  return g_off_x;
    case 1:  return g_off_y;
    case 2:  return g_off_u;
    default: return g_off_s;
    }
}

// Emit the EA into `dst` (masked to 16 bits) and add the mode's cycle
// cost. Clobbers W2 and (for the cycle add) W9/W1 via the helpers.
// Caller must have verified EAModeSupported.
static void EmitEA(emit_t& e, uint8_t ea_info, uint16_t operand, a64_reg_t dst)
{
    const uint32_t reg_off = EARegOffset(ea_info);
    switch (EA_MODE(ea_info))
    {
    case EA_NO_OFFSET:
        emit_ldrh_imm(&e, dst, A64_W19, reg_off);
        break;

    case EA_5BIT:
    case EA_OFFSET_8:
    {
        const int off = (EA_MODE(ea_info) == EA_5BIT)
            ? (int16_t)operand
            : (int)(signed char)(operand & 0xFF);
        emit_ldrh_imm(&e, dst, A64_W19, reg_off);
        if (off > 0)
            emit_add_w32_imm(&e, dst, dst, (uint32_t)off);
        else if (off < 0)
            emit_sub_w32_imm(&e, dst, dst, (uint32_t)(-off));
        if (off != 0)
            EmitAndImm(e, dst, dst, 0xFFFF);
        EmitCyclesConst(e, 1);
        break;
    }

    case EA_OFFSET_16:
        emit_ldrh_imm(&e, dst, A64_W19, reg_off);
        emit_movz_w32(&e, A64_W2, operand, 0);
        emit_add_w32(&e, dst, dst, A64_W2);
        EmitAndImm(e, dst, dst, 0xFFFF);
        EmitCyclesRuntime(e, g_off_nat43);
        break;

    case EA_PC_8:
        emit_movz_w32(&e, dst, operand, 0);   // pre-computed absolute
        EmitCyclesConst(e, 1);
        break;

    case EA_PC_16:
        emit_movz_w32(&e, dst, operand, 0);   // pre-computed absolute
        EmitCyclesRuntime(e, g_off_nat53);
        break;

    case EA_OFFSET_A:
    case EA_OFFSET_B:
        emit_ldrh_imm(&e, dst, A64_W19, reg_off);
        emit_ldrsb_w32_imm(&e, A64_W2, A64_W19,
                           EA_MODE(ea_info) == EA_OFFSET_A ? g_off_a : g_off_b);
        emit_add_w32(&e, dst, dst, A64_W2);
        EmitAndImm(e, dst, dst, 0xFFFF);
        EmitCyclesConst(e, 1);
        break;

    case EA_POST_INC1:
    case EA_POST_INC2:
        // ea = *reg, then *reg += n
        emit_ldrh_imm(&e, dst, A64_W19, reg_off);
        emit_add_w32_imm(&e, A64_W2, dst,
                         EA_MODE(ea_info) == EA_POST_INC1 ? 1u : 2u);
        emit_strh_imm(&e, A64_W2, A64_W19, reg_off);
        EmitCyclesRuntime(e, EA_MODE(ea_info) == EA_POST_INC1
                             ? g_off_nat21 : g_off_nat32);
        break;

    case EA_PRE_DEC1:
    case EA_PRE_DEC2:
        // *reg -= n, then ea = *reg
        emit_ldrh_imm(&e, dst, A64_W19, reg_off);
        emit_sub_w32_imm(&e, dst, dst,
                         EA_MODE(ea_info) == EA_PRE_DEC1 ? 1u : 2u);
        EmitAndImm(e, dst, dst, 0xFFFF);
        emit_strh_imm(&e, dst, A64_W19, reg_off);
        EmitCyclesRuntime(e, EA_MODE(ea_info) == EA_PRE_DEC1
                             ? g_off_nat21 : g_off_nat32);
        break;

    default:
        break;   // unreachable per EAModeSupported
    }
}

// LEAX/LEAY (Z from the 16-bit result) and LEAU/LEAS (no flags).
static void EmitInlineLea(emit_t& e, const DecodedInst& insn,
                          uint32_t dst_off, bool sets_z, uint8_t mask)
{
    EmitEA(e, insn.ea_info, insn.operand, A64_W0);
    emit_strh_imm(&e, A64_W0, A64_W19, dst_off);
    if (sets_z && (mask & CC_BIT_Z))
    {
        emit_cmp_w32_imm(&e, A64_W0, 0);
        emit_cset_w32(&e, A64_W2, A64_COND_EQ);
        emit_strb_imm(&e, A64_W2, A64_W19, g_off_cc + 2);
    }
    EmitCyclesConst(e, 4);
}

// LDA/LDB indexed: MemRead8(EA) -> acc, Z/N/V from the byte, +4.
static void EmitInlineLd8Idx(emit_t& e, const DecodedInst& insn,
                             uint32_t reg_off, uint8_t mask)
{
    g_emit_had_side_effects = true;   // touches guest memory
    EmitEA(e, insn.ea_info, insn.operand, A64_W0);
    EmitMemRead8FP(e);
    EmitAndImm(e, A64_W0, A64_W0, 0xFF);
    emit_strb_imm(&e, A64_W0, A64_W19, reg_off);
    EmitFlagsZNVFromReg(e, A64_W0, mask);
    EmitCyclesConst(e, 4);
}

// STA/STB indexed: MemWrite8(acc, EA), flags from acc, +4.
static void EmitInlineSt8Idx(emit_t& e, const DecodedInst& insn,
                             uint32_t reg_off, uint8_t mask)
{
    g_emit_had_side_effects = true;   // touches guest memory
    // EA must be computed into W0 and moved: EmitEA's cycle-add uses
    // W1 as scratch for the runtime-cost modes, so asking for the EA
    // directly in W1 hands mem_write8 a garbage address.
    EmitEA(e, insn.ea_info, insn.operand, A64_W0);
    emit_orr_w32(&e, A64_W1, A64_WZR, A64_W0);   // mov w1, w0 (address)
    emit_ldrb_imm(&e, A64_W0, A64_W19, reg_off);
    EmitFlagsZNVFromReg(e, A64_W0, mask);
    EmitMemWrite8FP(e);
    EmitCyclesConst(e, 4);
}

// TST indexed: flags from MemRead8(EA), no writeback, +NatEmuCycles65.
static void EmitInlineTstIdx(emit_t& e, const DecodedInst& insn, uint8_t mask)
{
    g_emit_had_side_effects = true;   // touches guest memory
    EmitEA(e, insn.ea_info, insn.operand, A64_W0);
    EmitMemRead8FP(e);
    EmitAndImm(e, A64_W0, A64_W0, 0xFF);
    EmitFlagsZNVFromReg(e, A64_W0, mask);
    EmitCyclesRuntime(e, g_off_nat65);
}

// LDX/LDU/LDD indexed: MemRead16(EA) -> reg16, Z/N(bit 15)/V=0, +5.
static void EmitInlineLd16Idx(emit_t& e, const DecodedInst& insn,
                              uint32_t reg_off, uint8_t mask)
{
    g_emit_had_side_effects = true;   // touches guest memory
    EmitEA(e, insn.ea_info, insn.operand, A64_W0);
    EmitMemRead16FP(e);
    emit_strh_imm(&e, A64_W0, A64_W19, reg_off);
    if (mask & CC_BIT_Z)
    {
        emit_cmp_w32_imm(&e, A64_W0, 0);
        emit_cset_w32(&e, A64_W2, A64_COND_EQ);
        emit_strb_imm(&e, A64_W2, A64_W19, g_off_cc + 2);
    }
    if (mask & CC_BIT_N)
    {
        emit_lsr_w32_imm(&e, A64_W2, A64_W0, 15);
        emit_strb_imm(&e, A64_W2, A64_W19, g_off_cc + 3);
    }
    if (mask & CC_BIT_V)
        emit_strb_imm(&e, A64_WZR, A64_W19, g_off_cc + 1);
    EmitCyclesConst(e, 5);
}

// STD/STX indexed: MemWrite16(reg16, EA), flags from reg16, +5.
static void EmitInlineSt16Idx(emit_t& e, const DecodedInst& insn,
                              uint32_t reg_off, uint8_t mask)
{
    g_emit_had_side_effects = true;   // touches guest memory
    // Same W1-scratch hazard as EmitInlineSt8Idx: EA via W0, then move.
    EmitEA(e, insn.ea_info, insn.operand, A64_W0);
    emit_orr_w32(&e, A64_W1, A64_WZR, A64_W0);   // mov w1, w0 (address)
    emit_ldrh_imm(&e, A64_W0, A64_W19, reg_off);
    if (mask & CC_BIT_Z)
    {
        emit_cmp_w32_imm(&e, A64_W0, 0);
        emit_cset_w32(&e, A64_W2, A64_COND_EQ);
        emit_strb_imm(&e, A64_W2, A64_W19, g_off_cc + 2);
    }
    if (mask & CC_BIT_N)
    {
        emit_lsr_w32_imm(&e, A64_W2, A64_W0, 15);
        emit_strb_imm(&e, A64_W2, A64_W19, g_off_cc + 3);
    }
    if (mask & CC_BIT_V)
        emit_strb_imm(&e, A64_WZR, A64_W19, g_off_cc + 1);
    EmitMemWrite16FP(e);
    EmitCyclesConst(e, 5);
}

// ---------- branch terminators ----------
// A branch as the block's last instruction needs no handler call: the
// fall-through PC is the running local_pc and the taken PC is
// fall + signed offset - both emit-time constants. The predicate is
// one to three cc[] byte loads. Cycle costs mirror the handlers
// exactly: short branches +3 flat, LBRA NatEmuCycles54, long
// conditionals +5 (+1 when taken).

enum class BranchPred : uint8_t
{
    Always,     // BRA
    Never,      // BRN
    Z,          // BEQ / BNE(inverted)
    N,          // BMI / BPL
    C,          // BLO / BHS
    V,          // BVS / BVC
    CorZ,       // BLS / BHI
    NxorV,      // BLT / BGE
    ZorNxorV,   // BLE / BGT
};

struct BranchDesc
{
    BranchPred pred;
    bool taken_when_zero;   // condition inverted (BNE, BPL, ...)
    bool is_long;
};

static bool LookupBranch(InstHandler h, BranchDesc& out)
{
    if (h == g_inlines.bra_r)  { out = { BranchPred::Always,   false, false }; return true; }
    if (h == g_inlines.brn_r)  { out = { BranchPred::Never,    false, false }; return true; }
    if (h == g_inlines.beq_r)  { out = { BranchPred::Z,        false, false }; return true; }
    if (h == g_inlines.bne_r)  { out = { BranchPred::Z,        true,  false }; return true; }
    if (h == g_inlines.bmi_r)  { out = { BranchPred::N,        false, false }; return true; }
    if (h == g_inlines.bpl_r)  { out = { BranchPred::N,        true,  false }; return true; }
    if (h == g_inlines.blo_r)  { out = { BranchPred::C,        false, false }; return true; }
    if (h == g_inlines.bhs_r)  { out = { BranchPred::C,        true,  false }; return true; }
    if (h == g_inlines.bvs_r)  { out = { BranchPred::V,        false, false }; return true; }
    if (h == g_inlines.bvc_r)  { out = { BranchPred::V,        true,  false }; return true; }
    if (h == g_inlines.bls_r)  { out = { BranchPred::CorZ,     false, false }; return true; }
    if (h == g_inlines.bhi_r)  { out = { BranchPred::CorZ,     true,  false }; return true; }
    if (h == g_inlines.blt_r)  { out = { BranchPred::NxorV,    false, false }; return true; }
    if (h == g_inlines.bge_r)  { out = { BranchPred::NxorV,    true,  false }; return true; }
    if (h == g_inlines.ble_r)  { out = { BranchPred::ZorNxorV, false, false }; return true; }
    if (h == g_inlines.bgt_r)  { out = { BranchPred::ZorNxorV, true,  false }; return true; }
    if (h == g_inlines.lbra_r) { out = { BranchPred::Always,   false, true  }; return true; }
    if (h == g_inlines.lbeq_r) { out = { BranchPred::Z,        false, true  }; return true; }
    if (h == g_inlines.lbne_r) { out = { BranchPred::Z,        true,  true  }; return true; }
    return false;
}

// Leaves the predicate value in W1 (nonzero = condition holds, before
// any taken_when_zero inversion).
static void EmitBranchPredicate(emit_t& e, BranchPred pred)
{
    switch (pred)
    {
    case BranchPred::Z:
        emit_ldrb_imm(&e, A64_W1, A64_W19, g_off_cc + 2);
        break;
    case BranchPred::N:
        emit_ldrb_imm(&e, A64_W1, A64_W19, g_off_cc + 3);
        break;
    case BranchPred::C:
        emit_ldrb_imm(&e, A64_W1, A64_W19, g_off_cc + 0);
        break;
    case BranchPred::V:
        emit_ldrb_imm(&e, A64_W1, A64_W19, g_off_cc + 1);
        break;
    case BranchPred::CorZ:
        emit_ldrb_imm(&e, A64_W1, A64_W19, g_off_cc + 0);
        emit_ldrb_imm(&e, A64_W2, A64_W19, g_off_cc + 2);
        emit_orr_w32(&e, A64_W1, A64_W1, A64_W2);
        break;
    case BranchPred::NxorV:
        emit_ldrb_imm(&e, A64_W1, A64_W19, g_off_cc + 3);
        emit_ldrb_imm(&e, A64_W2, A64_W19, g_off_cc + 1);
        emit_eor_w32(&e, A64_W1, A64_W1, A64_W2);
        break;
    case BranchPred::ZorNxorV:
        emit_ldrb_imm(&e, A64_W1, A64_W19, g_off_cc + 3);
        emit_ldrb_imm(&e, A64_W2, A64_W19, g_off_cc + 1);
        emit_eor_w32(&e, A64_W1, A64_W1, A64_W2);
        emit_ldrb_imm(&e, A64_W2, A64_W19, g_off_cc + 2);
        emit_orr_w32(&e, A64_W1, A64_W1, A64_W2);
        break;
    case BranchPred::Always:
    case BranchPred::Never:
        break;
    }
}

// fall_pc is local_pc after this instruction; the taken target comes
// from the pre-decoded signed offset. Stores PC (both paths) and adds
// the exact handler cycle cost.
static void EmitInlineBranch(emit_t& e, const DecodedInst& insn,
                             const BranchDesc& desc, uint16_t fall_pc)
{
    const uint16_t taken_pc = desc.is_long
        ? (uint16_t)(fall_pc + (int16_t)insn.operand)
        : (uint16_t)(fall_pc + (int8_t)(insn.operand & 0xFF));

    if (desc.pred == BranchPred::Always || desc.pred == BranchPred::Never)
    {
        const uint16_t pc = (desc.pred == BranchPred::Always) ? taken_pc : fall_pc;
        emit_movz_w32(&e, A64_W2, pc, 0);
        emit_strh_imm(&e, A64_W2, A64_W19, g_off_pc);
        if (desc.is_long)
            EmitCyclesRuntime(e, g_off_nat54);   // LBRA
        else
            EmitCyclesConst(e, 3);               // BRA / BRN
        return;
    }

    EmitBranchPredicate(e, desc.pred);

    // W2 = PC to store, W3 = cycles (long branches cost one more when
    // taken). Select via a forward branch over the reassignment.
    const uint16_t pc_if_zero    = desc.taken_when_zero ? taken_pc : fall_pc;
    const uint16_t pc_if_nonzero = desc.taken_when_zero ? fall_pc : taken_pc;
    const uint32_t cyc_if_zero    = desc.is_long ? (desc.taken_when_zero ? 6u : 5u) : 3u;
    const uint32_t cyc_if_nonzero = desc.is_long ? (desc.taken_when_zero ? 5u : 6u) : 3u;

    emit_movz_w32(&e, A64_W2, pc_if_zero, 0);
    if (desc.is_long)
        emit_movz_w32(&e, A64_W3, cyc_if_zero, 0);
    // skip the nonzero-case reassignment when W1 == 0
    emit_cbz_w32(&e, A64_W1, desc.is_long ? 12 : 8);
    emit_movz_w32(&e, A64_W2, pc_if_nonzero, 0);
    if (desc.is_long)
        emit_movz_w32(&e, A64_W3, cyc_if_nonzero, 0);

    emit_strh_imm(&e, A64_W2, A64_W19, g_off_pc);
    if (desc.is_long)
    {
        emit_add_w32(&e, A64_W21, A64_W21, A64_W3);
    }
    else
    {
        EmitCyclesConst(e, 3);
    }
}

// ---------- liveness analysis ----------
// Ported verbatim from BlockJit.cpp (target-independent logic).

static uint8_t InlinedHandlerWritesMask(InstHandler h)
{
    if (h == nullptr) return CC_UNKNOWN;

    if (h == g_inlines.lda_m  || h == g_inlines.ldb_m  ||
        h == g_inlines.ldd_m  || h == g_inlines.ldx_m  ||
        h == g_inlines.ldu_m  || h == g_inlines.lds_i  ||
        h == g_inlines.ldy_m  ||
        h == g_inlines.lda_d  || h == g_inlines.ldb_d  ||
        h == g_inlines.sta_d  || h == g_inlines.stb_d  ||
        h == g_inlines.lda_e  || h == g_inlines.ldb_e  ||
        h == g_inlines.sta_e  || h == g_inlines.stb_e  ||
        h == g_inlines.tsta_i || h == g_inlines.tstb_i ||
        h == g_inlines.inca_i || h == g_inlines.incb_i ||
        h == g_inlines.deca_i || h == g_inlines.decb_i)
    {
        return CC_BIT_Z | CC_BIT_N | CC_BIT_V;
    }

    if (h == g_inlines.anda_m || h == g_inlines.andb_m ||
        h == g_inlines.ora_m  || h == g_inlines.orb_m  ||
        h == g_inlines.eora_m || h == g_inlines.eorb_m ||
        h == g_inlines.bita_m || h == g_inlines.bitb_m)
    {
        return CC_BIT_Z | CC_BIT_N | CC_BIT_V;
    }

    // Compare/sub/add write all four (ADD also writes the untracked H).
    if (h == g_inlines.cmpa_m || h == g_inlines.cmpb_m ||
        h == g_inlines.suba_m || h == g_inlines.subb_m ||
        h == g_inlines.adda_m || h == g_inlines.addb_m)
    {
        return CC_ALL;
    }

    if (h == g_inlines.clra_i || h == g_inlines.clrb_i ||
        h == g_inlines.coma_i || h == g_inlines.comb_i ||
        h == g_inlines.nega_i || h == g_inlines.negb_i)
    {
        return CC_ALL;
    }

    if (h == g_inlines.lsra_i || h == g_inlines.asra_i)
    {
        return CC_BIT_C | CC_BIT_Z | CC_BIT_N;
    }

    if (h == g_inlines.asla_i)
    {
        return CC_ALL;
    }

    if (h == g_inlines.abx_i)
    {
        return 0;
    }

    // Indexed family (EA side effects touch registers, never CC).
    // LEAX/LEAY set Z from the result; LEAU/LEAS set nothing; the
    // indexed loads/stores/TST set Z/N and clear V like their
    // direct-page cousins. These were missing from this table, which
    // meant the CC_UNKNOWN structural guard silently blocked the whole
    // family from inlining - exactly what the guard is for, but the
    // masks belong here.
    if (h == g_inlines.leax_x || h == g_inlines.leay_x)
    {
        return CC_BIT_Z;
    }

    if (h == g_inlines.leau_x || h == g_inlines.leas_x)
    {
        return 0;
    }

    if (h == g_inlines.lda_x || h == g_inlines.ldb_x ||
        h == g_inlines.sta_x || h == g_inlines.stb_x ||
        h == g_inlines.tst_x ||
        h == g_inlines.ldx_x || h == g_inlines.ldu_x ||
        h == g_inlines.ldd_x || h == g_inlines.std_x ||
        h == g_inlines.stx_x)
    {
        return CC_BIT_Z | CC_BIT_N | CC_BIT_V;
    }

    // 16-bit load/store dp/ext: Z/N from the value, V=0, C untouched.
    {
        using namespace BlockJit;
        for (int r = 0; r < R16_COUNT; ++r)
        {
            if (h == g_inlines.ld16_dp[r]  || h == g_inlines.ld16_ext[r] ||
                h == g_inlines.st16_dp[r]  || h == g_inlines.st16_ext[r])
                return CC_BIT_Z | CC_BIT_N | CC_BIT_V;
        }
    }

    // 16-bit arithmetic: C/V/Z/N (no H).
    if (h == g_inlines.addd_m || h == g_inlines.addd_d ||
        h == g_inlines.addd_x || h == g_inlines.addd_e ||
        h == g_inlines.subd_m || h == g_inlines.subd_x ||
        h == g_inlines.cmpd_m || h == g_inlines.cmpd_x ||
        h == g_inlines.cmpx_m || h == g_inlines.cmpx_x ||
        h == g_inlines.cmpy_m ||
        h == g_inlines.cmpu_m || h == g_inlines.cmpu_x)
    {
        return CC_ALL;
    }

    // Memory-operand 8-bit ALU: logic ops write Z/N/V (V=0),
    // arithmetic writes all four (ADD also the untracked H).
    {
        using namespace BlockJit;
        for (int m = 0; m < MM_COUNT; ++m)
        {
            for (int op = 0; op < ALU8_OP_COUNT; ++op)
            {
                if (h == g_inlines.alu8_mem_a[op][m] ||
                    h == g_inlines.alu8_mem_b[op][m])
                {
                    return (op == ALU8_AND || op == ALU8_OR ||
                            op == ALU8_EOR || op == ALU8_BIT)
                        ? (uint8_t)(CC_BIT_Z | CC_BIT_N | CC_BIT_V)
                        : CC_ALL;
                }
            }
            // Memory RMW: exact per-op sets, straight from the handlers.
            // NEG/COM/CLR/ASL/ROL write all four; LSR/ROR/ASR leave V;
            // DEC/INC leave C.
            for (int op = 0; op < RMW8_OP_COUNT; ++op)
            {
                if (h == g_inlines.rmw8_mem[op][m])
                {
                    if (op == RMW8_LSR || op == RMW8_ROR || op == RMW8_ASR)
                        return CC_BIT_C | CC_BIT_Z | CC_BIT_N;
                    if (op == RMW8_DEC || op == RMW8_INC)
                        return CC_BIT_Z | CC_BIT_N | CC_BIT_V;
                    return CC_ALL;
                }
            }
        }
    }

    return CC_UNKNOWN;
}

// Flags a known-mask handler READS at run time. Every op in the
// original inline set read nothing - the analyzer's "live &= ~mask"
// walk silently relied on that. The memory ROR/ROL rotate the carry
// IN, so any upstream C-store they depend on must stay live or the
// rotate sees a stale cc[C] (found the hard way: ChaCha20's chained
// long rotates in the journal app decrypting garbage).
static uint8_t InlinedHandlerReadsMask(InstHandler h)
{
    using namespace BlockJit;
    for (int m = 0; m < MM_COUNT; ++m)
    {
        if (h == g_inlines.rmw8_mem[RMW8_ROR][m] ||
            h == g_inlines.rmw8_mem[RMW8_ROL][m])
            return CC_BIT_C;
    }
    return 0;
}

static void AnalyzeFlagLiveness(const CachedBlock& slot,
                                uint8_t* live_writes_out,
                                uint32_t* requested_out,
                                uint32_t* elided_out)
{
    uint8_t live = CC_ALL;
    uint32_t requested = 0;
    uint32_t elided    = 0;

    for (int i = (int)slot.num_insns - 1; i >= 0; --i)
    {
        const uint8_t mask = InlinedHandlerWritesMask(slot.insns[i].handler);

        if (mask == CC_UNKNOWN)
        {
            live_writes_out[i] = 0;
            live = CC_ALL;
            continue;
        }

        uint8_t bits = mask;
        while (bits) { requested++; bits &= bits - 1; }

        live_writes_out[i] = (uint8_t)(mask & live);

        uint8_t dead = (uint8_t)(mask & ~live);
        while (dead) { elided++; dead &= dead - 1; }

        live &= (uint8_t)~mask;
        live |= InlinedHandlerReadsMask(slot.insns[i].handler);
    }

    if (requested_out) *requested_out = requested;
    if (elided_out)    *elided_out    = elided;
}

// ---------- inline dispatch ----------

static int InlineRank(InstHandler h)
{
    const InstHandler order[] = {
        g_inlines.lda_m,  g_inlines.ldb_m,  g_inlines.ldd_m,  g_inlines.ldx_m,
        g_inlines.ldu_m,  g_inlines.lds_i,  g_inlines.ldy_m,  g_inlines.clra_i,
        g_inlines.clrb_i, g_inlines.lda_d,  g_inlines.ldb_d,  g_inlines.sta_d,
        g_inlines.stb_d,  g_inlines.lda_e,  g_inlines.ldb_e,  g_inlines.sta_e,
        g_inlines.stb_e,  g_inlines.tsta_i, g_inlines.tstb_i, g_inlines.inca_i,
        g_inlines.incb_i, g_inlines.deca_i, g_inlines.decb_i, g_inlines.coma_i,
        g_inlines.comb_i, g_inlines.nega_i, g_inlines.negb_i, g_inlines.lsra_i,
        g_inlines.asra_i, g_inlines.asla_i, g_inlines.abx_i,
        g_inlines.anda_m, g_inlines.andb_m, g_inlines.ora_m,  g_inlines.orb_m,
        g_inlines.eora_m, g_inlines.eorb_m, g_inlines.bita_m, g_inlines.bitb_m,
        g_inlines.cmpa_m, g_inlines.cmpb_m, g_inlines.suba_m, g_inlines.subb_m,
        g_inlines.adda_m, g_inlines.addb_m,
        g_inlines.leax_x, g_inlines.leay_x, g_inlines.leau_x, g_inlines.leas_x,
        g_inlines.lda_x,  g_inlines.ldb_x,  g_inlines.sta_x,  g_inlines.stb_x,
        g_inlines.tst_x,  g_inlines.ldx_x,  g_inlines.ldu_x,  g_inlines.ldd_x,
        g_inlines.std_x,  g_inlines.stx_x,
    };
    for (int i = 0; i < (int)(sizeof(order) / sizeof(order[0])); i++)
        if (h == order[i]) return i;
    return -1;
}

// ---------- 16-bit memory-operand families (fastmem-based) ----------

// Effective address for the MM_DP/MM_IDX/MM_EXT memory modes, into
// `dst`. DP builds dp.Reg|offset (low byte of dp.Reg is zero); EXT is
// the baked operand; IDX runs EmitEA, which also charges the EA mode's
// cycles exactly like the interpreter's EA_Fn helpers. Clobbers W2
// (and, for IDX, EmitEA's scratch set).
static void EmitEAForMode(emit_t& e, int mode, const DecodedInst& insn,
                          a64_reg_t dst)
{
    if (mode == BlockJit::MM_DP)
    {
        const uint8_t offset = (uint8_t)(insn.operand & 0xFF);
        emit_ldrh_imm(&e, dst, A64_W19, g_off_dp);
        if (offset != 0)
        {
            if (!emit_orr_w32_imm(&e, dst, dst, offset))
            {
                emit_movz_w32(&e, A64_W2, offset, 0);
                emit_orr_w32(&e, dst, dst, A64_W2);
            }
        }
    }
    else if (mode == BlockJit::MM_EXT)
    {
        emit_movz_w32(&e, dst, insn.operand, 0);
    }
    else
    {
        EmitEA(e, insn.ea_info, insn.operand, dst);
    }
}

// Z/N(bit15)/V=0 from the 16-bit value in `v` - the LD16/ST16 flag set.
static void EmitFlags16ZNV(emit_t& e, a64_reg_t v, uint8_t mask)
{
    if (mask & CC_BIT_Z)
    {
        emit_cmp_w32_imm(&e, v, 0);
        emit_cset_w32(&e, A64_W2, A64_COND_EQ);
        emit_strb_imm(&e, A64_W2, A64_W19, g_off_cc + 2);
    }
    if (mask & CC_BIT_N)
    {
        emit_lsr_w32_imm(&e, A64_W2, v, 15);
        emit_strb_imm(&e, A64_W2, A64_W19, g_off_cc + 3);
    }
    if (mask & CC_BIT_V)
        emit_strb_imm(&e, A64_WZR, A64_W19, g_off_cc + 1);
}

// LDD/LDX/LDU <dp and ext: MemRead16(EA) -> reg16, Z/N/V=0.
static void EmitInlineLd16Mem(emit_t& e, const DecodedInst& insn,
                              uint32_t reg_off, int mode, uint8_t mask)
{
    g_emit_had_side_effects = true;
    EmitEAForMode(e, mode, insn, A64_W0);
    EmitMemRead16FP(e);
    emit_strh_imm(&e, A64_W0, A64_W19, reg_off);
    EmitFlags16ZNV(e, A64_W0, mask);
    EmitCyclesRuntime(e, mode == BlockJit::MM_DP ? g_off_nat54 : g_off_nat65);
}

// STD/STX/STU <dp and ext: MemWrite16(reg16, EA), flags from the value.
static void EmitInlineSt16Mem(emit_t& e, const DecodedInst& insn,
                              uint32_t reg_off, int mode, uint8_t mask)
{
    g_emit_had_side_effects = true;
    EmitEAForMode(e, mode, insn, A64_W1);
    emit_ldrh_imm(&e, A64_W0, A64_W19, reg_off);
    EmitFlags16ZNV(e, A64_W0, mask);
    EmitMemWrite16FP(e);
    EmitCyclesRuntime(e, mode == BlockJit::MM_DP ? g_off_nat54 : g_off_nat65);
}

// 16-bit ADD/SUB/CMP against an immediate (mode < 0) or memory
// operand. Flag math mirrors the handlers exactly: C is the unsigned
// carry/borrow, V = C ^ bit15(res ^ old ^ operand) (OVERFLOW16 is
// XOR-symmetric in its value arguments), N/Z from the truncated
// result. No H - the 16-bit ops never touch it.
enum { A16_ADD, A16_SUB, A16_CMP };
static void EmitInlineArith16(emit_t& e, const DecodedInst& insn,
                              uint32_t reg_off, int op, int mode,
                              uint32_t nat_off, uint8_t mask)
{
    // Operand into W5 (preserved across the fastmem primitives' fast
    // paths; the slow helper call happens before W5 would be loaded
    // only in the memory-operand case, where W5 is loaded after).
    if (mode < 0)
    {
        emit_movz_w32(&e, A64_W5, insn.operand, 0);
    }
    else
    {
        g_emit_had_side_effects = true;
        EmitEAForMode(e, mode, insn, A64_W0);
        EmitMemRead16FP(e);
        emit_mov_w32_w32(&e, A64_W5, A64_W0);
    }
    emit_ldrh_imm(&e, A64_W0, A64_W19, reg_off);       // old value
    if (op == A16_ADD)
        emit_add_w32(&e, A64_W1, A64_W0, A64_W5);
    else
        emit_sub_w32(&e, A64_W1, A64_W0, A64_W5);
    EmitAndImm(e, A64_W3, A64_W1, 0xFFFF);             // result16

    // C into W2 unconditionally - it feeds V.
    if (op == A16_ADD)
    {
        emit_lsr_w32_imm(&e, A64_W2, A64_W1, 16);      // carry out
    }
    else
    {
        emit_cmp_w32_w32(&e, A64_W0, A64_W5);
        emit_cset_w32(&e, A64_W2, A64_COND_LO);        // borrow
    }
    if (mask & CC_BIT_C)
        emit_strb_imm(&e, A64_W2, A64_W19, g_off_cc + 0);
    if (mask & CC_BIT_V)
    {
        emit_eor_w32(&e, A64_W4, A64_W3, A64_W0);
        emit_eor_w32(&e, A64_W4, A64_W4, A64_W5);
        emit_lsr_w32_imm(&e, A64_W4, A64_W4, 15);
        EmitAndImm(e, A64_W4, A64_W4, 1);
        emit_eor_w32(&e, A64_W4, A64_W4, A64_W2);
        emit_strb_imm(&e, A64_W4, A64_W19, g_off_cc + 1);
    }
    if (op != A16_CMP)
        emit_strh_imm(&e, A64_W3, A64_W19, reg_off);
    if (mask & CC_BIT_Z)
    {
        emit_cmp_w32_imm(&e, A64_W3, 0);
        emit_cset_w32(&e, A64_W2, A64_COND_EQ);
        emit_strb_imm(&e, A64_W2, A64_W19, g_off_cc + 2);
    }
    if (mask & CC_BIT_N)
    {
        emit_lsr_w32_imm(&e, A64_W2, A64_W3, 15);
        emit_strb_imm(&e, A64_W2, A64_W19, g_off_cc + 3);
    }
    EmitCyclesRuntime(e, nat_off);
}

// ---------- 8-bit memory-operand ALU and RMW families ----------

// Shared 8-bit SUB/CMP/ADD flag+result body: old value in W0, operand
// byte in W5. Result byte lands in W3. Mirrors EmitInlineArith8Imm
// with a runtime operand.
static void EmitArith8Body(emit_t& e, uint32_t reg_off, bool is_add,
                           bool writeback, uint8_t mask)
{
    if (is_add)
        emit_add_w32(&e, A64_W1, A64_W0, A64_W5);
    else
        emit_sub_w32(&e, A64_W1, A64_W0, A64_W5);
    EmitAndImm(e, A64_W3, A64_W1, 0xFF);
    if (writeback)
        emit_strb_imm(&e, A64_W3, A64_W19, reg_off);

    if (is_add)
    {
        emit_lsr_w32_imm(&e, A64_W2, A64_W1, 8);       // carry out
    }
    else
    {
        emit_cmp_w32_w32(&e, A64_W0, A64_W5);
        emit_cset_w32(&e, A64_W2, A64_COND_LO);        // borrow
    }
    if (mask & CC_BIT_C)
        emit_strb_imm(&e, A64_W2, A64_W19, g_off_cc + 0);
    if (mask & CC_BIT_V)
    {
        emit_eor_w32(&e, A64_W4, A64_W3, A64_W0);
        emit_eor_w32(&e, A64_W4, A64_W4, A64_W5);
        emit_lsr_w32_imm(&e, A64_W4, A64_W4, 7);
        EmitAndImm(e, A64_W4, A64_W4, 1);
        emit_eor_w32(&e, A64_W4, A64_W4, A64_W2);
        emit_strb_imm(&e, A64_W4, A64_W19, g_off_cc + 1);
    }
    if (is_add)
    {
        // H = bit 4 of (old ^ operand ^ result), unconditional.
        emit_eor_w32(&e, A64_W4, A64_W0, A64_W3);
        emit_eor_w32(&e, A64_W4, A64_W4, A64_W5);
        emit_lsr_w32_imm(&e, A64_W4, A64_W4, 4);
        EmitAndImm(e, A64_W4, A64_W4, 1);
        emit_strb_imm(&e, A64_W4, A64_W19, g_off_cc + 5);
    }
    if (mask & CC_BIT_Z)
    {
        emit_cmp_w32_imm(&e, A64_W3, 0);
        emit_cset_w32(&e, A64_W2, A64_COND_EQ);
        emit_strb_imm(&e, A64_W2, A64_W19, g_off_cc + 2);
    }
    if (mask & CC_BIT_N)
    {
        emit_lsr_w32_imm(&e, A64_W2, A64_W3, 7);
        emit_strb_imm(&e, A64_W2, A64_W19, g_off_cc + 3);
    }
}

// ALU with a memory operand: SUBA/CMPA/ADDA/ANDA/ORA/EORA/BITA (and
// the B forms) against dp/indexed/extended. The operand byte is read
// through the RAM fast path; the accumulator loads AFTER the read so
// nothing lives across a possible slow-path helper call.
static void EmitInlineAlu8Mem(emit_t& e, const DecodedInst& insn, int op,
                              uint32_t reg_off, int mode, uint8_t mask)
{
    using namespace BlockJit;
    g_emit_had_side_effects = true;
    EmitEAForMode(e, mode, insn, A64_W0);
    EmitMemRead8FP(e);
    EmitAndImm(e, A64_W5, A64_W0, 0xFF);               // operand byte
    emit_ldrb_imm(&e, A64_W0, A64_W19, reg_off);       // accumulator

    switch (op)
    {
    case ALU8_SUB: EmitArith8Body(e, reg_off, false, true,  mask); break;
    case ALU8_CMP: EmitArith8Body(e, reg_off, false, false, mask); break;
    case ALU8_ADD: EmitArith8Body(e, reg_off, true,  true,  mask); break;
    default:
    {
        // AND/OR/EOR/BIT: result in W3, V=0, C untouched.
        if (op == ALU8_AND || op == ALU8_BIT)
            emit_and_w32(&e, A64_W3, A64_W0, A64_W5);
        else if (op == ALU8_OR)
            emit_orr_w32(&e, A64_W3, A64_W0, A64_W5);
        else
            emit_eor_w32(&e, A64_W3, A64_W0, A64_W5);
        if (op != ALU8_BIT)
            emit_strb_imm(&e, A64_W3, A64_W19, reg_off);
        EmitFlagsZNVFromReg(e, A64_W3, mask);
        break;
    }
    }
    if (mode == MM_DP)       EmitCyclesRuntime(e, g_off_nat43);
    else if (mode == MM_IDX) EmitCyclesConst(e, 4);
    else                     EmitCyclesRuntime(e, g_off_nat54);
}

// Memory read-modify-write: NEG/COM/LSR/ROR/ASR/ASL/ROL/DEC/INC/CLR
// against dp/indexed/extended. The EA is spilled to the state block's
// scratch slot so it survives a slow-path read (the helper call
// clobbers every caller-saved register). Flag order matches the
// handlers: flags first, then the write - so an I/O store that raises
// an interrupt sees the completed flag state.
static void EmitInlineRmw8(emit_t& e, const DecodedInst& insn, int op,
                           int mode, uint8_t mask)
{
    using namespace BlockJit;
    g_emit_had_side_effects = true;
    EmitEAForMode(e, mode, insn, A64_W0);

    if (op == RMW8_CLR)
    {
        // The handler writes 0 without reading first; mirror it.
        emit_orr_w32(&e, A64_W1, A64_WZR, A64_W0);     // address
        emit_movz_w32(&e, A64_W0, 0, 0);
        if (mask & CC_BIT_C) EmitCcConst(e, 0, 0);
        if (mask & CC_BIT_V) EmitCcConst(e, 1, 0);
        if (mask & CC_BIT_Z) EmitCcConst(e, 2, 1);
        if (mask & CC_BIT_N) EmitCcConst(e, 3, 0);
        EmitMemWrite8FP(e);
    }
    else
    {
        emit_strh_imm(&e, A64_W0, A64_W19, g_off_scratch16);
        EmitMemRead8FP(e);
        EmitAndImm(e, A64_W0, A64_W0, 0xFF);           // old value

        // Result byte into W3; flags per the handler bodies.
        switch (op)
        {
        case RMW8_NEG:
            emit_sub_w32(&e, A64_W3, A64_WZR, A64_W0);
            EmitAndImm(e, A64_W3, A64_W3, 0xFF);
            if (mask & CC_BIT_C)
            {
                emit_cmp_w32_imm(&e, A64_W3, 0);
                emit_cset_w32(&e, A64_W2, A64_COND_NE);
                emit_strb_imm(&e, A64_W2, A64_W19, g_off_cc + 0);
            }
            if (mask & CC_BIT_V)
            {
                emit_cmp_w32_imm(&e, A64_W0, 0x80);
                emit_cset_w32(&e, A64_W2, A64_COND_EQ);
                emit_strb_imm(&e, A64_W2, A64_W19, g_off_cc + 1);
            }
            break;
        case RMW8_COM:
            EmitEorImm(e, A64_W3, A64_W0, 0xFF);
            if (mask & CC_BIT_C) EmitCcConst(e, 0, 1);
            if (mask & CC_BIT_V) EmitCcConst(e, 1, 0);
            break;
        case RMW8_LSR:
            if (mask & CC_BIT_C)
            {
                EmitAndImm(e, A64_W2, A64_W0, 1);
                emit_strb_imm(&e, A64_W2, A64_W19, g_off_cc + 0);
            }
            emit_lsr_w32_imm(&e, A64_W3, A64_W0, 1);
            break;
        case RMW8_ROR:
            emit_ldrb_imm(&e, A64_W4, A64_W19, g_off_cc + 0);
            emit_lsl_w32_imm(&e, A64_W4, A64_W4, 7);   // old C into bit 7
            if (mask & CC_BIT_C)
            {
                EmitAndImm(e, A64_W2, A64_W0, 1);
                emit_strb_imm(&e, A64_W2, A64_W19, g_off_cc + 0);
            }
            emit_lsr_w32_imm(&e, A64_W3, A64_W0, 1);
            emit_orr_w32(&e, A64_W3, A64_W3, A64_W4);
            break;
        case RMW8_ASR:
            if (mask & CC_BIT_C)
            {
                EmitAndImm(e, A64_W2, A64_W0, 1);
                emit_strb_imm(&e, A64_W2, A64_W19, g_off_cc + 0);
            }
            EmitAndImm(e, A64_W3, A64_W0, 0x80);
            emit_lsr_w32_imm(&e, A64_W2, A64_W0, 1);
            emit_orr_w32(&e, A64_W3, A64_W3, A64_W2);
            break;
        case RMW8_ASL:
        case RMW8_ROL:
        {
            // ROL's incoming bit is the OLD carry - load it before the
            // new carry overwrites cc[C]. W5 is free in this family.
            if (op == RMW8_ROL)
                emit_ldrb_imm(&e, A64_W5, A64_W19, g_off_cc + 0);
            emit_lsr_w32_imm(&e, A64_W2, A64_W0, 7);   // new C
            if (mask & CC_BIT_C)
                emit_strb_imm(&e, A64_W2, A64_W19, g_off_cc + 0);
            if (mask & CC_BIT_V)
            {
                // V = newC ^ old bit 6, computed BEFORE the shift.
                emit_lsr_w32_imm(&e, A64_W4, A64_W0, 6);
                EmitAndImm(e, A64_W4, A64_W4, 1);
                emit_eor_w32(&e, A64_W4, A64_W4, A64_W2);
                emit_strb_imm(&e, A64_W4, A64_W19, g_off_cc + 1);
            }
            emit_lsl_w32_imm(&e, A64_W3, A64_W0, 1);
            if (op == RMW8_ROL)
                emit_orr_w32(&e, A64_W3, A64_W3, A64_W5);
            EmitAndImm(e, A64_W3, A64_W3, 0xFF);
            break;
        }
        case RMW8_DEC:
            emit_sub_w32_imm(&e, A64_W3, A64_W0, 1);
            EmitAndImm(e, A64_W3, A64_W3, 0xFF);
            if (mask & CC_BIT_V)
            {
                emit_cmp_w32_imm(&e, A64_W3, 0x7F);
                emit_cset_w32(&e, A64_W2, A64_COND_EQ);
                emit_strb_imm(&e, A64_W2, A64_W19, g_off_cc + 1);
            }
            break;
        default:  // RMW8_INC
            emit_add_w32_imm(&e, A64_W3, A64_W0, 1);
            EmitAndImm(e, A64_W3, A64_W3, 0xFF);
            if (mask & CC_BIT_V)
            {
                emit_cmp_w32_imm(&e, A64_W3, 0x80);
                emit_cset_w32(&e, A64_W2, A64_COND_EQ);
                emit_strb_imm(&e, A64_W2, A64_W19, g_off_cc + 1);
            }
            break;
        }

        // Z/N from the result byte (LSR forces N=0 via the shifted
        // value naturally - bit 7 of a >>1 result is 0).
        if (mask & CC_BIT_Z)
        {
            emit_cmp_w32_imm(&e, A64_W3, 0);
            emit_cset_w32(&e, A64_W2, A64_COND_EQ);
            emit_strb_imm(&e, A64_W2, A64_W19, g_off_cc + 2);
        }
        if (mask & CC_BIT_N)
        {
            emit_lsr_w32_imm(&e, A64_W2, A64_W3, 7);
            emit_strb_imm(&e, A64_W2, A64_W19, g_off_cc + 3);
        }
        emit_orr_w32(&e, A64_W0, A64_WZR, A64_W3);     // data
        emit_ldrh_imm(&e, A64_W1, A64_W19, g_off_scratch16);
        EmitMemWrite8FP(e);
    }
    if (mode == MM_DP)       EmitCyclesRuntime(e, g_off_nat65);
    else if (mode == MM_IDX) EmitCyclesConst(e, 6);
    else                     EmitCyclesRuntime(e, g_off_nat76);
}

static bool TryEmitInline(emit_t& e, const DecodedInst& insn, uint8_t mask)
{
    if (g_no_inline) return false;
    if (g_inline_max < 999 && (InlineRank(insn.handler) >= g_inline_max))
        return false;
    // Lockstep invariant with AnalyzeFlagLiveness: an op the analysis
    // classifies CC_UNKNOWN has live_writes == 0 (meaning "called
    // handler"), NOT "no flags live". Inlining it would silently drop
    // its flag writes - the exact bug this guard exists to prevent.
    // Costs one table walk per emitted instruction, at emit time only.
    if (InlinedHandlerWritesMask(insn.handler) == CC_UNKNOWN)
        return false;
    const InstHandler h = insn.handler;
    if (h == g_inlines.lda_m)  { EmitInlineLd8Imm (e, insn, g_off_a, mask); return true; }
    if (h == g_inlines.ldb_m)  { EmitInlineLd8Imm (e, insn, g_off_b, mask); return true; }
    if (h == g_inlines.ldd_m)  { EmitInlineLd16Imm(e, insn, g_off_d, 3, 0, mask); return true; }
    if (h == g_inlines.ldx_m)  { EmitInlineLd16Imm(e, insn, g_off_x, 3, 0, mask); return true; }
    if (h == g_inlines.ldu_m)  { EmitInlineLd16Imm(e, insn, g_off_u, 3, 0, mask); return true; }
    if (h == g_inlines.lds_i)  { EmitInlineLd16Imm(e, insn, g_off_s, 4, 0, mask); return true; }
    if (h == g_inlines.ldy_m)  { EmitInlineLd16Imm(e, insn, g_off_y, -1, g_off_nat54, mask); return true; }
    if (h == g_inlines.clra_i) { EmitInlineClr8   (e, g_off_a, mask); return true; }
    if (h == g_inlines.clrb_i) { EmitInlineClr8   (e, g_off_b, mask); return true; }
    if (h == g_inlines.lda_d)  { EmitInlineLd8Mem (e, insn, g_off_a, true,  mask); return true; }
    if (h == g_inlines.ldb_d)  { EmitInlineLd8Mem (e, insn, g_off_b, true,  mask); return true; }
    if (h == g_inlines.sta_d)  { EmitInlineSt8Mem (e, insn, g_off_a, true,  mask); return true; }
    if (h == g_inlines.stb_d)  { EmitInlineSt8Mem (e, insn, g_off_b, true,  mask); return true; }
    if (h == g_inlines.lda_e)  { EmitInlineLd8Mem (e, insn, g_off_a, false, mask); return true; }
    if (h == g_inlines.ldb_e)  { EmitInlineLd8Mem (e, insn, g_off_b, false, mask); return true; }
    if (h == g_inlines.sta_e)  { EmitInlineSt8Mem (e, insn, g_off_a, false, mask); return true; }
    if (h == g_inlines.stb_e)  { EmitInlineSt8Mem (e, insn, g_off_b, false, mask); return true; }
    if (h == g_inlines.tsta_i) { EmitInlineTst8   (e, g_off_a, mask); return true; }
    if (h == g_inlines.tstb_i) { EmitInlineTst8   (e, g_off_b, mask); return true; }
    if (h == g_inlines.inca_i) { EmitInlineIncDec8(e, g_off_a, true,  mask); return true; }
    if (h == g_inlines.incb_i) { EmitInlineIncDec8(e, g_off_b, true,  mask); return true; }
    if (h == g_inlines.deca_i) { EmitInlineIncDec8(e, g_off_a, false, mask); return true; }
    if (h == g_inlines.decb_i) { EmitInlineIncDec8(e, g_off_b, false, mask); return true; }
    if (h == g_inlines.coma_i) { EmitInlineCom8   (e, g_off_a, mask); return true; }
    if (h == g_inlines.comb_i) { EmitInlineCom8   (e, g_off_b, mask); return true; }
    if (h == g_inlines.nega_i) { EmitInlineNeg8   (e, g_off_a, mask); return true; }
    if (h == g_inlines.negb_i) { EmitInlineNeg8   (e, g_off_b, mask); return true; }
    if (h == g_inlines.lsra_i) { EmitInlineLsr8   (e, g_off_a, mask); return true; }
    if (h == g_inlines.asra_i) { EmitInlineAsr8   (e, g_off_a, mask); return true; }
    if (h == g_inlines.asla_i) { EmitInlineAsl8   (e, g_off_a, mask); return true; }
    if (h == g_inlines.abx_i)  { EmitInlineAbx    (e); return true; }
    if (h == g_inlines.anda_m) { EmitInlineLogic8Imm(e, insn, g_off_a, LogicOp::And, true,  mask); return true; }
    if (h == g_inlines.andb_m) { EmitInlineLogic8Imm(e, insn, g_off_b, LogicOp::And, true,  mask); return true; }
    if (h == g_inlines.ora_m)  { EmitInlineLogic8Imm(e, insn, g_off_a, LogicOp::Or,  true,  mask); return true; }
    if (h == g_inlines.orb_m)  { EmitInlineLogic8Imm(e, insn, g_off_b, LogicOp::Or,  true,  mask); return true; }
    if (h == g_inlines.eora_m) { EmitInlineLogic8Imm(e, insn, g_off_a, LogicOp::Eor, true,  mask); return true; }
    if (h == g_inlines.eorb_m) { EmitInlineLogic8Imm(e, insn, g_off_b, LogicOp::Eor, true,  mask); return true; }
    if (h == g_inlines.bita_m) { EmitInlineLogic8Imm(e, insn, g_off_a, LogicOp::And, false, mask); return true; }
    if (h == g_inlines.bitb_m) { EmitInlineLogic8Imm(e, insn, g_off_b, LogicOp::And, false, mask); return true; }
    if (h == g_inlines.cmpa_m) { EmitInlineArith8Imm(e, insn, g_off_a, false, false, mask); return true; }
    if (h == g_inlines.cmpb_m) { EmitInlineArith8Imm(e, insn, g_off_b, false, false, mask); return true; }
    if (h == g_inlines.suba_m) { EmitInlineArith8Imm(e, insn, g_off_a, false, true,  mask); return true; }
    if (h == g_inlines.subb_m) { EmitInlineArith8Imm(e, insn, g_off_b, false, true,  mask); return true; }
    if (h == g_inlines.adda_m) { EmitInlineArith8Imm(e, insn, g_off_a, true,  true,  mask); return true; }
    if (h == g_inlines.addb_m) { EmitInlineArith8Imm(e, insn, g_off_b, true,  true,  mask); return true; }
    if ((h == g_inlines.leax_x || h == g_inlines.leay_x ||
         h == g_inlines.leau_x || h == g_inlines.leas_x ||
         h == g_inlines.lda_x  || h == g_inlines.ldb_x  ||
         h == g_inlines.sta_x  || h == g_inlines.stb_x  ||
         h == g_inlines.tst_x  ||
         h == g_inlines.ldx_x  || h == g_inlines.ldu_x  ||
         h == g_inlines.ldd_x  || h == g_inlines.std_x  ||
         h == g_inlines.stx_x) && EAModeSupported(insn.ea_info))
    {
        if (h == g_inlines.leax_x) { EmitInlineLea(e, insn, g_off_x, true,  mask); return true; }
        if (h == g_inlines.leay_x) { EmitInlineLea(e, insn, g_off_y, true,  mask); return true; }
        if (h == g_inlines.leau_x) { EmitInlineLea(e, insn, g_off_u, false, mask); return true; }
        if (h == g_inlines.leas_x) { EmitInlineLea(e, insn, g_off_s, false, mask); return true; }
        if (h == g_inlines.lda_x)  { EmitInlineLd8Idx(e, insn, g_off_a, mask); return true; }
        if (h == g_inlines.ldb_x)  { EmitInlineLd8Idx(e, insn, g_off_b, mask); return true; }
        if (h == g_inlines.sta_x)  { EmitInlineSt8Idx(e, insn, g_off_a, mask); return true; }
        if (h == g_inlines.stb_x)  { EmitInlineSt8Idx(e, insn, g_off_b, mask); return true; }
        if (h == g_inlines.tst_x)  { EmitInlineTstIdx(e, insn, mask); return true; }
        if (h == g_inlines.ldx_x)  { EmitInlineLd16Idx(e, insn, g_off_x, mask); return true; }
        if (h == g_inlines.ldu_x)  { EmitInlineLd16Idx(e, insn, g_off_u, mask); return true; }
        if (h == g_inlines.ldd_x)  { EmitInlineLd16Idx(e, insn, g_off_d, mask); return true; }
        if (h == g_inlines.std_x)  { EmitInlineSt16Idx(e, insn, g_off_d, mask); return true; }
        if (h == g_inlines.stx_x)  { EmitInlineSt16Idx(e, insn, g_off_x, mask); return true; }
    }
    // 8-bit memory-operand ALU and RMW families (dp/idx/ext).
    if (std::getenv("VCC_NO_FAM_C") == nullptr)
    {
        using namespace BlockJit;
        for (int m = 0; m < MM_COUNT; ++m)
        {
            if (m == MM_IDX && !EAModeSupported(insn.ea_info))
                continue;
            if (std::getenv("VCC_NO_FAM_ALU8") == nullptr)
            for (int op = 0; op < ALU8_OP_COUNT; ++op)
            {
                if (h == g_inlines.alu8_mem_a[op][m])
                { EmitInlineAlu8Mem(e, insn, op, g_off_a, m, mask); return true; }
                if (h == g_inlines.alu8_mem_b[op][m])
                { EmitInlineAlu8Mem(e, insn, op, g_off_b, m, mask); return true; }
            }
            if (std::getenv("VCC_NO_FAM_RMW8") == nullptr)
            for (int op = 0; op < RMW8_OP_COUNT; ++op)
                if (h == g_inlines.rmw8_mem[op][m])
                { EmitInlineRmw8(e, insn, op, m, mask); return true; }
        }
    }

    // 16-bit loads/stores, dp/ext (fastmem).
    if (std::getenv("VCC_NO_FAM_LDST16") == nullptr)
    {
        using namespace BlockJit;
        static const uint32_t* kRegOff16[R16_COUNT] = { &g_off_d, &g_off_x, &g_off_u };
        for (int r = 0; r < R16_COUNT; ++r)
        {
            if (h == g_inlines.ld16_dp[r])  { EmitInlineLd16Mem(e, insn, *kRegOff16[r], MM_DP,  mask); return true; }
            if (h == g_inlines.ld16_ext[r]) { EmitInlineLd16Mem(e, insn, *kRegOff16[r], MM_EXT, mask); return true; }
            if (h == g_inlines.st16_dp[r])  { EmitInlineSt16Mem(e, insn, *kRegOff16[r], MM_DP,  mask); return true; }
            if (h == g_inlines.st16_ext[r]) { EmitInlineSt16Mem(e, insn, *kRegOff16[r], MM_EXT, mask); return true; }
        }
    }

    // 16-bit arithmetic; cycle charges mirror each handler.
    if (std::getenv("VCC_NO_FAM_ARITH16") == nullptr)
    {
        using namespace BlockJit;
        const bool ea_ok = EAModeSupported(insn.ea_info);
        if (h == g_inlines.addd_m) { EmitInlineArith16(e, insn, g_off_d, A16_ADD, -1,     g_off_nat43, mask); return true; }
        if (h == g_inlines.addd_d) { EmitInlineArith16(e, insn, g_off_d, A16_ADD, MM_DP,  g_off_nat64, mask); return true; }
        if (h == g_inlines.addd_x && ea_ok)
                                   { EmitInlineArith16(e, insn, g_off_d, A16_ADD, MM_IDX, g_off_nat65, mask); return true; }
        if (h == g_inlines.addd_e) { EmitInlineArith16(e, insn, g_off_d, A16_ADD, MM_EXT, g_off_nat76, mask); return true; }
        if (h == g_inlines.subd_m) { EmitInlineArith16(e, insn, g_off_d, A16_SUB, -1,     g_off_nat43, mask); return true; }
        if (h == g_inlines.subd_x && ea_ok)
                                   { EmitInlineArith16(e, insn, g_off_d, A16_SUB, MM_IDX, g_off_nat65, mask); return true; }
        if (h == g_inlines.cmpd_m) { EmitInlineArith16(e, insn, g_off_d, A16_CMP, -1,     g_off_nat54, mask); return true; }
        if (h == g_inlines.cmpd_x && ea_ok)
                                   { EmitInlineArith16(e, insn, g_off_d, A16_CMP, MM_IDX, g_off_nat76, mask); return true; }
        if (h == g_inlines.cmpx_m) { EmitInlineArith16(e, insn, g_off_x, A16_CMP, -1,     g_off_nat43, mask); return true; }
        if (h == g_inlines.cmpx_x && ea_ok)
                                   { EmitInlineArith16(e, insn, g_off_x, A16_CMP, MM_IDX, g_off_nat65, mask); return true; }
        if (h == g_inlines.cmpy_m) { EmitInlineArith16(e, insn, g_off_y, A16_CMP, -1,     g_off_nat54, mask); return true; }
        if (h == g_inlines.cmpu_m) { EmitInlineArith16(e, insn, g_off_u, A16_CMP, -1,     g_off_nat54, mask); return true; }
        if (h == g_inlines.cmpu_x && ea_ok)
                                   { EmitInlineArith16(e, insn, g_off_u, A16_CMP, MM_IDX, g_off_nat76, mask); return true; }
    }

    return false;
}

// ---------- block emitter ----------

// ---------- chain stub (block linking) ----------
//
// One shared stub per arena epoch. Every thunk's tail branches here
// instead of returning. The stub revalidates exactly what the
// dispatcher fast path would - cycle budget, pending interrupt/sync/
// halt work, and the cache slot for the new PC (tag + generation +
// native_entry) - then tail-jumps into the next thunk. Any failed
// check falls through to RET, which lands back in the dispatcher:
// every thunk's epilogue has already restored the frame (including
// the dispatcher's x30) before branching here, so stack depth stays
// constant across a chain of any length.
//
// Linking is indirect through the live cache slot, never a patched
// address: HD6309BlockInvalidate clearing a slot, or a generation
// bump, severs every link to it with no unlink bookkeeping.

// The registerized-state runner: C-callable entry that establishes the
// chain register convention - w21 = CycleCounter, w22 = CycleFor,
// x23 = &cpu_state, x24 = cache slot base - calls the thunk in x0, and
// spills w21 back when the thunk (or the chain it started) finally
// returns. All four are callee-saved, so thunk-internal calls to
// handlers and mem_read/mem_write preserve them for free. Thunks and
// the chain stub treat them as live throughout, which is also why any
// thunk entry MUST come through here.
static void EnsureThunkRunner()
{
    if (g_thunk_runner != nullptr)
        return;
    if (g_addrs.cycle_for == nullptr)
        return;

    constexpr size_t kRunnerBytes = 112;
    if (g_arena_used + kRunnerBytes > kArenaSize)
        return;

    uint8_t* const entry = g_arena_base + g_arena_used;
    emit_t e { entry, 0, (uint32_t)kRunnerBytes };
    const int64_t off_cycle_for = (const uint8_t*)g_addrs.cycle_for
                                - (const uint8_t*)g_addrs.base;
    if (off_cycle_for < 0 || off_cycle_for >= 4096 * 4 || (off_cycle_for % 4) != 0)
        return;

    jit_writable_begin();
    emit_stp_pre_sp(&e, A64_W29, A64_W30, -48);
    emit_stp_x64_off(&e, A64_W21, A64_W22, A64_SP, 16);
    emit_stp_x64_off(&e, A64_W23, A64_W24, A64_SP, 32);
    emit_add_x64_imm(&e, A64_W29, A64_SP, 0);
    emit_mov_x64_imm64(&e, A64_W23, (uint64_t)(uintptr_t)g_addrs.base);
    emit_mov_x64_imm64(&e, A64_W24, (uint64_t)(uintptr_t)g_addrs.chain_slot_base);
    emit_ldr_w32_imm(&e, A64_W21, A64_W23, g_off_cyc);
    emit_ldr_w32_imm(&e, A64_W22, A64_W23, (uint32_t)off_cycle_for);
    emit_blr(&e, A64_W0);
    emit_str_w32_imm(&e, A64_W21, A64_W23, g_off_cyc);
    emit_ldp_x64_off(&e, A64_W21, A64_W22, A64_SP, 16);
    emit_ldp_x64_off(&e, A64_W23, A64_W24, A64_SP, 32);
    emit_ldp_x64_post(&e, A64_W29, A64_W30, A64_SP, 48);
    emit_ret(&e);
    jit_writable_end();

    if (e.offset > e.capacity)
        return;
    __builtin___clear_cache((char*)entry, (char*)entry + e.offset);
    g_arena_used += e.offset;
    g_thunk_runner = entry;
}

static void EnsureChainStub()
{
    if (g_chain_stub != nullptr || g_no_link)
        return;
    if (g_addrs.chain_slot_base == nullptr || g_addrs.cycle_for == nullptr ||
        g_addrs.chain_slot_size == 0 || g_addrs.chain_slot_size > 0xFFFF)
    {
        g_no_link = true;   // core didn't provide the context
        return;
    }

    // The stub loads its operands base-relative off cpu_state - the
    // core keeps them there (Hd6309State chain-context fields). Verify
    // the layout it depends on: everything within immediate-load range
    // of base, the five pending flags packed into one zero-padded
    // 8-aligned quadword, the generation mirror in place, and the slot
    // stride a power of two so indexing is a shift.
    const uint8_t* cbase = (const uint8_t*)g_addrs.base;
    const int64_t off_cycle_for = (const uint8_t*)g_addrs.cycle_for         - cbase;
    const int64_t off_pending   = (const uint8_t*)g_addrs.chain_break       - cbase;
    const int64_t off_gen       = (const uint8_t*)g_addrs.generation_mirror - cbase;
    const int64_t off_runs      = (const uint8_t*)g_addrs.chain_runs        - cbase;
    auto in_range = [](int64_t off, int align) {
        return off >= 0 && off < 4096 * align && (off % align) == 0;
    };
    const uint32_t ssz = g_addrs.chain_slot_size;
    const bool size_pow2 = (ssz & (ssz - 1)) == 0;
    if (!in_range(off_cycle_for, 4) || !in_range(off_pending, 1) ||
        !in_range(off_gen, 4) || !in_range(off_runs, 8) || !size_pow2)
    {
        g_no_link = true;   // layout contract not met; run unlinked
        return;
    }
    uint32_t slot_shift = 0;
    while ((1u << slot_shift) < ssz) ++slot_shift;

    constexpr size_t kStubBytes = 320;
    if (g_arena_used + kStubBytes > kArenaSize)
        return;             // retried after the next arena flush

    uint8_t* const entry = g_arena_base + g_arena_used;
    emit_t e { entry, 0, (uint32_t)kStubBytes };

    // Forward branches to the RET at the end, patched once its position
    // is known. kind: 0 = b.cond, 1 = cbnz w, 2 = cbz x, 3 = cbnz x.
    struct Fixup { uint32_t at; int kind; a64_cond_t cond; a64_reg_t reg; };
    Fixup fixups[8];
    int nfix = 0;

    jit_writable_begin();

    // Budget exhausted -> bail. CycleCounter and CycleFor are LIVE in
    // w21/w22 and &cpu_state in x23 (runner convention) - no imm64
    // materializations and no state loads at all.
    emit_cmp_w32_w32(&e, A64_W21, A64_W22);
    fixups[nfix++] = { e.offset, 0, A64_COND_GE, A64_W0 };
    emit_b_cond(&e, A64_COND_GE, 0);

    // Any reason to leave the chain - a DELIVERABLE interrupt, SYNC
    // wait, or halted-insn - is the maintained ChainBreak byte.
    // Masked-but-asserted lines do not set it, so chains keep flowing
    // through code that runs with interrupts masked.
    emit_ldrb_imm(&e, A64_W12, A64_W23, (uint32_t)off_pending);
    fixups[nfix++] = { e.offset, 1, A64_COND_EQ, A64_W12 };
    emit_cbnz_w32(&e, A64_W12, 0);

    // w15 = next PC; x14 = &slot = slot_base(x24) + (pc & MASK) << shift
    emit_ldrh_imm(&e, A64_W15, A64_W23, g_off_pc);
    EmitAndImm(e, A64_W16, A64_W15, (uint32_t)BlockCache::CACHE_MASK);
    emit_lsl_x64_imm(&e, A64_W16, A64_W16, slot_shift);
    emit_add_x64(&e, A64_W14, A64_W24, A64_W16);

    // Slot must hold this PC in the current generation, with a thunk.
    emit_ldrh_imm(&e, A64_W17, A64_W14, (uint32_t)offsetof(CachedBlock, start_pc));
    emit_cmp_w32_w32(&e, A64_W17, A64_W15);
    fixups[nfix++] = { e.offset, 0, A64_COND_NE, A64_W0 };
    emit_b_cond(&e, A64_COND_NE, 0);
    emit_ldr_w32_imm(&e, A64_W17, A64_W14, (uint32_t)offsetof(CachedBlock, generation));
    emit_ldr_w32_imm(&e, A64_W12, A64_W23, (uint32_t)off_gen);
    emit_cmp_w32_w32(&e, A64_W17, A64_W12);
    fixups[nfix++] = { e.offset, 0, A64_COND_NE, A64_W0 };
    emit_b_cond(&e, A64_COND_NE, 0);

    // Whole next block must fit the budget with the shared overshoot
    // slack: cycles + total - kBudgetSlack <= CycleFor (same test as
    // the dispatcher's total_cycles <= remaining + slack). The
    // subtraction can go negative; the compare is signed.
    emit_ldrb_imm(&e, A64_W17, A64_W14, (uint32_t)offsetof(CachedBlock, total_cycles));
    emit_add_w32(&e, A64_W17, A64_W17, A64_W21);
    emit_sub_w32_imm(&e, A64_W17, A64_W17, (uint32_t)kBudgetSlack);
    emit_cmp_w32_w32(&e, A64_W17, A64_W22);
    fixups[nfix++] = { e.offset, 0, A64_COND_GT, A64_W0 };
    emit_b_cond(&e, A64_COND_GT, 0);

    emit_ldr_x64_imm(&e, A64_W16, A64_W14, (uint32_t)offsetof(CachedBlock, native_entry));
    fixups[nfix++] = { e.offset, 2, A64_COND_EQ, A64_W16 };
    emit_cbz_x64(&e, A64_W16, 0);

    // Transition counter, only when stats were asked for (a dependent
    // load-add-store on every chain hop is real money otherwise).
    static const bool jit_stats = std::getenv("VCC_JIT_STATS") != nullptr;
    if (jit_stats)
    {
        emit_ldr_x64_imm(&e, A64_W13, A64_W23, (uint32_t)off_runs);
        emit_add_x64_imm(&e, A64_W13, A64_W13, 1);
        emit_str_x64_imm(&e, A64_W13, A64_W23, (uint32_t)off_runs);
    }

    emit_br(&e, A64_W16);

    // bail: back to the dispatcher.
    const uint32_t bail_at = e.offset;
    emit_ret(&e);

    for (int i = 0; i < nfix; ++i)
    {
        emit_t p { entry + fixups[i].at, 0, 4 };
        const int32_t delta = (int32_t)(bail_at - fixups[i].at);
        switch (fixups[i].kind)
        {
        case 0: emit_b_cond(&p, fixups[i].cond, delta); break;
        case 1: emit_cbnz_w32(&p, fixups[i].reg, delta); break;
        case 2: emit_cbz_x64(&p, fixups[i].reg, delta); break;
        default: emit_cbnz_x64(&p, fixups[i].reg, delta); break;
        }
    }

    jit_writable_end();

    if (e.offset > e.capacity)
    {
        // Should be impossible with the 320-byte budget; refuse to link
        // rather than run truncated code.
        g_no_link = true;
        return;
    }

    __builtin___clear_cache((char*)entry, (char*)entry + e.offset);
    g_arena_used += e.offset;
    g_chain_stub = entry;
}

NativeEntry EmitBlock(const CachedBlock& slot)
{
    if (g_disabled || g_arena_base == nullptr ||
        g_addrs.base == nullptr || g_addrs.pc == nullptr)
        return nullptr;

    // Thunks assume the registerized-state convention (CycleCounter in
    // w21); without the runner to establish it, refuse to emit.
    EnsureThunkRunner();
    if (g_thunk_runner == nullptr)
        return nullptr;
    EnsureChainStub();

    const size_t needed =
        kPrologueBytes + (size_t)slot.num_insns * kMaxBytesPerInsn + kEpilogueBytes;
    if (g_arena_used + needed > kArenaSize)
    {
        ++g_emit_failures;
        return nullptr;
    }

    // Backward liveness pass for cc[] writes - same analysis as the
    // x86-32 backend; the forward pass emits only the live subset.
    uint8_t live_writes[BlockCache::MAX_BLOCK_INSNS];
    uint32_t cc_requested = 0;
    uint32_t cc_elided    = 0;
    AnalyzeFlagLiveness(slot, live_writes, &cc_requested, &cc_elided);
    g_cc_writes_requested += cc_requested;
    g_cc_writes_elided    += cc_elided;

    uint8_t* const entry = g_arena_base + g_arena_used;
    emit_t e { entry, 0, (uint32_t)needed };
    g_emit_had_side_effects = false;

    jit_writable_begin();

    // Prologue. x23 already holds &cpu_state (runner convention), so
    // x19 is one register move instead of an imm64 materialization.
    emit_stp_pre_sp(&e, A64_W29, A64_W30, -32);
    emit_stp_x64_off(&e, A64_W19, A64_W20, A64_SP, 16);
    emit_add_x64_imm(&e, A64_W29, A64_SP, 0);   // mov x29, sp (ORR would read XZR)
    emit_orr_x64_lsl(&e, A64_W19, A64_XZR, A64_W23, 0);   // mov x19, x23
    emit_mov_x64_imm64(&e, A64_W20, (uint64_t)(uintptr_t)&slot.insns[0]);

    // PC-skipping state, same policy as the x86 backend: inlined ops
    // don't read PC, so the write is deferred until the next handler
    // call (which needs the post-insn PC pre-set) or the block tail.
    uint16_t local_pc = slot.start_pc;
    bool     pc_dirty = false;

    // Superblock guard exits: each mid-block branch that goes its
    // taken way must store its taken PC and leave through the block
    // tail. The conditional jumps are emitted with placeholder offsets
    // and patched once the exit snippets exist after the main tail.
    // kind: 0 = cbz w1, 1 = cbnz w1 (guard direction diverged),
    //       2 = b.ne (called guard, bare exit),
    //       3 = cbnz x12 (interrupt pending at a loop back-edge; exit
    //           along the RECORDED path so the dispatcher can deliver).
    struct GuardExit
    {
        uint32_t branch_at;   // offset of the branch insn to patch
        uint8_t  kind;
        uint16_t exit_pc;     // PC to store on exit (kinds 0/1/3)
        uint32_t extra_cycles;// exit-minus-recorded cycle delta (long branches)
    };
    GuardExit guard_exits[BlockCache::MAX_BLOCK_INSNS * 2];   // direction + pending per guard
    int num_guard_exits = 0;

    for (int i = 0; i < (int)slot.num_insns; ++i)
    {
        const DecodedInst& insn = slot.insns[i];
        local_pc = (uint16_t)(local_pc + insn.length);

        // Branch terminators inline as the last instruction only: the
        // baked fall-through/taken PCs assume the block ends here.
        BranchDesc bdesc;
        if (i == (int)slot.num_insns - 1 && !g_no_inline &&
            LookupBranch(insn.handler, bdesc))
        {
            EmitInlineBranch(e, insn, bdesc, local_pc);
            pc_dirty = false;   // the branch stored PC itself
            ++g_insns_inlined;
            ++g_pc_writes_skipped;
            continue;
        }

        // Mid-block branch = trace guard. taken_mask says which way the
        // recording went: bit clear = fall-through (exit if taken), bit
        // set = taken (the trace FOLLOWS the branch; exit if it falls
        // through). Either exit stores the not-recorded successor PC
        // and leaves through the block tail.
        if (i < (int)slot.num_insns - 1 && LookupBranch(insn.handler, bdesc))
        {
            const bool taken_dir = ((slot.taken_mask >> i) & 1u) != 0;
            const uint16_t taken_pc = bdesc.is_long
                ? (uint16_t)(local_pc + (int16_t)insn.operand)
                : (uint16_t)(local_pc + (int8_t)(insn.operand & 0xFF));
            const uint16_t fall_pc = local_pc;
            // The PC the recorded path continues at, and the PC a
            // diverging outcome exits with.
            const uint16_t cont_pc = taken_dir ? taken_pc : fall_pc;
            const uint16_t exit_pc = taken_dir ? fall_pc : taken_pc;

            // Impossible direction/predicate pairs (Always not-taken,
            // Never taken) mean corrupted block data - refuse.
            if ((bdesc.pred == BranchPred::Always && !taken_dir) ||
                (bdesc.pred == BranchPred::Never && taken_dir))
            {
                ++g_emit_failures;
                return nullptr;
            }

            if (!g_no_inline)
            {
                if (bdesc.pred == BranchPred::Never ||
                    bdesc.pred == BranchPred::Always)
                {
                    // BRN (fall) / BRA (taken): a costed unconditional,
                    // no exit possible.
                    if (bdesc.is_long)
                        EmitCyclesRuntime(e, g_off_nat54);
                    else
                        EmitCyclesConst(e, 3);
                }
                else
                {
                    EmitBranchPredicate(e, bdesc.pred);
                    // Recorded-path cycles now; the exit snippet adds
                    // the delta (long branches: taken costs one more).
                    const uint32_t cont_cyc = bdesc.is_long ? (taken_dir ? 6u : 5u) : 3u;
                    const uint32_t exit_cyc = bdesc.is_long ? (taken_dir ? 5u : 6u) : 3u;
                    EmitCyclesConst(e, cont_cyc);
                    // Exit when the runtime outcome is NOT the recorded
                    // direction: for fall-recordings that's "taken", for
                    // taken-recordings it's "not taken" - which flips
                    // the cbz/cbnz sense.
                    const bool exit_on_zero = taken_dir ? !bdesc.taken_when_zero
                                                        : bdesc.taken_when_zero;
                    guard_exits[num_guard_exits++] = {
                        e.offset, (uint8_t)(exit_on_zero ? 0 : 1), exit_pc,
                        (exit_cyc > cont_cyc) ? exit_cyc - cont_cyc : 0u };
                    if (exit_on_zero)
                        emit_cbz_w32(&e, A64_W1, 0);
                    else
                        emit_cbnz_w32(&e, A64_W1, 0);
                }
                // Loop back-edge (taken direction): bound interrupt
                // latency to one loop iteration - if the dispatcher
                // would act (ChainBreak byte: deliverable interrupt,
                // sync, halt; NOT masked lines), exit along the
                // RECORDED path and let it. One load + one branch.
                if (taken_dir && g_off_pending >= 0)
                {
                    emit_ldrb_imm(&e, A64_W12, A64_W19, (uint32_t)g_off_pending);
                    guard_exits[num_guard_exits++] = {
                        e.offset, 3, cont_pc, 0u };
                    emit_cbnz_w32(&e, A64_W12, 0);
                }
                local_pc = cont_pc;
                pc_dirty = true;    // recorded-path PC still deferred
                ++g_insns_inlined;
                ++g_pc_writes_skipped;
                continue;
            }

            // Inlining off: call the handler, then check whether it
            // left the recorded path. PC and cycles are already correct
            // either way, so the exit is bare.
            g_emit_had_side_effects = true;
            emit_movz_w32(&e, A64_W1, fall_pc, 0);
            emit_strh_imm(&e, A64_W1, A64_W19, g_off_pc);
            ++g_pc_writes_emitted;
            emit_str_w32_imm(&e, A64_W21, A64_W19, g_off_cyc);
            emit_add_x64_imm(&e, A64_W0, A64_W20, (uint32_t)(i * sizeof(DecodedInst)));
            emit_mov_x64_imm64(&e, A64_W16, (uint64_t)(uintptr_t)insn.handler);
            emit_blr(&e, A64_W16);
            emit_ldr_w32_imm(&e, A64_W21, A64_W19, g_off_cyc);
            emit_ldrh_imm(&e, A64_W2, A64_W19, g_off_pc);
            emit_movz_w32(&e, A64_W3, cont_pc, 0);
            emit_cmp_w32_w32(&e, A64_W2, A64_W3);
            guard_exits[num_guard_exits++] = { e.offset, 2, 0, 0u };
            emit_b_cond(&e, A64_COND_NE, 0);
            local_pc = cont_pc;
            pc_dirty = false;
            ++g_insns_called;
            continue;
        }

        // A taken_mask bit on anything but a recognized branch means
        // the PC chain below would be wrong - refuse the block.
        if (i < (int)slot.num_insns - 1 && ((slot.taken_mask >> i) & 1u))
        {
            ++g_emit_failures;
            return nullptr;
        }

        // JMP indexed as terminator: PC = EA, +3 cycles.
        if (i == (int)slot.num_insns - 1 && !g_no_inline &&
            insn.handler == g_inlines.jmp_x && EAModeSupported(insn.ea_info))
        {
            EmitEA(e, insn.ea_info, insn.operand, A64_W0);
            emit_strh_imm(&e, A64_W0, A64_W19, g_off_pc);
            EmitCyclesConst(e, 3);
            pc_dirty = false;
            ++g_insns_inlined;
            ++g_pc_writes_skipped;
            continue;
        }

        if (TryEmitInline(e, insn, live_writes[i]))
        {
            pc_dirty = true;
            ++g_pc_writes_skipped;
            ++g_insns_inlined;
        }
        else
        {
            // Pre-set PC to the post-instruction value, exactly as the
            // interpreter dispatch loop would before the handler call.
            emit_movz_w32(&e, A64_W1, local_pc, 0);
            emit_strh_imm(&e, A64_W1, A64_W19, g_off_pc);
            ++g_pc_writes_emitted;
            pc_dirty = false;

            // handler(&slot.insns[i]) - handlers read and add cycles in
            // cpu_state, so sync the registerized counter around the call.
            g_emit_had_side_effects = true;
            emit_str_w32_imm(&e, A64_W21, A64_W19, g_off_cyc);
            emit_add_x64_imm(&e, A64_W0, A64_W20, (uint32_t)(i * sizeof(DecodedInst)));
            emit_mov_x64_imm64(&e, A64_W16, (uint64_t)(uintptr_t)insn.handler);
            emit_blr(&e, A64_W16);
            emit_ldr_w32_imm(&e, A64_W21, A64_W19, g_off_cyc);
            ++g_insns_called;
        }
    }

    // Flush PC if the block ended on an inlined op, so the dispatcher's
    // next-block lookup sees the right address.
    if (pc_dirty)
    {
        emit_movz_w32(&e, A64_W1, local_pc, 0);
        emit_strh_imm(&e, A64_W1, A64_W19, g_off_pc);
        ++g_pc_writes_emitted;
    }

    // Epilogue, then chain: with the frame restored (dispatcher's x30
    // back in place), branch to the shared stub, which either tail-
    // jumps into the next block's thunk or RETs to the dispatcher.
    emit_ldp_x64_off(&e, A64_W19, A64_W20, A64_SP, 16);
    emit_ldp_x64_post(&e, A64_W29, A64_W30, A64_SP, 32);
    if (g_chain_stub != nullptr)
        emit_b(&e, (int32_t)(g_chain_stub - (entry + e.offset)));
    else
        emit_ret(&e);

    // Guard exit snippets: store the taken PC (bare exits already have
    // PC and cycles right), pop the frame, and leave through the same
    // door as the main tail. Then patch each guard's conditional jump.
    uint32_t bare_exit_at = 0;
    bool bare_exit_emitted = false;
    for (int gi = 0; gi < num_guard_exits; ++gi)
    {
        GuardExit& ge = guard_exits[gi];
        uint32_t target;
        if (ge.kind == 2)
        {
            if (!bare_exit_emitted)
            {
                bare_exit_at = e.offset;
                bare_exit_emitted = true;
                emit_ldp_x64_off(&e, A64_W19, A64_W20, A64_SP, 16);
                emit_ldp_x64_post(&e, A64_W29, A64_W30, A64_SP, 32);
                if (g_chain_stub != nullptr)
                    emit_b(&e, (int32_t)(g_chain_stub - (entry + e.offset)));
                else
                    emit_ret(&e);
            }
            target = bare_exit_at;
        }
        else
        {
            target = e.offset;
            emit_movz_w32(&e, A64_W2, ge.exit_pc, 0);
            emit_strh_imm(&e, A64_W2, A64_W19, g_off_pc);
            if (ge.extra_cycles != 0)
                emit_add_w32_imm(&e, A64_W21, A64_W21, ge.extra_cycles);
            emit_ldp_x64_off(&e, A64_W19, A64_W20, A64_SP, 16);
            emit_ldp_x64_post(&e, A64_W29, A64_W30, A64_SP, 32);
            if (g_chain_stub != nullptr)
                emit_b(&e, (int32_t)(g_chain_stub - (entry + e.offset)));
            else
                emit_ret(&e);
        }

        emit_t p { entry + ge.branch_at, 0, 4 };
        const int32_t delta = (int32_t)(target - ge.branch_at);
        switch (ge.kind)
        {
        case 0:  emit_cbz_w32(&p, A64_W1, delta); break;
        case 1:  emit_cbnz_w32(&p, A64_W1, delta); break;
        case 2:  emit_b_cond(&p, A64_COND_NE, delta); break;
        default: emit_cbnz_w32(&p, A64_W12, delta); break;
        }
    }

    jit_writable_end();

    if (e.offset > e.capacity)
    {
        // Budget bug: refuse the block rather than run truncated code.
        ++g_emit_failures;
        return nullptr;
    }

    __builtin___clear_cache((char*)entry, (char*)entry + e.offset);


    g_arena_used += e.offset;
    ++g_blocks_emitted;

    return reinterpret_cast<NativeEntry>(entry);
}

// True when the most recent EmitBlock produced a thunk with no memory
// accesses or handler calls - safe to run twice from a snapshot for
// differential verification (VCC_VERIFY_PURE in hd6309.cpp).
ThunkRunner GetThunkRunner()
{
    return (ThunkRunner)(void*)g_thunk_runner;
}

bool EmitBlockWasPure()
{
    return !g_emit_had_side_effects;
}

Stats GetStats()
{
    Stats s;
    s.arena_size          = kArenaSize;
    s.arena_used          = g_arena_used;
    s.blocks_emitted      = g_blocks_emitted;
    s.emit_failures       = g_emit_failures;
    s.insns_called        = g_insns_called;
    s.insns_inlined       = g_insns_inlined;
    s.pc_writes_emitted   = g_pc_writes_emitted;
    s.pc_writes_skipped   = g_pc_writes_skipped;
    s.cc_writes_requested = g_cc_writes_requested;
    s.cc_writes_elided    = g_cc_writes_elided;
    return s;
}

} // namespace BlockJit
