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

// x86-64 backend for the Level-1/2 JIT (see BlockJit.h for the contract,
// BlockJitA64.cpp for the arm64 sibling this mirrors 1:1, and the retired
// x86-32 BlockJit.cpp for the flag-capture idioms it inherits). Runs on
// both Win64 and SysV x86-64; the only ABI differences are the two
// argument registers and Win64's 32-byte shadow space, both handled at
// compile time below.
//
// Register convention (all callee-saved on BOTH ABIs, established by the
// emitted thunk runner exactly like the arm64 backend's w21/w22/x23/x24):
//   R13  = &cpu_state (Hd6309State base) — every field is [r13 + disp]
//   EBX  = CycleCounter (registerized across whole chains)
//   BPL  = A_REG, R12B = B_REG (registerized accumulators; handler call
//          sites spill/reload them, MMU fallback calls don't need to)
//   R15  = block-cache slot base (chain stub indexing)
//   R14  = &slot.insns[0] (per-thunk, movabs in the prologue; the runner
//          saves/restores the dispatcher's r14 around the whole chain)
//   RAX, RCX, RDX, R8-R11 = scratch (caller-saved on both ABIs; RSI/RDI
//          are deliberately untouched except as SysV argument registers)
// CycleFor is read from memory in the chain stub (two loads per hop)
// so R12 can carry the B accumulator instead.
//
// Thunk shape:
//   sub  rsp, 40                ; Win64 shadow (32) + alignment (8);
//                               ; harmless overallocation on SysV
//   movabs r14, &slot.insns[0]
//   per instruction: inline body, or
//     mov  word [r13+PC], post_insn_pc
//     mov  [r13+CYC], ebx       ; handlers see the memory CycleCounter
//     lea  ARG0, [r14 + i*sizeof(DecodedInst)]
//     movabs rax, handler ; call rax
//     mov  ebx, [r13+CYC]
//   add  rsp, 40
//   jmp  chain_stub             ; or ret when linking is off
//
// Because the tail branch happens after add rsp, stack depth is constant
// across a chain of any length and the chain stub's RET lands in the
// runner. x86 icache is coherent with data writes, so unlike arm64 there
// is no cache-flush or W^X bracketing; the arena is RWX for its lifetime
// (same posture as the retired x86-32 backend). Win64 note: emitted code
// registers no unwind info — nothing here throws, and the handlers are
// plain C.
//
// Where arm64 computes 6309 flags explicitly, x86 captures most of them
// from EFLAGS with byte-granular RMW directly on state memory + setcc
// into the cc[] bytes — inc/dec give V on the exact 0x7F/0x80 wraps, neg
// gives C=(input!=0) and V=(input==0x80), shifts give C from the shifted
// bit and shl's OF is the 6309 V formula, and add/sub/cmp give C, V, Z,
// N in one instruction. Only ADD's half-carry H needs arithmetic.

#include "BlockJit.h"
#include "BlockCache.h"
#include "emit_x64.h"
#include <cstdlib>
#include <cstdio>
#include <cstring>
#ifdef _WIN32
#include <Windows.h>
#else
#include <sys/mman.h>
#endif

namespace BlockJit
{

// Same 16 MB arena budget as the other backends.
static constexpr size_t kArenaSize = 16 * 1024 * 1024;

// Calling convention: first two integer args.
#ifdef _WIN32
static constexpr int ARG0 = X64_RCX;
static constexpr int ARG1 = X64_RDX;
#else
static constexpr int ARG0 = X64_RDI;
static constexpr int ARG1 = X64_RSI;
#endif

// Pinned registers (see file header). The guest accumulators are
// register-resident across whole chains: A lives in BPL and B in R12B
// (both callee-saved on both ABIs), established/spilled by the runner
// exactly like the cycle counter. Handler call sites spill and reload
// them (handlers read the memory copies); the MMU functions the memory
// fast paths fall back to never touch CPU registers, so those calls
// need no spill - which is the point of using callee-saved registers.
// CycleFor moved to memory reads in the chain stub to free R12.
static constexpr int R_STATE = X64_R13;
static constexpr int R_INSNS = X64_R14;
static constexpr int R_SLOTS = X64_R15;
static constexpr int R_CYC   = X64_RBX;   // 32-bit view
static constexpr int R_ACC_A = X64_RBP;   // BPL = A_REG
static constexpr int R_ACC_B = X64_R12;   // R12B = B_REG

// Thunk stack adjustment: Win64 shadow space + alignment. Entry rsp is
// 8 mod 16 (call pushed the return address); -40 restores 16-alignment
// at every call site inside the thunk.
static constexpr int8_t kThunkFrame = 40;

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
static bool               g_emit_had_side_effects = false;

// Block linking (see BlockJitA64.cpp for the design discussion).
static uint8_t*           g_chain_stub = nullptr;
static uint8_t*           g_thunk_runner = nullptr;
static bool               g_no_link    = false;
// RAM fast path: inline the MMU's direct-RAM case into thunks (bank
// pointer tables + the block cache's write-watch fast-reject bitmap),
// falling back to the MemRead8/MemWrite8 calls for everything else.
static bool               g_fastmem    = false;

// Field offsets from CpuAddrs.base, computed once at Init. x86-64 disp32
// forms reach anywhere in the struct, so unlike arm64 there are no
// immediate-range restrictions to verify.
static int32_t g_off_pc  = 0;
static int32_t g_off_a   = 0;
static int32_t g_off_b   = 0;
static int32_t g_off_d   = 0;
static int32_t g_off_x   = 0;
static int32_t g_off_y   = 0;
static int32_t g_off_u   = 0;
static int32_t g_off_s   = 0;
static int32_t g_off_dp  = 0;
static int32_t g_off_cc  = 0;   // cc[C]=+0, cc[V]=+1, cc[Z]=+2, cc[N]=+3
static int32_t g_off_cyc = 0;
static int32_t g_off_nat21 = 0;
static int32_t g_off_nat31 = 0;
static int32_t g_off_nat43 = 0;
static int32_t g_off_nat54 = 0;
static int32_t g_off_nat32 = 0;
static int32_t g_off_nat53 = 0;
static int32_t g_off_nat65 = 0;
static int32_t g_off_nat51 = 0;
static int32_t g_off_nat64 = 0;
static int32_t g_off_nat76 = 0;
static int32_t g_off_nat87 = 0;
static int64_t g_off_pending = -1;   // ChainBreak byte in cpu_state

// ---------- cc[] bit ordering (matches BlockJit.cpp / hd6309.cpp) ----------

static constexpr uint8_t CC_BIT_C = 1u << 0;
static constexpr uint8_t CC_BIT_V = 1u << 1;
static constexpr uint8_t CC_BIT_Z = 1u << 2;
static constexpr uint8_t CC_BIT_N = 1u << 3;
static constexpr uint8_t CC_ALL   = CC_BIT_C | CC_BIT_V | CC_BIT_Z | CC_BIT_N;
static constexpr uint8_t CC_UNKNOWN = 0xFF;

void Init(const CpuAddrs& addrs, const InlineableHandlers& handlers)
{
    g_addrs = addrs;
    g_inlines = handlers;

    g_disabled = std::getenv("VCC_NO_JIT") != nullptr;
    g_no_inline = std::getenv("VCC_NO_INLINE") != nullptr;
    if (const char* im = std::getenv("VCC_INLINE_MAX")) g_inline_max = std::atoi(im);
    // Linking must be off under the differential verifiers: they run
    // ONE block from a snapshot and compare, and a chained thunk would
    // run its successors too before returning.
    g_no_link = std::getenv("VCC_NO_LINK") != nullptr ||
                std::getenv("VCC_VERIFY_PC") != nullptr ||
                std::getenv("VCC_VERIFY_PURE") != nullptr;

    // RAM fast path: needs the MMU bank tables and the watch bitmap.
    // VCC_NO_FASTMEM disables it for A/B runs; VCC_LOG_WRITES implies
    // it off because the inline stores would bypass the write log.
    g_fastmem = std::getenv("VCC_NO_FASTMEM") == nullptr &&
                std::getenv("VCC_LOG_WRITES") == nullptr &&
                addrs.fastmem_read_banks != nullptr &&
                addrs.fastmem_write_banks != nullptr &&
                addrs.fastmem_watch_bitmap != nullptr;

    if (g_arena_base == nullptr && !g_disabled)
    {
#ifdef _WIN32
        g_arena_base = (uint8_t*)VirtualAlloc(
            nullptr, kArenaSize, MEM_RESERVE | MEM_COMMIT,
            PAGE_EXECUTE_READWRITE);
#else
        void* mem = mmap(nullptr, kArenaSize, PROT_READ | PROT_WRITE | PROT_EXEC,
                         MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
        g_arena_base = (mem == MAP_FAILED) ? nullptr : (uint8_t*)mem;
#endif
    }

    if (g_addrs.base != nullptr && g_addrs.pc != nullptr)
    {
        const uint8_t* base = (const uint8_t*)g_addrs.base;
        g_off_pc    = (int32_t)((uint8_t*)g_addrs.pc            - base);
        g_off_a     = (int32_t)((uint8_t*)g_addrs.a             - base);
        g_off_b     = (int32_t)((uint8_t*)g_addrs.b             - base);
        g_off_d     = (int32_t)((uint8_t*)g_addrs.d             - base);
        g_off_x     = (int32_t)((uint8_t*)g_addrs.x             - base);
        g_off_y     = (int32_t)((uint8_t*)g_addrs.y             - base);
        g_off_u     = (int32_t)((uint8_t*)g_addrs.u             - base);
        g_off_s     = (int32_t)((uint8_t*)g_addrs.s             - base);
        g_off_dp    = (int32_t)((uint8_t*)g_addrs.dp            - base);
        g_off_cc    = (int32_t)((uint8_t*)g_addrs.cc            - base);
        g_off_cyc   = (int32_t)((uint8_t*)g_addrs.cycle_counter - base);
        g_off_nat21 = (int32_t)((uint8_t*)g_addrs.nat_cycles_21 - base);
        g_off_nat31 = (int32_t)((uint8_t*)g_addrs.nat_cycles_31 - base);
        g_off_nat43 = (int32_t)((uint8_t*)g_addrs.nat_cycles_43 - base);
        g_off_nat54 = (int32_t)((uint8_t*)g_addrs.nat_cycles_54 - base);
        g_off_nat32 = (int32_t)((uint8_t*)g_addrs.nat_cycles_32 - base);
        g_off_nat53 = (int32_t)((uint8_t*)g_addrs.nat_cycles_53 - base);
        g_off_nat65 = (int32_t)((uint8_t*)g_addrs.nat_cycles_65 - base);
        g_off_nat51 = (int32_t)((uint8_t*)g_addrs.nat_cycles_51 - base);
        g_off_nat64 = (int32_t)((uint8_t*)g_addrs.nat_cycles_64 - base);
        g_off_nat76 = (int32_t)((uint8_t*)g_addrs.nat_cycles_76 - base);
        g_off_nat87 = (int32_t)((uint8_t*)g_addrs.nat_cycles_87 - base);
        g_off_pending = (g_addrs.chain_break != nullptr)
            ? (int64_t)((uint8_t*)g_addrs.chain_break - base) : -1;
        if (g_off_pending < 0 || g_off_pending > INT32_MAX)
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

// ---------- size budget (bytes of x86-64 code) ----------

// Prologue: sub rsp (4) + movabs r14 (10).
static constexpr size_t kPrologueBytes = 16;
// Per instruction, worst case: an indexed memory-RMW with every flag
// live — EA, the read fast path, the modify+setcc body, the EA
// spill/reload, and the write fast path with its bitmap check — lands
// around 220 bytes. This is only the arena RESERVATION per
// instruction; actual consumption is the emitted size.
static constexpr size_t kMaxBytesPerInsn = 288;
// Epilogue: final PC flush (11) + add rsp (4) + jmp rel32 (5).
static constexpr size_t kEpilogueBytes = 24;

// ---------- small emit helpers ----------

// CycleCounter += constant  (registerized in ebx)
static void EmitCyclesConst(emit_t& e, uint32_t n)
{
    emit_add_r_imm(&e, R_CYC, (int32_t)n);
}

// CycleCounter += NatEmuCyclesNN (live byte)
static void EmitCyclesRuntime(emit_t& e, int32_t nat_offset)
{
    emit_load8u(&e, X64_RAX, R_STATE, nat_offset);
    emit_add_rr(&e, R_CYC, X64_RAX);
}

// cc[idx] = 0 or 1
static void EmitCcConst(emit_t& e, int idx, unsigned value)
{
    emit_mov_mem8_imm8(&e, R_STATE, g_off_cc + idx, value ? 1 : 0);
}

// Update cc[Z]/cc[N]/cc[V] from an 8-bit value held zero-extended in
// `reg` — Z/N via one 8-bit test + setcc, V as a constant 0, matching
// the interpreter's rules for loads/stores/TST.
static void EmitFlagsZNVFromReg8(emit_t& e, int reg, uint8_t mask)
{
    if (mask & (CC_BIT_Z | CC_BIT_N))
    {
        emit_test_r8_r8(&e, reg, reg);
        if (mask & CC_BIT_Z)
            emit_setcc_mem(&e, X64_CC_E, R_STATE, g_off_cc + 2);
        if (mask & CC_BIT_N)
            emit_setcc_mem(&e, X64_CC_S, R_STATE, g_off_cc + 3);
    }
    if (mask & CC_BIT_V)
        emit_mov_mem8_imm8(&e, R_STATE, g_off_cc + 1, 0);
}

// Same for a 16-bit value (Z from all 16 bits, N from bit 15).
static void EmitFlagsZNVFromReg16(emit_t& e, int reg, uint8_t mask)
{
    if (mask & (CC_BIT_Z | CC_BIT_N))
    {
        emit_test_r16_r16(&e, reg, reg);
        if (mask & CC_BIT_Z)
            emit_setcc_mem(&e, X64_CC_E, R_STATE, g_off_cc + 2);
        if (mask & CC_BIT_N)
            emit_setcc_mem(&e, X64_CC_S, R_STATE, g_off_cc + 3);
    }
    if (mask & CC_BIT_V)
        emit_mov_mem8_imm8(&e, R_STATE, g_off_cc + 1, 0);
}

// movabs rax, fn ; call rax — the arena can sit anywhere relative to
// the text segment, so calls are always absolute-through-register.
static void EmitCallAbs(emit_t& e, const void* fn)
{
    emit_mov_r64_imm64(&e, X64_RAX, (uint64_t)(uintptr_t)fn);
    emit_call_r(&e, X64_RAX);
}

// Host register holding a guest accumulator, or -1 when the offset is
// a memory-resident register (X/Y/U/S/dp...).
static int AccHostReg(int32_t reg_off)
{
    if (reg_off == g_off_a) return R_ACC_A;
    if (reg_off == g_off_b) return R_ACC_B;
    return -1;
}

// Spill/reload the register-resident accumulators around instruction-
// handler calls (handlers read and write the cpu_state copies).
static void EmitSpillAcc(emit_t& e)
{
    emit_store8(&e, R_ACC_A, R_STATE, g_off_a);
    emit_store8(&e, R_ACC_B, R_STATE, g_off_b);
}

static void EmitReloadAcc(emit_t& e)
{
    emit_load8u(&e, R_ACC_A, R_STATE, g_off_a);
    emit_load8u(&e, R_ACC_B, R_STATE, g_off_b);
}

// D = A:B, composed from / decomposed into the accumulator registers.
// Both clobber the scratch register.
static void EmitLoadD(emit_t& e, int dst, int scratch)
{
    emit_movzx_r32_r8(&e, dst, R_ACC_A);
    emit_shl_r_imm(&e, dst, 8);
    emit_movzx_r32_r8(&e, scratch, R_ACC_B);
    emit_or_rr(&e, dst, scratch);
}

static void EmitStoreD(emit_t& e, int src, int scratch)
{
    emit_mov_rr(&e, scratch, src);
    emit_shr_r_imm(&e, scratch, 8);
    emit_mov_r8_r8(&e, R_ACC_A, scratch);
    emit_mov_r8_r8(&e, R_ACC_B, src);
}

// Bounds-checked rel32 patch: if the field fell past the capacity the
// bytes were never written (emit_byte drops them), so patching would
// scribble out of bounds; the end-of-emit capacity check is about to
// refuse the block anyway.
static void SafePatchRel32(emit_t& e, uint32_t field_at, uint32_t target)
{
    if (field_at + 4 <= e.capacity && target <= e.capacity)
        emit_patch_rel32(&e, field_at, target);
}

// ---------- guest memory access (RAM fast path) ----------
//
// Uniform register contract across the four helpers:
//   R9D = guest address (zero-extended 16-bit), consumed
//   R8D = data (writes only; low 8/16 bits), preserved until the call
//   EAX = result (reads only; zero-extended 8/16-bit)
// All other caller-saved registers are clobbered; the pinned set
// survives. With g_fastmem off, each helper is exactly the old
// marshal-and-call sequence.
//
// The fast path replicates MemRead8/MemWrite8's RAM case against the
// MMU-maintained bank tables: bank = addr >> 13; a null bank pointer
// (ports, ROM writes, cart space, the $FE00+ tail via bank 7, vector-
// ROM pages) falls back to the C call. Stores additionally run
// InvalidateIfCached's fast-reject bitmap test - a set bit means "a
// cached block may live in this 256-byte page", and the write takes
// the full MemWrite8 path so the real invalidation machinery runs.
// The bank POINTERS are loaded at run time from the tables, so MMU
// rebuilds propagate to existing thunks with no re-emission or
// patching, exactly like the chain stub's indirect-through-slot rule.

// EAX = MemRead8(R9D)
static void EmitMemRead8(emit_t& e)
{
    if (!g_fastmem)
    {
        emit_mov_rr(&e, ARG0, X64_R9);
        EmitCallAbs(e, (const void*)g_addrs.mem_read8);
        emit_movzx_r32_r8(&e, X64_RAX, X64_RAX);
        return;
    }
    emit_mov_rr(&e, X64_R8, X64_R9);
    emit_shr_r_imm(&e, X64_R8, 13);
    emit_mov_r64_imm64(&e, X64_R10, (uint64_t)(uintptr_t)g_addrs.fastmem_read_banks);
    emit_load64_sib(&e, X64_R10, X64_R10, X64_R8, 3, 0);
    emit_test_r64_r64(&e, X64_R10, X64_R10);
    emit_jcc_rel32(&e, X64_CC_E, 0);
    const uint32_t fix_slow = emit_pos(&e) - 4;
    emit_and_r_imm(&e, X64_R9, 0x1FFF);
    emit_load8u_sib(&e, X64_RAX, X64_R10, X64_R9, 0);
    emit_jmp_rel32(&e, 0);
    const uint32_t fix_done = emit_pos(&e) - 4;
    SafePatchRel32(e, fix_slow, emit_pos(&e));
    emit_mov_rr(&e, ARG0, X64_R9);
    EmitCallAbs(e, (const void*)g_addrs.mem_read8);
    emit_movzx_r32_r8(&e, X64_RAX, X64_RAX);
    SafePatchRel32(e, fix_done, emit_pos(&e));
}

// MemWrite8(R8D low byte, R9D)
static void EmitMemWrite8(emit_t& e)
{
    if (!g_fastmem)
    {
        emit_mov_rr(&e, ARG0, X64_R8);
        emit_mov_rr(&e, ARG1, X64_R9);
        EmitCallAbs(e, (const void*)g_addrs.mem_write8);
        return;
    }
    uint32_t fix_slow[2];
    int nslow = 0;
    emit_mov_rr(&e, X64_RAX, X64_R9);
    emit_shr_r_imm(&e, X64_RAX, 13);
    emit_mov_r64_imm64(&e, X64_R10, (uint64_t)(uintptr_t)g_addrs.fastmem_write_banks);
    emit_load64_sib(&e, X64_R10, X64_R10, X64_RAX, 3, 0);
    emit_test_r64_r64(&e, X64_R10, X64_R10);
    emit_jcc_rel32(&e, X64_CC_E, 0);
    fix_slow[nslow++] = emit_pos(&e) - 4;
    // Watch bitmap: bit (addr>>8)&7 of byte (addr>>11).
    emit_mov_r64_imm64(&e, X64_R11, (uint64_t)(uintptr_t)g_addrs.fastmem_watch_bitmap);
    emit_mov_rr(&e, X64_RAX, X64_R9);
    emit_shr_r_imm(&e, X64_RAX, 11);
    emit_load8u_sib(&e, X64_RAX, X64_R11, X64_RAX, 0);
    emit_mov_rr(&e, X64_RCX, X64_R9);
    emit_shr_r_imm(&e, X64_RCX, 8);
    emit_and_r_imm(&e, X64_RCX, 7);
    emit_bt_rr(&e, X64_RAX, X64_RCX);
    emit_jcc_rel32(&e, X64_CC_B, 0);        // CF=1: watched page
    fix_slow[nslow++] = emit_pos(&e) - 4;
    emit_mov_rr(&e, X64_RAX, X64_R9);
    emit_and_r_imm(&e, X64_RAX, 0x1FFF);
    emit_store8_sib(&e, X64_R8, X64_R10, X64_RAX, 0);
    emit_jmp_rel32(&e, 0);
    const uint32_t fix_done = emit_pos(&e) - 4;
    for (int i = 0; i < nslow; ++i)
        SafePatchRel32(e, fix_slow[i], emit_pos(&e));
    emit_mov_rr(&e, ARG0, X64_R8);
    emit_mov_rr(&e, ARG1, X64_R9);
    EmitCallAbs(e, (const void*)g_addrs.mem_write8);
    SafePatchRel32(e, fix_done, emit_pos(&e));
}

// EAX = MemRead16(R9D) - big-endian pair. Fast only when both bytes
// sit in the same bank ((addr & 0x1FFF) != 0x1FFF); the wrap at
// $FFFF lands in bank 7 (always null) so it needs no special case.
static void EmitMemRead16(emit_t& e)
{
    if (!g_fastmem)
    {
        emit_mov_rr(&e, ARG0, X64_R9);
        EmitCallAbs(e, (const void*)g_addrs.mem_read16);
        emit_movzx_r32_r16(&e, X64_RAX, X64_RAX);
        return;
    }
    uint32_t fix_slow[2];
    int nslow = 0;
    emit_mov_rr(&e, X64_RAX, X64_R9);
    emit_and_r_imm(&e, X64_RAX, 0x1FFF);
    emit_cmp_r_imm(&e, X64_RAX, 0x1FFF);
    emit_jcc_rel32(&e, X64_CC_E, 0);        // bank-edge pair
    fix_slow[nslow++] = emit_pos(&e) - 4;
    emit_mov_rr(&e, X64_R8, X64_R9);
    emit_shr_r_imm(&e, X64_R8, 13);
    emit_mov_r64_imm64(&e, X64_R10, (uint64_t)(uintptr_t)g_addrs.fastmem_read_banks);
    emit_load64_sib(&e, X64_R10, X64_R10, X64_R8, 3, 0);
    emit_test_r64_r64(&e, X64_R10, X64_R10);
    emit_jcc_rel32(&e, X64_CC_E, 0);
    fix_slow[nslow++] = emit_pos(&e) - 4;
    // EAX still holds addr & 0x1FFF.
    emit_load8u_sib(&e, X64_RDX, X64_R10, X64_RAX, 1);   // low byte
    emit_load8u_sib(&e, X64_RAX, X64_R10, X64_RAX, 0);   // high byte
    emit_shl_r_imm(&e, X64_RAX, 8);
    emit_or_rr(&e, X64_RAX, X64_RDX);
    emit_jmp_rel32(&e, 0);
    const uint32_t fix_done = emit_pos(&e) - 4;
    for (int i = 0; i < nslow; ++i)
        SafePatchRel32(e, fix_slow[i], emit_pos(&e));
    emit_mov_rr(&e, ARG0, X64_R9);
    EmitCallAbs(e, (const void*)g_addrs.mem_read16);
    emit_movzx_r32_r16(&e, X64_RAX, X64_RAX);
    SafePatchRel32(e, fix_done, emit_pos(&e));
}

// MemWrite16(R8D low 16 bits, R9D) - big-endian pair. Fast only when
// (addr & 0xFF) != 0xFF, which keeps both bytes in the same bank AND
// the same 256-byte watch page, so one bitmap test covers the pair.
static void EmitMemWrite16(emit_t& e)
{
    if (!g_fastmem)
    {
        emit_mov_rr(&e, ARG0, X64_R8);
        emit_mov_rr(&e, ARG1, X64_R9);
        EmitCallAbs(e, (const void*)g_addrs.mem_write16);
        return;
    }
    uint32_t fix_slow[3];
    int nslow = 0;
    emit_mov_rr(&e, X64_RAX, X64_R9);
    emit_and_r_imm(&e, X64_RAX, 0xFF);
    emit_cmp_r_imm(&e, X64_RAX, 0xFF);
    emit_jcc_rel32(&e, X64_CC_E, 0);        // page/bank-edge pair
    fix_slow[nslow++] = emit_pos(&e) - 4;
    emit_mov_rr(&e, X64_RCX, X64_R9);
    emit_shr_r_imm(&e, X64_RCX, 13);
    emit_mov_r64_imm64(&e, X64_R10, (uint64_t)(uintptr_t)g_addrs.fastmem_write_banks);
    emit_load64_sib(&e, X64_R10, X64_R10, X64_RCX, 3, 0);
    emit_test_r64_r64(&e, X64_R10, X64_R10);
    emit_jcc_rel32(&e, X64_CC_E, 0);
    fix_slow[nslow++] = emit_pos(&e) - 4;
    emit_mov_r64_imm64(&e, X64_R11, (uint64_t)(uintptr_t)g_addrs.fastmem_watch_bitmap);
    emit_mov_rr(&e, X64_RAX, X64_R9);
    emit_shr_r_imm(&e, X64_RAX, 11);
    emit_load8u_sib(&e, X64_RAX, X64_R11, X64_RAX, 0);
    emit_mov_rr(&e, X64_RCX, X64_R9);
    emit_shr_r_imm(&e, X64_RCX, 8);
    emit_and_r_imm(&e, X64_RCX, 7);
    emit_bt_rr(&e, X64_RAX, X64_RCX);
    emit_jcc_rel32(&e, X64_CC_B, 0);
    fix_slow[nslow++] = emit_pos(&e) - 4;
    emit_mov_rr(&e, X64_RAX, X64_R9);
    emit_and_r_imm(&e, X64_RAX, 0x1FFF);
    emit_mov_rr(&e, X64_RDX, X64_R8);
    emit_shr_r_imm(&e, X64_RDX, 8);
    emit_store8_sib(&e, X64_RDX, X64_R10, X64_RAX, 0);   // high byte first
    emit_store8_sib(&e, X64_R8,  X64_R10, X64_RAX, 1);   // then low
    emit_jmp_rel32(&e, 0);
    const uint32_t fix_done = emit_pos(&e) - 4;
    for (int i = 0; i < nslow; ++i)
        SafePatchRel32(e, fix_slow[i], emit_pos(&e));
    emit_mov_rr(&e, ARG0, X64_R8);
    emit_mov_rr(&e, ARG1, X64_R9);
    EmitCallAbs(e, (const void*)g_addrs.mem_write16);
    SafePatchRel32(e, fix_done, emit_pos(&e));
}

// ---------- inline emitters ----------
// Each mirrors its interpreter handler exactly (see BlockJitA64.cpp for
// the semantics discussion); the flag_mask gates which cc[] stores are
// emitted, per the backward liveness pass.

// LDA/LDB #imm - flags are compile-time constants of the immediate.
// The accumulators are register-resident.
static void EmitInlineLd8Imm(emit_t& e, const DecodedInst& insn,
                             int32_t reg_off, uint8_t mask)
{
    const uint8_t imm = (uint8_t)(insn.operand & 0xFF);
    emit_mov_r8_imm8(&e, AccHostReg(reg_off), imm);
    if (mask & CC_BIT_N) EmitCcConst(e, 3, (imm & 0x80) ? 1 : 0);
    if (mask & CC_BIT_Z) EmitCcConst(e, 2, (imm == 0) ? 1 : 0);
    if (mask & CC_BIT_V) EmitCcConst(e, 1, 0);
    EmitCyclesConst(e, 2);
}

// LDD/LDX/LDU #imm16 (constant cycles) and LDY/LDS (runtime cycles).
// D decomposes into the accumulator registers; the rest stay memory.
static void EmitInlineLd16Imm(emit_t& e, const DecodedInst& insn,
                              int32_t reg_off, int const_cycles,
                              int32_t nat_offset, uint8_t mask)
{
    const uint16_t imm = insn.operand;
    if (reg_off == g_off_d)
    {
        emit_mov_r8_imm8(&e, R_ACC_A, (uint8_t)(imm >> 8));
        emit_mov_r8_imm8(&e, R_ACC_B, (uint8_t)(imm & 0xFF));
    }
    else
    {
        emit_mov_mem16_imm16(&e, R_STATE, reg_off, imm);
    }
    if (mask & CC_BIT_N) EmitCcConst(e, 3, (imm & 0x8000) ? 1 : 0);
    if (mask & CC_BIT_Z) EmitCcConst(e, 2, (imm == 0) ? 1 : 0);
    if (mask & CC_BIT_V) EmitCcConst(e, 1, 0);
    if (const_cycles >= 0)
        EmitCyclesConst(e, (uint32_t)const_cycles);
    else
        EmitCyclesRuntime(e, nat_offset);
}

// CLRA/CLRB: register = 0, CC = fixed pattern C=0 V=0 Z=1 N=0.
static void EmitInlineClr8(emit_t& e, int32_t reg_off, uint8_t mask)
{
    emit_mov_r8_imm8(&e, AccHostReg(reg_off), 0);
    if (mask & CC_BIT_C) EmitCcConst(e, 0, 0);
    if (mask & CC_BIT_V) EmitCcConst(e, 1, 0);
    if (mask & CC_BIT_Z) EmitCcConst(e, 2, 1);
    if (mask & CC_BIT_N) EmitCcConst(e, 3, 0);
    EmitCyclesRuntime(e, g_off_nat21);
}

// LDA/LDB <dp and ext: effective address, guest read, register store,
// Z/N/V from the loaded byte.
static void EmitInlineLd8Mem(emit_t& e, const DecodedInst& insn,
                             int32_t reg_off, bool direct_page,
                             uint8_t mask)
{
    if (direct_page)
    {
        const uint8_t offset = (uint8_t)(insn.operand & 0xFF);
        emit_load16u(&e, X64_R9, R_STATE, g_off_dp);
        if (offset != 0)
            emit_or_r_imm(&e, X64_R9, offset);   // dp low byte is always 0
    }
    else
    {
        emit_mov_r32_imm32(&e, X64_R9, insn.operand);
    }
    g_emit_had_side_effects = true;
    EmitMemRead8(e);
    emit_mov_r8_r8(&e, AccHostReg(reg_off), X64_RAX);
    EmitFlagsZNVFromReg8(e, X64_RAX, mask);
    EmitCyclesRuntime(e, direct_page ? g_off_nat43 : g_off_nat54);
}

// STA/STB <dp and ext: flags from the register value (not memory),
// then the guest write.
static void EmitInlineSt8Mem(emit_t& e, const DecodedInst& insn,
                             int32_t reg_off, bool direct_page,
                             uint8_t mask)
{
    emit_movzx_r32_r8(&e, X64_R8, AccHostReg(reg_off));
    EmitFlagsZNVFromReg8(e, X64_R8, mask);
    if (direct_page)
    {
        const uint8_t offset = (uint8_t)(insn.operand & 0xFF);
        emit_load16u(&e, X64_R9, R_STATE, g_off_dp);
        if (offset != 0)
            emit_or_r_imm(&e, X64_R9, offset);
    }
    else
    {
        emit_mov_r32_imm32(&e, X64_R9, insn.operand);
    }
    g_emit_had_side_effects = true;
    EmitMemWrite8(e);
    EmitCyclesRuntime(e, direct_page ? g_off_nat43 : g_off_nat54);
}

// TSTA/TSTB: Z/N from the register, V=0, C untouched.
static void EmitInlineTst8(emit_t& e, int32_t reg_off, uint8_t mask)
{
    if (mask & (CC_BIT_Z | CC_BIT_N))
    {
        const int acc = AccHostReg(reg_off);
        emit_test_r8_r8(&e, acc, acc);
        if (mask & CC_BIT_Z)
            emit_setcc_mem(&e, X64_CC_E, R_STATE, g_off_cc + 2);
        if (mask & CC_BIT_N)
            emit_setcc_mem(&e, X64_CC_S, R_STATE, g_off_cc + 3);
    }
    if (mask & CC_BIT_V)
        EmitCcConst(e, 1, 0);
    EmitCyclesRuntime(e, g_off_nat21);
}

// INCA/INCB, DECA/DECB: add/sub r8, 1 sets OF on exactly the
// 0x7F->0x80 / 0x80->0x7F wraps — the 6309 V rules. The host CF is
// clobbered but 6309 INC/DEC never write C, so no setc is emitted.
static void EmitInlineIncDec8(emit_t& e, int32_t reg_off, bool is_inc,
                              uint8_t mask)
{
    emit_alu_r8_imm8(&e, is_inc ? X64_ALU_ADD : X64_ALU_SUB,
                     AccHostReg(reg_off), 1);
    if (mask & CC_BIT_Z) emit_setcc_mem(&e, X64_CC_E, R_STATE, g_off_cc + 2);
    if (mask & CC_BIT_N) emit_setcc_mem(&e, X64_CC_S, R_STATE, g_off_cc + 3);
    if (mask & CC_BIT_V) emit_setcc_mem(&e, X64_CC_O, R_STATE, g_off_cc + 1);
    EmitCyclesRuntime(e, g_off_nat21);
}

// COMA/COMB: ones-complement via xor r8, 0xFF (sets ZF/SF); V=0, C=1.
static void EmitInlineCom8(emit_t& e, int32_t reg_off, uint8_t mask)
{
    emit_alu_r8_imm8(&e, X64_ALU_XOR, AccHostReg(reg_off), 0xFF);
    if (mask & CC_BIT_Z) emit_setcc_mem(&e, X64_CC_E, R_STATE, g_off_cc + 2);
    if (mask & CC_BIT_N) emit_setcc_mem(&e, X64_CC_S, R_STATE, g_off_cc + 3);
    if (mask & CC_BIT_V) EmitCcConst(e, 1, 0);
    if (mask & CC_BIT_C) EmitCcConst(e, 0, 1);
    EmitCyclesRuntime(e, g_off_nat21);
}

// NEGA/NEGB: x86 neg r8 sets CF=(input!=0), OF=(input==0x80), ZF/SF
// from the result — all four 6309 NEG flag rules from one instruction.
static void EmitInlineNeg8(emit_t& e, int32_t reg_off, uint8_t mask)
{
    emit_neg_r8(&e, AccHostReg(reg_off));
    if (mask & CC_BIT_C) emit_setcc_mem(&e, X64_CC_B, R_STATE, g_off_cc + 0);
    if (mask & CC_BIT_V) emit_setcc_mem(&e, X64_CC_O, R_STATE, g_off_cc + 1);
    if (mask & CC_BIT_Z) emit_setcc_mem(&e, X64_CC_E, R_STATE, g_off_cc + 2);
    if (mask & CC_BIT_N) emit_setcc_mem(&e, X64_CC_S, R_STATE, g_off_cc + 3);
    EmitCyclesRuntime(e, g_off_nat21);
}

// LSRA: C = old bit 0, Z from result, N = 0 (top bit cleared). V untouched.
static void EmitInlineLsr8(emit_t& e, int32_t reg_off, uint8_t mask)
{
    emit_shift_r8_1(&e, X64_SHIFT_SHR, AccHostReg(reg_off));
    if (mask & CC_BIT_C) emit_setcc_mem(&e, X64_CC_B, R_STATE, g_off_cc + 0);
    if (mask & CC_BIT_Z) emit_setcc_mem(&e, X64_CC_E, R_STATE, g_off_cc + 2);
    if (mask & CC_BIT_N) EmitCcConst(e, 3, 0);
    EmitCyclesRuntime(e, g_off_nat21);
}

// ASRA: sign bit propagates; C = old bit 0, Z/N from result. V untouched.
static void EmitInlineAsr8(emit_t& e, int32_t reg_off, uint8_t mask)
{
    emit_shift_r8_1(&e, X64_SHIFT_SAR, AccHostReg(reg_off));
    if (mask & CC_BIT_C) emit_setcc_mem(&e, X64_CC_B, R_STATE, g_off_cc + 0);
    if (mask & CC_BIT_Z) emit_setcc_mem(&e, X64_CC_E, R_STATE, g_off_cc + 2);
    if (mask & CC_BIT_N) emit_setcc_mem(&e, X64_CC_S, R_STATE, g_off_cc + 3);
    EmitCyclesRuntime(e, g_off_nat21);
}

// ASLA/LSLA: C = old bit 7; a 1-bit shl's OF is (new MSB != CF) =
// old bit 6 ^ old bit 7, exactly the 6309 V formula; Z/N from result.
static void EmitInlineAsl8(emit_t& e, int32_t reg_off, uint8_t mask)
{
    emit_shift_r8_1(&e, X64_SHIFT_SHL, AccHostReg(reg_off));
    if (mask & CC_BIT_C) emit_setcc_mem(&e, X64_CC_B, R_STATE, g_off_cc + 0);
    if (mask & CC_BIT_V) emit_setcc_mem(&e, X64_CC_O, R_STATE, g_off_cc + 1);
    if (mask & CC_BIT_Z) emit_setcc_mem(&e, X64_CC_E, R_STATE, g_off_cc + 2);
    if (mask & CC_BIT_N) emit_setcc_mem(&e, X64_CC_S, R_STATE, g_off_cc + 3);
    EmitCyclesRuntime(e, g_off_nat21);
}

// ABX: X += B, no flags.
static void EmitInlineAbx(emit_t& e)
{
    emit_movzx_r32_r8(&e, X64_RAX, R_ACC_B);
    emit_add_mem16_r16(&e, R_STATE, g_off_x, X64_RAX);
    EmitCyclesRuntime(e, g_off_nat31);
}

// ANDA/ANDB/ORA/ORB/EORA/EORB #imm (writeback RMW on the state byte)
// and BITA/BITB #imm (test, flags only): ZF/SF from the op, V=0, +2.
enum class LogicOp : uint8_t { And, Or, Eor };

static void EmitInlineLogic8Imm(emit_t& e, const DecodedInst& insn,
                                int32_t reg_off, LogicOp op,
                                bool writeback, uint8_t mask)
{
    const uint8_t imm = (uint8_t)(insn.operand & 0xFF);
    const int acc = AccHostReg(reg_off);
    if (writeback)
    {
        const int alu = (op == LogicOp::And) ? X64_ALU_AND
                      : (op == LogicOp::Or)  ? X64_ALU_OR
                                             : X64_ALU_XOR;
        emit_alu_r8_imm8(&e, alu, acc, imm);
    }
    else
    {
        // BIT = AND without writeback: flag it on a copy.
        emit_movzx_r32_r8(&e, X64_RAX, acc);
        emit_alu_r8_imm8(&e, X64_ALU_AND, X64_RAX, imm);
    }
    if (mask & CC_BIT_Z) emit_setcc_mem(&e, X64_CC_E, R_STATE, g_off_cc + 2);
    if (mask & CC_BIT_N) emit_setcc_mem(&e, X64_CC_S, R_STATE, g_off_cc + 3);
    if (mask & CC_BIT_V) EmitCcConst(e, 1, 0);
    EmitCyclesConst(e, 2);
}

// CMPA/CMPB (no writeback), SUBA/SUBB, ADDA/ADDB #imm. x86 sub/cmp/add
// on the state byte produce C (CF), V (OF), Z, N directly — the 6309
// formulas are the standard two's-complement rules. ADD additionally
// writes the untracked half-carry cc[H] (index 5) unconditionally like
// the handler, which x86 tracks only in AF (no setcc), so ADD takes the
// register path and computes H = bit 4 of (old ^ imm ^ result).
static void EmitInlineArith8Imm(emit_t& e, const DecodedInst& insn,
                                int32_t reg_off, bool is_add,
                                bool writeback, uint8_t mask)
{
    const uint8_t imm = (uint8_t)(insn.operand & 0xFF);
    const int acc = AccHostReg(reg_off);
    if (!is_add)
    {
        emit_alu_r8_imm8(&e, writeback ? X64_ALU_SUB : X64_ALU_CMP, acc, imm);
        if (mask & CC_BIT_C) emit_setcc_mem(&e, X64_CC_B, R_STATE, g_off_cc + 0);
        if (mask & CC_BIT_V) emit_setcc_mem(&e, X64_CC_O, R_STATE, g_off_cc + 1);
        if (mask & CC_BIT_Z) emit_setcc_mem(&e, X64_CC_E, R_STATE, g_off_cc + 2);
        if (mask & CC_BIT_N) emit_setcc_mem(&e, X64_CC_S, R_STATE, g_off_cc + 3);
    }
    else
    {
        emit_movzx_r32_r8(&e, X64_RAX, acc);              // old value
        emit_alu_r8_imm8(&e, X64_ALU_ADD, acc, imm);      // 8-bit add: flags
        // setcc preserves flags; capture everything before the H math.
        if (mask & CC_BIT_C) emit_setcc_mem(&e, X64_CC_B, R_STATE, g_off_cc + 0);
        if (mask & CC_BIT_V) emit_setcc_mem(&e, X64_CC_O, R_STATE, g_off_cc + 1);
        if (mask & CC_BIT_Z) emit_setcc_mem(&e, X64_CC_E, R_STATE, g_off_cc + 2);
        if (mask & CC_BIT_N) emit_setcc_mem(&e, X64_CC_S, R_STATE, g_off_cc + 3);
        // H = bit 4 of (old ^ imm ^ result), unconditional per the handler.
        emit_movzx_r32_r8(&e, X64_RDX, acc);              // result
        emit_xor_rr(&e, X64_RAX, X64_RDX);
        if (imm & 0x10)
            emit_xor_r_imm(&e, X64_RAX, 0x10);
        emit_shr_r_imm(&e, X64_RAX, 4);
        emit_and_r_imm(&e, X64_RAX, 1);
        emit_store8(&e, X64_RAX, R_STATE, g_off_cc + 5);
    }
    EmitCyclesConst(e, 2);
}

// ---------- indexed effective addresses ----------
// Mirrors BlockJitA64.cpp exactly: the postbyte was resolved at decode
// time, so the addressing mode is an emit-time constant; the direct
// modes inline and everything else falls back to the call path. Cycle
// adds mirror the EA_Fn_* helpers in hd6309.cpp.

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

static int32_t EARegOffset(uint8_t ea_info)
{
    switch (EA_REG(ea_info))
    {
    case 0:  return g_off_x;
    case 1:  return g_off_y;
    case 2:  return g_off_u;
    default: return g_off_s;
    }
}

// Emit the EA into `dst` (a scratch register, zero-extended 16 bits)
// and add the mode's cycle cost. Clobbers RAX (the cycle-add and the
// register-offset modes use it); callers keep their own values out of
// RAX and dst across this.
static void EmitEA(emit_t& e, uint8_t ea_info, uint16_t operand, int dst)
{
    const int32_t reg_off = EARegOffset(ea_info);
    switch (EA_MODE(ea_info))
    {
    case EA_NO_OFFSET:
        emit_load16u(&e, dst, R_STATE, reg_off);
        break;

    case EA_5BIT:
    case EA_OFFSET_8:
    {
        const int off = (EA_MODE(ea_info) == EA_5BIT)
            ? (int16_t)operand
            : (int)(signed char)(operand & 0xFF);
        emit_load16u(&e, dst, R_STATE, reg_off);
        if (off != 0)
        {
            emit_add_r_imm(&e, dst, off);
            emit_movzx_r32_r16(&e, dst, dst);
        }
        EmitCyclesConst(e, 1);
        break;
    }

    case EA_OFFSET_16:
        emit_load16u(&e, dst, R_STATE, reg_off);
        emit_add_r_imm(&e, dst, (int32_t)operand);
        emit_movzx_r32_r16(&e, dst, dst);
        EmitCyclesRuntime(e, g_off_nat43);
        break;

    case EA_PC_8:
        emit_mov_r32_imm32(&e, dst, operand);   // pre-computed absolute
        EmitCyclesConst(e, 1);
        break;

    case EA_PC_16:
        emit_mov_r32_imm32(&e, dst, operand);   // pre-computed absolute
        EmitCyclesRuntime(e, g_off_nat53);
        break;

    case EA_OFFSET_A:
    case EA_OFFSET_B:
        emit_load16u(&e, dst, R_STATE, reg_off);
        emit_movsx_r32_r8(&e, X64_RAX,
                          EA_MODE(ea_info) == EA_OFFSET_A ? R_ACC_A : R_ACC_B);
        emit_add_rr(&e, dst, X64_RAX);
        emit_movzx_r32_r16(&e, dst, dst);
        EmitCyclesConst(e, 1);
        break;

    case EA_POST_INC1:
    case EA_POST_INC2:
        // ea = *reg, then *reg += n (store truncates to 16 bits)
        emit_load16u(&e, dst, R_STATE, reg_off);
        emit_lea32(&e, X64_RAX, dst,
                   EA_MODE(ea_info) == EA_POST_INC1 ? 1 : 2);
        emit_store16(&e, X64_RAX, R_STATE, reg_off);
        EmitCyclesRuntime(e, EA_MODE(ea_info) == EA_POST_INC1
                             ? g_off_nat21 : g_off_nat32);
        break;

    case EA_PRE_DEC1:
    case EA_PRE_DEC2:
        // *reg -= n, then ea = *reg
        emit_load16u(&e, dst, R_STATE, reg_off);
        emit_add_r_imm(&e, dst,
                       EA_MODE(ea_info) == EA_PRE_DEC1 ? -1 : -2);
        emit_movzx_r32_r16(&e, dst, dst);
        emit_store16(&e, dst, R_STATE, reg_off);
        EmitCyclesRuntime(e, EA_MODE(ea_info) == EA_PRE_DEC1
                             ? g_off_nat21 : g_off_nat32);
        break;

    default:
        break;   // unreachable per EAModeSupported
    }
}

// LEAX/LEAY (Z from the 16-bit result) and LEAU/LEAS (no flags).
static void EmitInlineLea(emit_t& e, const DecodedInst& insn,
                          int32_t dst_off, bool sets_z, uint8_t mask)
{
    EmitEA(e, insn.ea_info, insn.operand, X64_R9);
    emit_store16(&e, X64_R9, R_STATE, dst_off);
    if (sets_z && (mask & CC_BIT_Z))
    {
        emit_test_r16_r16(&e, X64_R9, X64_R9);
        emit_setcc_mem(&e, X64_CC_E, R_STATE, g_off_cc + 2);
    }
    EmitCyclesConst(e, 4);
}

// LDA/LDB indexed: read8(EA) -> acc, Z/N/V from the byte, +4.
static void EmitInlineLd8Idx(emit_t& e, const DecodedInst& insn,
                             int32_t reg_off, uint8_t mask)
{
    g_emit_had_side_effects = true;   // touches guest memory
    EmitEA(e, insn.ea_info, insn.operand, X64_R9);
    EmitMemRead8(e);
    emit_mov_r8_r8(&e, AccHostReg(reg_off), X64_RAX);
    EmitFlagsZNVFromReg8(e, X64_RAX, mask);
    EmitCyclesConst(e, 4);
}

// STA/STB indexed: write8(acc, EA), flags from acc, +4.
static void EmitInlineSt8Idx(emit_t& e, const DecodedInst& insn,
                             int32_t reg_off, uint8_t mask)
{
    g_emit_had_side_effects = true;   // touches guest memory
    EmitEA(e, insn.ea_info, insn.operand, X64_R9);
    emit_movzx_r32_r8(&e, X64_R8, AccHostReg(reg_off));
    EmitFlagsZNVFromReg8(e, X64_R8, mask);
    EmitMemWrite8(e);
    EmitCyclesConst(e, 4);
}

// TST indexed: flags from read8(EA), no writeback, +NatEmuCycles65.
static void EmitInlineTstIdx(emit_t& e, const DecodedInst& insn, uint8_t mask)
{
    g_emit_had_side_effects = true;   // touches guest memory
    EmitEA(e, insn.ea_info, insn.operand, X64_R9);
    EmitMemRead8(e);
    EmitFlagsZNVFromReg8(e, X64_RAX, mask);
    EmitCyclesRuntime(e, g_off_nat65);
}

// LDX/LDU/LDD indexed: read16(EA) -> reg16, Z/N(bit 15)/V=0, +5.
// D decomposes into the accumulator registers.
static void EmitInlineLd16Idx(emit_t& e, const DecodedInst& insn,
                              int32_t reg_off, uint8_t mask)
{
    g_emit_had_side_effects = true;   // touches guest memory
    EmitEA(e, insn.ea_info, insn.operand, X64_R9);
    EmitMemRead16(e);
    if (reg_off == g_off_d)
        EmitStoreD(e, X64_RAX, X64_RCX);
    else
        emit_store16(&e, X64_RAX, R_STATE, reg_off);
    EmitFlagsZNVFromReg16(e, X64_RAX, mask);
    EmitCyclesConst(e, 5);
}

// STD/STX indexed: write16(reg16, EA), flags from reg16, +5.
static void EmitInlineSt16Idx(emit_t& e, const DecodedInst& insn,
                              int32_t reg_off, uint8_t mask)
{
    g_emit_had_side_effects = true;   // touches guest memory
    EmitEA(e, insn.ea_info, insn.operand, X64_R9);
    if (reg_off == g_off_d)
        EmitLoadD(e, X64_R8, X64_RCX);
    else
        emit_load16u(&e, X64_R8, R_STATE, reg_off);
    EmitFlagsZNVFromReg16(e, X64_R8, mask);
    EmitMemWrite16(e);
    EmitCyclesConst(e, 5);
}

// ---------- memory-operand ALU / RMW / 16-bit / call-return families ----------

// EA into R9 for the three memory modes. MM_IDX callers must have
// verified EAModeSupported first.
static void EmitEAToR9(emit_t& e, int mode, const DecodedInst& insn)
{
    if (mode == MM_DP)
    {
        emit_load16u(&e, X64_R9, R_STATE, g_off_dp);
        const uint8_t off = (uint8_t)(insn.operand & 0xFF);
        if (off != 0)
            emit_or_r_imm(&e, X64_R9, off);
    }
    else if (mode == MM_EXT)
    {
        emit_mov_r32_imm32(&e, X64_R9, insn.operand);
    }
    else
    {
        EmitEA(e, insn.ea_info, insn.operand, X64_R9);
    }
}

// Cycle charge for a family/mode pair, mirroring the handlers exactly.
static void EmitCyclesForMode(emit_t& e, int mode,
                              int32_t dp_nat, uint32_t idx_const, int32_t ext_nat)
{
    if (mode == MM_DP)       EmitCyclesRuntime(e, dp_nat);
    else if (mode == MM_IDX) EmitCyclesConst(e, idx_const);
    else                     EmitCyclesRuntime(e, ext_nat);
}

// SUBA/CMPA/ADDA/ANDA/ORA/EORA/BITA (and B forms) with a memory
// operand: read8(EA) then the same 8-bit host-flag capture as the
// immediate forms. ADD also writes the untracked half-carry cc[H].
// Cycles: dp NatEmuCycles43, indexed +4, ext NatEmuCycles54.
static void EmitInlineAlu8Mem(emit_t& e, const DecodedInst& insn,
                              int op, int32_t reg_off, int mode, uint8_t mask)
{
    g_emit_had_side_effects = true;
    const int acc = AccHostReg(reg_off);
    EmitEAToR9(e, mode, insn);
    EmitMemRead8(e);                                  // eax = operand byte
    switch (op)
    {
    case ALU8_SUB:
    case ALU8_CMP:
        emit_alu_r8_r8(&e, (op == ALU8_SUB) ? X64_ALU_SUB : X64_ALU_CMP,
                       acc, X64_RAX);
        if (mask & CC_BIT_C) emit_setcc_mem(&e, X64_CC_B, R_STATE, g_off_cc + 0);
        if (mask & CC_BIT_V) emit_setcc_mem(&e, X64_CC_O, R_STATE, g_off_cc + 1);
        if (mask & CC_BIT_Z) emit_setcc_mem(&e, X64_CC_E, R_STATE, g_off_cc + 2);
        if (mask & CC_BIT_N) emit_setcc_mem(&e, X64_CC_S, R_STATE, g_off_cc + 3);
        break;
    case ALU8_ADD:
        emit_movzx_r32_r8(&e, X64_RCX, acc);          // save old for H
        emit_alu_r8_r8(&e, X64_ALU_ADD, acc, X64_RAX);
        if (mask & CC_BIT_C) emit_setcc_mem(&e, X64_CC_B, R_STATE, g_off_cc + 0);
        if (mask & CC_BIT_V) emit_setcc_mem(&e, X64_CC_O, R_STATE, g_off_cc + 1);
        if (mask & CC_BIT_Z) emit_setcc_mem(&e, X64_CC_E, R_STATE, g_off_cc + 2);
        if (mask & CC_BIT_N) emit_setcc_mem(&e, X64_CC_S, R_STATE, g_off_cc + 3);
        // H = bit 4 of (old ^ operand ^ result), unconditional.
        emit_movzx_r32_r8(&e, X64_RDX, acc);          // result
        emit_xor_rr(&e, X64_RCX, X64_RAX);
        emit_xor_rr(&e, X64_RCX, X64_RDX);
        emit_shr_r_imm(&e, X64_RCX, 4);
        emit_and_r_imm(&e, X64_RCX, 1);
        emit_store8(&e, X64_RCX, R_STATE, g_off_cc + 5);
        break;
    case ALU8_AND:
    case ALU8_OR:
    case ALU8_EOR:
        emit_alu_r8_r8(&e, (op == ALU8_AND) ? X64_ALU_AND
                         : (op == ALU8_OR)  ? X64_ALU_OR : X64_ALU_XOR,
                       acc, X64_RAX);
        if (mask & CC_BIT_Z) emit_setcc_mem(&e, X64_CC_E, R_STATE, g_off_cc + 2);
        if (mask & CC_BIT_N) emit_setcc_mem(&e, X64_CC_S, R_STATE, g_off_cc + 3);
        if (mask & CC_BIT_V) EmitCcConst(e, 1, 0);
        break;
    default:   // ALU8_BIT: AND flags, no writeback
        emit_test_r8_r8(&e, acc, X64_RAX);
        if (mask & CC_BIT_Z) emit_setcc_mem(&e, X64_CC_E, R_STATE, g_off_cc + 2);
        if (mask & CC_BIT_N) emit_setcc_mem(&e, X64_CC_S, R_STATE, g_off_cc + 3);
        if (mask & CC_BIT_V) EmitCcConst(e, 1, 0);
        break;
    }
    EmitCyclesForMode(e, mode, g_off_nat43, 4, g_off_nat54);
}

// Memory RMW: NEG/COM/LSR/ROR/ASR/ASL/ROL/DEC/INC/CLR on dp/idx/ext.
// The EA survives the read in the thunk's own stack slot ([rsp+32] -
// above the Win64 shadow the helpers' slow-path calls hand to their
// callees). CLR mirrors its handler: write 0, no read. ROR/ROL consume
// the OLD cc[C] byte (captured before the setcc overwrites it) - which
// is why they carry a reads-mask in the liveness analysis below.
// Cycles: dp NatEmuCycles65, indexed +6, ext NatEmuCycles76.
static void EmitInlineRmw8(emit_t& e, const DecodedInst& insn,
                           int op, int mode, uint8_t mask)
{
    g_emit_had_side_effects = true;
    EmitEAToR9(e, mode, insn);

    if (op == RMW8_CLR)
    {
        emit_mov_r32_imm32(&e, X64_R8, 0);
        EmitMemWrite8(e);
        if (mask & CC_BIT_C) EmitCcConst(e, 0, 0);
        if (mask & CC_BIT_N) EmitCcConst(e, 3, 0);
        if (mask & CC_BIT_V) EmitCcConst(e, 1, 0);
        if (mask & CC_BIT_Z) EmitCcConst(e, 2, 1);
        EmitCyclesForMode(e, mode, g_off_nat65, 6, g_off_nat76);
        return;
    }

    emit_store32(&e, X64_R9, X64_RSP, 32);   // save EA across the read
    EmitMemRead8(e);                         // eax = original byte
    emit_mov_rr(&e, X64_RDX, X64_RAX);
    switch (op)
    {
    case RMW8_NEG:
        emit_neg_r8(&e, X64_RDX);
        if (mask & CC_BIT_C) emit_setcc_mem(&e, X64_CC_B, R_STATE, g_off_cc + 0);
        if (mask & CC_BIT_V) emit_setcc_mem(&e, X64_CC_O, R_STATE, g_off_cc + 1);
        if (mask & CC_BIT_Z) emit_setcc_mem(&e, X64_CC_E, R_STATE, g_off_cc + 2);
        if (mask & CC_BIT_N) emit_setcc_mem(&e, X64_CC_S, R_STATE, g_off_cc + 3);
        break;
    case RMW8_COM:
        emit_alu_r8_imm8(&e, X64_ALU_XOR, X64_RDX, 0xFF);
        if (mask & CC_BIT_Z) emit_setcc_mem(&e, X64_CC_E, R_STATE, g_off_cc + 2);
        if (mask & CC_BIT_N) emit_setcc_mem(&e, X64_CC_S, R_STATE, g_off_cc + 3);
        if (mask & CC_BIT_V) EmitCcConst(e, 1, 0);
        if (mask & CC_BIT_C) EmitCcConst(e, 0, 1);
        break;
    case RMW8_LSR:
        emit_shift_r8_1(&e, X64_SHIFT_SHR, X64_RDX);
        if (mask & CC_BIT_C) emit_setcc_mem(&e, X64_CC_B, R_STATE, g_off_cc + 0);
        if (mask & CC_BIT_Z) emit_setcc_mem(&e, X64_CC_E, R_STATE, g_off_cc + 2);
        if (mask & CC_BIT_N) EmitCcConst(e, 3, 0);
        break;
    case RMW8_ROR:
        emit_load8u(&e, X64_RCX, R_STATE, g_off_cc + 0);   // OLD carry
        emit_shl_r_imm(&e, X64_RCX, 7);
        emit_shift_r8_1(&e, X64_SHIFT_SHR, X64_RDX);
        if (mask & CC_BIT_C) emit_setcc_mem(&e, X64_CC_B, R_STATE, g_off_cc + 0);
        emit_alu_r8_r8(&e, X64_ALU_OR, X64_RDX, X64_RCX);
        emit_test_r8_r8(&e, X64_RDX, X64_RDX);
        if (mask & CC_BIT_Z) emit_setcc_mem(&e, X64_CC_E, R_STATE, g_off_cc + 2);
        if (mask & CC_BIT_N) emit_setcc_mem(&e, X64_CC_S, R_STATE, g_off_cc + 3);
        break;
    case RMW8_ASR:
        emit_shift_r8_1(&e, X64_SHIFT_SAR, X64_RDX);
        if (mask & CC_BIT_C) emit_setcc_mem(&e, X64_CC_B, R_STATE, g_off_cc + 0);
        if (mask & CC_BIT_Z) emit_setcc_mem(&e, X64_CC_E, R_STATE, g_off_cc + 2);
        if (mask & CC_BIT_N) emit_setcc_mem(&e, X64_CC_S, R_STATE, g_off_cc + 3);
        break;
    case RMW8_ASL:
        emit_shift_r8_1(&e, X64_SHIFT_SHL, X64_RDX);
        if (mask & CC_BIT_C) emit_setcc_mem(&e, X64_CC_B, R_STATE, g_off_cc + 0);
        if (mask & CC_BIT_V) emit_setcc_mem(&e, X64_CC_O, R_STATE, g_off_cc + 1);
        if (mask & CC_BIT_Z) emit_setcc_mem(&e, X64_CC_E, R_STATE, g_off_cc + 2);
        if (mask & CC_BIT_N) emit_setcc_mem(&e, X64_CC_S, R_STATE, g_off_cc + 3);
        break;
    case RMW8_ROL:
        emit_load8u(&e, X64_RCX, R_STATE, g_off_cc + 0);   // OLD carry (0/1)
        emit_shift_r8_1(&e, X64_SHIFT_SHL, X64_RDX);
        if (mask & CC_BIT_C) emit_setcc_mem(&e, X64_CC_B, R_STATE, g_off_cc + 0);
        if (mask & CC_BIT_V) emit_setcc_mem(&e, X64_CC_O, R_STATE, g_off_cc + 1);
        emit_alu_r8_r8(&e, X64_ALU_OR, X64_RDX, X64_RCX);
        emit_test_r8_r8(&e, X64_RDX, X64_RDX);
        if (mask & CC_BIT_Z) emit_setcc_mem(&e, X64_CC_E, R_STATE, g_off_cc + 2);
        if (mask & CC_BIT_N) emit_setcc_mem(&e, X64_CC_S, R_STATE, g_off_cc + 3);
        break;
    case RMW8_DEC:
        emit_alu_r8_imm8(&e, X64_ALU_SUB, X64_RDX, 1);
        if (mask & CC_BIT_Z) emit_setcc_mem(&e, X64_CC_E, R_STATE, g_off_cc + 2);
        if (mask & CC_BIT_N) emit_setcc_mem(&e, X64_CC_S, R_STATE, g_off_cc + 3);
        if (mask & CC_BIT_V) emit_setcc_mem(&e, X64_CC_O, R_STATE, g_off_cc + 1);
        break;
    default:   // RMW8_INC
        emit_alu_r8_imm8(&e, X64_ALU_ADD, X64_RDX, 1);
        if (mask & CC_BIT_Z) emit_setcc_mem(&e, X64_CC_E, R_STATE, g_off_cc + 2);
        if (mask & CC_BIT_N) emit_setcc_mem(&e, X64_CC_S, R_STATE, g_off_cc + 3);
        if (mask & CC_BIT_V) emit_setcc_mem(&e, X64_CC_O, R_STATE, g_off_cc + 1);
        break;
    }
    emit_mov_rr(&e, X64_R8, X64_RDX);
    emit_load32(&e, X64_R9, X64_RSP, 32);    // restore EA
    EmitMemWrite8(e);
    EmitCyclesForMode(e, mode, g_off_nat65, 6, g_off_nat76);
}

// LDD/LDX/LDU and STD/STX/STU with dp/ext operands (the indexed forms
// already inline above). Cycles: dp NatEmuCycles54, ext NatEmuCycles65.
static void EmitInlineLd16Mem(emit_t& e, const DecodedInst& insn,
                              int32_t reg_off, int mode, uint8_t mask)
{
    g_emit_had_side_effects = true;
    EmitEAToR9(e, mode, insn);
    EmitMemRead16(e);
    if (reg_off == g_off_d)
        EmitStoreD(e, X64_RAX, X64_RCX);
    else
        emit_store16(&e, X64_RAX, R_STATE, reg_off);
    EmitFlagsZNVFromReg16(e, X64_RAX, mask);
    EmitCyclesRuntime(e, mode == MM_DP ? g_off_nat54 : g_off_nat65);
}

static void EmitInlineSt16Mem(emit_t& e, const DecodedInst& insn,
                              int32_t reg_off, int mode, uint8_t mask)
{
    g_emit_had_side_effects = true;
    if (reg_off == g_off_d)
        EmitLoadD(e, X64_R8, X64_RCX);
    else
        emit_load16u(&e, X64_R8, R_STATE, reg_off);
    EmitFlagsZNVFromReg16(e, X64_R8, mask);
    EmitEAToR9(e, mode, insn);
    EmitMemWrite16(e);
    EmitCyclesRuntime(e, mode == MM_DP ? g_off_nat54 : g_off_nat65);
}

// ADDD/SUBD/CMPD/CMPX/CMPY/CMPU: 16-bit host add/sub/cmp gives the
// 6309's C (bit 16), V (16-bit signed overflow), Z, N directly.
// source_mode -1 = immediate (operand baked); else MM_*. The cycle
// charge is passed in because it varies per (op, mode).
static void EmitInlineArith16(emit_t& e, const DecodedInst& insn,
                              int32_t dst_off, int alu, int source_mode,
                              int const_cycles, int32_t nat_offset,
                              uint8_t mask)
{
    if (source_mode < 0)
    {
        emit_mov_r32_imm32(&e, X64_RAX, insn.operand);
    }
    else
    {
        g_emit_had_side_effects = true;
        EmitEAToR9(e, source_mode, insn);
        EmitMemRead16(e);
    }
    if (dst_off == g_off_d)
        EmitLoadD(e, X64_RDX, X64_RCX);
    else
        emit_load16u(&e, X64_RDX, R_STATE, dst_off);
    emit_alu16_rr(&e, alu, X64_RDX, X64_RAX);
    if (mask & CC_BIT_C) emit_setcc_mem(&e, X64_CC_B, R_STATE, g_off_cc + 0);
    if (mask & CC_BIT_V) emit_setcc_mem(&e, X64_CC_O, R_STATE, g_off_cc + 1);
    if (mask & CC_BIT_Z) emit_setcc_mem(&e, X64_CC_E, R_STATE, g_off_cc + 2);
    if (mask & CC_BIT_N) emit_setcc_mem(&e, X64_CC_S, R_STATE, g_off_cc + 3);
    if (alu != X64_ALU_CMP)
    {
        if (dst_off == g_off_d)
            EmitStoreD(e, X64_RDX, X64_RCX);
        else
            emit_store16(&e, X64_RDX, R_STATE, dst_off);
    }
    if (const_cycles >= 0)
        EmitCyclesConst(e, (uint32_t)const_cycles);
    else
        EmitCyclesRuntime(e, nat_offset);
}

// ---------- call/return terminators (block's LAST instruction only) ----------

// JMP ext: PC = baked operand, NatEmuCycles43.
static void EmitInlineJmpE(emit_t& e, const DecodedInst& insn)
{
    emit_mov_mem16_imm16(&e, R_STATE, g_off_pc, insn.operand);
    EmitCyclesRuntime(e, g_off_nat43);
}

// RTS: PC = read16(S), S += 2, NatEmuCycles51. The handler reads both
// bytes at the OLD S (msb first), which the captured R9 preserves.
static void EmitInlineRts(emit_t& e)
{
    g_emit_had_side_effects = true;
    emit_load16u(&e, X64_R9, R_STATE, g_off_s);
    emit_lea32(&e, X64_RAX, X64_R9, 2);
    emit_store16(&e, X64_RAX, R_STATE, g_off_s);
    EmitMemRead16(e);
    emit_store16(&e, X64_RAX, R_STATE, g_off_pc);
    EmitCyclesRuntime(e, g_off_nat51);
}

// Shared push-return-address sequence for BSR/JSR: S -= 2, then
// write16(ret_pc, S) - same bytes at the same addresses as the
// handlers' two MemWrite8 calls (they store lsb then msb; both bytes
// land within the same instruction, so the order is guest-invisible).
static void EmitPushReturn(emit_t& e, uint16_t ret_pc)
{
    emit_load16u(&e, X64_RAX, R_STATE, g_off_s);
    emit_add_r_imm(&e, X64_RAX, -2);
    emit_movzx_r32_r16(&e, X64_RAX, X64_RAX);
    emit_store16(&e, X64_RAX, R_STATE, g_off_s);
    emit_mov_rr(&e, X64_R9, X64_RAX);
    emit_mov_r32_imm32(&e, X64_R8, ret_pc);
    EmitMemWrite16(e);
}

// BSR (target = ret_pc + signed8) and JSR ext (target = baked operand).
static void EmitInlineBsrJsrE(emit_t& e, const DecodedInst& insn,
                              uint16_t ret_pc, bool is_jsr_ext)
{
    g_emit_had_side_effects = true;
    const uint16_t target = is_jsr_ext
        ? insn.operand
        : (uint16_t)(ret_pc + (int8_t)(insn.operand & 0xFF));
    EmitPushReturn(e, ret_pc);
    emit_mov_mem16_imm16(&e, R_STATE, g_off_pc, target);
    EmitCyclesRuntime(e, is_jsr_ext ? g_off_nat87 : g_off_nat76);
}

// JSR indexed: EA first (the handler computes it before the push, so
// ,S-relative modes see the pre-push S), stashed across the push.
static void EmitInlineJsrX(emit_t& e, const DecodedInst& insn, uint16_t ret_pc)
{
    g_emit_had_side_effects = true;
    EmitEA(e, insn.ea_info, insn.operand, X64_R9);
    emit_store32(&e, X64_R9, X64_RSP, 32);
    EmitPushReturn(e, ret_pc);
    emit_load32(&e, X64_RAX, X64_RSP, 32);
    emit_store16(&e, X64_RAX, R_STATE, g_off_pc);
    EmitCyclesRuntime(e, g_off_nat76);
}

// ---------- branch terminators ----------
// Same table and predicate model as the arm64 backend; the select is a
// cmov instead of a branch-over-reassignment.

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

// Leaves the predicate value in EAX (nonzero = condition holds, before
// any taken_when_zero inversion).
static void EmitBranchPredicate(emit_t& e, BranchPred pred)
{
    switch (pred)
    {
    case BranchPred::Z:
        emit_load8u(&e, X64_RAX, R_STATE, g_off_cc + 2);
        break;
    case BranchPred::N:
        emit_load8u(&e, X64_RAX, R_STATE, g_off_cc + 3);
        break;
    case BranchPred::C:
        emit_load8u(&e, X64_RAX, R_STATE, g_off_cc + 0);
        break;
    case BranchPred::V:
        emit_load8u(&e, X64_RAX, R_STATE, g_off_cc + 1);
        break;
    case BranchPred::CorZ:
        emit_load8u(&e, X64_RAX, R_STATE, g_off_cc + 0);
        emit_load8u(&e, X64_RDX, R_STATE, g_off_cc + 2);
        emit_or_rr(&e, X64_RAX, X64_RDX);
        break;
    case BranchPred::NxorV:
        emit_load8u(&e, X64_RAX, R_STATE, g_off_cc + 3);
        emit_load8u(&e, X64_RDX, R_STATE, g_off_cc + 1);
        emit_xor_rr(&e, X64_RAX, X64_RDX);
        break;
    case BranchPred::ZorNxorV:
        emit_load8u(&e, X64_RAX, R_STATE, g_off_cc + 3);
        emit_load8u(&e, X64_RDX, R_STATE, g_off_cc + 1);
        emit_xor_rr(&e, X64_RAX, X64_RDX);
        emit_load8u(&e, X64_RDX, R_STATE, g_off_cc + 2);
        emit_or_rr(&e, X64_RAX, X64_RDX);
        break;
    case BranchPred::Always:
    case BranchPred::Never:
        break;
    }
}

// fall_pc is local_pc after this instruction; the taken target comes
// from the pre-decoded signed offset. Stores PC (both paths) and adds
// the exact handler cycle cost (long branches cost one more when taken).
static void EmitInlineBranch(emit_t& e, const DecodedInst& insn,
                             const BranchDesc& desc, uint16_t fall_pc)
{
    const uint16_t taken_pc = desc.is_long
        ? (uint16_t)(fall_pc + (int16_t)insn.operand)
        : (uint16_t)(fall_pc + (int8_t)(insn.operand & 0xFF));

    if (desc.pred == BranchPred::Always || desc.pred == BranchPred::Never)
    {
        const uint16_t pc = (desc.pred == BranchPred::Always) ? taken_pc : fall_pc;
        emit_mov_mem16_imm16(&e, R_STATE, g_off_pc, pc);
        if (desc.is_long)
            EmitCyclesRuntime(e, g_off_nat54);   // LBRA
        else
            EmitCyclesConst(e, 3);               // BRA / BRN
        return;
    }

    EmitBranchPredicate(e, desc.pred);

    const uint16_t pc_if_zero    = desc.taken_when_zero ? taken_pc : fall_pc;
    const uint16_t pc_if_nonzero = desc.taken_when_zero ? fall_pc : taken_pc;
    const uint32_t cyc_if_zero    = desc.is_long ? (desc.taken_when_zero ? 6u : 5u) : 3u;
    const uint32_t cyc_if_nonzero = desc.is_long ? (desc.taken_when_zero ? 5u : 6u) : 3u;

    emit_mov_r32_imm32(&e, X64_RCX, pc_if_zero);
    emit_mov_r32_imm32(&e, X64_RDX, pc_if_nonzero);
    emit_test_rr(&e, X64_RAX, X64_RAX);
    emit_cmovcc(&e, X64_CC_NE, X64_RCX, X64_RDX);
    emit_store16(&e, X64_RCX, R_STATE, g_off_pc);
    if (desc.is_long)
    {
        emit_mov_r32_imm32(&e, X64_R8,  cyc_if_zero);
        emit_mov_r32_imm32(&e, X64_R10, cyc_if_nonzero);
        emit_cmovcc(&e, X64_CC_NE, X64_R8, X64_R10);
        emit_add_rr(&e, R_CYC, X64_R8);
    }
    else
    {
        EmitCyclesConst(e, 3);
    }
}

// ---------- liveness analysis ----------
// Ported verbatim from BlockJitA64.cpp (target-independent logic).

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

    // Memory-operand ALU: sub/cmp/add write all four (add also the
    // untracked H); the logic ops write Z/N/V.
    for (int m = 0; m < MM_COUNT; ++m)
        for (int op = 0; op < ALU8_OP_COUNT; ++op)
            if (h == g_inlines.alu8_mem_a[op][m] ||
                h == g_inlines.alu8_mem_b[op][m])
                return (op <= ALU8_ADD) ? CC_ALL
                                        : (uint8_t)(CC_BIT_Z | CC_BIT_N | CC_BIT_V);

    // Memory RMW, per the handlers: NEG/COM/ASL/ROL/CLR write all four;
    // LSR/ASR/ROR write C/Z/N (V untouched); DEC/INC write Z/N/V.
    for (int m = 0; m < MM_COUNT; ++m)
        for (int op = 0; op < RMW8_OP_COUNT; ++op)
            if (h == g_inlines.rmw8_mem[op][m])
                switch (op)
                {
                case RMW8_NEG: case RMW8_COM: case RMW8_ASL:
                case RMW8_ROL: case RMW8_CLR:
                    return CC_ALL;
                case RMW8_LSR: case RMW8_ASR: case RMW8_ROR:
                    return CC_BIT_C | CC_BIT_Z | CC_BIT_N;
                default:   // DEC, INC
                    return CC_BIT_Z | CC_BIT_N | CC_BIT_V;
                }

    // 16-bit loads/stores (dp/ext): Z/N/V like their indexed cousins.
    for (int r = 0; r < R16_COUNT; ++r)
        if (h == g_inlines.ld16_dp[r]  || h == g_inlines.ld16_ext[r] ||
            h == g_inlines.st16_dp[r]  || h == g_inlines.st16_ext[r])
            return CC_BIT_Z | CC_BIT_N | CC_BIT_V;

    // 16-bit arithmetic: C/V/Z/N.
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

    // The call/return terminators (jmp_e/rts_i/bsr_r2/jsr_e/jsr_x) stay
    // CC_UNKNOWN deliberately: they inline only via the last-instruction
    // hook in EmitBlock, and CC_UNKNOWN keeps TryEmitInline refusing
    // them anywhere else.

    return CC_UNKNOWN;
}

// cc[] bits an inlined handler READS. ROL/ROR consume the incoming
// carry; everything else in the inline set reads nothing. Feeding this
// into the backward liveness pass keeps DSE from eliding a prior op's
// carry write that a following ROL/ROR would then read stale.
static uint8_t InlinedHandlerReadsMask(InstHandler h)
{
    for (int m = 0; m < MM_COUNT; ++m)
        if (h == g_inlines.rmw8_mem[RMW8_ROR][m] ||
            h == g_inlines.rmw8_mem[RMW8_ROL][m])
            return CC_BIT_C;
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

        // Backward transfer with the op's reads: writes kill liveness,
        // reads (ROL/ROR's carry-in) create it.
        live = (uint8_t)((live & (uint8_t)~mask) |
                         InlinedHandlerReadsMask(slot.insns[i].handler));
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

    // Families appended after the flat list, in stable index order, so
    // VCC_INLINE_MAX bisection covers them too.
    int base = (int)(sizeof(order) / sizeof(order[0]));
    for (int op = 0; op < ALU8_OP_COUNT; ++op)
        for (int m = 0; m < MM_COUNT; ++m)
        {
            if (h == g_inlines.alu8_mem_a[op][m]) return base + (op * MM_COUNT + m) * 2;
            if (h == g_inlines.alu8_mem_b[op][m]) return base + (op * MM_COUNT + m) * 2 + 1;
        }
    base += ALU8_OP_COUNT * MM_COUNT * 2;
    for (int op = 0; op < RMW8_OP_COUNT; ++op)
        for (int m = 0; m < MM_COUNT; ++m)
            if (h == g_inlines.rmw8_mem[op][m]) return base + op * MM_COUNT + m;
    base += RMW8_OP_COUNT * MM_COUNT;
    for (int r = 0; r < R16_COUNT; ++r)
    {
        if (h == g_inlines.ld16_dp[r])  return base + r * 4;
        if (h == g_inlines.ld16_ext[r]) return base + r * 4 + 1;
        if (h == g_inlines.st16_dp[r])  return base + r * 4 + 2;
        if (h == g_inlines.st16_ext[r]) return base + r * 4 + 3;
    }
    base += R16_COUNT * 4;
    const InstHandler a16[] = {
        g_inlines.addd_m, g_inlines.addd_d, g_inlines.addd_x, g_inlines.addd_e,
        g_inlines.subd_m, g_inlines.subd_x,
        g_inlines.cmpd_m, g_inlines.cmpd_x,
        g_inlines.cmpx_m, g_inlines.cmpx_x,
        g_inlines.cmpy_m, g_inlines.cmpu_m, g_inlines.cmpu_x,
    };
    for (int i = 0; i < (int)(sizeof(a16) / sizeof(a16[0])); ++i)
        if (h == a16[i]) return base + i;

    return -1;
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

    // Memory-operand ALU and RMW families (dp/idx/ext). Indexed modes
    // still require a supported EA.
    for (int m = 0; m < MM_COUNT; ++m)
    {
        if (m == MM_IDX && !EAModeSupported(insn.ea_info))
            continue;
        for (int op = 0; op < ALU8_OP_COUNT; ++op)
        {
            if (h == g_inlines.alu8_mem_a[op][m])
            { EmitInlineAlu8Mem(e, insn, op, g_off_a, m, mask); return true; }
            if (h == g_inlines.alu8_mem_b[op][m])
            { EmitInlineAlu8Mem(e, insn, op, g_off_b, m, mask); return true; }
        }
        for (int op = 0; op < RMW8_OP_COUNT; ++op)
            if (h == g_inlines.rmw8_mem[op][m])
            { EmitInlineRmw8(e, insn, op, m, mask); return true; }
    }

    // 16-bit loads/stores, dp/ext.
    {
        static const int32_t* kRegOff[R16_COUNT] = { &g_off_d, &g_off_x, &g_off_u };
        for (int r = 0; r < R16_COUNT; ++r)
        {
            if (h == g_inlines.ld16_dp[r])  { EmitInlineLd16Mem(e, insn, *kRegOff[r], MM_DP,  mask); return true; }
            if (h == g_inlines.ld16_ext[r]) { EmitInlineLd16Mem(e, insn, *kRegOff[r], MM_EXT, mask); return true; }
            if (h == g_inlines.st16_dp[r])  { EmitInlineSt16Mem(e, insn, *kRegOff[r], MM_DP,  mask); return true; }
            if (h == g_inlines.st16_ext[r]) { EmitInlineSt16Mem(e, insn, *kRegOff[r], MM_EXT, mask); return true; }
        }
    }

    // 16-bit arithmetic. Cycle charges verified against each handler.
    if (h == g_inlines.addd_m) { EmitInlineArith16(e, insn, g_off_d, X64_ALU_ADD, -1,     -1, g_off_nat43, mask); return true; }
    if (h == g_inlines.addd_d) { EmitInlineArith16(e, insn, g_off_d, X64_ALU_ADD, MM_DP,  -1, g_off_nat64, mask); return true; }
    if (h == g_inlines.addd_x && EAModeSupported(insn.ea_info))
                               { EmitInlineArith16(e, insn, g_off_d, X64_ALU_ADD, MM_IDX, -1, g_off_nat65, mask); return true; }
    if (h == g_inlines.addd_e) { EmitInlineArith16(e, insn, g_off_d, X64_ALU_ADD, MM_EXT, -1, g_off_nat76, mask); return true; }
    if (h == g_inlines.subd_m) { EmitInlineArith16(e, insn, g_off_d, X64_ALU_SUB, -1,     -1, g_off_nat43, mask); return true; }
    if (h == g_inlines.subd_x && EAModeSupported(insn.ea_info))
                               { EmitInlineArith16(e, insn, g_off_d, X64_ALU_SUB, MM_IDX, -1, g_off_nat65, mask); return true; }
    if (h == g_inlines.cmpd_m) { EmitInlineArith16(e, insn, g_off_d, X64_ALU_CMP, -1,     -1, g_off_nat54, mask); return true; }
    if (h == g_inlines.cmpd_x && EAModeSupported(insn.ea_info))
                               { EmitInlineArith16(e, insn, g_off_d, X64_ALU_CMP, MM_IDX, -1, g_off_nat76, mask); return true; }
    if (h == g_inlines.cmpx_m) { EmitInlineArith16(e, insn, g_off_x, X64_ALU_CMP, -1,     -1, g_off_nat43, mask); return true; }
    if (h == g_inlines.cmpx_x && EAModeSupported(insn.ea_info))
                               { EmitInlineArith16(e, insn, g_off_x, X64_ALU_CMP, MM_IDX, -1, g_off_nat65, mask); return true; }
    if (h == g_inlines.cmpy_m) { EmitInlineArith16(e, insn, g_off_y, X64_ALU_CMP, -1,     -1, g_off_nat54, mask); return true; }
    if (h == g_inlines.cmpu_m) { EmitInlineArith16(e, insn, g_off_u, X64_ALU_CMP, -1,     -1, g_off_nat54, mask); return true; }
    if (h == g_inlines.cmpu_x && EAModeSupported(insn.ea_info))
                               { EmitInlineArith16(e, insn, g_off_u, X64_ALU_CMP, MM_IDX, -1, g_off_nat76, mask); return true; }

    return false;
}

// ---------- thunk runner and chain stub ----------
// Same design as the arm64 backend: one emitted runner establishes the
// pinned-register convention and is the ONLY entry into any thunk; one
// shared chain stub revalidates what the dispatcher fast path would
// (cycle budget, ChainBreak, slot tag + generation + native_entry) and
// tail-jumps into the next thunk, RETing back to the runner otherwise.
// Linking is indirect through the live cache slot, never a patched
// address, so invalidation severs links with no unlink bookkeeping.

static void EnsureThunkRunner()
{
    if (g_thunk_runner != nullptr)
        return;
    if (g_addrs.cycle_for == nullptr)
        return;

    const int64_t off_cycle_for = (const uint8_t*)g_addrs.cycle_for
                                - (const uint8_t*)g_addrs.base;
    if (off_cycle_for < 0 || off_cycle_for > INT32_MAX)
        return;

    constexpr size_t kRunnerBytes = 128;
    if (g_arena_used + kRunnerBytes > kArenaSize)
        return;

    uint8_t* const entry = g_arena_base + g_arena_used;
    emit_t e { entry, 0, (uint32_t)kRunnerBytes };

    // Entry rsp is 8 mod 16; six pushes make it 8 again; the 40-byte
    // adjustment (32 shadow + 8 align) makes the call site 0 mod 16,
    // so the thunk enters at 8 mod 16 — which kThunkFrame then
    // corrects at every interior call site. The runner establishes the
    // full register convention: cycles in ebx, A in bpl, B in r12b,
    // state base in r13, slot base in r15; it spills cycles and the
    // accumulators back to cpu_state when the chain finally returns.
    emit_push(&e, X64_RBX);
    emit_push(&e, X64_RBP);
    emit_push(&e, X64_R12);
    emit_push(&e, X64_R13);
    emit_push(&e, X64_R14);
    emit_push(&e, X64_R15);
    emit_sub_rsp_imm8(&e, 40);
    emit_mov_r64_r64(&e, X64_RAX, ARG0);
    emit_mov_r64_imm64(&e, R_STATE, (uint64_t)(uintptr_t)g_addrs.base);
    emit_mov_r64_imm64(&e, R_SLOTS, (uint64_t)(uintptr_t)g_addrs.chain_slot_base);
    emit_load32(&e, R_CYC, R_STATE, g_off_cyc);
    emit_load8u(&e, R_ACC_A, R_STATE, g_off_a);
    emit_load8u(&e, R_ACC_B, R_STATE, g_off_b);
    emit_call_r(&e, X64_RAX);
    emit_store32(&e, R_CYC, R_STATE, g_off_cyc);
    emit_store8(&e, R_ACC_A, R_STATE, g_off_a);
    emit_store8(&e, R_ACC_B, R_STATE, g_off_b);
    emit_add_rsp_imm8(&e, 40);
    emit_pop(&e, X64_R15);
    emit_pop(&e, X64_R14);
    emit_pop(&e, X64_R13);
    emit_pop(&e, X64_R12);
    emit_pop(&e, X64_RBP);
    emit_pop(&e, X64_RBX);
    emit_x64_ret(&e);

    if (e.offset > e.capacity)
        return;
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

    const uint8_t* cbase = (const uint8_t*)g_addrs.base;
    const int64_t off_pending = (const uint8_t*)g_addrs.chain_break       - cbase;
    const int64_t off_gen     = (const uint8_t*)g_addrs.generation_mirror - cbase;
    const int64_t off_runs    = (const uint8_t*)g_addrs.chain_runs        - cbase;
    auto in_range = [](int64_t off) { return off >= 0 && off <= INT32_MAX; };
    const uint32_t ssz = g_addrs.chain_slot_size;
    const bool size_pow2 = (ssz & (ssz - 1)) == 0;
    if (!in_range(off_pending) || !in_range(off_gen) || !in_range(off_runs) ||
        !size_pow2)
    {
        g_no_link = true;   // layout contract not met; run unlinked
        return;
    }
    uint32_t slot_shift = 0;
    while ((1u << slot_shift) < ssz) ++slot_shift;

    constexpr size_t kStubBytes = 192;
    if (g_arena_used + kStubBytes > kArenaSize)
        return;             // retried after the next arena flush

    uint8_t* const entry = g_arena_base + g_arena_used;
    emit_t e { entry, 0, (uint32_t)kStubBytes };

    // Forward jcc rel32 fields to patch once the bail RET's position is
    // known. Each entry is the offset of the 4-byte displacement.
    uint32_t fixups[8];
    int nfix = 0;

    // Budget exhausted -> bail. CycleCounter is LIVE in ebx and
    // &cpu_state in r13 (runner convention); CycleFor is a memory read
    // now that r12 carries the B accumulator.
    const int64_t off_cycle_for_stub = (const uint8_t*)g_addrs.cycle_for - cbase;
    emit_cmp_r32_mem32(&e, R_CYC, R_STATE, (int32_t)off_cycle_for_stub);
    emit_jcc_rel32(&e, X64_CC_GE, 0);
    fixups[nfix++] = emit_pos(&e) - 4;

    // Any reason to leave the chain is the maintained ChainBreak byte.
    emit_alu_mem8_imm8(&e, X64_ALU_CMP, R_STATE, (int32_t)off_pending, 0);
    emit_jcc_rel32(&e, X64_CC_NE, 0);
    fixups[nfix++] = emit_pos(&e) - 4;

    // r8d = next PC; r9 = &slot = slot_base + (pc & MASK) << shift
    emit_load16u(&e, X64_R8, R_STATE, g_off_pc);
    emit_mov_rr(&e, X64_R9, X64_R8);
    emit_and_r_imm(&e, X64_R9, (int32_t)BlockCache::CACHE_MASK);
    emit_shl_r64_imm(&e, X64_R9, (uint8_t)slot_shift);
    emit_add_r64_r64(&e, X64_R9, R_SLOTS);

    // Slot must hold this PC in the current generation, with a thunk.
    emit_cmp_mem16_r16(&e, X64_R9, (int32_t)offsetof(CachedBlock, start_pc), X64_R8);
    emit_jcc_rel32(&e, X64_CC_NE, 0);
    fixups[nfix++] = emit_pos(&e) - 4;
    emit_load32(&e, X64_RAX, X64_R9, (int32_t)offsetof(CachedBlock, generation));
    emit_cmp_r32_mem32(&e, X64_RAX, R_STATE, (int32_t)off_gen);
    emit_jcc_rel32(&e, X64_CC_NE, 0);
    fixups[nfix++] = emit_pos(&e) - 4;

    // Whole next block must fit the budget with the shared overshoot
    // slack: cycles + total - kBudgetSlack <= CycleFor (signed).
    emit_load8u(&e, X64_RAX, X64_R9, (int32_t)offsetof(CachedBlock, total_cycles));
    emit_add_rr(&e, X64_RAX, R_CYC);
    emit_add_r_imm(&e, X64_RAX, -(int32_t)kBudgetSlack);
    emit_cmp_r32_mem32(&e, X64_RAX, R_STATE, (int32_t)off_cycle_for_stub);
    emit_jcc_rel32(&e, X64_CC_G, 0);
    fixups[nfix++] = emit_pos(&e) - 4;

    emit_load64(&e, X64_RAX, X64_R9, (int32_t)offsetof(CachedBlock, native_entry));
    emit_test_r64_r64(&e, X64_RAX, X64_RAX);
    emit_jcc_rel32(&e, X64_CC_E, 0);
    fixups[nfix++] = emit_pos(&e) - 4;

    // Transition counter, only when stats were asked for.
    static const bool jit_stats = std::getenv("VCC_JIT_STATS") != nullptr;
    if (jit_stats)
        emit_add_mem64_imm8(&e, R_STATE, (int32_t)off_runs, 1);

    emit_jmp_r(&e, X64_RAX);

    // bail: back to the runner.
    const uint32_t bail_at = emit_pos(&e);
    emit_x64_ret(&e);

    for (int i = 0; i < nfix; ++i)
        SafePatchRel32(e, fixups[i], bail_at);

    if (e.offset > e.capacity)
    {
        g_no_link = true;   // refuse to link rather than run truncated code
        return;
    }

    g_arena_used += e.offset;
    g_chain_stub = entry;
}

// ---------- block emitter ----------

NativeEntry EmitBlock(const CachedBlock& slot)
{
    if (g_disabled || g_arena_base == nullptr ||
        g_addrs.base == nullptr || g_addrs.pc == nullptr)
        return nullptr;

    // Thunks assume the pinned-register convention; without the runner
    // to establish it, refuse to emit.
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

    // Backward liveness pass for cc[] writes; the forward pass emits
    // only the live subset.
    uint8_t live_writes[BlockCache::MAX_BLOCK_INSNS];
    uint32_t cc_requested = 0;
    uint32_t cc_elided    = 0;
    AnalyzeFlagLiveness(slot, live_writes, &cc_requested, &cc_elided);
    g_cc_writes_requested += cc_requested;
    g_cc_writes_elided    += cc_elided;

    uint8_t* const entry = g_arena_base + g_arena_used;
    emit_t e { entry, 0, (uint32_t)needed };
    g_emit_had_side_effects = false;

    // Prologue: alignment/shadow adjustment plus the per-block operand
    // base. The pinned state registers are already live (runner).
    emit_sub_rsp_imm8(&e, kThunkFrame);
    emit_mov_r64_imm64(&e, R_INSNS, (uint64_t)(uintptr_t)&slot.insns[0]);

    // PC-skipping state, same policy as the other backends: inlined ops
    // don't read PC, so the write is deferred until the next handler
    // call (which needs the post-insn PC pre-set) or the block tail.
    uint16_t local_pc = slot.start_pc;
    bool     pc_dirty = false;

    // Superblock guard exits (see BlockJitA64.cpp for the full design
    // notes). Each records the offset of a jcc's rel32 field, patched
    // to its exit snippet after the main tail is emitted.
    // kind: 0/1 = inline guard (direction diverged; 0 jz, 1 jnz),
    //       2 = called guard (bare exit; PC and cycles already right),
    //       3 = interrupt pending at a loop back-edge (exit along the
    //           RECORDED path so the dispatcher can deliver).
    struct GuardExit
    {
        uint32_t field_at;    // offset of the rel32 field to patch
        uint8_t  kind;
        uint16_t exit_pc;     // PC to store on exit (kinds 0/1/3)
        uint32_t extra_cycles;// exit-minus-recorded cycle delta (long branches)
    };
    GuardExit guard_exits[BlockCache::MAX_BLOCK_INSNS * 2];
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

        // Mid-block branch = trace guard (taken_mask bit says which way
        // the recording went; a diverging outcome exits with the other
        // successor's PC through an exit snippet).
        if (i < (int)slot.num_insns - 1 && LookupBranch(insn.handler, bdesc))
        {
            const bool taken_dir = ((slot.taken_mask >> i) & 1u) != 0;
            const uint16_t taken_pc = bdesc.is_long
                ? (uint16_t)(local_pc + (int16_t)insn.operand)
                : (uint16_t)(local_pc + (int8_t)(insn.operand & 0xFF));
            const uint16_t fall_pc = local_pc;
            const uint16_t cont_pc = taken_dir ? taken_pc : fall_pc;
            const uint16_t exit_pc = taken_dir ? fall_pc : taken_pc;

            // Impossible direction/predicate pairs mean corrupted block
            // data - refuse.
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
                    const uint32_t cont_cyc = bdesc.is_long ? (taken_dir ? 6u : 5u) : 3u;
                    const uint32_t exit_cyc = bdesc.is_long ? (taken_dir ? 5u : 6u) : 3u;
                    EmitCyclesConst(e, cont_cyc);
                    // Exit when the runtime outcome is NOT the recorded
                    // direction.
                    const bool exit_on_zero = taken_dir ? !bdesc.taken_when_zero
                                                        : bdesc.taken_when_zero;
                    emit_test_rr(&e, X64_RAX, X64_RAX);
                    emit_jcc_rel32(&e, exit_on_zero ? X64_CC_E : X64_CC_NE, 0);
                    guard_exits[num_guard_exits++] = {
                        emit_pos(&e) - 4, (uint8_t)(exit_on_zero ? 0 : 1), exit_pc,
                        (exit_cyc > cont_cyc) ? exit_cyc - cont_cyc : 0u };
                }
                // Loop back-edge (taken direction): bound interrupt
                // latency to one loop iteration via the ChainBreak byte.
                if (taken_dir && g_off_pending >= 0)
                {
                    emit_alu_mem8_imm8(&e, X64_ALU_CMP, R_STATE,
                                       (int32_t)g_off_pending, 0);
                    emit_jcc_rel32(&e, X64_CC_NE, 0);
                    guard_exits[num_guard_exits++] = {
                        emit_pos(&e) - 4, 3, cont_pc, 0u };
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
            emit_mov_mem16_imm16(&e, R_STATE, g_off_pc, fall_pc);
            ++g_pc_writes_emitted;
            emit_store32(&e, R_CYC, R_STATE, g_off_cyc);
            EmitSpillAcc(e);
            emit_lea64(&e, ARG0, R_INSNS, (int32_t)(i * sizeof(DecodedInst)));
            EmitCallAbs(e, (const void*)insn.handler);
            emit_load32(&e, R_CYC, R_STATE, g_off_cyc);
            EmitReloadAcc(e);
            emit_load16u(&e, X64_RAX, R_STATE, g_off_pc);
            emit_cmp_r_imm(&e, X64_RAX, cont_pc);
            emit_jcc_rel32(&e, X64_CC_NE, 0);
            guard_exits[num_guard_exits++] = { emit_pos(&e) - 4, 2, 0, 0u };
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
            EmitEA(e, insn.ea_info, insn.operand, X64_R9);
            emit_store16(&e, X64_R9, R_STATE, g_off_pc);
            EmitCyclesConst(e, 3);
            pc_dirty = false;
            ++g_insns_inlined;
            ++g_pc_writes_skipped;
            continue;
        }

        // Call/return terminators, last instruction only: both PCs (and
        // the pushed return address, = local_pc) are emit-time values.
        if (i == (int)slot.num_insns - 1 && !g_no_inline)
        {
            bool done = false;
            if (insn.handler == g_inlines.jmp_e)
            { EmitInlineJmpE(e, insn); done = true; }
            else if (insn.handler == g_inlines.rts_i)
            { EmitInlineRts(e); done = true; }
            else if (insn.handler == g_inlines.bsr_r2)
            { EmitInlineBsrJsrE(e, insn, local_pc, false); done = true; }
            else if (insn.handler == g_inlines.jsr_e)
            { EmitInlineBsrJsrE(e, insn, local_pc, true); done = true; }
            else if (insn.handler == g_inlines.jsr_x &&
                     EAModeSupported(insn.ea_info))
            { EmitInlineJsrX(e, insn, local_pc); done = true; }
            if (done)
            {
                pc_dirty = false;   // the terminator stored PC itself
                ++g_insns_inlined;
                ++g_pc_writes_skipped;
                continue;
            }
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
            emit_mov_mem16_imm16(&e, R_STATE, g_off_pc, local_pc);
            ++g_pc_writes_emitted;
            pc_dirty = false;

            // handler(&slot.insns[i]) - handlers read and add cycles in
            // cpu_state, so sync the registerized counter AND the
            // accumulators around the call.
            g_emit_had_side_effects = true;
            emit_store32(&e, R_CYC, R_STATE, g_off_cyc);
            EmitSpillAcc(e);
            emit_lea64(&e, ARG0, R_INSNS, (int32_t)(i * sizeof(DecodedInst)));
            EmitCallAbs(e, (const void*)insn.handler);
            emit_load32(&e, R_CYC, R_STATE, g_off_cyc);
            EmitReloadAcc(e);
            ++g_insns_called;
        }
    }

    // Flush PC if the block ended on an inlined op, so the dispatcher's
    // next-block lookup sees the right address.
    if (pc_dirty)
    {
        emit_mov_mem16_imm16(&e, R_STATE, g_off_pc, local_pc);
        ++g_pc_writes_emitted;
    }

    // Epilogue, then chain: with the stack adjustment undone (the
    // runner's return address back on top), jump to the shared stub,
    // which either tail-jumps into the next block's thunk or RETs to
    // the runner. Stack depth is constant across a chain of any length.
    emit_add_rsp_imm8(&e, kThunkFrame);
    if (g_chain_stub != nullptr)
        emit_jmp_rel32(&e, (int32_t)(g_chain_stub - (entry + emit_pos(&e) + 5)));
    else
        emit_x64_ret(&e);

    // Guard exit snippets: store the exit PC (bare exits already have
    // PC and cycles right), undo the stack adjustment, and leave
    // through the same door as the main tail. Then patch each guard's
    // rel32 field.
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
                bare_exit_at = emit_pos(&e);
                bare_exit_emitted = true;
                emit_add_rsp_imm8(&e, kThunkFrame);
                if (g_chain_stub != nullptr)
                    emit_jmp_rel32(&e, (int32_t)(g_chain_stub - (entry + emit_pos(&e) + 5)));
                else
                    emit_x64_ret(&e);
            }
            target = bare_exit_at;
        }
        else
        {
            target = emit_pos(&e);
            emit_mov_mem16_imm16(&e, R_STATE, g_off_pc, ge.exit_pc);
            if (ge.extra_cycles != 0)
                emit_add_r_imm(&e, R_CYC, (int32_t)ge.extra_cycles);
            emit_add_rsp_imm8(&e, kThunkFrame);
            if (g_chain_stub != nullptr)
                emit_jmp_rel32(&e, (int32_t)(g_chain_stub - (entry + emit_pos(&e) + 5)));
            else
                emit_x64_ret(&e);
        }

        SafePatchRel32(e, ge.field_at, target);
    }

    if (e.offset > e.capacity)
    {
        // Budget bug: refuse the block rather than run truncated code.
        ++g_emit_failures;
        return nullptr;
    }

    // x86 icache is coherent with data writes - no flush needed.

    g_arena_used += e.offset;
    ++g_blocks_emitted;

    return reinterpret_cast<NativeEntry>(entry);
}

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
