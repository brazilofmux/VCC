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
static uint32_t           g_blocks_emitted = 0;
static uint32_t           g_emit_failures  = 0;
static uint32_t           g_insns_called   = 0;
static uint32_t           g_insns_inlined  = 0;
static uint32_t           g_pc_writes_emitted = 0;
static uint32_t           g_pc_writes_skipped = 0;
static uint32_t           g_cc_writes_requested = 0;
static uint32_t           g_cc_writes_elided    = 0;

// Field offsets from CpuAddrs.base, computed once at Init. Every one
// fits the unsigned-immediate load/store forms with room to spare.
static uint32_t g_off_pc = 0;

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
        g_off_pc = (uint32_t)((uint8_t*)g_addrs.pc - (uint8_t*)g_addrs.base);
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
}

// ---------- size budget (bytes of arm64 code) ----------

// Prologue: 3 frame insns + two imm64 materializations (up to 4 each).
static constexpr size_t kPrologueBytes = (3 + 4 + 4) * 4;
// Per called insn: movz + strh + add + imm64(<=4) + blr = 8 insns.
static constexpr size_t kMaxBytesPerInsn = 8 * 4;
// Epilogue: 2 ldp + ret, plus a final PC flush when the block ends on
// an inlined op (level-2, later).
static constexpr size_t kEpilogueBytes = (2 + 1 + 2) * 4;

// ---------- block emitter ----------

NativeEntry EmitBlock(const CachedBlock& slot)
{
    if (g_disabled || g_arena_base == nullptr ||
        g_addrs.base == nullptr || g_addrs.pc == nullptr)
        return nullptr;

    const size_t needed =
        kPrologueBytes + (size_t)slot.num_insns * kMaxBytesPerInsn + kEpilogueBytes;
    if (g_arena_used + needed > kArenaSize)
    {
        ++g_emit_failures;
        return nullptr;
    }

    uint8_t* const entry = g_arena_base + g_arena_used;
    emit_t e { entry, 0, (uint32_t)needed };

    jit_writable_begin();

    // Prologue.
    emit_stp_pre_sp(&e, A64_W29, A64_W30, -32);
    emit_stp_x64_off(&e, A64_W19, A64_W20, A64_SP, 16);
    emit_mov_x64_x64(&e, A64_W29, A64_SP);
    emit_mov_x64_imm64(&e, A64_W19, (uint64_t)(uintptr_t)g_addrs.base);
    emit_mov_x64_imm64(&e, A64_W20, (uint64_t)(uintptr_t)&slot.insns[0]);

    uint16_t local_pc = slot.start_pc;

    for (int i = 0; i < (int)slot.num_insns; ++i)
    {
        const DecodedInst& insn = slot.insns[i];
        local_pc = (uint16_t)(local_pc + insn.length);

        // Pre-set PC to the post-instruction value, exactly as the
        // interpreter dispatch loop would before calling the handler.
        emit_movz_w32(&e, A64_W1, local_pc, 0);
        emit_strh_imm(&e, A64_W1, A64_W19, g_off_pc);
        ++g_pc_writes_emitted;

        // handler(&slot.insns[i])
        emit_add_x64_imm(&e, A64_W0, A64_W20, (uint32_t)(i * sizeof(DecodedInst)));
        emit_mov_x64_imm64(&e, A64_W16, (uint64_t)(uintptr_t)insn.handler);
        emit_blr(&e, A64_W16);
        ++g_insns_called;
    }

    // Epilogue.
    emit_ldp_x64_off(&e, A64_W19, A64_W20, A64_SP, 16);
    emit_ldp_x64_post(&e, A64_W29, A64_W30, A64_SP, 32);
    emit_ret(&e);

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
