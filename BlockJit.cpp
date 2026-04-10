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

#include "BlockJit.h"
#include "BlockCache.h"
#include <cstring>
#include <Windows.h>

namespace BlockJit
{

// 16 MB code arena. ROM pre-population emits ~1.8 MB of thunks per pass
// at level-1; level-2 inlining bumps that ~10% in the worst case. We
// can absorb several SoftReset cycles before the arena fills, after
// which EmitBlock starts returning nullptr and new blocks fall back to
// the interpreter (correct, just slower). Real arena lifecycle is a
// level-3 concern.
static constexpr size_t kArenaSize = 16 * 1024 * 1024;

static uint8_t*           g_arena_base   = nullptr;
static size_t             g_arena_used   = 0;
static CpuAddrs           g_addrs        {};
static InlineableHandlers g_inlines      {};
static uint32_t           g_blocks_emitted = 0;
static uint32_t           g_emit_failures  = 0;
static uint32_t           g_insns_called   = 0;
static uint32_t           g_insns_inlined  = 0;

void Init(const CpuAddrs& addrs, const InlineableHandlers& handlers)
{
    g_addrs = addrs;
    g_inlines = handlers;

    if (g_arena_base == nullptr)
    {
        // PAGE_EXECUTE_READWRITE: emit and execute from the same pages.
        // No DEP toggling needed. CFG-hardened binaries would need
        // VirtualAlloc2 + indirect call thunks; we're not there yet.
        g_arena_base = (uint8_t*)VirtualAlloc(
            nullptr, kArenaSize, MEM_RESERVE | MEM_COMMIT,
            PAGE_EXECUTE_READWRITE);
    }
    g_arena_used = 0;
    g_blocks_emitted = 0;
    g_emit_failures = 0;
    g_insns_called = 0;
    g_insns_inlined = 0;
}

void Reset()
{
    g_arena_used = 0;
    g_blocks_emitted = 0;
    g_emit_failures = 0;
    g_insns_called = 0;
    g_insns_inlined = 0;
}

// ---------- low-level x86 emitter helpers ----------

// mov word ptr [imm32], imm16   - 9 bytes (66 C7 05 <addr32> <imm16>)
static void EmitMovMem16Imm16(uint8_t*& p, const void* addr, uint16_t imm)
{
    const uint32_t addr_imm = (uint32_t)(uintptr_t)addr;
    p[0] = 0x66;
    p[1] = 0xC7;
    p[2] = 0x05;
    std::memcpy(p + 3, &addr_imm, 4);
    std::memcpy(p + 7, &imm,      2);
    p += 9;
}

// mov byte ptr [imm32], imm8    - 7 bytes (C6 05 <addr32> <imm8>)
static void EmitMovMem8Imm8(uint8_t*& p, const void* addr, uint8_t imm)
{
    const uint32_t addr_imm = (uint32_t)(uintptr_t)addr;
    p[0] = 0xC6;
    p[1] = 0x05;
    std::memcpy(p + 2, &addr_imm, 4);
    p[6] = imm;
    p += 7;
}

// add dword ptr [imm32], imm8 (sign-extended)  - 7 bytes (83 05 <addr32> <imm8>)
static void EmitAddMem32Imm8(uint8_t*& p, const void* addr, int8_t imm)
{
    const uint32_t addr_imm = (uint32_t)(uintptr_t)addr;
    p[0] = 0x83;
    p[1] = 0x05;
    std::memcpy(p + 2, &addr_imm, 4);
    p[6] = (uint8_t)imm;
    p += 7;
}

// push imm32                    - 5 bytes (68 <imm32>)
static void EmitPushImm32(uint8_t*& p, uint32_t imm)
{
    p[0] = 0x68;
    std::memcpy(p + 1, &imm, 4);
    p += 5;
}

// call rel32                    - 5 bytes (E8 <rel32>)
static void EmitCallRel32(uint8_t*& p, const void* target)
{
    const uint8_t* const call_site_end = p + 5;
    const int32_t rel = (int32_t)((intptr_t)target - (intptr_t)call_site_end);
    p[0] = 0xE8;
    std::memcpy(p + 1, &rel, 4);
    p += 5;
}

// add esp, 4                    - 3 bytes (83 C4 04)
static void EmitAddEsp4(uint8_t*& p)
{
    p[0] = 0x83;
    p[1] = 0xC4;
    p[2] = 0x04;
    p += 3;
}

// ---------- size constants ----------

// Always-emitted PC write before each instruction:
//   mov word ptr [PC], local_pc   = 9 bytes
static constexpr size_t kPcWriteBytes = 9;

// Level-1 call sequence:
//   push imm32 + call rel32 + add esp, 4   = 13 bytes
static constexpr size_t kCallSeqBytes = 5 + 5 + 3;

// Level-2 inline sizes (worst case for any registered handler):
//   8-bit immediate load (LDA #imm / LDB #imm):
//     mov [reg], imm8       7
//     mov [cc[N]], imm8     7
//     mov [cc[Z]], imm8     7
//     mov [cc[V]], 0        7
//     add [cycles], 2       7
//     ----------------------- 35
//   16-bit immediate load (LDD/LDX/LDU #imm16):
//     mov word [reg], imm16 9
//     mov [cc[N]], imm8     7
//     mov [cc[Z]], imm8     7
//     mov [cc[V]], 0        7
//     add [cycles], 3       7
//     ----------------------- 37
// Use 40 as a safe per-instruction upper bound including the PC write.
static constexpr size_t kMaxBytesPerInsn = kPcWriteBytes + 40;
static constexpr size_t kEpilogueBytes = 1;  // ret

// ---------- inline emitters ----------

// LDA #imm and LDB #imm have identical structure - only the destination
// register byte address differs. The immediate is in insn.operand (low
// byte). N/Z/V are precomputable from the constant immediate.
static void EmitInlineLd8Imm(uint8_t*& p, const DecodedInst& insn,
                             uint8_t* reg_addr)
{
    const uint8_t imm = (uint8_t)(insn.operand & 0xFF);
    EmitMovMem8Imm8(p, reg_addr,        imm);
    EmitMovMem8Imm8(p, g_addrs.cc + 3,  (imm & 0x80) ? 1 : 0);   // cc[N=3]
    EmitMovMem8Imm8(p, g_addrs.cc + 2,  (imm == 0)   ? 1 : 0);   // cc[Z=2]
    EmitMovMem8Imm8(p, g_addrs.cc + 1,  0);                      // cc[V=1]
    EmitAddMem32Imm8(p, g_addrs.cycle_counter, 2);
}

// LDD/LDX/LDU #imm16. Same shape as the 8-bit version, just a 16-bit
// destination and a 16-bit operand. cc[N] tests the high bit of the
// 16-bit value.
static void EmitInlineLd16Imm(uint8_t*& p, const DecodedInst& insn,
                              uint16_t* reg_addr, int8_t cycles)
{
    const uint16_t imm = insn.operand;
    EmitMovMem16Imm16(p, reg_addr,      imm);
    EmitMovMem8Imm8(p, g_addrs.cc + 3,  (imm & 0x8000) ? 1 : 0); // cc[N]
    EmitMovMem8Imm8(p, g_addrs.cc + 2,  (imm == 0)     ? 1 : 0); // cc[Z]
    EmitMovMem8Imm8(p, g_addrs.cc + 1,  0);                      // cc[V]
    EmitAddMem32Imm8(p, g_addrs.cycle_counter, cycles);
}

// Returns true and emits if the handler matches a registered inlineable
// handler; returns false otherwise so the caller emits the call sequence.
static bool TryEmitInline(uint8_t*& p, const DecodedInst& insn)
{
    const InstHandler h = insn.handler;
    if (h == g_inlines.lda_m) { EmitInlineLd8Imm (p, insn, g_addrs.a); return true; }
    if (h == g_inlines.ldb_m) { EmitInlineLd8Imm (p, insn, g_addrs.b); return true; }
    if (h == g_inlines.ldd_m) { EmitInlineLd16Imm(p, insn, g_addrs.d, 3); return true; }
    if (h == g_inlines.ldx_m) { EmitInlineLd16Imm(p, insn, g_addrs.x, 3); return true; }
    if (h == g_inlines.ldu_m) { EmitInlineLd16Imm(p, insn, g_addrs.u, 3); return true; }
    return false;
}

// ---------- block emitter ----------

NativeEntry EmitBlock(const CachedBlock& slot)
{
    if (g_arena_base == nullptr || g_addrs.pc == nullptr)
        return nullptr;

    const size_t needed = (size_t)slot.num_insns * kMaxBytesPerInsn + kEpilogueBytes;
    if (g_arena_used + needed > kArenaSize)
    {
        ++g_emit_failures;
        return nullptr;
    }

    uint8_t* const entry = g_arena_base + g_arena_used;
    uint8_t* p = entry;

    uint16_t local_pc = slot.start_pc;

    for (int i = 0; i < (int)slot.num_insns; ++i)
    {
        const DecodedInst& insn = slot.insns[i];
        local_pc = (uint16_t)(local_pc + insn.length);

        // Always pre-set PC_REG to the post-instruction value before
        // emitting the body. Inlined ops don't read PC themselves, but
        // keeping the write per-insn matches the interpreter loop's
        // observable state at every dispatch boundary - we can elide
        // it for inlined sequences in level-3.
        EmitMovMem16Imm16(p, g_addrs.pc, local_pc);

        if (TryEmitInline(p, insn))
        {
            ++g_insns_inlined;
        }
        else
        {
            // Level-1 fallback: __cdecl call into the interpreter handler.
            EmitPushImm32(p, (uint32_t)(uintptr_t)&insn);
            EmitCallRel32(p, (const void*)insn.handler);
            EmitAddEsp4(p);
            ++g_insns_called;
        }
    }

    // ret
    *p++ = 0xC3;

    g_arena_used = (size_t)(p - g_arena_base);
    ++g_blocks_emitted;

    return reinterpret_cast<NativeEntry>(entry);
}

Stats GetStats()
{
    Stats s;
    s.arena_size     = kArenaSize;
    s.arena_used     = g_arena_used;
    s.blocks_emitted = g_blocks_emitted;
    s.emit_failures  = g_emit_failures;
    s.insns_called   = g_insns_called;
    s.insns_inlined  = g_insns_inlined;
    return s;
}

} // namespace BlockJit
