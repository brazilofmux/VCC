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

// 16 MB code arena. ROM pre-population emits ~1 MB of thunks per pass.
// We can absorb ~16 SoftReset cycles before the arena fills, after
// which EmitBlock starts returning nullptr and new blocks fall back to
// the interpreter (correct, just slower). Real arena lifecycle is a
// level-2 concern.
static constexpr size_t kArenaSize = 16 * 1024 * 1024;

static uint8_t*  g_arena_base   = nullptr;
static size_t    g_arena_used   = 0;
static uint16_t* g_pc_reg_addr  = nullptr;
static uint32_t  g_blocks_emitted = 0;
static uint32_t  g_emit_failures  = 0;

void Init(uint16_t* pc_reg_addr)
{
    g_pc_reg_addr = pc_reg_addr;

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
}

void Reset()
{
    g_arena_used = 0;
    g_blocks_emitted = 0;
    g_emit_failures = 0;
}

// Per-instruction emitted size:
//   mov word ptr [imm32], imm16   - 9 bytes (66 C7 05 [4] [2])
//   push imm32                    - 5 bytes (68 [4])
//   call rel32                    - 5 bytes (E8 [4])
//   add esp, 4                    - 3 bytes (83 C4 04)
// Plus 1 byte ret at the end of the block.
static constexpr size_t kBytesPerInsn = 9 + 5 + 5 + 3;
static constexpr size_t kEpilogueBytes = 1;

NativeEntry EmitBlock(const CachedBlock& slot)
{
    if (g_arena_base == nullptr || g_pc_reg_addr == nullptr)
        return nullptr;

    const size_t needed = (size_t)slot.num_insns * kBytesPerInsn + kEpilogueBytes;
    if (g_arena_used + needed > kArenaSize)
    {
        ++g_emit_failures;
        return nullptr;
    }

    uint8_t* const entry = g_arena_base + g_arena_used;
    uint8_t* p = entry;

    uint16_t local_pc = slot.start_pc;
    const uint32_t pc_addr_imm = (uint32_t)(uintptr_t)g_pc_reg_addr;

    for (int i = 0; i < (int)slot.num_insns; ++i)
    {
        const DecodedInst& insn = slot.insns[i];
        local_pc = (uint16_t)(local_pc + insn.length);

        // mov word ptr [pc_addr], local_pc
        //   66 C7 05 <addr32> <imm16>
        p[0] = 0x66;
        p[1] = 0xC7;
        p[2] = 0x05;
        std::memcpy(p + 3, &pc_addr_imm, 4);
        std::memcpy(p + 7, &local_pc,    2);
        p += 9;

        // push imm32 = &slot.insns[i]
        //   68 <imm32>
        const uint32_t insn_addr_imm = (uint32_t)(uintptr_t)&insn;
        p[0] = 0x68;
        std::memcpy(p + 1, &insn_addr_imm, 4);
        p += 5;

        // call rel32 = handler
        //   E8 <rel32>
        // rel32 is relative to the BYTE AFTER the call instruction.
        const uint8_t* const call_site_end = p + 5;
        const int32_t rel = (int32_t)(
            (intptr_t)(void*)insn.handler - (intptr_t)call_site_end);
        p[0] = 0xE8;
        std::memcpy(p + 1, &rel, 4);
        p += 5;

        // add esp, 4
        //   83 C4 04
        p[0] = 0x83;
        p[1] = 0xC4;
        p[2] = 0x04;
        p += 3;
    }

    // ret
    *p++ = 0xC3;

    g_arena_used = (size_t)(p - g_arena_base);
    ++g_blocks_emitted;

    return reinterpret_cast<NativeEntry>(entry);
}

Stats GetStats()
{
    return Stats{ kArenaSize, g_arena_used, g_blocks_emitted, g_emit_failures };
}

} // namespace BlockJit
