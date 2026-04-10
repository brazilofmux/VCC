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
static uint32_t           g_pc_writes_emitted = 0;
static uint32_t           g_pc_writes_skipped = 0;

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
    g_pc_writes_emitted = 0;
    g_pc_writes_skipped = 0;
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

// movzx eax, byte ptr [imm32]   - 7 bytes (0F B6 05 <addr32>)
static void EmitMovzxEaxMem8(uint8_t*& p, const void* addr)
{
    const uint32_t addr_imm = (uint32_t)(uintptr_t)addr;
    p[0] = 0x0F;
    p[1] = 0xB6;
    p[2] = 0x05;
    std::memcpy(p + 3, &addr_imm, 4);
    p += 7;
}

// add dword ptr [imm32], eax    - 6 bytes (01 05 <addr32>)
static void EmitAddMem32Eax(uint8_t*& p, const void* addr)
{
    const uint32_t addr_imm = (uint32_t)(uintptr_t)addr;
    p[0] = 0x01;
    p[1] = 0x05;
    std::memcpy(p + 2, &addr_imm, 4);
    p += 6;
}

// Add a runtime cycle byte (e.g., NatEmuCycles21) into CycleCounter.
// 13 bytes total (movzx eax, [addr]; add [cycle_counter], eax).
static void EmitAddCyclesRuntime(uint8_t*& p, const void* nat_cycles_addr)
{
    EmitMovzxEaxMem8(p, nat_cycles_addr);
    EmitAddMem32Eax(p, g_addrs.cycle_counter);
}

// ---------- size constants ----------

//   mov word ptr [PC], local_pc      = 9 bytes
static constexpr size_t kPcWriteBytes = 9;

// Level-1 call sequence (after PC write):
//   push imm32 + call rel32 + add esp, 4   = 13 bytes
static constexpr size_t kCallSeqBytes = 5 + 5 + 3;

// Worst-case inline body across all level-2 handlers. CLRA/CLRB are
// the largest at:
//   mov [reg], 0          7
//   mov [cc[C]], 0        7
//   mov [cc[V]], 0        7
//   mov [cc[Z]], 1        7
//   mov [cc[N]], 0        7
//   movzx eax, [cycles21] 7
//   add [cycles], eax     6
//   --------------------- 48
// PC-write skipping means inlined ops do NOT prepend a PC write, so
// the per-instruction budget for an inlined op is just the body size.
// For called ops the budget is kPcWriteBytes + kCallSeqBytes = 22.
// Use 56 as a safe upper bound that covers either case plus headroom.
static constexpr size_t kMaxBytesPerInsn = 56;

// Block epilogue: 1 byte ret + up to 9 bytes for the final PC flush
// when the last instruction in the block was inlined.
static constexpr size_t kEpilogueBytes = 1 + kPcWriteBytes;

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

// LDD/LDX/LDU #imm16: 16-bit destination, constant cycle cost. cc[N]
// tests the high bit of the 16-bit value.
static void EmitInlineLd16ImmConstCycles(uint8_t*& p, const DecodedInst& insn,
                                         uint16_t* reg_addr, int8_t cycles)
{
    const uint16_t imm = insn.operand;
    EmitMovMem16Imm16(p, reg_addr,      imm);
    EmitMovMem8Imm8(p, g_addrs.cc + 3,  (imm & 0x8000) ? 1 : 0); // cc[N]
    EmitMovMem8Imm8(p, g_addrs.cc + 2,  (imm == 0)     ? 1 : 0); // cc[Z]
    EmitMovMem8Imm8(p, g_addrs.cc + 1,  0);                      // cc[V]
    EmitAddMem32Imm8(p, g_addrs.cycle_counter, cycles);
}

// LDY/LDS #imm16: same as the const-cycles 16-bit version, but the
// cycle cost lives in a runtime byte (NatEmuCycles54) that flips when
// the CPU mode changes. We read it via movzx + add.
static void EmitInlineLd16ImmRuntimeCycles(uint8_t*& p, const DecodedInst& insn,
                                           uint16_t* reg_addr,
                                           const void* nat_cycles_addr)
{
    const uint16_t imm = insn.operand;
    EmitMovMem16Imm16(p, reg_addr,      imm);
    EmitMovMem8Imm8(p, g_addrs.cc + 3,  (imm & 0x8000) ? 1 : 0);
    EmitMovMem8Imm8(p, g_addrs.cc + 2,  (imm == 0)     ? 1 : 0);
    EmitMovMem8Imm8(p, g_addrs.cc + 1,  0);
    EmitAddCyclesRuntime(p, nat_cycles_addr);
}

// CLRA/CLRB: store 0 to the target register and set CC to a fixed
// pattern (C=0, V=0, Z=1, N=0). Touches cc[C] which the load
// emitters do NOT, so this is its own emitter rather than reusing
// the load shape. Cycle cost is the runtime NatEmuCycles21 byte.
static void EmitInlineClr8(uint8_t*& p, uint8_t* reg_addr)
{
    EmitMovMem8Imm8(p, reg_addr,        0);
    EmitMovMem8Imm8(p, g_addrs.cc + 0,  0);  // C
    EmitMovMem8Imm8(p, g_addrs.cc + 1,  0);  // V
    EmitMovMem8Imm8(p, g_addrs.cc + 2,  1);  // Z = 1
    EmitMovMem8Imm8(p, g_addrs.cc + 3,  0);  // N
    EmitAddCyclesRuntime(p, g_addrs.nat_cycles_21);
}

// Returns true and emits if the handler matches a registered inlineable
// handler; returns false otherwise so the caller emits the call sequence.
static bool TryEmitInline(uint8_t*& p, const DecodedInst& insn)
{
    const InstHandler h = insn.handler;
    if (h == g_inlines.lda_m)  { EmitInlineLd8Imm             (p, insn, g_addrs.a);                          return true; }
    if (h == g_inlines.ldb_m)  { EmitInlineLd8Imm             (p, insn, g_addrs.b);                          return true; }
    if (h == g_inlines.ldd_m)  { EmitInlineLd16ImmConstCycles (p, insn, g_addrs.d, 3);                       return true; }
    if (h == g_inlines.ldx_m)  { EmitInlineLd16ImmConstCycles (p, insn, g_addrs.x, 3);                       return true; }
    if (h == g_inlines.ldu_m)  { EmitInlineLd16ImmConstCycles (p, insn, g_addrs.u, 3);                       return true; }
    if (h == g_inlines.lds_i)  { EmitInlineLd16ImmConstCycles (p, insn, g_addrs.s, 4);                       return true; }
    if (h == g_inlines.ldy_m)  { EmitInlineLd16ImmRuntimeCycles(p, insn, g_addrs.y, g_addrs.nat_cycles_54);  return true; }
    if (h == g_inlines.clra_i) { EmitInlineClr8               (p, g_addrs.a);                                return true; }
    if (h == g_inlines.clrb_i) { EmitInlineClr8               (p, g_addrs.b);                                return true; }
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

    // PC-skipping state. The dispatcher arranges PC_REG == slot.start_pc
    // before calling native_entry, so PC matches the "current" position
    // until we either run a called handler (which needs the post-insn
    // PC pre-set) or finish the block (which needs PC pointing past
    // the last instruction so the outer loop's next-block lookup
    // works). For sequences of inlined ops we just track the running
    // local_pc and skip the writes; the next call OR the block tail
    // flushes whatever's pending.
    uint16_t local_pc = slot.start_pc;
    bool     pc_dirty = false;

    for (int i = 0; i < (int)slot.num_insns; ++i)
    {
        const DecodedInst& insn = slot.insns[i];
        local_pc = (uint16_t)(local_pc + insn.length);

        if (TryEmitInline(p, insn))
        {
            // Inlined ops don't read PC, so skip the per-insn PC
            // write. Just remember PC is now stale relative to local_pc.
            pc_dirty = true;
            ++g_pc_writes_skipped;
            ++g_insns_inlined;
        }
        else
        {
            // Called handler: must pre-set PC to local_pc (the post-
            // this-instruction value) so the handler sees the same
            // state the interpreter loop would have set up.
            EmitMovMem16Imm16(p, g_addrs.pc, local_pc);
            ++g_pc_writes_emitted;
            pc_dirty = false;

            EmitPushImm32(p, (uint32_t)(uintptr_t)&insn);
            EmitCallRel32(p, (const void*)insn.handler);
            EmitAddEsp4(p);
            ++g_insns_called;
        }
    }

    // If the block ended on an inlined op, flush PC so the outer
    // dispatcher's "look up next block at PC_REG" sees the right
    // address. Terminator-ending blocks always end on a called
    // handler (we don't inline branches/jumps/returns), so this
    // flush only fires for blocks that hit MAX_BLOCK_INSNS with
    // an inlined non-terminator at the end.
    if (pc_dirty)
    {
        EmitMovMem16Imm16(p, g_addrs.pc, local_pc);
        ++g_pc_writes_emitted;
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
    s.arena_size        = kArenaSize;
    s.arena_used        = g_arena_used;
    s.blocks_emitted    = g_blocks_emitted;
    s.emit_failures     = g_emit_failures;
    s.insns_called      = g_insns_called;
    s.insns_inlined     = g_insns_inlined;
    s.pc_writes_emitted = g_pc_writes_emitted;
    s.pc_writes_skipped = g_pc_writes_skipped;
    return s;
}

} // namespace BlockJit
