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

// mov al, byte ptr [imm32]      - 5 bytes (A0 <addr32>). Short-form
// load; only al has this one-byte-opcode encoding.
static void EmitMovAlMem(uint8_t*& p, const void* addr)
{
    const uint32_t addr_imm = (uint32_t)(uintptr_t)addr;
    p[0] = 0xA0;
    std::memcpy(p + 1, &addr_imm, 4);
    p += 5;
}

// mov byte ptr [imm32], al      - 5 bytes (A2 <addr32>). Short-form
// store paired with EmitMovAlMem above.
static void EmitMovMemAl(uint8_t*& p, const void* addr)
{
    const uint32_t addr_imm = (uint32_t)(uintptr_t)addr;
    p[0] = 0xA2;
    std::memcpy(p + 1, &addr_imm, 4);
    p += 5;
}

// movzx eax, word ptr [imm32]   - 7 bytes (0F B7 05 <addr32>). Used
// to load dp.Reg into eax for the DP effective-address calculation.
static void EmitMovzxEaxMem16(uint8_t*& p, const void* addr)
{
    const uint32_t addr_imm = (uint32_t)(uintptr_t)addr;
    p[0] = 0x0F;
    p[1] = 0xB7;
    p[2] = 0x05;
    std::memcpy(p + 3, &addr_imm, 4);
    p += 7;
}

// movzx edx, word ptr [imm32]   - 7 bytes (0F B7 15 <addr32>). Used
// when eax is already holding a separate value (STA_D holds A_REG in
// eax while computing the effective address in edx).
static void EmitMovzxEdxMem16(uint8_t*& p, const void* addr)
{
    const uint32_t addr_imm = (uint32_t)(uintptr_t)addr;
    p[0] = 0x0F;
    p[1] = 0xB7;
    p[2] = 0x15;
    std::memcpy(p + 3, &addr_imm, 4);
    p += 7;
}

// or al, imm8                   - 2 bytes (0C <imm8>). Short-form OR
// used to fold the DP offset into the low byte of the EA; correct
// because dp.Reg's low byte is guaranteed zero.
static void EmitOrAlImm8(uint8_t*& p, uint8_t imm)
{
    p[0] = 0x0C;
    p[1] = imm;
    p += 2;
}

// or dl, imm8                   - 3 bytes (80 CA <imm8>). No short
// form exists for dl; we pay an extra byte here to keep A_REG in al
// for STA_D.
static void EmitOrDlImm8(uint8_t*& p, uint8_t imm)
{
    p[0] = 0x80;
    p[1] = 0xCA;
    p[2] = imm;
    p += 3;
}

// push eax                      - 1 byte (50)
static void EmitPushEax(uint8_t*& p)
{
    p[0] = 0x50;
    p += 1;
}

// push edx                      - 1 byte (52)
static void EmitPushEdx(uint8_t*& p)
{
    p[0] = 0x52;
    p += 1;
}

// test al, al                   - 2 bytes (84 C0). Sets ZF and SF
// from the low byte only — exactly what the 6309 N/Z flags need.
static void EmitTestAlAl(uint8_t*& p)
{
    p[0] = 0x84;
    p[1] = 0xC0;
    p += 2;
}

// sete byte ptr [imm32]         - 7 bytes (0F 94 05 <addr32>). Stores
// 1 when ZF=1, else 0. Pairs naturally with the cc[Z] byte.
static void EmitSeteMem(uint8_t*& p, const void* addr)
{
    const uint32_t addr_imm = (uint32_t)(uintptr_t)addr;
    p[0] = 0x0F;
    p[1] = 0x94;
    p[2] = 0x05;
    std::memcpy(p + 3, &addr_imm, 4);
    p += 7;
}

// sets byte ptr [imm32]         - 7 bytes (0F 98 05 <addr32>). Stores
// 1 when SF=1 (high bit of the tested byte), else 0. cc[N].
static void EmitSetsMem(uint8_t*& p, const void* addr)
{
    const uint32_t addr_imm = (uint32_t)(uintptr_t)addr;
    p[0] = 0x0F;
    p[1] = 0x98;
    p[2] = 0x05;
    std::memcpy(p + 3, &addr_imm, 4);
    p += 7;
}

// add esp, imm8 (sign-extended) - 3 bytes (83 C4 <imm8>). Used to
// clean up __cdecl stack frames of 4 or 8 bytes.
static void EmitAddEspImm8(uint8_t*& p, uint8_t imm)
{
    p[0] = 0x83;
    p[1] = 0xC4;
    p[2] = imm;
    p += 3;
}

// cmp byte ptr [imm32], imm8    - 7 bytes (80 3D <addr32> <imm8>).
// Reads the register byte and sets SF/ZF so TSTA can set cc[N]/cc[Z]
// from a single cmp instead of loading into a register first.
static void EmitCmpMem8Imm8(uint8_t*& p, const void* addr, uint8_t imm)
{
    const uint32_t addr_imm = (uint32_t)(uintptr_t)addr;
    p[0] = 0x80;
    p[1] = 0x3D;                 // /7 group, mod=00, r/m=101 (disp32)
    std::memcpy(p + 2, &addr_imm, 4);
    p[6] = imm;
    p += 7;
}

// xor byte ptr [imm32], imm8    - 7 bytes (80 35 <addr32> <imm8>).
// Used for COMA/COMB: xor with 0xFF is the ones-complement and sets
// SF/ZF/PF cleanly (CF/OF are always cleared by xor).
static void EmitXorMem8Imm8(uint8_t*& p, const void* addr, uint8_t imm)
{
    const uint32_t addr_imm = (uint32_t)(uintptr_t)addr;
    p[0] = 0x80;
    p[1] = 0x35;                 // /6 group
    std::memcpy(p + 2, &addr_imm, 4);
    p[6] = imm;
    p += 7;
}

// inc byte ptr [imm32]          - 6 bytes (FE 05 <addr32>). Sets
// SF/ZF/OF from the result; CF is unchanged - matches 6309 INC which
// leaves carry alone and sets V only on 0x7F->0x80 overflow.
static void EmitIncMem8(uint8_t*& p, const void* addr)
{
    const uint32_t addr_imm = (uint32_t)(uintptr_t)addr;
    p[0] = 0xFE;
    p[1] = 0x05;                 // /0
    std::memcpy(p + 2, &addr_imm, 4);
    p += 6;
}

// dec byte ptr [imm32]          - 6 bytes (FE 0D <addr32>). Same
// flag story as inc: SF/ZF/OF updated, CF preserved. For 6309 DEC,
// the OF flag is 1 exactly when the result is 0x7F, matching
// cc[V] = (A_REG == 0x7F).
static void EmitDecMem8(uint8_t*& p, const void* addr)
{
    const uint32_t addr_imm = (uint32_t)(uintptr_t)addr;
    p[0] = 0xFE;
    p[1] = 0x0D;                 // /1
    std::memcpy(p + 2, &addr_imm, 4);
    p += 6;
}

// neg byte ptr [imm32]          - 6 bytes (F6 1D <addr32>). Sets
// SF/ZF, CF = (input != 0), OF = (input == 0x80) — which exactly
// matches 6309 NEG's cc[C] and cc[V] rules.
static void EmitNegMem8(uint8_t*& p, const void* addr)
{
    const uint32_t addr_imm = (uint32_t)(uintptr_t)addr;
    p[0] = 0xF6;
    p[1] = 0x1D;                 // /3
    std::memcpy(p + 2, &addr_imm, 4);
    p += 6;
}

// shl byte ptr [imm32], 1       - 6 bytes (D0 25 <addr32>). CF gets
// the shifted-out MSB, OF is (CF != new MSB) which is the 6309 V
// formula (old_bit7 XOR old_bit6).
static void EmitShlMem8By1(uint8_t*& p, const void* addr)
{
    const uint32_t addr_imm = (uint32_t)(uintptr_t)addr;
    p[0] = 0xD0;
    p[1] = 0x25;                 // /4
    std::memcpy(p + 2, &addr_imm, 4);
    p += 6;
}

// shr byte ptr [imm32], 1       - 6 bytes (D0 2D <addr32>). Logical
// shift: CF = old LSB; SF always 0 in the result (top bit cleared);
// OF on a 1-bit shift = old MSB - we don't consume it for LSR.
static void EmitShrMem8By1(uint8_t*& p, const void* addr)
{
    const uint32_t addr_imm = (uint32_t)(uintptr_t)addr;
    p[0] = 0xD0;
    p[1] = 0x2D;                 // /5
    std::memcpy(p + 2, &addr_imm, 4);
    p += 6;
}

// sar byte ptr [imm32], 1       - 6 bytes (D0 3D <addr32>). Arith
// shift: CF = old LSB; the sign bit propagates so SF reflects the
// preserved sign (matches 6309 ASRA's cc[N] rule).
static void EmitSarMem8By1(uint8_t*& p, const void* addr)
{
    const uint32_t addr_imm = (uint32_t)(uintptr_t)addr;
    p[0] = 0xD0;
    p[1] = 0x3D;                 // /7
    std::memcpy(p + 2, &addr_imm, 4);
    p += 6;
}

// seto byte ptr [imm32]         - 7 bytes (0F 90 05 <addr32>). Pairs
// with inc/dec/neg/shl to capture the 6309 V flag directly from x86
// OF without any arithmetic.
static void EmitSetoMem(uint8_t*& p, const void* addr)
{
    const uint32_t addr_imm = (uint32_t)(uintptr_t)addr;
    p[0] = 0x0F;
    p[1] = 0x90;
    p[2] = 0x05;
    std::memcpy(p + 3, &addr_imm, 4);
    p += 7;
}

// setc byte ptr [imm32]         - 7 bytes (0F 92 05 <addr32>). Pairs
// with neg/shl/shr/sar to write cc[C] from x86 CF.
static void EmitSetcMem(uint8_t*& p, const void* addr)
{
    const uint32_t addr_imm = (uint32_t)(uintptr_t)addr;
    p[0] = 0x0F;
    p[1] = 0x92;
    p[2] = 0x05;
    std::memcpy(p + 3, &addr_imm, 4);
    p += 7;
}

// add word ptr [imm32], ax      - 7 bytes (66 01 05 <addr32>). Used
// by ABX to fold B_REG (widened into ax via movzx) into X_REG.
static void EmitAddMem16Ax(uint8_t*& p, const void* addr)
{
    const uint32_t addr_imm = (uint32_t)(uintptr_t)addr;
    p[0] = 0x66;                 // operand size prefix
    p[1] = 0x01;                 // add r/m16, r16
    p[2] = 0x05;
    std::memcpy(p + 3, &addr_imm, 4);
    p += 7;
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

// Worst-case inline body across all level-2 handlers. STA_D is the
// largest at:
//   movzx eax, [A_REG]     7
//   test al, al            2
//   sete [cc[Z]]           7
//   sets [cc[N]]           7
//   mov [cc[V]], 0         7
//   movzx edx, [dp.Reg]    7
//   or dl, offset_imm8     3
//   push edx               1
//   push eax               1
//   call MemWrite8         5
//   add esp, 8             3
//   movzx eax, [cycles43]  7
//   add [cycles], eax      6
//   --------------------- 63
// PC-write skipping means inlined ops do NOT prepend a PC write, so
// the per-instruction budget for an inlined op is just the body size.
// For called ops the budget is kPcWriteBytes + kCallSeqBytes = 22.
// Use 64 as a safe upper bound that covers either case plus one
// byte of headroom.
static constexpr size_t kMaxBytesPerInsn = 64;

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

// After an 8-bit memory load or store, update cc[Z]/cc[N]/cc[V] from
// the low byte currently in al. The caller is responsible for
// ensuring al holds the value the 6309 handler would flag-test
// (the byte just read, or A_REG/B_REG for stores).
static void EmitFlags8FromAl(uint8_t*& p)
{
    EmitTestAlAl(p);                         // test al, al
    EmitSeteMem(p, g_addrs.cc + 2);          // cc[Z]
    EmitSetsMem(p, g_addrs.cc + 3);          // cc[N]
    EmitMovMem8Imm8(p, g_addrs.cc + 1, 0);   // cc[V] = 0
}

// LDA <dp / LDB <dp. Compute effective address as dp.Reg | offset,
// call MemRead8, store the byte to A/B, then update Z/N/V from the
// returned byte. Cycles come from the runtime NatEmuCycles43 byte.
static void EmitInlineLd8Dp(uint8_t*& p, const DecodedInst& insn,
                            uint8_t* reg_addr)
{
    const uint8_t offset = (uint8_t)(insn.operand & 0xFF);

    // eax = dp.Reg | offset, then push and call MemRead8.
    EmitMovzxEaxMem16(p, g_addrs.dp);        // movzx eax, word [dp]
    if (offset != 0)
        EmitOrAlImm8(p, offset);             // or al, offset_imm8
    EmitPushEax(p);                          // push eax
    EmitCallRel32(p, (const void*)g_addrs.mem_read8);
    EmitAddEsp4(p);                          // add esp, 4

    // al now holds the memory byte. Store it and flag-test.
    EmitMovMemAl(p, reg_addr);               // mov [reg], al
    EmitFlags8FromAl(p);
    EmitAddCyclesRuntime(p, g_addrs.nat_cycles_43);
}

// LDA ext / LDB ext. The 16-bit effective address is known at emit
// time and gets baked as a push imm32, so we skip the dp math and
// just call. Uses NatEmuCycles54.
static void EmitInlineLd8Ext(uint8_t*& p, const DecodedInst& insn,
                             uint8_t* reg_addr)
{
    const uint16_t addr = insn.operand;

    EmitPushImm32(p, addr);                  // push imm32 (zero-extended addr)
    EmitCallRel32(p, (const void*)g_addrs.mem_read8);
    EmitAddEsp4(p);

    EmitMovMemAl(p, reg_addr);
    EmitFlags8FromAl(p);
    EmitAddCyclesRuntime(p, g_addrs.nat_cycles_54);
}

// STA <dp / STB <dp. Load A/B into al (for the call AND the flag
// test), compute effective address in edx, push both, call
// MemWrite8, update flags from al, accumulate runtime cycles.
// The flag update uses the value in A/B, not anything from memory,
// matching the interpreter handler.
static void EmitInlineSt8Dp(uint8_t*& p, const DecodedInst& insn,
                            uint8_t* reg_addr)
{
    const uint8_t offset = (uint8_t)(insn.operand & 0xFF);

    // Load data into eax zero-extended. movzx (7 bytes) is two
    // bytes longer than mov al (5 bytes) but zero-extends for the
    // push imm arg promotion the C ABI requires.
    EmitMovzxEaxMem8(p, reg_addr);           // movzx eax, byte [reg]

    // Flag-test while al still has the value.
    EmitFlags8FromAl(p);

    // Compute address in edx = dp.Reg | offset.
    EmitMovzxEdxMem16(p, g_addrs.dp);
    if (offset != 0)
        EmitOrDlImm8(p, offset);

    // cdecl: push args right-to-left -> push address then data.
    EmitPushEdx(p);                          // push edx (address, arg1)
    EmitPushEax(p);                          // push eax (data,   arg0)
    EmitCallRel32(p, (const void*)g_addrs.mem_write8);
    EmitAddEspImm8(p, 8);                    // add esp, 8

    EmitAddCyclesRuntime(p, g_addrs.nat_cycles_43);
}

// STA ext / STB ext. Same shape as STA_D but the address is a baked
// push imm32, not a runtime dp.Reg|offset computation. Uses
// NatEmuCycles54.
static void EmitInlineSt8Ext(uint8_t*& p, const DecodedInst& insn,
                             uint8_t* reg_addr)
{
    const uint16_t addr = insn.operand;

    EmitMovzxEaxMem8(p, reg_addr);           // movzx eax, byte [reg]
    EmitFlags8FromAl(p);

    EmitPushImm32(p, addr);                  // push address imm
    EmitPushEax(p);                          // push data
    EmitCallRel32(p, (const void*)g_addrs.mem_write8);
    EmitAddEspImm8(p, 8);

    EmitAddCyclesRuntime(p, g_addrs.nat_cycles_54);
}

// TSTA / TSTB. cmp m8, 0 sets SF/ZF without modifying the register;
// 6309 semantics clear cc[V] and leave cc[C] alone, so we write Z/N
// from the flags and V as a constant. Cycle cost is NatEmuCycles21.
static void EmitInlineTst8(uint8_t*& p, uint8_t* reg_addr)
{
    EmitCmpMem8Imm8(p, reg_addr, 0);
    EmitSeteMem(p, g_addrs.cc + 2);          // cc[Z]
    EmitSetsMem(p, g_addrs.cc + 3);          // cc[N]
    EmitMovMem8Imm8(p, g_addrs.cc + 1, 0);   // cc[V] = 0
    EmitAddCyclesRuntime(p, g_addrs.nat_cycles_21);
}

// INCA / INCB. x86 inc m8 sets SF/ZF from the result and OF exactly
// when the value wraps from 0x7F to 0x80 - which is the 6309 V rule
// verbatim. CF is left alone, matching "C untouched".
static void EmitInlineInc8(uint8_t*& p, uint8_t* reg_addr)
{
    EmitIncMem8(p, reg_addr);
    EmitSeteMem(p, g_addrs.cc + 2);          // cc[Z]
    EmitSetsMem(p, g_addrs.cc + 3);          // cc[N]
    EmitSetoMem(p, g_addrs.cc + 1);          // cc[V]
    EmitAddCyclesRuntime(p, g_addrs.nat_cycles_21);
}

// DECA / DECB. Same shape as INC8 with dec m8 - OF is 1 exactly on
// the 0x80 -> 0x7F wrap, which is the 6309 V rule for DEC.
static void EmitInlineDec8(uint8_t*& p, uint8_t* reg_addr)
{
    EmitDecMem8(p, reg_addr);
    EmitSeteMem(p, g_addrs.cc + 2);          // cc[Z]
    EmitSetsMem(p, g_addrs.cc + 3);          // cc[N]
    EmitSetoMem(p, g_addrs.cc + 1);          // cc[V]
    EmitAddCyclesRuntime(p, g_addrs.nat_cycles_21);
}

// COMA / COMB. Ones-complement via xor m8, 0xFF. xor sets SF/ZF/PF
// and clears CF/OF, so we pick up Z/N from the flags and hard-wire
// cc[V]=0 and cc[C]=1 per the 6309 spec.
static void EmitInlineCom8(uint8_t*& p, uint8_t* reg_addr)
{
    EmitXorMem8Imm8(p, reg_addr, 0xFF);
    EmitSeteMem(p, g_addrs.cc + 2);          // cc[Z]
    EmitSetsMem(p, g_addrs.cc + 3);          // cc[N]
    EmitMovMem8Imm8(p, g_addrs.cc + 1, 0);   // cc[V] = 0
    EmitMovMem8Imm8(p, g_addrs.cc + 0, 1);   // cc[C] = 1 (per spec)
    EmitAddCyclesRuntime(p, g_addrs.nat_cycles_21);
}

// NEGA / NEGB. x86 neg m8 sets: CF = (input != 0); OF = (input ==
// 0x80); SF/ZF from the result. Those are exactly the four 6309 NEG
// flag rules, so we can fire four setcc writes straight from the
// same flag state.
static void EmitInlineNeg8(uint8_t*& p, uint8_t* reg_addr)
{
    EmitNegMem8(p, reg_addr);
    EmitSeteMem(p, g_addrs.cc + 2);          // cc[Z]
    EmitSetsMem(p, g_addrs.cc + 3);          // cc[N]
    EmitSetoMem(p, g_addrs.cc + 1);          // cc[V]
    EmitSetcMem(p, g_addrs.cc + 0);          // cc[C]
    EmitAddCyclesRuntime(p, g_addrs.nat_cycles_21);
}

// LSRA. Logical shift right by 1: CF gets the shifted-out LSB (the
// 6309 C rule), ZF reflects the new value (cc[Z]), and the top bit
// of the result is always 0 so we hard-wire cc[N] to 0 rather than
// copy SF. V is left untouched per the 6309 spec, so no write.
static void EmitInlineLsr8(uint8_t*& p, uint8_t* reg_addr)
{
    EmitShrMem8By1(p, reg_addr);
    EmitSetcMem(p, g_addrs.cc + 0);          // cc[C]
    EmitSeteMem(p, g_addrs.cc + 2);          // cc[Z]
    EmitMovMem8Imm8(p, g_addrs.cc + 3, 0);   // cc[N] = 0
    EmitAddCyclesRuntime(p, g_addrs.nat_cycles_21);
}

// ASRA. Arithmetic shift right: sar m8, 1 - sign bit replicates so
// SF gives us the correct cc[N], ZF cc[Z], CF cc[C]. V unchanged.
static void EmitInlineAsr8(uint8_t*& p, uint8_t* reg_addr)
{
    EmitSarMem8By1(p, reg_addr);
    EmitSetcMem(p, g_addrs.cc + 0);          // cc[C]
    EmitSeteMem(p, g_addrs.cc + 2);          // cc[Z]
    EmitSetsMem(p, g_addrs.cc + 3);          // cc[N]
    EmitAddCyclesRuntime(p, g_addrs.nat_cycles_21);
}

// ASLA (also LSLA - same opcode, aliased mnemonic). shl m8, 1:
// CF = old MSB (cc[C]); OF = new-MSB != CF = old_bit6 XOR old_bit7
// which matches the 6309 V formula (cc[C] ^ ((A_REG & 0x40) >> 6));
// SF/ZF from the result.
static void EmitInlineAsl8(uint8_t*& p, uint8_t* reg_addr)
{
    EmitShlMem8By1(p, reg_addr);
    EmitSetcMem(p, g_addrs.cc + 0);          // cc[C]
    EmitSetoMem(p, g_addrs.cc + 1);          // cc[V]
    EmitSeteMem(p, g_addrs.cc + 2);          // cc[Z]
    EmitSetsMem(p, g_addrs.cc + 3);          // cc[N]
    EmitAddCyclesRuntime(p, g_addrs.nat_cycles_21);
}

// ABX. X_REG += B_REG. Touches no flags in 6309 semantics. We widen
// B_REG through movzx (upper bits of eax become zero), then add its
// low word into X_REG via an operand-size-prefixed add m16, r16.
// Cycle cost is NatEmuCycles31, not the usual 21.
static void EmitInlineAbx(uint8_t*& p)
{
    EmitMovzxEaxMem8(p, g_addrs.b);
    EmitAddMem16Ax(p, g_addrs.x);
    EmitAddCyclesRuntime(p, g_addrs.nat_cycles_31);
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
    if (h == g_inlines.lda_d)  { EmitInlineLd8Dp              (p, insn, g_addrs.a);                          return true; }
    if (h == g_inlines.ldb_d)  { EmitInlineLd8Dp              (p, insn, g_addrs.b);                          return true; }
    if (h == g_inlines.sta_d)  { EmitInlineSt8Dp              (p, insn, g_addrs.a);                          return true; }
    if (h == g_inlines.stb_d)  { EmitInlineSt8Dp              (p, insn, g_addrs.b);                          return true; }
    if (h == g_inlines.lda_e)  { EmitInlineLd8Ext             (p, insn, g_addrs.a);                          return true; }
    if (h == g_inlines.ldb_e)  { EmitInlineLd8Ext             (p, insn, g_addrs.b);                          return true; }
    if (h == g_inlines.sta_e)  { EmitInlineSt8Ext             (p, insn, g_addrs.a);                          return true; }
    if (h == g_inlines.stb_e)  { EmitInlineSt8Ext             (p, insn, g_addrs.b);                          return true; }
    if (h == g_inlines.tsta_i) { EmitInlineTst8               (p, g_addrs.a);                                return true; }
    if (h == g_inlines.tstb_i) { EmitInlineTst8               (p, g_addrs.b);                                return true; }
    if (h == g_inlines.inca_i) { EmitInlineInc8               (p, g_addrs.a);                                return true; }
    if (h == g_inlines.incb_i) { EmitInlineInc8               (p, g_addrs.b);                                return true; }
    if (h == g_inlines.deca_i) { EmitInlineDec8               (p, g_addrs.a);                                return true; }
    if (h == g_inlines.decb_i) { EmitInlineDec8               (p, g_addrs.b);                                return true; }
    if (h == g_inlines.coma_i) { EmitInlineCom8               (p, g_addrs.a);                                return true; }
    if (h == g_inlines.comb_i) { EmitInlineCom8               (p, g_addrs.b);                                return true; }
    if (h == g_inlines.nega_i) { EmitInlineNeg8               (p, g_addrs.a);                                return true; }
    if (h == g_inlines.negb_i) { EmitInlineNeg8               (p, g_addrs.b);                                return true; }
    if (h == g_inlines.lsra_i) { EmitInlineLsr8               (p, g_addrs.a);                                return true; }
    if (h == g_inlines.asra_i) { EmitInlineAsr8               (p, g_addrs.a);                                return true; }
    if (h == g_inlines.asla_i) { EmitInlineAsl8               (p, g_addrs.a);                                return true; }
    if (h == g_inlines.abx_i)  { EmitInlineAbx                (p);                                           return true; }
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
