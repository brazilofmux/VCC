#pragma once
/*
Copyright 2025 by the VCC Project Contributors.
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

// EAModes: Pre-decoded effective address calculation modes for indexed
// addressing on the 6809/6309.
//
// At decode time, the indexed postbyte is resolved into an EA mode index
// and register index. At runtime, a table of small EA calculator functions
// is indexed by the mode — no switch statement, no postbyte interpretation.
//
// The ea_info byte in DecodedInst packs both:
//   bits 0-5: EA mode (0-63)
//   bits 6-7: register index (0-3 → maps to xfreg16[reg+1])

#include <cstdint>

// EA mode indices. Each corresponds to one small EA calculator function.
enum EAMode : uint8_t
{
    // --- Direct (non-indirect) modes ---
    EA_POST_INC1 = 0,   // ea = *reg; (*reg)++
    EA_POST_INC2,        // ea = *reg; (*reg) += 2
    EA_PRE_DEC1,         // (*reg)--; ea = *reg
    EA_PRE_DEC2,         // (*reg) -= 2; ea = *reg
    EA_NO_OFFSET,        // ea = *reg
    EA_OFFSET_B,         // ea = *reg + (signed char)B
    EA_OFFSET_A,         // ea = *reg + (signed char)A
    EA_OFFSET_E,         // ea = *reg + (signed char)E        (6309)
    EA_OFFSET_8,         // ea = *reg + (signed char)operand
    EA_OFFSET_16,        // ea = *reg + operand
    EA_OFFSET_F,         // ea = *reg + (signed char)F        (6309)
    EA_OFFSET_D,         // ea = *reg + D
    EA_PC_8,             // ea = operand (pre-computed at decode time)
    EA_PC_16,            // ea = operand (pre-computed at decode time)
    EA_OFFSET_W,         // ea = *reg + W                     (6309)
    EA_5BIT,             // ea = *reg + (signed short)operand (sign-extended 5-bit)
    EA_W_NO_OFFSET,      // ea = W                            (6309)
    EA_W_OFFSET_16,      // ea = W + operand                  (6309)
    EA_W_POST_INC,       // ea = W; W += 2                    (6309)
    EA_W_PRE_DEC,        // W -= 2; ea = W                    (6309)

    // --- Indirect modes (add MemRead16 dereference) ---
    EA_I_POST_INC2,      // ea = [*reg]; (*reg) += 2
    EA_I_ILLEGAL,        // illegal indexed mode (case 18)
    EA_I_PRE_DEC2,       // (*reg) -= 2; ea = [*reg]
    EA_I_NO_OFFSET,      // ea = [*reg]
    EA_I_OFFSET_B,       // ea = [*reg + (signed char)B]
    EA_I_OFFSET_A,       // ea = [*reg + (signed char)A]
    EA_I_OFFSET_E,       // ea = [*reg + (signed char)E]      (6309)
    EA_I_OFFSET_8,       // ea = [*reg + (signed char)operand]
    EA_I_OFFSET_16,      // ea = [*reg + operand]
    EA_I_OFFSET_F,       // ea = [*reg + (signed char)F]      (6309)
    EA_I_OFFSET_D,       // ea = [*reg + D]
    EA_I_PC_8,           // ea = [operand] (pre-computed addr)
    EA_I_PC_16,          // ea = [operand] (pre-computed addr)
    EA_I_OFFSET_W,       // ea = [*reg + W]                   (6309)
    EA_I_W_NO_OFFSET,    // ea = [W]                          (6309)
    EA_I_W_OFFSET_16,    // ea = [W + operand]                (6309)
    EA_I_W_POST_INC,     // ea = [W]; W += 2                  (6309)
    EA_I_W_PRE_DEC,      // W -= 2; ea = [W]                  (6309)
    EA_I_EXT,            // ea = [operand] (extended indirect)

    EA_MODE_COUNT
};

// Pack/unpack ea_info byte
#define EA_MODE(info)  ((EAMode)((info) & 0x3F))
#define EA_REG(info)   (((info) >> 6) & 3)
#define MAKE_EA_INFO(mode, reg) ((uint8_t)(((reg) << 6) | (mode)))

// EA calculator function type.
// reg: pointer to the index register (X, Y, U, or S via xfreg16[])
// operand: pre-decoded offset/address from DecodedInst
// Returns the effective address.
typedef unsigned short (*EACalcFunc)(unsigned short* reg, unsigned short operand);
