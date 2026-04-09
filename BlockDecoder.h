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

// BlockDecoder: Pre-decodes a recorded basic block into DecodedInst arrays.
//
// Given a block's start PC and instruction count, walks the instruction
// bytes in memory and fills in handler, operand, postbyte, and length
// for each instruction. This is called once when a block is first cached.
// Subsequent executions use the pre-decoded data directly.
//
// Page 2/3 instructions are stored with the Page_2/Page_3 handler and
// decoded length — the handler still does its own inner dispatch for now.
// As individual page 2/3 handlers are converted, the decoder can be
// extended to look through the prefix.

#include <cstdint>
#include "DecodedInst.h"
#include "tcc1014mmu.h"

// Forward declarations — these are defined in hd6309.cpp
extern InstHandler JmpVec1[256];
extern InstHandler JmpVec2[256];
extern InstHandler JmpVec3[256];

// Compute extra bytes consumed by an indexed addressing mode postbyte,
// beyond the postbyte itself. Returns 0, 1, or 2.
inline int IndexedExtraBytes(uint8_t postbyte)
{
    if (!(postbyte & 0x80))
        return 0;  // 5-bit signed offset, no extra bytes

    switch (postbyte & 0x1F)
    {
    case 8:  case 24:           // 8-bit offset / indirect 8-bit offset
    case 12: case 28:           // 8-bit PC relative / indirect
        return 1;

    case 9:  case 25:           // 16-bit offset / indirect 16-bit offset
    case 13: case 29:           // 16-bit PC relative / indirect
    case 31:                    // extended indirect [addr]
        return 2;

    case 15:                    // W-reg modes (6309)
    case 16:                    // indirect W-reg modes (6309)
        // Register field (bits 6-5) determines sub-mode:
        // reg=1 has a 16-bit offset, others have no extra bytes
        return ((postbyte >> 5) & 3) == 1 ? 2 : 0;

    default:
        return 0;               // register offsets, auto inc/dec, etc.
    }
}

// Static instruction length table for page 1 opcodes.
// 0 = illegal or special (page prefix). For indexed modes, this is the
// base length (opcode + any immediate byte before the postbyte + postbyte);
// IndexedExtraBytes() is added on top.
//
// Encoding: positive values are fixed lengths. Negative values (stored as
// uint8_t >= 0x80) indicate indexed mode: the base is (value & 0x7F),
// and the postbyte at offset (base-1) from the opcode determines extra bytes.
//
// We use a simple scheme: IDX_BASE(n) = 0x80 | n
#define IDX_BASE(n) (0x80 | (n))

static const uint8_t Page1InsLen[256] = {
// 0x00-0x0F: Direct page ops
// Most are 2 bytes. OIM(01),AIM(02),EIM(05),TIM(0B) are 3 bytes (imm+DP)
   2, 3, 3, 2, 2, 3, 2, 2,  2, 2, 2, 3, 2, 2, 2, 2,
// 0x10-0x1F: Misc
// 10=Page2(0), 11=Page3(0), 12=NOP(1), 13=SYNC(1), 14=SEXW(1), 15=HALT(1)
// 16=LBRA(3), 17=LBSR(3), 18=illegal(1), 19=DAA(1)
// 1A=ORCC(2), 1B=illegal(1), 1C=ANDCC(2), 1D=SEX(1), 1E=EXG(2), 1F=TFR(2)
   0, 0, 1, 1, 1, 1, 3, 3,  1, 1, 2, 1, 2, 1, 2, 2,
// 0x20-0x2F: Relative branches (all 2 bytes), 8D=BSR also 2 bytes
   2, 2, 2, 2, 2, 2, 2, 2,  2, 2, 2, 2, 2, 2, 2, 2,
// 0x30-0x3F: Misc
// 30-33=LEA indexed(2+idx), 34-37=PSH/PUL(2), 38=illegal(1)
// 39=RTS(1), 3A=ABX(1), 3B=RTI(1), 3C=CWAI(2), 3D=MUL(1), 3E=RESET(1), 3F=SWI(1)
   IDX_BASE(2), IDX_BASE(2), IDX_BASE(2), IDX_BASE(2),
   2, 2, 2, 2,
   1, 1, 1, 1, 2, 1, 1, 1,
// 0x40-0x4F: Inherent A ops (all 1 byte)
   1, 1, 1, 1, 1, 1, 1, 1,  1, 1, 1, 1, 1, 1, 1, 1,
// 0x50-0x5F: Inherent B ops (all 1 byte)
   1, 1, 1, 1, 1, 1, 1, 1,  1, 1, 1, 1, 1, 1, 1, 1,
// 0x60-0x6F: Indexed ops
// Most are IDX_BASE(2). OIM(61),AIM(62),EIM(65),TIM(6B) are IDX_BASE(3)
   IDX_BASE(2), IDX_BASE(3), IDX_BASE(3), IDX_BASE(2),
   IDX_BASE(2), IDX_BASE(3), IDX_BASE(2), IDX_BASE(2),
   IDX_BASE(2), IDX_BASE(2), IDX_BASE(2), IDX_BASE(3),
   IDX_BASE(2), IDX_BASE(2), IDX_BASE(2), IDX_BASE(2),
// 0x70-0x7F: Extended ops
// Most are 3 bytes. OIM(71),AIM(72),EIM(75),TIM(7B) are 4 bytes
   3, 4, 4, 3, 3, 4, 3, 3,  3, 3, 3, 4, 3, 3, 3, 3,
// 0x80-0x8F: Immediate ops
// Most are 2 (imm8). 83=SUBD(3), 8C=CMPX(3), 8D=BSR(2/rel), 8E=LDX(3)
// 87,8F = illegal
   2, 2, 2, 3, 2, 2, 2, 1,  2, 2, 2, 2, 3, 2, 3, 1,
// 0x90-0x9F: Direct ops (all 2 bytes)
   2, 2, 2, 2, 2, 2, 2, 2,  2, 2, 2, 2, 2, 2, 2, 2,
// 0xA0-0xAF: Indexed ops (all IDX_BASE(2))
   IDX_BASE(2), IDX_BASE(2), IDX_BASE(2), IDX_BASE(2),
   IDX_BASE(2), IDX_BASE(2), IDX_BASE(2), IDX_BASE(2),
   IDX_BASE(2), IDX_BASE(2), IDX_BASE(2), IDX_BASE(2),
   IDX_BASE(2), IDX_BASE(2), IDX_BASE(2), IDX_BASE(2),
// 0xB0-0xBF: Extended ops (all 3 bytes)
   3, 3, 3, 3, 3, 3, 3, 3,  3, 3, 3, 3, 3, 3, 3, 3,
// 0xC0-0xCF: Immediate ops
// Most are 2 (imm8). C3=ADDD(3), CC=LDD(3), CD=LDQ(5), CE=LDU(3)
// C7,CF = illegal
   2, 2, 2, 3, 2, 2, 2, 1,  2, 2, 2, 2, 3, 5, 3, 1,
// 0xD0-0xDF: Direct ops (all 2 bytes)
   2, 2, 2, 2, 2, 2, 2, 2,  2, 2, 2, 2, 2, 2, 2, 2,
// 0xE0-0xEF: Indexed ops (all IDX_BASE(2))
   IDX_BASE(2), IDX_BASE(2), IDX_BASE(2), IDX_BASE(2),
   IDX_BASE(2), IDX_BASE(2), IDX_BASE(2), IDX_BASE(2),
   IDX_BASE(2), IDX_BASE(2), IDX_BASE(2), IDX_BASE(2),
   IDX_BASE(2), IDX_BASE(2), IDX_BASE(2), IDX_BASE(2),
// 0xF0-0xFF: Extended ops (all 3 bytes)
   3, 3, 3, 3, 3, 3, 3, 3,  3, 3, 3, 3, 3, 3, 3, 3,
};

// Page 2 (0x10 prefix) instruction lengths. These are the length of the
// instruction AFTER the prefix byte (so total = 1 + this value).
// The structure mirrors page 1 but with different opcodes valid.
static const uint8_t Page2InsLen[256] = {
// 0x00-0x0F: all illegal (use 1 as safe default — the prefix already consumed 1)
   1, 1, 1, 1, 1, 1, 1, 1,  1, 1, 1, 1, 1, 1, 1, 1,
// 0x10-0x1F: all illegal
   1, 1, 1, 1, 1, 1, 1, 1,  1, 1, 1, 1, 1, 1, 1, 1,
// 0x20-0x2F: long relative branches (all 3 bytes: opcode + 16-bit offset)
   3, 3, 3, 3, 3, 3, 3, 3,  3, 3, 3, 3, 3, 3, 3, 3,
// 0x30-0x3F: 30=ADDR(2), 31=ADCR(2), 32=SUBR(2), 33=SBCR(2),
//            34=ANDR(2), 35=ORR(2), 36=EORR(2), 37=CMPR(2),
//            38=PSHSW(1), 39=PULSW(1), 3A=PSHUW(1), 3B=PULUW(1)
//            3C-3E=illegal, 3F=SWI2(1)
   2, 2, 2, 2, 2, 2, 2, 2,  1, 1, 1, 1, 1, 1, 1, 1,
// 0x40-0x5F: 6309 inherent ops (NEGD, COMD, etc.) — all 1 byte
   1, 1, 1, 1, 1, 1, 1, 1,  1, 1, 1, 1, 1, 1, 1, 1,
   1, 1, 1, 1, 1, 1, 1, 1,  1, 1, 1, 1, 1, 1, 1, 1,
// 0x60-0x6F: all illegal
   1, 1, 1, 1, 1, 1, 1, 1,  1, 1, 1, 1, 1, 1, 1, 1,
// 0x70-0x7F: all illegal
   1, 1, 1, 1, 1, 1, 1, 1,  1, 1, 1, 1, 1, 1, 1, 1,
// 0x80-0x8F: immediate ops
// 83=CMPD(3), 8C=CMPY(3), 8E=LDY(3), others illegal
   1, 1, 1, 3, 1, 1, 1, 1,  1, 1, 1, 1, 3, 1, 3, 1,
// 0x90-0x9F: direct ops
// 93=CMPD(2), 9C=CMPY(2), 9E=LDY(2), 9F=STY(2), others illegal
   1, 1, 1, 2, 1, 1, 1, 1,  1, 1, 1, 1, 2, 1, 2, 2,
// 0xA0-0xAF: indexed ops
// A3=CMPD, AC=CMPY, AE=LDY, AF=STY — IDX_BASE(2), others illegal
   1, 1, 1, IDX_BASE(2), 1, 1, 1, 1,
   1, 1, 1, 1, IDX_BASE(2), 1, IDX_BASE(2), IDX_BASE(2),
// 0xB0-0xBF: extended ops
// B3=CMPD(3), BC=CMPY(3), BE=LDY(3), BF=STY(3), others illegal
   1, 1, 1, 3, 1, 1, 1, 1,  1, 1, 1, 1, 3, 1, 3, 3,
// 0xC0-0xCF: CE=LDS imm(3), others illegal
   1, 1, 1, 1, 1, 1, 1, 1,  1, 1, 1, 1, 1, 1, 3, 1,
// 0xD0-0xDF: DC=LDMD(2), DE=LDS(2), DF=STS(2), others illegal
   1, 1, 1, 1, 1, 1, 1, 1,  1, 1, 1, 1, 2, 1, 2, 2,
// 0xE0-0xEF: EC=??, EE=LDS, EF=STS indexed
   1, 1, 1, 1, 1, 1, 1, 1,
   1, 1, 1, 1, 1, 1, IDX_BASE(2), IDX_BASE(2),
// 0xF0-0xFF: FE=LDS(3), FF=STS(3), others illegal
   1, 1, 1, 1, 1, 1, 1, 1,  1, 1, 1, 1, 1, 1, 3, 3,
};

// Page 3 (0x11 prefix) instruction lengths (after the prefix byte).
static const uint8_t Page3InsLen[256] = {
// 0x00-0x2F: all illegal
   1, 1, 1, 1, 1, 1, 1, 1,  1, 1, 1, 1, 1, 1, 1, 1,
   1, 1, 1, 1, 1, 1, 1, 1,  1, 1, 1, 1, 1, 1, 1, 1,
   1, 1, 1, 1, 1, 1, 1, 1,  1, 1, 1, 1, 1, 1, 1, 1,
// 0x30-0x3F: 3F=SWI3(1), rest illegal
   1, 1, 1, 1, 1, 1, 1, 1,  1, 1, 1, 1, 1, 1, 1, 1,
// 0x40-0x7F: all illegal
   1, 1, 1, 1, 1, 1, 1, 1,  1, 1, 1, 1, 1, 1, 1, 1,
   1, 1, 1, 1, 1, 1, 1, 1,  1, 1, 1, 1, 1, 1, 1, 1,
   1, 1, 1, 1, 1, 1, 1, 1,  1, 1, 1, 1, 1, 1, 1, 1,
   1, 1, 1, 1, 1, 1, 1, 1,  1, 1, 1, 1, 1, 1, 1, 1,
// 0x80-0x8F: 83=CMPU(3), 8C=CMPS(3), others illegal
   1, 1, 1, 3, 1, 1, 1, 1,  1, 1, 1, 1, 3, 1, 1, 1,
// 0x90-0x9F: 93=CMPU(2), 9C=CMPS(2), others illegal
   1, 1, 1, 2, 1, 1, 1, 1,  1, 1, 1, 1, 2, 1, 1, 1,
// 0xA0-0xAF: A3=CMPU indexed, AC=CMPS indexed
   1, 1, 1, IDX_BASE(2), 1, 1, 1, 1,
   1, 1, 1, 1, IDX_BASE(2), 1, 1, 1,
// 0xB0-0xBF: B3=CMPU(3), BC=CMPS(3), others illegal
   1, 1, 1, 3, 1, 1, 1, 1,  1, 1, 1, 1, 3, 1, 1, 1,
// 0xC0-0xFF: all illegal
   1, 1, 1, 1, 1, 1, 1, 1,  1, 1, 1, 1, 1, 1, 1, 1,
   1, 1, 1, 1, 1, 1, 1, 1,  1, 1, 1, 1, 1, 1, 1, 1,
   1, 1, 1, 1, 1, 1, 1, 1,  1, 1, 1, 1, 1, 1, 1, 1,
   1, 1, 1, 1, 1, 1, 1, 1,  1, 1, 1, 1, 1, 1, 1, 1,
};

#undef IDX_BASE

// Compute the total byte length of an instruction given its length-table
// entry and the memory at pc (for reading the indexed postbyte).
// pc points to the opcode byte.
inline int ComputeInsLength(uint8_t lenEntry, uint16_t pc, int postbyteOffset)
{
    if (!(lenEntry & 0x80))
        return lenEntry;  // fixed-length instruction

    // Indexed mode: base length + extra from postbyte
    int base = lenEntry & 0x7F;
    // The postbyte is at pc + postbyteOffset (usually 1, or 2 for OIM/AIM etc.)
    uint8_t postbyte = MemRead8(pc + postbyteOffset);
    return base + IndexedExtraBytes(postbyte);
}

// Decode a block of num_insns instructions starting at start_pc.
// Fills the out[] array with handler, operand, postbyte, and length.
// Returns the PC after the last decoded instruction (should match end_pc).
inline uint16_t DecodeBlock(uint16_t start_pc, int num_insns, DecodedInst* out)
{
    uint16_t pc = start_pc;

    for (int i = 0; i < num_insns; i++)
    {
        uint8_t opcode = MemRead8(pc);
        DecodedInst& inst = out[i];

        if (opcode == 0x10)
        {
            // Page 2 prefix: store Page_2 handler, decode full length
            inst.handler = JmpVec1[0x10];
            uint8_t op2 = MemRead8(pc + 1);
            uint8_t lenEntry = Page2InsLen[op2];
            int innerLen = (lenEntry & 0x80)
                ? (lenEntry & 0x7F) + IndexedExtraBytes(MemRead8(pc + 2))
                : lenEntry;
            inst.length = (uint8_t)(1 + innerLen);  // prefix + inner
            inst.operand = op2;
            inst.postbyte = 0;
        }
        else if (opcode == 0x11)
        {
            // Page 3 prefix: store Page_3 handler, decode full length
            inst.handler = JmpVec1[0x11];
            uint8_t op3 = MemRead8(pc + 1);
            uint8_t lenEntry = Page3InsLen[op3];
            int innerLen = (lenEntry & 0x80)
                ? (lenEntry & 0x7F) + IndexedExtraBytes(MemRead8(pc + 2))
                : lenEntry;
            inst.length = (uint8_t)(1 + innerLen);
            inst.operand = op3;
            inst.postbyte = 0;
        }
        else
        {
            // Page 1 instruction
            inst.handler = JmpVec1[opcode];
            uint8_t lenEntry = Page1InsLen[opcode];

            if (lenEntry & 0x80)
            {
                // Indexed mode
                int base = lenEntry & 0x7F;
                // Postbyte position: for regular indexed ops it's at pc+1,
                // for OIM/AIM/EIM/TIM indexed (base=3) it's at pc+2
                // (because there's an immediate byte between opcode and postbyte)
                int pbOffset = (base >= 3) ? 2 : 1;
                uint8_t pb = MemRead8(pc + pbOffset);
                inst.postbyte = pb;
                inst.length = (uint8_t)(base + IndexedExtraBytes(pb));

                // Extract operand: for base=3, the immediate byte at pc+1
                if (base >= 3)
                    inst.operand = MemRead8(pc + 1);
                else
                    inst.operand = 0;
            }
            else if (lenEntry == 0)
            {
                // Illegal or page prefix that shouldn't appear here
                inst.length = 1;
                inst.operand = 0;
                inst.postbyte = 0;
            }
            else
            {
                // Fixed-length instruction: extract operand bytes
                inst.length = lenEntry;
                inst.postbyte = 0;

                switch (lenEntry)
                {
                case 1:
                    inst.operand = 0;
                    break;
                case 2:
                    inst.operand = MemRead8(pc + 1);
                    break;
                case 3:
                    inst.operand = MemRead16(pc + 1);
                    break;
                default:
                    // 4+ bytes (e.g., OIM/AIM extended = 4, LDQ imm = 5)
                    // Store first operand word; handler reads the rest
                    inst.operand = MemRead16(pc + 1);
                    break;
                }
            }
        }

        pc += inst.length;
    }

    return pc;
}
