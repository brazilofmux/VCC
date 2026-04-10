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
// Page 2 instructions can now be decoded directly to their handlers.
// Page 3 still uses the Page_3 compatibility wrapper for now.

#include <cstdint>
#include "DecodedInst.h"
#include "EAModes.h"
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
//
// IMPORTANT: any entry here must agree with what JmpVec2[op2]'s handler
// actually consumes. Mismatches cause the block decoder to walk wrong byte
// boundaries and produce gibberish for subsequent instructions in the block.
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
// 0x80-0x8F: immediate ops (all 3 bytes after prefix: opcode + 16-bit imm)
// 80=SUBW, 81=CMPW, 82=SBCD, 83=CMPD, 84=ANDD, 85=BITD, 86=LDW,
// 87=illegal, 88=EORD, 89=ADCD, 8A=ORD, 8B=ADDW, 8C=CMPY, 8D=illegal,
// 8E=LDY, 8F=illegal
   3, 3, 3, 3, 3, 3, 3, 1,  3, 3, 3, 3, 3, 1, 3, 1,
// 0x90-0x9F: direct ops (all 2 bytes after prefix: opcode + DP byte)
// 90=SUBW, 91=CMPW, 92=SBCD, 93=CMPD, 94=ANDD, 95=BITD, 96=LDW, 97=STW,
// 98=EORD, 99=ADCD, 9A=ORD, 9B=ADDW, 9C=CMPY, 9D=illegal,
// 9E=LDY, 9F=STY
   2, 2, 2, 2, 2, 2, 2, 2,  2, 2, 2, 2, 2, 1, 2, 2,
// 0xA0-0xAF: indexed ops (IDX_BASE(2): opcode + postbyte + extras)
// A0=SUBW, A1=CMPW, A2=SBCD, A3=CMPD, A4=ANDD, A5=BITD, A6=LDW, A7=STW,
// A8=EORD, A9=ADCD, AA=ORD, AB=ADDW, AC=CMPY, AD=illegal, AE=LDY, AF=STY
   IDX_BASE(2), IDX_BASE(2), IDX_BASE(2), IDX_BASE(2),
   IDX_BASE(2), IDX_BASE(2), IDX_BASE(2), IDX_BASE(2),
   IDX_BASE(2), IDX_BASE(2), IDX_BASE(2), IDX_BASE(2),
   IDX_BASE(2), 1, IDX_BASE(2), IDX_BASE(2),
// 0xB0-0xBF: extended ops (all 3 bytes after prefix: opcode + 16-bit addr)
// B0=SUBW, B1=CMPW, B2=SBCD, B3=CMPD, B4=ANDD, B5=BITD, B6=LDW, B7=STW,
// B8=EORD, B9=ADCD, BA=ORD, BB=ADDW, BC=CMPY, BD=illegal, BE=LDY, BF=STY
   3, 3, 3, 3, 3, 3, 3, 3,  3, 3, 3, 3, 3, 1, 3, 3,
// 0xC0-0xCF: CE=LDS imm(3), others illegal
   1, 1, 1, 1, 1, 1, 1, 1,  1, 1, 1, 1, 1, 1, 3, 1,
// 0xD0-0xDF: DC=LDQ direct(2), DD=STQ direct(2), DE=LDS direct(2), DF=STS direct(2)
   1, 1, 1, 1, 1, 1, 1, 1,  1, 1, 1, 1, 2, 2, 2, 2,
// 0xE0-0xEF: EC=LDQ idx, ED=STQ idx, EE=LDS idx, EF=STS idx (all IDX_BASE(2))
   1, 1, 1, 1, 1, 1, 1, 1,
   1, 1, 1, 1, IDX_BASE(2), IDX_BASE(2), IDX_BASE(2), IDX_BASE(2),
// 0xF0-0xFF: FC=LDQ ext(3), FD=STQ ext(3), FE=LDS ext(3), FF=STS ext(3)
   1, 1, 1, 1, 1, 1, 1, 1,  1, 1, 1, 1, 3, 3, 3, 3,
};

// Page 3 (0x11 prefix) instruction lengths (after the prefix byte).
//
// IMPORTANT: any entry here must agree with what JmpVec3[op3]'s handler
// actually consumes. See Page2InsLen comment for the same warning.
static const uint8_t Page3InsLen[256] = {
// 0x00-0x2F: all illegal
   1, 1, 1, 1, 1, 1, 1, 1,  1, 1, 1, 1, 1, 1, 1, 1,
   1, 1, 1, 1, 1, 1, 1, 1,  1, 1, 1, 1, 1, 1, 1, 1,
   1, 1, 1, 1, 1, 1, 1, 1,  1, 1, 1, 1, 1, 1, 1, 1,
// 0x30-0x3F: 30-37 bit ops(3), 38-3B TFM(2), 3C=BITMD(2), 3D=LDMD(2), 3F=SWI3(1)
   3, 3, 3, 3, 3, 3, 3, 3,  2, 2, 2, 2, 2, 2, 1, 1,
// 0x40-0x7F: 6309 inherent E/F ops (1 byte each); others illegal (also 1)
   1, 1, 1, 1, 1, 1, 1, 1,  1, 1, 1, 1, 1, 1, 1, 1,
   1, 1, 1, 1, 1, 1, 1, 1,  1, 1, 1, 1, 1, 1, 1, 1,
   1, 1, 1, 1, 1, 1, 1, 1,  1, 1, 1, 1, 1, 1, 1, 1,
   1, 1, 1, 1, 1, 1, 1, 1,  1, 1, 1, 1, 1, 1, 1, 1,
// 0x80-0x8F: immediate ops
// 80=SUBE imm8(2), 81=CMPE imm8(2), 83=CMPU imm16(3), 86=LDE imm8(2),
// 8B=ADDE imm8(2), 8C=CMPS imm16(3), 8D=DIVD imm8(2), 8E=DIVQ imm16(3),
// 8F=MULD imm16(3); others illegal
   2, 2, 1, 3, 1, 1, 2, 1,  1, 1, 1, 2, 3, 2, 3, 3,
// 0x90-0x9F: direct ops (2 bytes if valid: opcode + DP)
// 90=SUBE, 91=CMPE, 93=CMPU, 96=LDE, 97=STE, 9B=ADDE, 9C=CMPS,
// 9D=DIVD, 9E=DIVQ, 9F=MULD
   2, 2, 1, 2, 1, 1, 2, 2,  1, 1, 1, 2, 2, 2, 2, 2,
// 0xA0-0xAF: indexed ops (IDX_BASE(2) if valid)
// A0=SUBE, A1=CMPE, A3=CMPU, A6=LDE, A7=STE, AB=ADDE, AC=CMPS,
// AD=DIVD, AE=DIVQ, AF=MULD
   IDX_BASE(2), IDX_BASE(2), 1, IDX_BASE(2),
   1, 1, IDX_BASE(2), IDX_BASE(2),
   1, 1, 1, IDX_BASE(2),
   IDX_BASE(2), IDX_BASE(2), IDX_BASE(2), IDX_BASE(2),
// 0xB0-0xBF: extended ops (3 bytes if valid: opcode + 16-bit addr)
// B0=SUBE, B1=CMPE, B3=CMPU, B6=LDE, B7=STE, BB=ADDE, BC=CMPS,
// BD=DIVD, BE=DIVQ, BF=MULD
   3, 3, 1, 3, 1, 1, 3, 3,  1, 1, 1, 3, 3, 3, 3, 3,
// 0xC0-0xCF: F-register immediate ops.
// C0/C1/C6/CB use imm8 (2 bytes after prefix), others illegal.
   2, 2, 1, 1, 1, 1, 2, 1,  1, 1, 1, 2, 1, 1, 1, 1,
// 0xD0-0xDF: F-register direct ops.
// D0/D1/D6/D7/DB use direct (2 bytes after prefix), others illegal.
   2, 2, 1, 1, 1, 1, 2, 2,  1, 1, 1, 2, 1, 1, 1, 1,
// 0xE0-0xEF: F-register indexed ops.
// E0/E1/E6/E7/EB use indexed (postbyte follows second opcode).
   IDX_BASE(2), IDX_BASE(2), 1, 1, 1, 1, IDX_BASE(2), IDX_BASE(2),
   1, 1, 1, IDX_BASE(2), 1, 1, 1, 1,
// 0xF0-0xFF: F-register extended ops.
// F0/F1/F6/F7/FB use extended (3 bytes after prefix), others illegal.
   3, 3, 1, 1, 1, 1, 3, 3,  1, 1, 1, 3, 1, 1, 1, 1,
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
    uint8_t postbyte = MemFetch8(pc + postbyteOffset);
    return base + IndexedExtraBytes(postbyte);
}

// Returns the total byte length of the instruction at pc using the same
// length tables that drive DecodeBlock(). This is used by the recorder to
// verify that a non-terminating instruction really fell through to its
// sequential next PC; if not, recording is abandoned instead of caching a
// block with hidden control flow.
inline int GetInstructionLengthAt(uint16_t pc)
{
    uint8_t opcode = MemFetch8(pc);

    if (opcode == 0x10)
    {
        uint8_t op2 = MemFetch8(pc + 1);
        uint8_t lenEntry = Page2InsLen[op2];
        return 1 + ComputeInsLength(lenEntry, pc + 1, (lenEntry & 0x80) ? (((lenEntry & 0x7F) >= 3) ? 2 : 1) : 0);
    }

    if (opcode == 0x11)
    {
        uint8_t op3 = MemFetch8(pc + 1);
        uint8_t lenEntry = Page3InsLen[op3];
        return 1 + ComputeInsLength(lenEntry, pc + 1, (lenEntry & 0x80) ? 1 : 0);
    }

    uint8_t lenEntry = Page1InsLen[opcode];
    return ComputeInsLength(lenEntry, pc, (lenEntry & 0x80) ? (((lenEntry & 0x7F) >= 3) ? 2 : 1) : 0);
}

// Resolve an indexed postbyte into an EAMode and extract the operand.
// pc_at_postbyte: address of the postbyte in memory (for reading offset bytes)
// Sets mode, reg, and operand. Returns number of extra bytes after postbyte.
inline int DecodeIndexedPostbyte(uint8_t pb, uint16_t pc_at_postbyte,
                                 EAMode& mode, uint8_t& reg, uint16_t& operand)
{
    reg = (pb >> 5) & 3;

    if (!(pb & 0x80))
    {
        // 5-bit signed offset: sign-extend bits 0-4
        signed char offset = (pb & 0x1F);
        if (offset & 0x10) offset |= 0xE0;  // sign extend bit 4
        mode = EA_5BIT;
        operand = (unsigned short)(signed short)offset;
        return 0;
    }

    int extra = 0;
    switch (pb & 0x1F)
    {
    case 0:  mode = EA_POST_INC1; break;
    case 1:  mode = EA_POST_INC2; break;
    case 2:  mode = EA_PRE_DEC1; break;
    case 3:  mode = EA_PRE_DEC2; break;
    case 4:  mode = EA_NO_OFFSET; break;
    case 5:  mode = EA_OFFSET_B; break;
    case 6:  mode = EA_OFFSET_A; break;
    case 7:  mode = EA_OFFSET_E; break;
    case 8:  // 8-bit offset
        mode = EA_OFFSET_8;
        operand = MemFetch8(pc_at_postbyte + 1);
        extra = 1;
        break;
    case 9:  // 16-bit offset
        mode = EA_OFFSET_16;
        operand = MemFetch16(pc_at_postbyte + 1);
        extra = 2;
        break;
    case 10: mode = EA_OFFSET_F; break;
    case 11: mode = EA_OFFSET_D; break;
    case 12: // 8-bit PC-relative: pre-compute absolute address
        {
            signed char off8 = (signed char)MemFetch8(pc_at_postbyte + 1);
            // EA = (address_of_offset_byte) + offset + 1
            // address_of_offset_byte = pc_at_postbyte + 1
            operand = (uint16_t)((pc_at_postbyte + 1) + off8 + 1);
            mode = EA_PC_8;
            extra = 1;
        }
        break;
    case 13: // 16-bit PC-relative: pre-compute absolute address
        {
            uint16_t off16 = MemFetch16(pc_at_postbyte + 1);
            // EA = (address_of_offset_word) + offset + 2
            operand = (uint16_t)((pc_at_postbyte + 1) + off16 + 2);
            mode = EA_PC_16;
            extra = 2;
        }
        break;
    case 14: mode = EA_OFFSET_W; break;
    case 15: // W-reg direct modes (6309), sub-mode from register field
        switch (reg) {
        case 0: mode = EA_W_NO_OFFSET; break;
        case 1: mode = EA_W_OFFSET_16;
                operand = MemFetch16(pc_at_postbyte + 1);
                extra = 2; break;
        case 2: mode = EA_W_POST_INC; break;
        case 3: mode = EA_W_PRE_DEC; break;
        }
        break;
    case 16: // W-reg indirect modes (6309)
        switch (reg) {
        case 0: mode = EA_I_W_NO_OFFSET; break;
        case 1: mode = EA_I_W_OFFSET_16;
                operand = MemFetch16(pc_at_postbyte + 1);
                extra = 2; break;
        case 2: mode = EA_I_W_POST_INC; break;
        case 3: mode = EA_I_W_PRE_DEC; break;
        }
        break;
    case 17: mode = EA_I_POST_INC2; break;
    case 18: mode = EA_I_ILLEGAL; break;
    case 19: mode = EA_I_PRE_DEC2; break;
    case 20: mode = EA_I_NO_OFFSET; break;
    case 21: mode = EA_I_OFFSET_B; break;
    case 22: mode = EA_I_OFFSET_A; break;
    case 23: mode = EA_I_OFFSET_E; break;
    case 24: // indirect 8-bit offset
        mode = EA_I_OFFSET_8;
        operand = MemFetch8(pc_at_postbyte + 1);
        extra = 1;
        break;
    case 25: // indirect 16-bit offset
        mode = EA_I_OFFSET_16;
        operand = MemFetch16(pc_at_postbyte + 1);
        extra = 2;
        break;
    case 26: mode = EA_I_OFFSET_F; break;
    case 27: mode = EA_I_OFFSET_D; break;
    case 28: // indirect 8-bit PC-relative
        {
            signed char off8 = (signed char)MemFetch8(pc_at_postbyte + 1);
            operand = (uint16_t)((pc_at_postbyte + 1) + off8 + 1);
            mode = EA_I_PC_8;
            extra = 1;
        }
        break;
    case 29: // indirect 16-bit PC-relative
        {
            uint16_t off16 = MemFetch16(pc_at_postbyte + 1);
            operand = (uint16_t)((pc_at_postbyte + 1) + off16 + 2);
            mode = EA_I_PC_16;
            extra = 2;
        }
        break;
    case 30: mode = EA_I_OFFSET_W; break;
    case 31: // extended indirect
        mode = EA_I_EXT;
        operand = MemFetch16(pc_at_postbyte + 1);
        extra = 2;
        break;
    default:
        mode = EA_I_ILLEGAL;
        break;
    }
    return extra;
}

// Decode a block of num_insns instructions starting at start_pc.
// Fills the out[] array with handler, operand, ea_info, and length.
// Returns the PC after the last decoded instruction.
inline uint16_t DecodeBlock(uint16_t start_pc, int num_insns, DecodedInst* out)
{
    uint16_t pc = start_pc;

    for (int i = 0; i < num_insns; i++)
    {
        uint8_t opcode = MemFetch8(pc);
        DecodedInst& inst = out[i];
        inst.aux = 0;

        if (opcode == 0x10)
        {
            // Page 2 prefix: decode through to the actual handler.
            uint8_t op2 = MemFetch8(pc + 1);
            uint8_t lenEntry = Page2InsLen[op2];
            inst.handler = JmpVec2[op2];

            if (lenEntry & 0x80)
            {
                // Indexed mode: page-2 indexed opcodes have the same inner
                // layout as page-1 indexed forms, just with a prefix byte.
                int base = lenEntry & 0x7F;
                int pbOffset = (base >= 3) ? 3 : 2;
                uint8_t pb = MemFetch8(pc + pbOffset);

                EAMode mode;
                uint8_t reg;
                uint16_t operand = 0;
                if (base >= 3)
                    inst.aux = MemFetch8(pc + 2);  // immediate byte before postbyte

                DecodeIndexedPostbyte(pb, pc + pbOffset, mode, reg, operand);
                inst.ea_info = MAKE_EA_INFO(mode, reg);
                inst.length = (uint8_t)(1 + base + IndexedExtraBytes(pb));
                inst.operand = operand;
            }
            else
            {
                int innerLen = lenEntry;
                inst.length = (uint8_t)(1 + innerLen);
                inst.ea_info = 0;

                switch (innerLen)
                {
                case 1:
                    inst.operand = 0;
                    break;
                case 2:
                    inst.operand = MemFetch8(pc + 2);
                    break;
                case 3:
                    inst.operand = MemFetch16(pc + 2);
                    break;
                default:
                    inst.operand = MemFetch16(pc + 2);
                    break;
                }
            }
        }
        else if (opcode == 0x11)
        {
            uint8_t op3 = MemFetch8(pc + 1);
            uint8_t lenEntry = Page3InsLen[op3];
            bool directDispatch = false;

            switch (op3)
            {
            case 0x30: case 0x31: case 0x32: case 0x33:
            case 0x34: case 0x35: case 0x36: case 0x37:
            case 0x38: case 0x39: case 0x3A: case 0x3B:
            case 0x3C: case 0x3D:
            case 0x43: case 0x4A: case 0x4C: case 0x4D: case 0x4F:
            case 0x53: case 0x5A: case 0x5C: case 0x5D: case 0x5F:
            case 0x80: case 0x81: case 0x83: case 0x86: case 0x8B: case 0x8C:
            case 0x8D: case 0x8E: case 0x8F:
            case 0x90: case 0x91: case 0x93: case 0x96: case 0x97:
            case 0x9B: case 0x9C: case 0x9D: case 0x9E: case 0x9F:
            case 0xA0: case 0xA1: case 0xA3: case 0xA6: case 0xA7:
            case 0xAB: case 0xAC: case 0xAD: case 0xAE: case 0xAF:
            case 0xB0: case 0xB1: case 0xB3: case 0xB6: case 0xB7:
            case 0xBB: case 0xBC: case 0xBD: case 0xBE: case 0xBF:
            case 0xC0: case 0xC1: case 0xC6: case 0xCB:
            case 0xD0: case 0xD1: case 0xD6: case 0xD7: case 0xDB:
            case 0xE0: case 0xE1: case 0xE6: case 0xE7: case 0xEB:
            case 0xF0: case 0xF1: case 0xF6: case 0xF7: case 0xFB:
                directDispatch = true;
                break;
            default:
                break;
            }

            inst.handler = directDispatch ? JmpVec3[op3] : JmpVec1[0x11];

            if (directDispatch)
            {
                if (lenEntry & 0x80)
                {
                    uint8_t pb = MemFetch8(pc + 2);
                    EAMode mode;
                    uint8_t reg;
                    uint16_t operand = 0;
                    DecodeIndexedPostbyte(pb, pc + 2, mode, reg, operand);
                    inst.ea_info = MAKE_EA_INFO(mode, reg);
                    inst.operand = operand;
                    inst.length = (uint8_t)(1 + (lenEntry & 0x7F) + IndexedExtraBytes(pb));
                }
                else
                {
                    int innerLen = lenEntry;
                    inst.length = (uint8_t)(1 + innerLen);
                    inst.ea_info = 0;

                    switch (innerLen)
                    {
                    case 1:
                        inst.operand = 0;
                        break;
                    case 2:
                        inst.operand = MemFetch8(pc + 2);
                        break;
                    case 3:
                        inst.operand = MemFetch16(pc + 2);
                        break;
                    default:
                        inst.operand = MemFetch16(pc + 2);
                        break;
                    }
                }
            }
            else
            {
                int innerLen = (lenEntry & 0x80)
                    ? (lenEntry & 0x7F) + IndexedExtraBytes(MemFetch8(pc + 2))
                    : lenEntry;
                inst.length = (uint8_t)(1 + innerLen);
                inst.ea_info = 0;
                inst.operand = op3;
            }
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
                // Postbyte position: for OIM/AIM/EIM/TIM indexed (base=3),
                // there's an immediate byte between opcode and postbyte
                int pbOffset = (base >= 3) ? 2 : 1;
                uint8_t pb = MemFetch8(pc + pbOffset);

                // Decode the postbyte into EA mode + register + operand
                EAMode mode;
                uint8_t reg;
                uint16_t operand = 0;
                // For base>=3 (OIM etc.), the immediate byte is the operand;
                // preserve it and let the indexed decode only set offset.
                if (base >= 3)
                    inst.aux = MemFetch8(pc + 1);  // immediate byte for OIM/AIM/EIM/TIM

                DecodeIndexedPostbyte(pb, pc + pbOffset, mode, reg, operand);
                // For base>=3, operand was overwritten by DecodeIndexedPostbyte
                // for modes that have offset bytes. For modes without offset
                // bytes, operand still has the immediate byte. That's correct
                // because OIM/AIM/EIM/TIM handlers read the immediate separately.
                // Actually, for OIM etc. we should NOT overwrite operand with
                // indexed data — leave it as the immediate byte. The EA
                // calculation uses the register and mode but not operand for
                // most modes. For modes with offsets, the operand is correctly
                // set by DecodeIndexedPostbyte. This is fine because OIM etc.
                // are not yet converted to use pre-decoded EA.
                (void)operand;

                inst.ea_info = MAKE_EA_INFO(mode, reg);
                inst.length = (uint8_t)(base + IndexedExtraBytes(pb));

                inst.operand = operand;
            }
            else if (lenEntry == 0)
            {
                // Illegal or page prefix
                inst.length = 1;
                inst.operand = 0;
                inst.ea_info = 0;
            }
            else
            {
                // Fixed-length instruction
                inst.length = lenEntry;
                inst.ea_info = 0;

                if (opcode == 0x01 || opcode == 0x02 || opcode == 0x05 || opcode == 0x0B)
                {
                    // OIM/AIM/EIM/TIM direct: imm8 then dp8
                    inst.aux = MemFetch8(pc + 1);
                    inst.operand = MemFetch8(pc + 2);
                }
                else if (opcode == 0x71 || opcode == 0x72 || opcode == 0x75 || opcode == 0x7B)
                {
                    // OIM/AIM/EIM/TIM extended: imm8 then ext16
                    inst.aux = MemFetch8(pc + 1);
                    inst.operand = MemFetch16(pc + 2);
                }
                else
                {
                    switch (lenEntry)
                    {
                    case 1:
                        inst.operand = 0;
                        break;
                    case 2:
                        inst.operand = MemFetch8(pc + 1);
                        break;
                    case 3:
                        inst.operand = MemFetch16(pc + 1);
                        break;
                    default:
                        inst.operand = MemFetch16(pc + 1);
                        break;
                    }
                }
            }
        }

        pc += inst.length;
    }

    return pc;
}
