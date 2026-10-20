#pragma once
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

// InsnLengths: static instruction-length tables for the 6809 / 6309
// instruction set, plus the small helper for indexed-mode postbyte
// length adjustment.
//
// These tables are pure data + pure logic. They have no dependencies on
// the runtime memory subsystem and can be included from any TU - the
// block decoder, the ROM reachability analyzer, anything else that needs
// to walk an instruction stream byte by byte.
//
// IMPORTANT: any entry in these tables must agree with what the
// corresponding handler (JmpVec1 / JmpVec2 / JmpVec3) actually consumes.
// Mismatches cause downstream code to walk wrong byte boundaries.

#include <cstdint>

// Encoding: positive values are fixed lengths. Negative values (stored as
// uint8_t >= 0x80) indicate indexed mode: the base is (value & 0x7F),
// and the postbyte at offset (base-1) from the opcode determines extra
// bytes via IndexedExtraBytes().
#define IDX_BASE(n) (0x80 | (n))

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

// Page 1 instruction lengths.
// 0 = illegal or special (page prefix). For indexed modes, this is the
// base length (opcode + any immediate byte before the postbyte + postbyte);
// IndexedExtraBytes() is added on top.
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
static const uint8_t Page2InsLen[256] = {
// 0x00-0x0F: all illegal (use 1 as safe default - the prefix already consumed 1)
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
// 0x40-0x5F: 6309 inherent ops (NEGD, COMD, etc.) - all 1 byte
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
