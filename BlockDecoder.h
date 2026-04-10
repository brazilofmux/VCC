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
#include "InsnLengths.h"
#include "tcc1014mmu.h"

// Forward declarations — these are defined in hd6309.cpp
extern InstHandler JmpVec1[256];
extern InstHandler JmpVec2[256];
extern InstHandler JmpVec3[256];

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
