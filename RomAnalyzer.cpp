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

#include "RomAnalyzer.h"
#include "InsnLengths.h"

namespace VCC
{

// Helpers for reading bytes from a flat ROM buffer (no MMU, no MemFetch).
static inline uint8_t RomByte(const uint8_t* rom, size_t rom_size, uint16_t off)
{
    return (off < rom_size) ? rom[off] : 0xFF;
}

static inline uint16_t RomWord(const uint8_t* rom, size_t rom_size, uint16_t off)
{
    // 6809 is big-endian: high byte first.
    return ((uint16_t)RomByte(rom, rom_size, off) << 8)
         | (uint16_t)RomByte(rom, rom_size, (uint16_t)(off + 1));
}

// Compute the actual byte length of the instruction at offset `off` in the
// ROM, using the same length tables as the runtime block decoder. Handles
// page-1, page-2 prefix, and page-3 prefix. Returns 0 on error (off out of
// range or illegal opcode encoding that we can't size).
static int InstructionLengthAt(const uint8_t* rom, size_t rom_size, uint16_t off)
{
    if (off >= rom_size)
        return 0;

    uint8_t opcode = rom[off];

    auto resolve = [&](uint8_t lenEntry, int prefixBytes, int postbyteOffset) -> int {
        if (!(lenEntry & 0x80))
            return prefixBytes + lenEntry;  // fixed-length

        int base = lenEntry & 0x7F;
        uint16_t pbAddr = (uint16_t)(off + prefixBytes + postbyteOffset);
        if (pbAddr >= rom_size)
            return 0;
        uint8_t pb = rom[pbAddr];
        return prefixBytes + base + IndexedExtraBytes(pb);
    };

    if (opcode == 0x10)
    {
        if ((size_t)(off + 1) >= rom_size) return 0;
        uint8_t op2 = rom[off + 1];
        uint8_t lenEntry = Page2InsLen[op2];
        // For indexed forms (lenEntry has 0x80 bit), the postbyte sits at
        // offset (base-1) past the second opcode. For OIM/AIM-style with
        // an extra immediate byte before the postbyte, base is >= 3.
        int pbOffset = (lenEntry & 0x80) ? (((lenEntry & 0x7F) >= 3) ? 2 : 1) : 0;
        return resolve(lenEntry, /*prefixBytes=*/1, pbOffset);
    }
    if (opcode == 0x11)
    {
        if ((size_t)(off + 1) >= rom_size) return 0;
        uint8_t op3 = rom[off + 1];
        uint8_t lenEntry = Page3InsLen[op3];
        int pbOffset = (lenEntry & 0x80) ? 1 : 0;
        return resolve(lenEntry, /*prefixBytes=*/1, pbOffset);
    }

    uint8_t lenEntry = Page1InsLen[opcode];
    if (lenEntry == 0)
        return 1;  // page prefix that wasn't handled above; shouldn't happen
    int pbOffset = (lenEntry & 0x80) ? (((lenEntry & 0x7F) >= 3) ? 1 : 0) : 0;
    return resolve(lenEntry, /*prefixBytes=*/0, pbOffset);
}

// Classification of how an instruction ends control flow.
enum class FlowKind
{
    FallThrough,         // continue at off + length
    BranchTaken,         // unconditional jump/branch to a known target
    BranchConditional,   // both fall-through AND a known branch target
    SubroutineCall,      // BSR / LBSR / JSR with known target; both target and fall-through reachable
    Terminator,          // RTS, RTI, JMP indexed, etc. - stop tracing
};

struct FlowInfo
{
    FlowKind kind;
    uint16_t target_logical;  // valid for BranchTaken / BranchConditional / SubroutineCall
};

// Inspect the instruction at `off` and tell us what kind of control flow
// it has. We only resolve STATIC targets (PC-relative branches, extended
// absolute JMP/JSR/LBRA/LBSR, page-2 long conditional branches). Indexed
// and computed forms become Terminator.
static FlowInfo ClassifyFlow(const uint8_t* rom, size_t rom_size,
                              uint16_t off, uint16_t rom_base, int insn_length)
{
    if (off >= rom_size)
        return { FlowKind::Terminator, 0 };

    uint8_t opcode = rom[off];

    auto pc_after = [&](void) -> uint16_t {
        return (uint16_t)(rom_base + off + insn_length);
    };

    // ----- Page 1 -----
    switch (opcode)
    {
    // Short relative branches (2 bytes total): opcode + signed 8-bit offset
    case 0x20:  // BRA - unconditional
        {
            int8_t off8 = (int8_t)RomByte(rom, rom_size, (uint16_t)(off + 1));
            return { FlowKind::BranchTaken, (uint16_t)(pc_after() + off8) };
        }
    case 0x21: // BRN (branch never) - effectively NOP, but classified as conditional w/ no taken effect
        return { FlowKind::FallThrough, 0 };

    case 0x22: case 0x23: case 0x24: case 0x25: case 0x26: case 0x27:
    case 0x28: case 0x29: case 0x2A: case 0x2B: case 0x2C: case 0x2D:
    case 0x2E: case 0x2F:  // BHI..BLE - conditional
        {
            int8_t off8 = (int8_t)RomByte(rom, rom_size, (uint16_t)(off + 1));
            return { FlowKind::BranchConditional, (uint16_t)(pc_after() + off8) };
        }

    case 0x16:  // LBRA - unconditional, 16-bit signed offset
        {
            int16_t off16 = (int16_t)RomWord(rom, rom_size, (uint16_t)(off + 1));
            return { FlowKind::BranchTaken, (uint16_t)(pc_after() + off16) };
        }
    case 0x17:  // LBSR
        {
            int16_t off16 = (int16_t)RomWord(rom, rom_size, (uint16_t)(off + 1));
            return { FlowKind::SubroutineCall, (uint16_t)(pc_after() + off16) };
        }

    case 0x8D:  // BSR - 8-bit signed offset
        {
            int8_t off8 = (int8_t)RomByte(rom, rom_size, (uint16_t)(off + 1));
            return { FlowKind::SubroutineCall, (uint16_t)(pc_after() + off8) };
        }

    case 0x7E:  // JMP extended (16-bit absolute)
        return { FlowKind::BranchTaken, RomWord(rom, rom_size, (uint16_t)(off + 1)) };
    case 0xBD:  // JSR extended
        return { FlowKind::SubroutineCall, RomWord(rom, rom_size, (uint16_t)(off + 1)) };

    case 0x39:  // RTS
    case 0x3B:  // RTI
    case 0x3F:  // SWI - vectors out of our analysis scope
    case 0x13:  // SYNC
    case 0x15:  // HALT (6309)
    case 0x3E:  // RESET (undocumented)
        return { FlowKind::Terminator, 0 };

    // JMP/JSR direct, indexed: target depends on DP register or index
    // register and is unresolvable statically.
    case 0x0E:  // JMP direct
    case 0x6E:  // JMP indexed
    case 0x9D:  // JSR direct
    case 0xAD:  // JSR indexed
        return { FlowKind::Terminator, 0 };

    // EXG / TFR can target PC; PULS / PULU can pull PC. Conservative:
    // treat as terminators (we don't track register data flow).
    case 0x1E: case 0x1F:        // EXG, TFR
    case 0x35: case 0x37:        // PULS, PULU
        return { FlowKind::Terminator, 0 };

    default:
        break;
    }

    // ----- Page 2 (long conditional branches and SWI2) -----
    if (opcode == 0x10 && (size_t)(off + 1) < rom_size)
    {
        uint8_t op2 = rom[off + 1];
        if (op2 >= 0x21 && op2 <= 0x2F)
        {
            // 1021..102F: long conditional branches (LBRN..LBLE), 4 bytes total.
            int16_t off16 = (int16_t)RomWord(rom, rom_size, (uint16_t)(off + 2));
            // LBRN (1021) is "branch never" — falls through unconditionally.
            if (op2 == 0x21)
                return { FlowKind::FallThrough, 0 };
            return { FlowKind::BranchConditional, (uint16_t)(pc_after() + off16) };
        }
        if (op2 == 0x3F)  // SWI2
            return { FlowKind::Terminator, 0 };
    }

    // ----- Page 3 -----
    if (opcode == 0x11 && (size_t)(off + 1) < rom_size)
    {
        uint8_t op3 = rom[off + 1];
        if (op3 == 0x3F)  // SWI3
            return { FlowKind::Terminator, 0 };
        // TFM (1138-113B) repeats the same instruction until counter reaches
        // zero. From the analyzer's perspective it falls through to the next
        // instruction once done.
    }

    // Everything else: ordinary fall-through.
    return { FlowKind::FallThrough, 0 };
}

// Add a logical address as an offset to the worklist if it falls inside
// the ROM and isn't already visited.
static void TryAddTarget(uint16_t target_logical,
                          uint16_t rom_base, size_t rom_size,
                          std::set<uint16_t>& visited,
                          std::vector<uint16_t>& worklist,
                          int& branches_followed,
                          int& targets_outside_rom)
{
    if (target_logical < rom_base
        || (size_t)(target_logical - rom_base) >= rom_size)
    {
        ++targets_outside_rom;
        return;
    }
    uint16_t target_offset = (uint16_t)(target_logical - rom_base);
    if (visited.insert(target_offset).second)
    {
        worklist.push_back(target_offset);
    }
    ++branches_followed;
}

RomAnalysisResult AnalyzeRom(
    const uint8_t* rom_bytes,
    size_t rom_size,
    uint16_t rom_base,
    const std::vector<uint16_t>& seed_offsets)
{
    RomAnalysisResult result {};

    if (!rom_bytes || rom_size == 0)
        return result;

    std::vector<uint16_t> worklist;
    for (uint16_t seed : seed_offsets)
    {
        if (seed < rom_size && result.entry_offsets.insert(seed).second)
            worklist.push_back(seed);
    }

    while (!worklist.empty())
    {
        uint16_t off = worklist.back();
        worklist.pop_back();
        ++result.instructions_decoded;

        int len = InstructionLengthAt(rom_bytes, rom_size, off);
        if (len <= 0 || (size_t)(off + len) > rom_size)
        {
            // Bad / illegal instruction or runs off the end - stop here.
            ++result.unresolved_terminators;
            continue;
        }

        FlowInfo flow = ClassifyFlow(rom_bytes, rom_size, off, rom_base, len);

        switch (flow.kind)
        {
        case FlowKind::FallThrough:
            {
                uint16_t next = (uint16_t)(off + len);
                if (next < rom_size && result.entry_offsets.insert(next).second)
                    worklist.push_back(next);
            }
            break;

        case FlowKind::BranchTaken:
            TryAddTarget(flow.target_logical, rom_base, rom_size,
                         result.entry_offsets, worklist,
                         result.branches_followed, result.targets_outside_rom);
            break;

        case FlowKind::BranchConditional:
            // Both the target and the fall-through are reachable.
            {
                TryAddTarget(flow.target_logical, rom_base, rom_size,
                             result.entry_offsets, worklist,
                             result.branches_followed, result.targets_outside_rom);
                uint16_t next = (uint16_t)(off + len);
                if (next < rom_size && result.entry_offsets.insert(next).second)
                    worklist.push_back(next);
            }
            break;

        case FlowKind::SubroutineCall:
            // Both the callee and the return point (fall-through) are reachable.
            {
                TryAddTarget(flow.target_logical, rom_base, rom_size,
                             result.entry_offsets, worklist,
                             result.branches_followed, result.targets_outside_rom);
                uint16_t next = (uint16_t)(off + len);
                if (next < rom_size && result.entry_offsets.insert(next).second)
                    worklist.push_back(next);
            }
            break;

        case FlowKind::Terminator:
            ++result.unresolved_terminators;
            break;
        }
    }

    return result;
}

// Heuristic: does the byte region at this offset look like real 6809
// code, or is it uninitialized data (a run of 0x00 or 0xFF)?
//
// Many CoCo3 ROM vectors point at "vector slots" that the OS plugs into
// at runtime; the ROM itself just contains 0x00 or 0xFF placeholders
// there. If we naively followed those vectors, the analyzer would walk
// through garbage bytes treating them as instructions and report
// hundreds of bogus entry points.
//
// The check is intentionally conservative: a region of 8 consecutive
// identical bytes is almost certainly not code. Real instruction streams
// have far more byte variety.
static bool LooksLikeRomCode(const uint8_t* rom_bytes, size_t rom_size, uint16_t off)
{
    if ((size_t)off + 8 > rom_size)
        return false;

    uint8_t first = rom_bytes[off];
    for (int i = 1; i < 8; i++)
    {
        if (rom_bytes[off + i] != first)
            return true;
    }
    return false;  // 8 identical bytes in a row - very unlikely to be code
}

RomAnalysisResult AnalyzeRomLinearSweep(
    const uint8_t* rom_bytes,
    size_t rom_size,
    uint16_t rom_base)
{
    (void)rom_base;  // unused for linear sweep - all addressing is by offset

    RomAnalysisResult result {};
    if (!rom_bytes || rom_size == 0)
        return result;

    // Walk byte by byte starting at offset 0. For each offset, decode the
    // instruction; if it's valid (length > 0 and fits in the ROM), record
    // it as an entry point and advance by the instruction length. If
    // decode fails, skip 1 byte and try again - this matches what a real
    // disassembler does for "data inside code" regions.
    uint16_t off = 0;
    while ((size_t)off < rom_size)
    {
        ++result.instructions_decoded;
        int len = InstructionLengthAt(rom_bytes, rom_size, off);
        if (len <= 0 || (size_t)(off + len) > rom_size)
        {
            ++result.unresolved_terminators;
            // Skip a single byte and try again. This re-syncs after data.
            ++off;
            continue;
        }
        result.entry_offsets.insert(off);
        off = (uint16_t)(off + len);
    }
    return result;
}

std::vector<uint16_t> ReadStandardVectors(
    const uint8_t* rom_bytes,
    size_t rom_size,
    uint16_t rom_base)
{
    std::vector<uint16_t> seeds;
    if (!rom_bytes || rom_size < 16)
        return seeds;

    // Vectors live at logical $FFF0..$FFFF. The ROM only contains them
    // if it occupies the top of the address space, i.e. its last byte
    // is at logical $FFFF.
    uint32_t rom_end_logical = (uint32_t)rom_base + rom_size;
    if (rom_end_logical != 0x10000u)
        return seeds;

    // Read the 8 vectors at offset (rom_size - 16). Each is a big-endian
    // 16-bit logical address. Drop ones that fall outside the ROM, and
    // drop ones that point to obvious uninitialized data (all 0x00 / 0xFF
    // runs - those are vector slots the OS plugs into at runtime).
    uint16_t vec_offset = (uint16_t)(rom_size - 16);
    for (int i = 0; i < 8; i++)
    {
        uint16_t off = (uint16_t)(vec_offset + i * 2);
        uint16_t target_logical = ((uint16_t)rom_bytes[off] << 8)
                                | (uint16_t)rom_bytes[off + 1];

        if (target_logical < rom_base
            || (size_t)(target_logical - rom_base) >= rom_size)
        {
            continue;  // points outside the ROM
        }

        uint16_t target_offset = (uint16_t)(target_logical - rom_base);
        if (!LooksLikeRomCode(rom_bytes, rom_size, target_offset))
            continue;  // looks like uninitialized data

        seeds.push_back(target_offset);
    }
    return seeds;
}

} // namespace VCC
