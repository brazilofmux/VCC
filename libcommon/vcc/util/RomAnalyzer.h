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

// RomAnalyzer: reachability analysis over a ROM byte sequence.
//
// Given a ROM image, a logical base address it will be mapped at, and a
// list of seed entry-point offsets, the analyzer walks forward following
// fall-through and statically resolvable control flow (relative branches,
// long branches, JMP/JSR extended-absolute, etc.) and reports every
// in-ROM byte offset that is the start of an instruction.
//
// Output is keyed by ROM-relative OFFSETS, not logical addresses, so the
// result is independent of where the ROM ends up mapped at runtime. The
// caller adds the base address back when feeding the result into the
// block cache.
//
// Unresolvable terminators (RTS/RTI, JMP indexed, JSR indexed, EXG/TFR
// targeting PC, PULS/PULU pulling PC, JMP through a pointer table, ...)
// stop tracing along that path. They're counted in the result so we can
// see how many "edges of the world" the analyzer hit.

#include <cstddef>
#include <cstdint>
#include <set>
#include <vector>
#include <vcc/util/RomBlockStore.h>

namespace VCC
{

struct RomAnalysisResult
{
    // ROM-relative offsets of every byte that is the start of an
    // instruction reachable from the seeds. Always within [0, rom_size).
    std::set<uint16_t> entry_offsets;

    int instructions_decoded;     // total Decode() calls (some may be repeats stopped by visited check)
    int branches_followed;        // statically resolvable control-flow edges added to the worklist
    int unresolved_terminators;   // RTS, JMP indexed, etc.
    int targets_outside_rom;      // branch / jump target fell outside the ROM range
};

// Analyze a ROM. seed_offsets are starting points for the trace; the
// caller is responsible for figuring out where they live. For ROMs that
// occupy the top of the address space (so the 6809 vectors at $FFF0-$FFFF
// land inside the ROM), use ReadStandardVectors() to get the seeds.
RomAnalysisResult AnalyzeRom(
    const uint8_t* rom_bytes,
    size_t rom_size,
    uint16_t rom_base,
    const std::vector<uint16_t>& seed_offsets);

// Read the 8 standard 6809 reset / interrupt / SWI vectors from the top
// of the ROM ($FFF0-$FFFF in logical space). Returns the discovered
// entry points as ROM-relative offsets, ready to feed to AnalyzeRom().
//
// This only makes sense for a ROM that ends at logical $FFFF (so the
// vector area falls inside the ROM). Returns an empty list otherwise.
//
// Vectors that point to obviously-uninitialized memory (8 consecutive
// identical bytes - typically 0x00 or 0xFF) are dropped, since those
// are vector slots that the OS plugs into at runtime rather than ROM
// handlers.
std::vector<uint16_t> ReadStandardVectors(
    const uint8_t* rom_bytes,
    size_t rom_size,
    uint16_t rom_base);

// Linear-sweep analyzer: walk straight through the ROM byte by byte,
// decoding each opcode and chaining to (offset + length). Equivalent to
// what an annotated 6809 disassembler does. Catches every byte that's
// the start of a valid-looking instruction in the natural decode chain
// from offset 0.
//
// Linear sweep finds CODE THAT THE STATIC TRACE MISSES (everything
// behind dispatch tables, indirect jumps, computed branches) at the
// cost of also flagging data regions whose bytes happen to decode as
// valid instructions. For the block cache, false positives are
// harmless: a "block" starting at a data offset is never entered at
// runtime so it just sits unused.
//
// Returns a result with the same shape as AnalyzeRom but populated
// purely from the linear sweep. Caller can union this with the static
// trace's result for "definitely code OR maybe code".
RomAnalysisResult AnalyzeRomLinearSweep(
    const uint8_t* rom_bytes,
    size_t rom_size,
    uint16_t rom_base);

// Walk forward from each entry offset and produce a PrebuiltBlock
// describing the resulting block (start_offset, num_insns, byte_length).
// Each block ends at the same terminators the runtime recorder uses
// (branch / JSR / RTS / etc.), or at MAX_BLOCK_INSNS, whichever comes
// first.
//
// `entry_offsets` is typically the union of static-trace and linear-
// sweep results. The output is suitable for stashing in RomBlockStore.
std::vector<PrebuiltBlock> BuildPrebuiltBlocks(
    const uint8_t* rom_bytes,
    size_t rom_size,
    uint16_t rom_base,
    const std::set<uint16_t>& entry_offsets);

} // namespace VCC
