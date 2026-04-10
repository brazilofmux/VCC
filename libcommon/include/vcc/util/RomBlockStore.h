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

// RomBlockStore: holds the offline ROM analyzer's output (block boundaries
// discovered by static trace + linear sweep) keyed by ROM fingerprint, so
// the runtime block cache can be pre-populated at every CPU reset without
// re-running the analyzer.
//
// The store deliberately does NOT hold runtime DecodedInst arrays. Those
// contain function pointers tied to a specific CPU engine (HD6309 vs
// MC6809) and would force libcommon to know about hd6309.cpp's handlers.
// Instead, the store holds offset+length descriptors and the runtime side
// re-decodes them via DecodeBlock at insertion time. The decoded result is
// identical because the ROM bytes are stable.

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace VCC
{

// One pre-built block descriptor: where it starts in the ROM, how many
// instructions it contains, and the total byte length of those
// instructions (so end_offset = start_offset + byte_length). Cycle cost
// is intentionally not stored - the runtime estimates it from num_insns
// when rehydrating.
struct PrebuiltBlock
{
    uint16_t start_offset;  // ROM-relative offset of the first instruction
    uint8_t  num_insns;     // 1..MAX_BLOCK_INSNS
    uint8_t  byte_length;   // sum of instruction lengths in this block
};

// Per-ROM stash: the rom_base it was analyzed at, plus the list of blocks.
// Knowing the base lets us rehydrate to the correct logical addresses
// even if the ROM's mapping changes between sessions (e.g., the same
// fingerprint mounted in a different slot).
struct RomBlockStoreEntry
{
    uint16_t rom_base;
    std::vector<PrebuiltBlock> blocks;
};

class RomBlockStore
{
public:
    // Add (or replace) the pre-built block list for a ROM identified by
    // its content fingerprint. Replacing is OK - the analysis is
    // deterministic so the new list will match the old one.
    void AddRomBlocks(uint32_t fingerprint, uint16_t rom_base,
                      std::vector<PrebuiltBlock> blocks);

    // Look up the entry for a fingerprint. Returns nullptr if no entry
    // has been added for that ROM.
    const RomBlockStoreEntry* Lookup(uint32_t fingerprint) const;

    // Drop all entries (e.g., on hard reset).
    void Clear();

    // Number of distinct ROMs in the store.
    size_t Size() const { return entries_.size(); }

    // Iterate over all (fingerprint, entry) pairs. Used by the CPU
    // engine to drain every registered ROM into the live block cache.
    using map_type = std::unordered_map<uint32_t, RomBlockStoreEntry>;
    map_type::const_iterator begin() const { return entries_.begin(); }
    map_type::const_iterator end()   const { return entries_.end(); }

private:
    map_type entries_;
};

// Process-wide singleton accessor. The store is shared across the main
// VCC executable and any cartridge DLLs that link libcommon, so it can
// hold pre-built blocks for both internal and cartridge ROMs.
RomBlockStore& GetRomBlockStore();

} // namespace VCC
