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

// RomDatabase: identify ROM images by content fingerprint.
//
// At ROM load time we compute a CRC32 over the byte sequence and look it up
// in a small registry of known ROMs. The registry maps fingerprint to a
// human-readable name (e.g. "CoCo3 ECB 1.0", "Disk ECB 1.1") and a kind
// classification (internal ROM, disk ROM, cartridge ROM, etc.). Unknown
// ROMs are still usable - identification is just metadata for logging
// today, and a key for the future ROM block analyzer.

#include <cstddef>
#include <cstdint>

namespace VCC
{

enum class RomKind
{
    Unknown,
    CocoBasic,        // Color BASIC (CoCo 1)
    CocoExtBasic,     // Extended Color BASIC (CoCo 1/2)
    Coco3Internal,    // CoCo 3 internal ROM (Super ECB + ECB + CB)
    DiskBasic,        // Disk Extended Color BASIC
    HardDiskBasic,    // HDB-DOS variants
    Cartridge,        // Generic cartridge ROM
    DragonBasic,      // Dragon 32/64 BASIC
};

struct RomInfo
{
    uint32_t   fingerprint;  // CRC32 of the ROM bytes
    size_t     size;         // ROM size in bytes
    RomKind    kind;         // classification
    const char* name;        // human-readable name, never null (e.g. "Unknown ROM")
};

// Compute the CRC32 of a byte buffer using the Ethernet/IEEE 802.3
// polynomial (0xEDB88320, reflected). Standard zlib-compatible CRC32.
uint32_t Crc32(const uint8_t* data, size_t len);

// Identify a ROM by content. Always returns a populated RomInfo - if the
// ROM is not in the registry, kind = Unknown and name = "Unknown ROM".
// The returned name pointer is valid for the program's lifetime.
RomInfo IdentifyRom(const uint8_t* data, size_t len);

} // namespace VCC
