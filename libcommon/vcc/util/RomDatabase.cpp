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

#include <vcc/util/RomDatabase.h>

namespace VCC
{

// Standard CRC32 (zlib / IEEE 802.3) reflected, polynomial 0xEDB88320.
// Computed lazily on first call into a 256-entry table for byte-at-a-time
// CRC update. ~1KB of cache, fast enough for any ROM size we care about.
static uint32_t crc32_table[256];
static bool     crc32_table_built = false;

static void BuildCrc32Table()
{
    for (uint32_t i = 0; i < 256; i++)
    {
        uint32_t c = i;
        for (int j = 0; j < 8; j++)
            c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
        crc32_table[i] = c;
    }
    crc32_table_built = true;
}

uint32_t Crc32(const uint8_t* data, size_t len)
{
    if (!crc32_table_built)
        BuildCrc32Table();

    uint32_t c = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++)
        c = crc32_table[(c ^ data[i]) & 0xFF] ^ (c >> 8);
    return c ^ 0xFFFFFFFFu;
}

// Registry of known ROMs. Add entries here as we encounter them in the wild.
// The order does not matter; lookups are linear and the registry is small.
//
// To add an entry: run VCC with the new ROM, look at the OutputDebugString
// log for "[ROM] ... Unknown ROM size=N crc=0xXXXXXXXX". Add a row.
static const RomInfo kKnownRoms[] = {
    // CoCo3 internal ROM (32KB: Color BASIC + ECB + Super ECB).
    // The standard MAME / VCC dump, typically named coco3.rom.
    { 0xB4C88D6Cu, 32768, RomKind::Coco3Internal, "CoCo3 Internal ROM (32K)" },

    // Disk Extended Color BASIC 1.1 (Tandy). The canonical FD-502 disk
    // ROM, typically named disk11.rom.
    { 0x0B9C5415u,  8192, RomKind::DiskBasic,     "Disk ECB 1.1 (Tandy)" },

    // RGB-DOS (alternative disk ROM with extended directory / drive
    // support). Typically named rgbdos.rom.
    { 0xE548C0A3u,  8192, RomKind::DiskBasic,     "RGB-DOS" },

    // CoCo 1 Color BASIC 1.0 (8KB). The earliest BASIC ROM, used by
    // the original 1980 Color Computer. From the MAME coco set.
    { 0x00B50AAAu,  8192, RomKind::CocoBasic,     "Color BASIC 1.0" },
};

static const size_t kKnownRomCount = sizeof(kKnownRoms) / sizeof(kKnownRoms[0]);

RomInfo IdentifyRom(const uint8_t* data, size_t len)
{
    RomInfo result {};
    result.fingerprint = Crc32(data, len);
    result.size        = len;
    result.kind        = RomKind::Unknown;
    result.name        = "Unknown ROM";

    for (size_t i = 0; i < kKnownRomCount; i++)
    {
        if (kKnownRoms[i].fingerprint == result.fingerprint
            && kKnownRoms[i].size == len)
        {
            result.kind = kKnownRoms[i].kind;
            result.name = kKnownRoms[i].name;
            break;
        }
    }

    return result;
}

} // namespace VCC
