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

#include <vcc/util/RomBlockStore.h>

namespace VCC
{

void RomBlockStore::AddRomBlocks(uint32_t fingerprint, uint16_t rom_base,
                                  std::vector<PrebuiltBlock> blocks)
{
    RomBlockStoreEntry& entry = entries_[fingerprint];
    entry.rom_base = rom_base;
    entry.blocks = std::move(blocks);
}

const RomBlockStoreEntry* RomBlockStore::Lookup(uint32_t fingerprint) const
{
    auto it = entries_.find(fingerprint);
    if (it == entries_.end())
        return nullptr;
    return &it->second;
}

void RomBlockStore::Clear()
{
    entries_.clear();
}

RomBlockStore& GetRomBlockStore()
{
    static RomBlockStore instance;
    return instance;
}

} // namespace VCC
