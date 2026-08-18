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

// Null backend for the Level-1/2 JIT on hosts where the x86-32 emitter
// in BlockJit.cpp cannot run (everything except Win32|x86 builds).
//
// EmitBlock refusing (returning nullptr) is the emitter's documented
// fallback contract: the dispatch loop leaves native_entry unset and the
// block runs through the threaded interpreter + block cache, which is a
// first-class configuration per docs/porting-macos.md. The arm64 backend
// replaces this file when it lands (see the "Prior art" section there
// for the donor code).
//
// Built only by CMake; the MSBuild projects keep compiling BlockJit.cpp.

#include "BlockJit.h"

namespace BlockJit
{

void Init(const CpuAddrs&, const InlineableHandlers&)
{
}

void Reset()
{
}

NativeEntry EmitBlock(const CachedBlock&)
{
    return nullptr;
}

Stats GetStats()
{
    return Stats{};
}

}
