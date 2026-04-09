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

// DecodedInst: Pre-decoded instruction for the threaded interpreter.
//
// When a block is first executed, each instruction is decoded into one of
// these structs. On subsequent executions, the handler is called directly
// with the pre-extracted operand — no MemRead8 for operand fetch, no
// byte-by-byte PC advancement, no opcode switch dispatch.
//
// Handlers that haven't been updated to use the pre-decoded operand can
// ignore the struct and read from memory as before. This allows incremental
// conversion.

#include <cstdint>

struct DecodedInst;
typedef void (*InstHandler)(const DecodedInst*);

struct DecodedInst
{
    InstHandler handler;        // handler function pointer (4 bytes on x86)
    uint16_t    operand;        // pre-extracted operand: immediate value,
                                // direct-page offset, extended address,
                                // or signed branch offset
    uint8_t     postbyte;       // indexed mode postbyte (0 if N/A)
    uint8_t     length;         // total instruction length in bytes
};
// sizeof(DecodedInst) = 8 bytes on x86 (4+2+1+1)
