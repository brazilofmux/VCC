#ifndef __TCC1014MMU_H__
#define __TCC1014MMU_H__
#include <array>


/*
Copyright 2015 by Joseph Forgione
This file is part of VCC (Virtual Color Computer).

    VCC (Virtual Color Computer) is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    VCC (Virtual Color Computer) is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with VCC (Virtual Color Computer).  If not, see <http://www.gnu.org/licenses/>.
*/
#include "Debugger.h"

namespace VCC
{

	struct MMUState
	{
		bool Enabled;
		int ActiveTask;
		int RamVectors;
		int RomMap;

		std::array<int, 8>    Task0;
		std::array<int, 8>    Task1;
	};

}

VCC::MMUState GetMMUState();
void GetMMUPage(size_t page, std::array<unsigned char, 8192>& outBuffer);

void MemWrite8(unsigned char,unsigned short );
void MemWrite16(unsigned short,unsigned short );

unsigned short MemRead16(unsigned short);
unsigned char MemRead8(unsigned short);

// Fast instruction-fetch primitive. Uses a cached pointer to the current
// 8KB RAM page so the common case (PC inside a RAM page that's already
// been resolved) is a single load with no MMU lookup, no port check, no
// ROM/RAM discrimination, and no mutex.
//
// The cache is invalidated whenever any MMU state changes (task swap,
// register write, ROM map, MapType, MMU enable). When the cache misses
// (different page, or non-RAM), MemFetch8_Slow falls back to MemRead8.
//
// Use this ONLY for instruction fetches and decoder byte walks. Data reads
// must go through MemRead8 because they may target ports or vector RAM.
extern unsigned char* gFetchPagePtr;
extern unsigned int   gFetchPageMask;  // matches (addr & 0xE000) when cache valid
unsigned char MemFetch8_Slow(unsigned short address);

static inline unsigned char MemFetch8(unsigned short address)
{
	// The < 0xFE00 guard is required: addresses in 0xFE00-0xFFFF live in
	// the same 8KB bucket as 0xE000-0xFDFF, but they may be RAM vectors or
	// port pass-throughs. Routing them through the cached page pointer
	// would return garbage. Force the slow path for that high tail.
	if (address < 0xFE00 && (unsigned int)(address & 0xE000) == gFetchPageMask)
		return gFetchPagePtr[address & 0x1FFF];
	return MemFetch8_Slow(address);
}

// Big-endian 16-bit instruction-fetch primitive built on MemFetch8.
// Used by the block decoder to read 16-bit immediates / addresses /
// branch offsets that live in the instruction stream.
static inline unsigned short MemFetch16(unsigned short address)
{
	return ((unsigned short)MemFetch8(address) << 8)
	     |  (unsigned short)MemFetch8((unsigned short)(address + 1));
}
unsigned char SafeMemRead8(unsigned short);
unsigned char * MmuInit(unsigned char);
unsigned char *	Getint_rom_pointer();
unsigned short GetMem(unsigned long);
void SetMem(unsigned long, unsigned short);
bool MemCheckWrite(unsigned short address);

void __fastcall fMemWrite8(unsigned char,unsigned short );
unsigned char __fastcall fMemRead8(unsigned short);

void SetMapType(unsigned char);
void LoadRom();
void Set_MmuTask(unsigned char);
void SetMmuRegister(unsigned char,unsigned char);
void Set_MmuEnabled (unsigned char );
void SetRomMap(unsigned char );
void SetVectors(unsigned char);
void MmuReset();
void SetDistoRamBank(unsigned char);
void SetMmuPrefix(unsigned char);
unsigned char * Get_mem_pointer();

// Block cache invalidation callbacks. Set by the active CPU engine.
typedef void (*BlockInvalidateFunc)(unsigned short address);
typedef void (*BlockInvalidateAllFunc)();
extern BlockInvalidateFunc gBlockInvalidate;       // per-address, called from MemWrite8
extern BlockInvalidateAllFunc gBlockInvalidateAll;  // full flush, called on MMU changes

// Identification of the currently-loaded internal ROM. Set by LoadRom()
// after fingerprinting; consumed by the CPU engine when populating the
// block cache from RomBlockStore at reset time. Zero if no ROM is loaded.
unsigned int  GetInternalRomFingerprint();
unsigned short GetInternalRomBase();
unsigned int  GetInternalRomSize();

// FIXME: These need to be turned into an enum and the signature of functions
// that use them updated.
#define _128K	0
#define _512K	1
#define _2M		2

#endif
