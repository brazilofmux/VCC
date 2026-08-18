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

// Headless driver (docs/porting-macos.md stage 1): boot the CoCo 3 core
// with no shell at all - no window, no audio, no throttle, no carts -
// run it flat out for a number of frames, then print the effective
// speed and an ASCII dump of the text screen. With a coco3.rom next to
// the binary (or passed as argv[1]) the dump shows the BASIC copyright
// banner: proof the whole portable core works on this host.
//
// The boot sequence mirrors DoHardReset() in Vcc.cpp minus the UI.
// Usage: vcc-headless [rom-path] [frames]

#include "defines.h"
#include "MachineDefs.h"
#include "tcc1014mmu.h"
#include "tcc1014registers.h"
#include "tcc1014graphics.h"
#include "mc6821.h"
#include "coco3.h"
#include "hd6309.h"
#include "mc6809.h"
#include "pakinterface.h"
#include "Vcc.h"
#include <chrono>
#include <cstdio>
#include <cstring>

// Globals normally owned by Vcc.cpp (the Win32 shell).
SystemState EmuState;
void (*CPUInit)() = nullptr;
int  (*CPUExec)(int) = nullptr;
void (*CPUReset)() = nullptr;
void (*CPUAssertInterupt)(InterruptSource, Interrupt) = nullptr;
void (*CPUDeAssertInterupt)(InterruptSource, Interrupt) = nullptr;
void (*CPUForcePC)(unsigned short) = nullptr;
void (*CPUSetBreakpoints)(const std::vector<unsigned short>&) = nullptr;
void (*CPUSetTraceTriggers)(const std::vector<unsigned short>&) = nullptr;
VCC::CPUState (*CPUGetState)() = nullptr;

// ROM path handed to the GetExtRomPath stub in stubs.cpp.
char gHeadlessRomPath[512] = "coco3.rom";

// 32-bit render target; the GIME renderer needs somewhere to draw even
// though nobody looks at it here.
static unsigned int Framebuffer[640 * 480];

// CoCo VDG screen code -> printable ASCII (both the inverse 0x00-0x3F
// and normal 0x40-0x7F ranges fold to the same glyphs).
static char ScreenChar(unsigned char c)
{
	const unsigned char v = c & 0x3F;
	const char ch = (v < 32) ? (char)(v + 64) : (char)v;
	return (ch < 32 || ch > 126) ? ' ' : ch;
}

static void DumpTextScreen()
{
	std::printf("+--------------------------------+\n");
	for (int row = 0; row < 16; ++row)
	{
		char line[33];
		for (int col = 0; col < 32; ++col)
			line[col] = ScreenChar(SafeMemRead8((unsigned short)(0x0400 + row * 32 + col)));
		line[32] = '\0';
		std::printf("|%s|\n", line);
	}
	std::printf("+--------------------------------+\n");
}

int main(int argc, char** argv)
{
	if (argc > 1)
		std::snprintf(gHeadlessRomPath, sizeof(gHeadlessRomPath), "%s", argv[1]);
	const int frames = (argc > 2) ? std::atoi(argv[2]) : 600;

	EmuState.RamSize = 1;           // 512K
	EmuState.CpuType = 1;           // HD6309
	EmuState.FrameSkip = 1;
	EmuState.EmulationRunning = 1;
	EmuState.BitDepth = 3;          // 32-bit surface
	EmuState.PTRsurface32 = Framebuffer;
	EmuState.SurfacePitch = 640;

	// DoHardReset(), sans UI.
	EmuState.RamBuffer = MmuInit(EmuState.RamSize);
	EmuState.WRamBuffer = (unsigned short*)EmuState.RamBuffer;
	if (EmuState.RamBuffer == nullptr)
	{
		std::fprintf(stderr, "vcc-headless: MmuInit failed\n");
		return 1;
	}

	CPUInit             = HD6309Init;
	CPUExec             = HD6309Exec;
	CPUReset            = HD6309Reset;
	CPUAssertInterupt   = HD6309AssertInterupt;
	CPUDeAssertInterupt = HD6309DeAssertInterupt;
	CPUForcePC          = HD6309ForcePC;
	CPUSetBreakpoints   = HD6309SetBreakpoints;
	CPUGetState         = HD6309GetState;
	CPUSetTraceTriggers = HD6309SetTraceTriggers;

	PiaReset();
	mc6883_reset();
	CPUInit();
	CPUReset();
	GimeReset();
	UpdateBusPointer();
	EmuState.TurboSpeedFlag = 1;
	ResetBus();
	SetCPUMultiplyerFlag(0);
	SetClockSpeed(1);

	std::printf("vcc-headless: rom=%s frames=%d cpu=HD6309 ram=512K\n",
	            gHeadlessRomPath, frames);

	const auto start = std::chrono::steady_clock::now();
	for (int i = 0; i < frames; ++i)
		RenderFrame(&EmuState);
	const auto end = std::chrono::steady_clock::now();

	const double wall = std::chrono::duration<double>(end - start).count();
	const double emulated = frames / 60.0;
	const double ratio = (wall > 0.0) ? emulated / wall : 0.0;

	std::printf("rendered %d frames (%.1fs emulated) in %.3fs wall: %.1fx realtime\n",
	            frames, emulated, wall, ratio);
	std::printf("effective CPU speed ~%.1f MHz (0.894 MHz machine)\n", 0.894 * ratio);

	char jitstats[512];
	HD6309GetBlockStatsText(jitstats, sizeof(jitstats));
	std::printf("%s\n", jitstats);

	DumpTextScreen();
	return 0;
}
