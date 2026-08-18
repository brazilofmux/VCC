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
// with no shell - no window, no audio, no throttle - run it flat out,
// then print the effective speed and an ASCII dump of the text screen.
//
// Reads the same vcc.ini as the full emulator (~/.config/vcc/vcc.ini):
// RAM size, CPU type, and the [Module] OnBoot cartridge - so an MPI
// with FD502 + hard disk configuration boots here exactly as it would
// on Windows, including NitrOS-9 from the configured boot floppy/VHD.
// Cartridge modules load from <exedir>/<name>.so via the dlopen shim;
// FD502 finds disk11.rom/rgbdos.rom in <exedir> too.
//
// The boot sequence mirrors DoHardReset() in Vcc.cpp minus the UI.
// Usage: vcc-headless [rom-path] [frames] [text-to-type]
//        (text is typed starting at frame 150; "\n" means ENTER)

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
#include <vcc/util/settings.h>
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

// Provided by headless/stubs.cpp.
VCC::Util::settings& Setting();
void HeadlessTypeText(const char* text);
void HeadlessKeyboardTick();

// 32-bit render target; the GIME renderer needs somewhere to draw even
// though nobody looks at it here.
static unsigned int Framebuffer[640 * 480];

// CoCo VDG screen code -> printable ASCII (both the inverse 0x00-0x3F
// and normal 0x40-0x7F ranges fold to the same glyphs).
static char VdgChar(unsigned char c)
{
	const unsigned char v = c & 0x3F;
	const char ch = (v < 32) ? (char)(v + 64) : (char)v;
	return (ch < 32 || ch > 126) ? ' ' : ch;
}

static void DumpTextScreen()
{
	const int bpr = GetBytesPerRow();
	if (bpr == 40 || bpr == 80)
	{
		// GIME hardware text: 2 bytes per cell (character, attribute),
		// near-ASCII character codes. Same walk as coco3.cpp CopyText.
		const unsigned int start = GetStartOfVidram();
		std::printf("GIME %d-column text screen (vidram 0x%06X):\n", bpr, start);
		for (int row = 0; row < 24; ++row)
		{
			std::printf("|");
			for (int col = 0; col < bpr; ++col)
			{
				const unsigned char c =
					(unsigned char)GetMem(start + (unsigned long)row * bpr * 2 + col * 2);
				std::printf("%c", (c >= 32 && c <= 126) ? c : ' ');
			}
			std::printf("|\n");
		}
		return;
	}

	std::printf("VDG 32-column text screen:\n");
	std::printf("+--------------------------------+\n");
	for (int row = 0; row < 16; ++row)
	{
		char line[33];
		for (int col = 0; col < 32; ++col)
			line[col] = VdgChar(SafeMemRead8((unsigned short)(0x0400 + row * 32 + col)));
		line[32] = '\0';
		std::printf("|%s|\n", line);
	}
	std::printf("+--------------------------------+\n");
}

static void HardResetMachine()
{
	EmuState.RamBuffer = MmuInit(EmuState.RamSize);
	EmuState.WRamBuffer = (unsigned short*)EmuState.RamBuffer;
	if (EmuState.RamBuffer == nullptr)
	{
		std::fprintf(stderr, "vcc-headless: MmuInit failed\n");
		std::exit(1);
	}

	if (EmuState.CpuType == 1)
	{
		CPUInit             = HD6309Init;
		CPUExec             = HD6309Exec;
		CPUReset            = HD6309Reset;
		CPUAssertInterupt   = HD6309AssertInterupt;
		CPUDeAssertInterupt = HD6309DeAssertInterupt;
		CPUForcePC          = HD6309ForcePC;
		CPUSetBreakpoints   = HD6309SetBreakpoints;
		CPUGetState         = HD6309GetState;
		CPUSetTraceTriggers = HD6309SetTraceTriggers;
	}
	else
	{
		CPUInit             = MC6809Init;
		CPUExec             = MC6809Exec;
		CPUReset            = MC6809Reset;
		CPUAssertInterupt   = MC6809AssertInterupt;
		CPUDeAssertInterupt = MC6809DeAssertInterupt;
		CPUForcePC          = MC6809ForcePC;
		CPUSetBreakpoints   = MC6809SetBreakpoints;
		CPUGetState         = MC6809GetState;
		CPUSetTraceTriggers = MC6809SetTraceTriggers;
	}

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
}

int main(int argc, char** argv)
{
	if (argc > 1)
		std::snprintf(gHeadlessRomPath, sizeof(gHeadlessRomPath), "%s", argv[1]);
	const int frames = (argc > 2) ? std::atoi(argv[2]) : 600;
	const char* type_text = (argc > 3) ? argv[3] : nullptr;

	EmuState.RamSize = (unsigned char)Setting().read("Memory", "RamSize", 1);
	EmuState.CpuType = (unsigned char)Setting().read("CPU", "CpuType", 1);
	// The GIME renderer dominates headless wall time (~85% in
	// UpdateScreen32 at FrameSkip=1). VCC_FRAMESKIP=N renders only
	// every Nth frame - emulation is unaffected (HLINE still runs per
	// line) - so CPU-tier benchmarks measure the CPU, not the painter.
	EmuState.FrameSkip = 1;
	if (const char* fs = std::getenv("VCC_FRAMESKIP"))
	{
		const int n = std::atoi(fs);
		if (n >= 1 && n <= 255)
			EmuState.FrameSkip = (unsigned char)n;
	}
	EmuState.EmulationRunning = 1;
	EmuState.BitDepth = 3;          // 32-bit surface
	EmuState.PTRsurface32 = Framebuffer;
	EmuState.SurfacePitch = 640;

	HardResetMachine();

	// Load the boot cartridge the same way Vcc.cpp does at startup.
	char onboot[MAX_PATH] = "";
	Setting().read("Module", "OnBoot", "", onboot, MAX_PATH);
	if (onboot[0] != '\0')
	{
		const auto status = PakLoadCartridge(onboot);
		std::printf("vcc-headless: OnBoot module %s -> %s\n", onboot,
		            status == VCC::Core::cartridge_loader_status::success ? "loaded" : "FAILED");
		if (EmuState.ResetPending == 2)
		{
			HardResetMachine();
			EmuState.ResetPending = 0;
		}
	}

	std::printf("vcc-headless: rom=%s frames=%d cpu=%s ram=%s\n",
	            gHeadlessRomPath, frames,
	            EmuState.CpuType == 1 ? "HD6309" : "MC6809",
	            EmuState.RamSize == 0 ? "128K" : EmuState.RamSize == 1 ? "512K"
	          : EmuState.RamSize == 2 ? "2M" : "8M");
	if (type_text)
		std::printf("vcc-headless: will type %s at frame 150\n", type_text);

	const auto start = std::chrono::steady_clock::now();
	for (int i = 0; i < frames; ++i)
	{
		if (type_text && i == 150)
			HeadlessTypeText(type_text);
		HeadlessKeyboardTick();
		RenderFrame(&EmuState);
	}
	const auto end = std::chrono::steady_clock::now();

	const double wall = std::chrono::duration<double>(end - start).count();
	const double emulated = frames / 60.0;
	const double ratio = (wall > 0.0) ? emulated / wall : 0.0;

	std::printf("rendered %d frames (%.1fs emulated) in %.3fs wall: %.1fx realtime\n",
	            frames, emulated, wall, ratio);
	std::printf("effective CPU speed ~%.1f MHz (0.894 MHz machine)\n", 0.894 * ratio);

	if (EmuState.CpuType == 1)
	{
		char jitstats[512];
		HD6309GetBlockStatsText(jitstats, sizeof(jitstats));
		std::printf("%s\n", jitstats);
	}

	GetModuleStatus(&EmuState);
	std::printf("module status: %s\n", EmuState.StatusLine);

	DumpTextScreen();
	return 0;
}
