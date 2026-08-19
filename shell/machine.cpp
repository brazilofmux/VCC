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

// See machine.h. The boot sequence mirrors DoHardReset() in Vcc.cpp.

#include "machine.h"
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
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

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

// ---- config.cpp replacements ----

// Config lives at $XDG_CONFIG_HOME/vcc/vcc.ini (~/.config/vcc/vcc.ini) -
// same file format as the Windows build, sensible host location.
void GetIniFilePath(char* path)
{
	const char* xdg = std::getenv("XDG_CONFIG_HOME");
	std::string dir = (xdg && *xdg) ? xdg
	                                : std::string(std::getenv("HOME") ? std::getenv("HOME") : ".") + "/.config";
	dir += "/vcc/vcc.ini";
	std::strncpy(path, dir.c_str(), MAX_PATH - 1);
	path[MAX_PATH - 1] = '\0';
}

VCC::Util::settings& Setting()
{
	static VCC::Util::settings* instance = [] {
		char path[MAX_PATH];
		GetIniFilePath(path);
		return new VCC::Util::settings(path);
	}();
	return *instance;
}

void GetExtRomPath(char* path)
{
	std::strncpy(path, VccShell::RomPathOverride, MAX_PATH - 1);
	path[MAX_PATH - 1] = '\0';
}

int GetPaletteType()
{
	return 1;   // RGB
}

// ---- Vcc.cpp shell functions the chipset calls ----

void SetCPUMultiplyerFlag(unsigned char double_speed)
{
	// Functional: the GIME's $FF91 speed poke must work in every shell.
	EmuState.DoubleSpeedFlag = double_speed;
	const int mult = (EmuState.OverclockFlag) ? EmuState.DoubleSpeedMultiplyer : 2;
	SetClockSpeed(1);
	if (EmuState.DoubleSpeedFlag)
		SetClockSpeed((unsigned short)(mult * EmuState.TurboSpeedFlag));
}

void SetTurboMode(unsigned char data)
{
	EmuState.TurboSpeedFlag = data + 1;
	if (EmuState.DoubleSpeedFlag)
		SetCPUMultiplyerFlag(EmuState.DoubleSpeedFlag);
}

namespace VccShell
{

char RomPathOverride[512] = "";
unsigned int Framebuffer[640 * 480];

void HardResetMachine()
{
	EmuState.RamBuffer = MmuInit(EmuState.RamSize);
	EmuState.WRamBuffer = (unsigned short*)EmuState.RamBuffer;
	if (EmuState.RamBuffer == nullptr)
		return;

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

bool BootMachine()
{
	EmuState.RamSize = (unsigned char)Setting().read("Memory", "RamSize", 1);
	EmuState.CpuType = (unsigned char)Setting().read("CPU", "CpuType", 1);
	EmuState.FrameSkip = 1;
	EmuState.EmulationRunning = 1;
	EmuState.BitDepth = 3;          // 32-bit XRGB8888 surface
	EmuState.PTRsurface32 = Framebuffer;
	EmuState.SurfacePitch = 640;

	HardResetMachine();
	if (EmuState.RamBuffer == nullptr)
		return false;

	// Load the boot cartridge the same way Vcc.cpp does at startup.
	char onboot[MAX_PATH] = "";
	Setting().read("Module", "OnBoot", "", onboot, MAX_PATH);
	if (onboot[0] != '\0')
	{
		const auto status = PakLoadCartridge(onboot);
		std::fprintf(stderr, "vcc-shell: OnBoot module %s -> %s\n", onboot,
		             status == VCC::Core::cartridge_loader_status::success
		                 ? "loaded" : "FAILED");
		if (EmuState.ResetPending == 2)
		{
			HardResetMachine();
			EmuState.ResetPending = 0;
		}
	}
	return true;
}

}
