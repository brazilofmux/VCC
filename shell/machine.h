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

// Shared machine bootstrap for the non-Windows shells (headless, SDL).
// Owns the globals Vcc.cpp provides on Windows (EmuState, the CPU
// function pointers) plus the boot sequence: configuration comes from
// $XDG_CONFIG_HOME/vcc/vcc.ini exactly as the Windows build reads its
// own ini, including the [Module] OnBoot cartridge.

namespace VccShell
{
	// Optional override for the internal ROM path (GetExtRomPath). When
	// empty, the ini's ExternalBasicImage / <exedir>/coco3.rom rules
	// apply.
	extern char RomPathOverride[512];

	// The 640x480 XRGB8888 surface the GIME renderer draws into.
	// Wired into EmuState by InitMachine.
	extern unsigned int Framebuffer[640 * 480];

	// Read RamSize/CpuType from the ini, allocate RAM, install the CPU,
	// and reset the machine (DoHardReset minus the Win32 UI).
	void HardResetMachine();

	// Full cold boot: InitMachine state, hard reset, then load the
	// [Module] OnBoot cartridge and honor the reset it requests.
	// Returns false if RAM allocation failed.
	bool BootMachine();
}
