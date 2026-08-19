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

// Device-absent stubs shared by every non-Windows shell: no cassette
// deck, no joysticks, no debugger UI, and a screen "lock" that always
// succeeds because the render surface is a plain static buffer.
// Keyboard and audio are per-shell (scripted/dropped in headless, real
// in the SDL shell) and live with their shells.

#include "defines.h"
#include <string>

// ---- keyboardEdit.cpp ----

void PasteIntoQueue(const std::string&)
{
}

// ---- Cassette.cpp ----

unsigned char TapeFastLoad = 0;

void Motor(unsigned char)
{
}

unsigned char GetMotorState()
{
	return 0;
}

unsigned int GetTapeRate()
{
	return 0;
}

void LoadCassetteBuffer(unsigned char*, unsigned int* size)
{
	if (size) *size = 0;
}

void FlushCassetteBuffer(const unsigned char*, unsigned int*)
{
}

// ---- DirectDrawInterface.cpp ----

unsigned char LockScreen()
{
	return 0;   // success; the shell's static framebuffer is the surface
}

void UnlockScreen(SystemState*)
{
}

void Cls(unsigned int, SystemState*)
{
}

POINT GetForcedAspectBorderPadding()
{
	return POINT{};
}

// ---- throttle.cpp ----

float CalculateFPS()
{
	return 0.0f;
}

// ---- Disassembler.cpp (debugger UI) ----

namespace VCC
{
	void ApplyHaltpoints(bool)
	{
	}

	void KillHaltpoints()
	{
	}
}
