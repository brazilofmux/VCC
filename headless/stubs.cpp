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

// Shell stubs for the headless driver: everything the portable core
// links against that lives in the Win32 shell (Vcc.cpp, config.cpp,
// keyboard, joystick, cassette, audio, pakinterface, DirectDraw,
// throttle) or the debugger UI. Each stub is the "device absent"
// behavior: no keys down, no cassette, no cartridge, screen lock always
// succeeds. The SDL shell replaces this file function by function.

#include "defines.h"
#include "MachineDefs.h"
#include "coco3.h"
#include <cstring>
#include <string>

extern char gHeadlessRomPath[512];   // headless/main.cpp

// ---- config.cpp ----

void GetExtRomPath(char* path)
{
	std::strncpy(path, gHeadlessRomPath, MAX_PATH - 1);
	path[MAX_PATH - 1] = '\0';
}

int GetPaletteType()
{
	return 1;   // RGB
}

// ---- Vcc.cpp shell ----

void SetCPUMultiplyerFlag(unsigned char double_speed)
{
	// Functional: the GIME's $FF91 speed poke must still work headless.
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

// ---- keyboard.cpp / keyboardEdit.cpp ----

extern "C" unsigned char vccKeyboardGetScan(unsigned char)
{
	return 0xFF;   // no keys down (active-low rows)
}

void PasteIntoQueue(const std::string&)
{
}

// ---- joystickinput.cpp ----

int JS_Ramp_Clock = 0;

extern "C" void vccJoystickStartTandy(unsigned char)
{
}

extern "C" void vccJoystickStartCCMax()
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

// ---- Audio.cpp ----

void FlushAudioBuffer(unsigned int*, unsigned int)
{
}

int GetFreeBlockCount()
{
	return 1;   // "buffer exactly full": no expansion samples inserted
}

void ResetAudio()
{
}

unsigned char PauseAudio(unsigned char)
{
	return 0;
}

// ---- pakinterface.cpp (no cartridge inserted) ----

void PakTimer()
{
}

unsigned char PakReadPort(unsigned char)
{
	return 0;
}

void PakWritePort(unsigned char, unsigned char)
{
}

unsigned char PackMem8Read(unsigned short)
{
	return 0;
}

unsigned short PackAudioSample()
{
	return 0;
}

void ResetBus()
{
}

void UpdateBusPointer()
{
}

// ---- DirectDrawInterface.cpp ----

unsigned char LockScreen()
{
	return 0;   // success; the driver's static framebuffer is the surface
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

// ---- FileOps.cpp ----

BOOL PathRemoveFileSpec(char* path)
{
	char* last = nullptr;
	for (char* p = path; *p; ++p)
	{
		if (*p == '/' || *p == '\\')
			last = p;
	}
	if (last == nullptr)
		return FALSE;
	*last = '\0';
	return TRUE;
}
