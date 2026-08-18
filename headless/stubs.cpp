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
#include <vcc/util/settings.h>
#include <cstdlib>
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
//
// Scripted CoCo keyboard matrix in place of the Win32 scan-code
// translator: the driver queues text with HeadlessTypeText and ticks
// once per frame; the PIA scan sees each key held for a few frames
// with a gap before the next, which is plenty for the ROM's debounce.

namespace
{
	struct MatrixKey
	{
		unsigned char col;
		unsigned char row;
	};

	// Standard CoCo keyboard matrix: rows PA0-PA6, columns PB0-PB7.
	bool KeyForChar(char ch, MatrixKey& key)
	{
		if (ch >= 'a' && ch <= 'z')
			ch = (char)(ch - 'a' + 'A');
		if (ch == '\r' || ch == '\n') { key = {0, 6}; return true; }   // ENTER
		if (ch == ' ')                { key = {7, 3}; return true; }
		if (ch >= '@' && ch <= 'Z')
		{
			const unsigned char i = (unsigned char)(ch - '@');
			key = {(unsigned char)(i & 7), (unsigned char)(i >> 3)};
			return true;
		}
		if (ch >= '0' && ch <= '7') { key = {(unsigned char)(ch - '0'), 4}; return true; }
		if (ch == '8' || ch == '9') { key = {(unsigned char)(ch - '8'), 5}; return true; }
		switch (ch)
		{
		case ':': key = {2, 5}; return true;
		case ';': key = {3, 5}; return true;
		case ',': key = {4, 5}; return true;
		case '-': key = {5, 5}; return true;
		case '.': key = {6, 5}; return true;
		case '/': key = {7, 5}; return true;
		}
		return false;   // unsupported (shifted) character: skipped
	}

	std::string   gTypeText;
	size_t        gTypeIndex = 0;
	int           gTypePhase = 0;
	constexpr int kHoldFrames = 4;
	constexpr int kGapFrames  = 4;
}

void HeadlessTypeText(const char* text)
{
	gTypeText  = text ? text : "";
	gTypeIndex = 0;
	gTypePhase = 0;
}

void HeadlessKeyboardTick()
{
	if (gTypeIndex >= gTypeText.size())
		return;
	// '~' in the script is a one-second pause (60 frames), not a key.
	const int frames_for_char =
		(gTypeText[gTypeIndex] == '~') ? 60 : kHoldFrames + kGapFrames;
	if (++gTypePhase >= frames_for_char)
	{
		gTypePhase = 0;
		++gTypeIndex;
	}
}

extern "C" unsigned char vccKeyboardGetScan(unsigned char Col)
{
	unsigned char rows = 0;
	if (gTypeIndex < gTypeText.size() && gTypePhase < kHoldFrames)
	{
		MatrixKey key;
		if (KeyForChar(gTypeText[gTypeIndex], key))
		{
			const unsigned char active = (unsigned char)~Col;
			if (active & (unsigned char)(1u << key.col))
				rows = (unsigned char)(1u << key.row);
		}
	}
	// Rows are active-low; bit 7 (joystick comparator) idles high.
	return (unsigned char)((~rows & 0x7F) | 0x80);
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

