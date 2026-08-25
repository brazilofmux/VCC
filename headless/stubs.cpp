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

// Headless-specific shell pieces: a scripted CoCo keyboard (the smoke
// tests type through it) and an audio sink that discards samples. The
// shared machine bootstrap and device-absent stubs live in shell/.

#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <string>

int HD6309LiveCycles();

// ---- keyboard: scripted matrix ----
//
// The driver queues text with HeadlessTypeText and ticks once per
// frame; the PIA scan sees each key held for a few frames with a gap
// before the next, which is plenty for the ROM's debounce.

namespace
{
	struct MatrixKey
	{
		unsigned char col;
		unsigned char row;
		bool          shift;
		bool          ctrl;
	};

	// Standard CoCo keyboard matrix: rows PA0-PA6, columns PB0-PB7.
	// Shifted characters (quotes especially - LOADM needs them) are
	// synthesized by holding SHIFT (col 7, row 6) with the base key.
	bool KeyForChar(char ch, MatrixKey& key)
	{
		if (ch >= 'a' && ch <= 'z')
			ch = (char)(ch - 'a' + 'A');
		if (ch == '\r' || ch == '\n') { key = {0, 6, false}; return true; }   // ENTER
		if (ch == ' ')                { key = {7, 3, false}; return true; }
		if (ch >= '@' && ch <= 'Z')
		{
			const unsigned char i = (unsigned char)(ch - '@');
			key = {(unsigned char)(i & 7), (unsigned char)(i >> 3), false};
			return true;
		}
		if (ch >= '0' && ch <= '7') { key = {(unsigned char)(ch - '0'), 4, false}; return true; }
		if (ch == '8' || ch == '9') { key = {(unsigned char)(ch - '8'), 5, false}; return true; }
		switch (ch)
		{
		case ':': key = {2, 5, false}; return true;
		case ';': key = {3, 5, false}; return true;
		case ',': key = {4, 5, false}; return true;
		case '-': key = {5, 5, false}; return true;
		case '.': key = {6, 5, false}; return true;
		case '/': key = {7, 5, false}; return true;
		// Shifted number row: !"#$%&'()
		case '!':  key = {1, 4, true}; return true;
		case '"':  key = {2, 4, true}; return true;
		case '#':  key = {3, 4, true}; return true;
		case '$':  key = {4, 4, true}; return true;
		case '%':  key = {5, 4, true}; return true;
		case '&':  key = {6, 4, true}; return true;
		case '\'': key = {7, 4, true}; return true;
		case '(':  key = {0, 5, true}; return true;
		case ')':  key = {1, 5, true}; return true;
		case '=':  key = {5, 5, true}; return true;
		case '+':  key = {3, 5, true}; return true;
		case '*':  key = {2, 5, true}; return true;
		case '?':  key = {7, 5, true}; return true;
		case '<':  key = {4, 5, true}; return true;
		case '>':  key = {6, 5, true}; return true;
		// NitrOS-9's CoCo 3 keydrv reaches the missing ASCII through
		// CTRL combos; underscore (OS-9 filenames use it) is CTRL+'-'.
		case '_':  key = {5, 5, false, true}; return true;
		}
		return false;   // unsupported character: skipped
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
	// Diagnostic (VCC_LOG_SCAN): one line per matrix scan with the
	// live cycle position and feeder state - diffing these between
	// CPU tiers pinpoints guest keyboard-sampling divergence (this is
	// what caught the stale-latch phantom IRQ under scanline bursts).
	static const bool log_scan = getenv("VCC_LOG_SCAN") != nullptr;
	if (log_scan)
	{
		static long n = 0;
		std::fprintf(stderr, "[SCAN %ld] col=%02X cyc=%d idx=%zu ph=%d\n",
		             n++, Col, HD6309LiveCycles(),
		             gTypeIndex, gTypePhase);
	}
	unsigned char rows = 0;
	if (gTypeIndex < gTypeText.size() && gTypePhase < kHoldFrames)
	{
		MatrixKey key;
		if (KeyForChar(gTypeText[gTypeIndex], key))
		{
			const unsigned char active = (unsigned char)~Col;
			if (active & (unsigned char)(1u << key.col))
				rows = (unsigned char)(1u << key.row);
			// SHIFT (col 7) and CTRL (col 4) live on row 6; either is
			// held together with the base key when synthesized.
			if (key.shift && (active & 0x80u))
				rows |= (unsigned char)(1u << 6);
			if (key.ctrl && (active & 0x10u))
				rows |= (unsigned char)(1u << 6);
		}
	}
	// Rows are active-low; the joystick layer supplies the comparator
	// bit (sticks idle mid-range) and button bits.
	extern unsigned char vccJoystickGetScan(unsigned char);
	return vccJoystickGetScan((unsigned char)(~rows & 0x7F));
}

// ---- audio: discard ----

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
