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

// Headless driver (docs/porting-macos.md): boot the CoCo 3 core with no
// shell - no window, no audio, no throttle - run it flat out, then
// print the effective speed and an ASCII dump of the text screen.
// Machine bootstrap is shared with the SDL shell (shell/machine.cpp);
// the same vcc.ini drives both.
//
// Usage: vcc-headless [rom-path] [frames] [text-to-type]
//        (text is typed starting at frame 150; "\n" means ENTER and
//        "~" pauses one second)
// Env:   VCC_FRAMESKIP=N   render every Nth frame (CPU-bound benches)
//        VCC_NO_JIT / VCC_NO_INLINE / VCC_VERIFY_PURE / ... (see JIT)

#include "shell/machine.h"
#include "defines.h"
#include "tcc1014mmu.h"
#include "tcc1014graphics.h"
#include "coco3.h"
#include "hd6309.h"
#include "pakinterface.h"
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>

// Provided by headless/stubs.cpp.
void HeadlessTypeText(const char* text);
void HeadlessKeyboardTick();

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

int main(int argc, char** argv)
{
	// No ROM argument: leave the override empty so the shell falls back
	// to the ExternalBasicImage path in vcc.ini, same as vcc-sdl.
	if (argc > 1)
		std::snprintf(VccShell::RomPathOverride,
		              sizeof(VccShell::RomPathOverride), "%s", argv[1]);
	const int frames = (argc > 2) ? std::atoi(argv[2]) : 600;
	const char* type_text = (argc > 3) ? argv[3] : nullptr;

	if (!VccShell::BootMachine())
	{
		std::fprintf(stderr, "vcc-headless: machine boot failed\n");
		return 1;
	}

	// The GIME renderer dominates headless wall time at FrameSkip=1;
	// VCC_FRAMESKIP=N renders only every Nth frame (emulation is
	// unaffected) so CPU-tier benchmarks measure the CPU.
	if (const char* fs = std::getenv("VCC_FRAMESKIP"))
	{
		const int n = std::atoi(fs);
		if (n >= 1 && n <= 255)
			EmuState.FrameSkip = (unsigned char)n;
	}

	std::printf("vcc-headless: rom=%s frames=%d cpu=%s ram=%s\n",
	            VccShell::RomPathOverride[0] ? VccShell::RomPathOverride
	                                         : "(vcc.ini ExternalBasicImage)",
	            frames,
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

	// VCC_SHOT_FILE: save the rendered framebuffer as a BMP (32-bit
	// BGRX, top-down) - the renderer's XRGB8888 words are already in
	// BMP byte order. Visual regression hook, no SDL needed.
	if (const char* shot = std::getenv("VCC_SHOT_FILE"))
	{
		FILE* f = std::fopen(shot, "wb");
		if (f != nullptr)
		{
			const uint32_t data_size = 640 * 480 * 4;
			uint8_t hdr[54] = {};
			hdr[0] = 'B'; hdr[1] = 'M';
			const uint32_t file_size = 54 + data_size;
			std::memcpy(hdr + 2, &file_size, 4);
			const uint32_t data_off = 54;
			std::memcpy(hdr + 10, &data_off, 4);
			const uint32_t info_size = 40;
			std::memcpy(hdr + 14, &info_size, 4);
			const int32_t w = 640, h = -480;   // negative: top-down rows
			std::memcpy(hdr + 18, &w, 4);
			std::memcpy(hdr + 22, &h, 4);
			const uint16_t planes = 1, bpp = 32;
			std::memcpy(hdr + 26, &planes, 2);
			std::memcpy(hdr + 28, &bpp, 2);
			std::memcpy(hdr + 34, &data_size, 4);
			std::fwrite(hdr, 1, 54, f);
			std::fwrite(VccShell::Framebuffer, 1, data_size, f);
			std::fclose(f);
			std::printf("framebuffer saved to %s\n", shot);
		}
	}

	// VCC_DUMP_PIXELS: print a strip of framebuffer words so a shell
	// author can learn the renderer's channel order empirically.
	if (std::getenv("VCC_DUMP_PIXELS"))
	{
		std::printf("framebuffer row 100, cols 0-15:\n");
		for (int i = 0; i < 16; ++i)
			std::printf(" %08X", VccShell::Framebuffer[100 * 640 + i]);
		std::printf("\nframebuffer row 120, cols 300-315:\n");
		for (int i = 0; i < 16; ++i)
			std::printf(" %08X", VccShell::Framebuffer[120 * 640 + 300 + i]);
		std::printf("\n");
	}
	return 0;
}
