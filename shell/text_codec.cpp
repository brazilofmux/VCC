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

// See text_codec.h. The paste normalization leans on libutf's
// compressed-DFA ASCII approximation when built with it; the screen
// export handles the GIME hardware text modes and the VDG 32-column
// screen including SG4 semigraphics.

#include "shell/text_codec.h"
#include "defines.h"
#include "tcc1014graphics.h"
#include "tcc1014mmu.h"
#include <cstring>
#include <vector>

#ifdef VCC_HAVE_LIBUTF
extern "C" {
#include <utf/utf_types.h>
#include <utf/color_ops.h>
}
#endif

namespace VccShell
{

size_t Utf8ToCocoPaste(std::string& out, const char* utf8)
{
	out.clear();
	if (utf8 == nullptr)
		return 0;
	const size_t len = std::strlen(utf8);

	// Stage 1: UTF-8 down to ASCII.
	std::vector<unsigned char> ascii;
#ifdef VCC_HAVE_LIBUTF
	ascii.resize(len + 1);
	const size_t n = co_render_ascii(ascii.data(),
	                                 (const unsigned char*)utf8, len);
	ascii.resize(n);
#else
	ascii.reserve(len);
	for (size_t i = 0; i < len; ++i)
	{
		const unsigned char c = (unsigned char)utf8[i];
		if (c < 0x80)
			ascii.push_back(c);
		else if (c >= 0xC0)
			ascii.push_back('?');   // lead byte: one '?' per code point
	}
#endif

	// Stage 2: ASCII down to what the CoCo keyboard can type. Line
	// endings normalize to '\n' (ENTER); tabs become spaces; the rest
	// of the control range drops.
	out.reserve(ascii.size());
	for (size_t i = 0; i < ascii.size(); ++i)
	{
		unsigned char c = ascii[i];
		if (c == '\r')
		{
			if (i + 1 < ascii.size() && ascii[i + 1] == '\n')
				++i;
			c = '\n';
		}
		else if (c == '\t')
			c = ' ';
		if (c == '\n' || (c >= 32 && c <= 126))
			out.push_back((char)c);
	}
	return out.size();
}

namespace
{
	// SG4 semigraphics: luminance bits L3..L0 are upper-left,
	// upper-right, lower-left, lower-right quadrants. Unicode's
	// quadrant blocks cover all 16 patterns exactly.
	const char* const kQuadrant[16] = {
		" ",        // 0000
		"▗",   // 0001 lower right
		"▖",   // 0010 lower left
		"▄",   // 0011 lower half
		"▝",   // 0100 upper right
		"▐",   // 0101 right half
		"▞",   // 0110 diagonal
		"▟",   // 0111 all but upper left
		"▘",   // 1000 upper left
		"▚",   // 1001 diagonal
		"▌",   // 1010 left half
		"▙",   // 1011 all but upper right
		"▀",   // 1100 upper half
		"▜",   // 1101 all but lower left
		"▛",   // 1110 all but lower right
		"█",   // 1111 full block
	};

	void AppendTrimmed(std::string& out, const std::string& line)
	{
		size_t end = line.size();
		while (end > 0 && line[end - 1] == ' ')
			--end;
		out.append(line, 0, end);
		out.push_back('\n');
	}
}

std::string ScreenToUtf8()
{
	std::string out;

	const int bpr = GetBytesPerRow();
	if (bpr == 40 || bpr == 80)
	{
		// GIME hardware text: (character, attribute) byte pairs with
		// near-ASCII character codes. Same walk as the headless dump.
		const unsigned int start = GetStartOfVidram();
		for (int row = 0; row < 24; ++row)
		{
			std::string line;
			for (int col = 0; col < bpr; ++col)
			{
				const unsigned char c = (unsigned char)GetMem(
					start + (unsigned long)row * bpr * 2 + col * 2);
				line.push_back((c >= 32 && c <= 126) ? (char)c : ' ');
			}
			AppendTrimmed(out, line);
		}
	}
	else
	{
		// VDG 32-column screen at $0400: 0x00-0x3F inverse text,
		// 0x40-0x7F normal text, 0x80-0xFF SG4 semigraphics.
		for (int row = 0; row < 16; ++row)
		{
			std::string line;
			for (int col = 0; col < 32; ++col)
			{
				const unsigned char c = SafeMemRead8(
					(unsigned short)(0x0400 + row * 32 + col));
				if (c & 0x80)
				{
					line.append(kQuadrant[c & 0x0F]);
				}
				else
				{
					const unsigned char v = c & 0x3F;
					const char ch = (v < 32) ? (char)(v + 64) : (char)v;
					line.push_back((ch < 32 || ch > 126) ? ' ' : ch);
				}
			}
			AppendTrimmed(out, line);
		}
	}

	// Trim trailing blank lines.
	while (out.size() >= 2 && out[out.size() - 1] == '\n' &&
	       out[out.size() - 2] == '\n')
		out.pop_back();
	return out;
}

}   // namespace VccShell
