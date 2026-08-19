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
#pragma once

#include <string>

// Text conversion between the host's UTF-8 world and the CoCo.
//
// Paste direction: arbitrary clipboard UTF-8 becomes a stream of
// CoCo-typeable characters ('\n' meaning ENTER). When libutf is
// available (VCC_HAVE_LIBUTF), non-ASCII code points are perceptually
// approximated (e-acute -> e, smart quotes -> ", em dash -> -) by its
// compressed-DFA tr_ascii tables; otherwise non-ASCII is dropped.
//
// Copy direction: the CURRENT text screen - GIME 40/80-column hardware
// text or the VDG 32-column screen - rendered as UTF-8 lines with
// trailing blanks trimmed. VDG semigraphics (SG4) map to the Unicode
// quadrant-block characters, so a PMODE-less block drawing survives
// the trip to the host clipboard intact.

namespace VccShell
{
	// Returns the number of typeable characters produced.
	size_t Utf8ToCocoPaste(std::string& out, const char* utf8);

	// Empty string when no recognizable text screen is active.
	std::string ScreenToUtf8();
}
