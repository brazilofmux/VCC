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

// SDL shell (docs/porting-macos.md stage "SDL shell"): a window on the
// GIME renderer's 640x480 XRGB8888 surface, a real keyboard mapped
// onto the CoCo matrix, SDL audio fed from the core's per-frame sample
// buffer, and a 60 fps pacer. Machine bootstrap is shared with the
// headless driver (shell/machine.cpp); the same vcc.ini drives both.
//
// Keys: host keyboard types naturally (character-level mapping onto
// the CoCo matrix, shift synthesized or suppressed as the CoCo layout
// requires. Backspace is the CoCo left-arrow, Esc is BREAK, Home is
// CLEAR, F1/F2 are the CoCo function keys). F8 toggles turbo (no
// throttle); Cmd+Q or closing the window quits.
//
// Usage: vcc-sdl [rom-path]

#define SDL_MAIN_HANDLED
#include <SDL.h>

#include "shell/machine.h"
#include "shell/text_codec.h"
#include "defines.h"
#include "coco3.h"
#include "pakinterface.h"
#include "joystickinput.h"
#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>
#include <unordered_map>

// ---------------------------------------------------------------------
// Keyboard: character-level host-to-matrix mapping.
//
// The CoCo matrix (rows PA0-PA6, columns PB0-PB7):
//   row 0: @ A B C D E F G      row 4: 0 1 2 3 4 5 6 7
//   row 1: H I J K L M N O      row 5: 8 9 : ; , - . /
//   row 2: P Q R S T U V W      row 6: ENT CLR BRK ALT CTL F1 F2 SHIFT
//   row 3: X Y Z UP DN LT RT SPACE
//
// A host keypress resolves to a matrix key plus a shift disposition:
// Pass (host shift flows through - letters), Force (CoCo needs shift -
// e.g. '!' is shift-1), Suppress (CoCo key is unshifted even though
// the host used shift to type it - e.g. ':' on a US Mac is shift-';'
// but a plain key on the CoCo).

namespace
{

enum class ShiftMode : uint8_t { Pass, Force, Suppress };

struct CocoKey
{
	uint8_t   col;
	uint8_t   row;
	ShiftMode shift;
};

// Pressed host keys and the matrix keys they activated.
std::unordered_map<SDL_Keycode, CocoKey> gPressed;
int gHostShiftCount = 0;

bool CocoKeyForChar(char ch, CocoKey& out);

// Cmd+V paste: clipboard text (normalized by shell/text_codec) typed
// synthetically, one character per few frames - held long enough for
// the ROM's 60Hz keyboard scan, with a gap so repeated characters
// register as distinct presses.
std::string gPasteQueue;
size_t      gPasteIndex = 0;
int         gPastePhase = 0;
constexpr int kPasteHold = 2;
constexpr int kPasteGap  = 1;

void PasteTick()
{
	if (gPasteIndex >= gPasteQueue.size())
		return;
	if (++gPastePhase >= kPasteHold + kPasteGap)
	{
		gPastePhase = 0;
		++gPasteIndex;
		if (gPasteIndex >= gPasteQueue.size())
		{
			gPasteQueue.clear();
			gPasteIndex = 0;
		}
	}
}

// The paste character currently "held down", if any.
bool PasteActiveKey(CocoKey& out)
{
	if (gPasteIndex >= gPasteQueue.size() || gPastePhase >= kPasteHold)
		return false;
	const char c = gPasteQueue[gPasteIndex];
	if (c == '\n')
	{
		out = { 0, 6, ShiftMode::Suppress };   // ENTER
		return true;
	}
	return CocoKeyForChar(c, out);
}

bool CocoKeyForChar(char ch, CocoKey& out)
{
	if (ch >= 'a' && ch <= 'z')
		ch = (char)(ch - 'a' + 'A');
	if (ch >= '@' && ch <= 'Z')
	{
		const uint8_t i = (uint8_t)(ch - '@');
		out = { (uint8_t)(i & 7), (uint8_t)(i >> 3),
		        ch == '@' ? ShiftMode::Suppress : ShiftMode::Pass };
		return true;
	}
	if (ch >= '0' && ch <= '7') { out = { (uint8_t)(ch - '0'), 4, ShiftMode::Suppress }; return true; }
	if (ch == '8' || ch == '9') { out = { (uint8_t)(ch - '8'), 5, ShiftMode::Suppress }; return true; }
	switch (ch)
	{
	case ' ': out = { 7, 3, ShiftMode::Pass };     return true;
	case ':': out = { 2, 5, ShiftMode::Suppress }; return true;
	case ';': out = { 3, 5, ShiftMode::Suppress }; return true;
	case ',': out = { 4, 5, ShiftMode::Suppress }; return true;
	case '-': out = { 5, 5, ShiftMode::Suppress }; return true;
	case '.': out = { 6, 5, ShiftMode::Suppress }; return true;
	case '/': out = { 7, 5, ShiftMode::Suppress }; return true;
	// Shifted CoCo keys.
	case '!':  out = { 1, 4, ShiftMode::Force }; return true;
	case '"':  out = { 2, 4, ShiftMode::Force }; return true;
	case '#':  out = { 3, 4, ShiftMode::Force }; return true;
	case '$':  out = { 4, 4, ShiftMode::Force }; return true;
	case '%':  out = { 5, 4, ShiftMode::Force }; return true;
	case '&':  out = { 6, 4, ShiftMode::Force }; return true;
	case '\'': out = { 7, 4, ShiftMode::Force }; return true;
	case '(':  out = { 0, 5, ShiftMode::Force }; return true;
	case ')':  out = { 1, 5, ShiftMode::Force }; return true;
	case '*':  out = { 2, 5, ShiftMode::Force }; return true;
	case '+':  out = { 3, 5, ShiftMode::Force }; return true;
	case '<':  out = { 4, 5, ShiftMode::Force }; return true;
	case '=':  out = { 5, 5, ShiftMode::Force }; return true;
	case '>':  out = { 6, 5, ShiftMode::Force }; return true;
	case '?':  out = { 7, 5, ShiftMode::Force }; return true;
	}
	return false;
}

// US-layout shifted variant of an unshifted SDL keycode character.
char ShiftedChar(char c)
{
	if (c >= 'a' && c <= 'z')
		return c;   // letters handled by shift passthrough
	static const char* from = "1234567890-=[]\\;',./`";
	static const char* to   = "!@#$%^&*()_+{}|:\"<>?~";
	const char* p = std::strchr(from, c);
	return p ? to[p - from] : c;
}

bool CocoKeyForEvent(const SDL_Keysym& sym, CocoKey& out)
{
	// Non-character keys by scancode.
	switch (sym.scancode)
	{
	case SDL_SCANCODE_RETURN:
	case SDL_SCANCODE_KP_ENTER: out = { 0, 6, ShiftMode::Pass }; return true;
	case SDL_SCANCODE_HOME:     out = { 1, 6, ShiftMode::Pass }; return true;   // CLEAR
	case SDL_SCANCODE_ESCAPE:   out = { 2, 6, ShiftMode::Pass }; return true;   // BREAK
	case SDL_SCANCODE_UP:       out = { 3, 3, ShiftMode::Pass }; return true;
	case SDL_SCANCODE_DOWN:     out = { 4, 3, ShiftMode::Pass }; return true;
	case SDL_SCANCODE_LEFT:
	case SDL_SCANCODE_BACKSPACE:out = { 5, 3, ShiftMode::Pass }; return true;   // CoCo left arrow
	case SDL_SCANCODE_RIGHT:    out = { 6, 3, ShiftMode::Pass }; return true;
	case SDL_SCANCODE_LALT:
	case SDL_SCANCODE_RALT:     out = { 3, 6, ShiftMode::Pass }; return true;   // ALT
	case SDL_SCANCODE_LCTRL:
	case SDL_SCANCODE_RCTRL:    out = { 4, 6, ShiftMode::Pass }; return true;   // CTRL
	case SDL_SCANCODE_F1:       out = { 5, 6, ShiftMode::Pass }; return true;
	case SDL_SCANCODE_F2:       out = { 6, 6, ShiftMode::Pass }; return true;
	default:
		break;
	}

	// Printable characters: SDL keycodes are the unshifted US-layout
	// ASCII; apply the host shift to learn what the user meant to type,
	// then map that character onto the CoCo matrix.
	if (sym.sym >= 32 && sym.sym < 127)
	{
		char c = (char)sym.sym;
		if (sym.mod & KMOD_SHIFT)
			c = ShiftedChar(c);
		return CocoKeyForChar(c, out);
	}
	return false;
}

}   // namespace

// The PIA keyboard scan, fed from the pressed-key map. Called from the
// emulation (same thread as the SDL event pump) - no locking needed.
extern "C" unsigned char vccKeyboardGetScan(unsigned char Col)
{
	uint8_t rollover[8] = {};
	bool force = false, suppress = false;
	for (const auto& [keycode, key] : gPressed)
	{
		rollover[key.col] |= (uint8_t)(1u << key.row);
		if (key.shift == ShiftMode::Force)    force = true;
		if (key.shift == ShiftMode::Suppress) suppress = true;
	}
	{
		CocoKey pk;
		if (PasteActiveKey(pk))
		{
			rollover[pk.col] |= (uint8_t)(1u << pk.row);
			if (pk.shift == ShiftMode::Force)    force = true;
			if (pk.shift == ShiftMode::Suppress) suppress = true;
		}
	}
	const bool shift = force || (gHostShiftCount > 0 && !suppress);
	if (shift)
		rollover[7] |= 1u << 6;   // SHIFT key: col 7, row 6

	uint8_t rows = 0;
	const uint8_t active = (uint8_t)~Col;
	for (int col = 0; col < 8; ++col)
	{
		if (active & (1u << col))
			rows |= rollover[col];
	}
	// Rows are active-low. The joystick layer adds the comparator
	// (bit 7) and clears button bits, exactly like the Windows
	// keyboard.cpp merge.
	return vccJoystickGetScan((uint8_t)(~rows & 0x7F));
}

// ---------------------------------------------------------------------
// Audio: the core hands FlushAudioBuffer one frame of packed 16:16
// stereo samples at 44.1 kHz; queue them to SDL. GetFreeBlockCount
// reports how far the queue is below its target so the core's
// stretcher can top it up, matching the DirectSound block semantics.

namespace
{
	SDL_AudioDeviceID gAudioDev = 0;
	constexpr uint32_t kAudioRate = 44100;
	constexpr uint32_t kBlockBytes = (kAudioRate / 60) * 4;   // one frame
	constexpr uint32_t kTargetBlocks = 4;
}

void FlushAudioBuffer(unsigned int* buffer, unsigned int bytes)
{
	if (gAudioDev == 0 || buffer == nullptr || bytes == 0)
		return;
	// Cap the queue (turbo mode outruns the DAC); a quarter second of
	// backlog is the drop threshold.
	if (SDL_GetQueuedAudioSize(gAudioDev) > kAudioRate)
		return;
	SDL_QueueAudio(gAudioDev, buffer, bytes);
}

int GetFreeBlockCount()
{
	if (gAudioDev == 0)
		return 1;
	const uint32_t queued = SDL_GetQueuedAudioSize(gAudioDev);
	const uint32_t target = kBlockBytes * kTargetBlocks;
	if (queued >= target)
		return 0;
	return (int)((target - queued) / kBlockBytes);
}

void ResetAudio()
{
	if (gAudioDev != 0)
		SDL_ClearQueuedAudio(gAudioDev);
}

unsigned char PauseAudio(unsigned char pause)
{
	if (gAudioDev != 0)
		SDL_PauseAudioDevice(gAudioDev, pause ? 1 : 0);
	return pause;
}

// ---------------------------------------------------------------------

int main(int argc, char** argv)
{
	SDL_SetMainReady();

	if (argc > 1)
		std::snprintf(VccShell::RomPathOverride,
		              sizeof(VccShell::RomPathOverride), "%s", argv[1]);

	if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO) != 0)
	{
		std::fprintf(stderr, "vcc-sdl: SDL_Init failed: %s\n", SDL_GetError());
		return 1;
	}

	SDL_Window* window = SDL_CreateWindow(
		"VCC",
		SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
		1280, 960,
		SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
	if (window == nullptr)
	{
		std::fprintf(stderr, "vcc-sdl: window: %s\n", SDL_GetError());
		return 1;
	}

	SDL_Renderer* renderer = SDL_CreateRenderer(
		window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
	if (renderer == nullptr)
		renderer = SDL_CreateRenderer(window, -1, 0);
	SDL_RenderSetLogicalSize(renderer, 640, 480);

	SDL_Texture* texture = SDL_CreateTexture(
		renderer, SDL_PIXELFORMAT_XRGB8888,
		SDL_TEXTUREACCESS_STREAMING, 640, 480);

	SDL_AudioSpec want {}, have {};
	want.freq = kAudioRate;
	want.format = AUDIO_S16SYS;
	want.channels = 2;
	want.samples = 1024;
	gAudioDev = SDL_OpenAudioDevice(nullptr, 0, &want, &have, 0);
	if (gAudioDev != 0)
		SDL_PauseAudioDevice(gAudioDev, 0);
	else
		std::fprintf(stderr, "vcc-sdl: audio unavailable: %s\n", SDL_GetError());

	if (!VccShell::BootMachine())
	{
		std::fprintf(stderr, "vcc-sdl: machine boot failed\n");
		return 1;
	}
	if (gAudioDev != 0)
		SetAudioRate(kAudioRate);

	using clock = std::chrono::steady_clock;
	constexpr auto kFramePeriod = std::chrono::nanoseconds(1000000000ull / 60);
	auto deadline = clock::now() + kFramePeriod;

	bool running = true;
	bool turbo = false;
	int frames = 0;
	auto fps_mark = clock::now();
	int fps_frames = 0;

	// Self-screenshot: VCC_SHOT_FILE=<path.bmp> saves the emulator
	// surface at frame VCC_SHOT_FRAME (default 300). F12 saves
	// vcc-shot.bmp in the working directory any time. Reads the
	// machine's own 640x480 surface, so no OS capture permission is
	// involved - which also makes it the automated visual test hook.
	const char* shot_file = std::getenv("VCC_SHOT_FILE");
	const int shot_frame = std::getenv("VCC_SHOT_FRAME")
		? std::atoi(std::getenv("VCC_SHOT_FRAME")) : 300;
	auto save_shot = [](const char* path) {
		SDL_Surface* s = SDL_CreateRGBSurfaceWithFormatFrom(
			VccShell::Framebuffer, 640, 480, 32, 640 * 4,
			SDL_PIXELFORMAT_XRGB8888);
		if (s != nullptr)
		{
			SDL_SaveBMP(s, path);
			SDL_FreeSurface(s);
		}
	};

	while (running)
	{
		SDL_Event ev;
		while (SDL_PollEvent(&ev))
		{
			switch (ev.type)
			{
			case SDL_QUIT:
				running = false;
				break;

			case SDL_KEYDOWN:
				if (ev.key.repeat)
					break;
				if (ev.key.keysym.mod & KMOD_GUI)
				{
					// Host shortcuts; Cmd-modified keys never reach the
					// CoCo matrix.
					if (ev.key.keysym.sym == SDLK_q)
						running = false;
					else if (ev.key.keysym.sym == SDLK_v)
					{
						if (char* clip = SDL_GetClipboardText())
						{
							std::string queue;
							VccShell::Utf8ToCocoPaste(queue, clip);
							SDL_free(clip);
							gPasteQueue = std::move(queue);
							gPasteIndex = 0;
							gPastePhase = 0;
						}
					}
					else if (ev.key.keysym.sym == SDLK_c)
					{
						const std::string text = VccShell::ScreenToUtf8();
						if (!text.empty())
							SDL_SetClipboardText(text.c_str());
					}
					break;
				}
				if (ev.key.keysym.scancode == SDL_SCANCODE_F12)
				{
					save_shot("vcc-shot.bmp");
					break;
				}
				if (ev.key.keysym.scancode == SDL_SCANCODE_F8)
				{
					turbo = !turbo;
					if (turbo)
						ResetAudio();
					deadline = clock::now() + kFramePeriod;
					break;
				}
				if (ev.key.keysym.scancode == SDL_SCANCODE_LSHIFT ||
				    ev.key.keysym.scancode == SDL_SCANCODE_RSHIFT)
				{
					++gHostShiftCount;
					break;
				}
				{
					CocoKey key;
					if (CocoKeyForEvent(ev.key.keysym, key))
						gPressed[ev.key.keysym.sym] = key;
				}
				break;

			case SDL_MOUSEMOTION:
			{
				int w = 0, h = 0;
				SDL_GetWindowSize(window, &w, &h);
				if (w > 1 && h > 1)
				{
					unsigned int jx = (unsigned int)((ev.motion.x * 16383) / (w - 1));
					unsigned int jy = (unsigned int)((ev.motion.y * 16383) / (h - 1));
					joystick(jx, jy);
				}
				break;
			}

			case SDL_MOUSEBUTTONDOWN:
			case SDL_MOUSEBUTTONUP:
			{
				const unsigned char state =
					(ev.type == SDL_MOUSEBUTTONDOWN) ? 1 : 0;
				if (ev.button.button == SDL_BUTTON_LEFT)
					SetButtonStatus(0, state);
				else if (ev.button.button == SDL_BUTTON_RIGHT)
					SetButtonStatus(1, state);
				break;
			}

			case SDL_KEYUP:
				if (ev.key.keysym.scancode == SDL_SCANCODE_LSHIFT ||
				    ev.key.keysym.scancode == SDL_SCANCODE_RSHIFT)
				{
					if (gHostShiftCount > 0)
						--gHostShiftCount;
					break;
				}
				gPressed.erase(ev.key.keysym.sym);
				break;

			default:
				break;
			}
		}

		PasteTick();
		RenderFrame(&EmuState);
		++frames;
		++fps_frames;
		if (shot_file != nullptr && frames == shot_frame)
			save_shot(shot_file);

		SDL_UpdateTexture(texture, nullptr, VccShell::Framebuffer, 640 * 4);
		SDL_RenderClear(renderer);
		SDL_RenderCopy(renderer, texture, nullptr, nullptr);
		SDL_RenderPresent(renderer);

		// Title: fps + machine status, once a second.
		const auto now = clock::now();
		if (now - fps_mark >= std::chrono::seconds(1))
		{
			GetModuleStatus(&EmuState);
			char title[320];
			std::snprintf(title, sizeof(title), "VCC  |  %d fps%s  |  %s",
			              fps_frames, turbo ? " (turbo)" : "",
			              EmuState.StatusLine);
			SDL_SetWindowTitle(window, title);
			fps_mark = now;
			fps_frames = 0;
		}

		// 60 fps pacing; turbo runs flat out.
		if (!turbo)
		{
			if (now < deadline)
				std::this_thread::sleep_until(deadline);
			deadline += kFramePeriod;
			if (clock::now() > deadline + std::chrono::milliseconds(100))
				deadline = clock::now() + kFramePeriod;   // resync after stalls
		}
	}

	if (gAudioDev != 0)
		SDL_CloseAudioDevice(gAudioDev);
	SDL_DestroyTexture(texture);
	SDL_DestroyRenderer(renderer);
	SDL_DestroyWindow(window);
	SDL_Quit();
	return 0;
}
