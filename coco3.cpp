/*
Copyright 2015 by Joseph Forgione
This file is part of VCC (Virtual Color Computer).

    VCC (Virtual Color Computer) is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    VCC (Virtual Color Computer) is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with VCC (Virtual Color Computer).  If not, see <http://www.gnu.org/licenses/>.
*/

#include <vcc/util/host_services.h>
#include "stdio.h"
#include <math.h>
#include "defines.h"
#include "BuildConfig.h"
#include "tcc1014graphics.h"
#include "tcc1014registers.h"
#include "mc6821.h"
#include "hd6309.h"
#include "mc6809.h"
#include "pakinterface.h"
#include "audio.h"
#include "coco3.h"
#include "throttle.h"
#include "Vcc.h"
#include "Cassette.h"
#include "DirectDrawInterface.h"
#include <vcc/util/logger.h>
#include "keyboard.h"
#include <string>
#include <iostream>
#include "config.h"
#include "tcc1014mmu.h"
#include "EventHeap.h"


#if USE_DEBUG_AUDIOTAPE
#include "IDisplayDebug.h"
const int AudioHistorySize = 900;
struct AudioHistory
{
	char motorState;
	char audioState;
	int inputMin;
	int inputMax;
};
AudioHistory gAudioHistory[AudioHistorySize];
int gAudioHistoryCount = 0;
#endif

constexpr auto RENDERS_PER_BLINK_TOGGLE = 16u;

//****************************************
	static double SoundInterupt=0;
	static double NanosToSoundSample=SoundInterupt;
	static double NanosToAudioSample = SoundInterupt;
	static double CyclesPerSecord=(COLORBURST/4)*(TARGETFRAMERATE/FRAMESPERSECORD);
	static double LinesPerSecond= TARGETFRAMERATE * LINESPERSCREEN;
	static double NanosPerLine = NANOSECOND / LinesPerSecond;
	static double HSYNCWidthInNanos = 5000;
	static double CyclesPerLine = CyclesPerSecord / LinesPerSecond;
	// Nanos->cycles conversion factor for the CPUCycle slice loop. Only
	// SetClockSpeed changes any input, so it refreshes this cache; the
	// slice loop then pays one multiply instead of two multiplies and a
	// divide per slice.
	static double CyclesPerNano = CyclesPerLine / NanosPerLine;
	static double CycleDrift=0;
	static double CyclesThisLine=0;
	static unsigned int StateSwitch=0;
	unsigned int SoundRate=0;
//*****************************************************
static int MasterTimer=0; 
static unsigned int TimerClockRate=0;
static int TimerCycleCount=0;
static double MasterTickCounter = 0;
static unsigned int UnxlatedTickCounter = 0;
static double NanosThisLine=0;
static unsigned char BlinkPhase=1;
static unsigned int AudioBuffer[16384];
static unsigned char CassBuffer[8192];
static unsigned int AudioIndex = 0;
static unsigned int CassIndex = 0;
static unsigned int CassBufferSize = 0;
double NanosToInterrupt=0;
static int IntEnable=0;
static int SndEnable=1;
static int OverClock=1;
static unsigned char SoundOutputMode=0;	//Default to Speaker 1= Cassette
static double emulatedCycles;
double TimeToHSYNCLow = 0;
double TimeToHSYNCHigh = 0;
static unsigned char LastMotorState;

static int AudioFreeBlockCount;

static int clipcycle = 1, cyclewait=2000;
bool codepaste, PasteWithNew = false;
void AudioOut();
void CassOut();
void CassIn();
void (*AudioEvent)()=AudioOut;

//--- Event Heap ---
static EventHeap eventHeap;
static int evtTimerInterrupt = -1;
static int evtAudioSample = -1;

// GIME timer state beyond the event itself (see SetMasterTickCounter).
static bool gTimerOverdue = true;
// True only while CPUExec runs inside CPUCycle, i.e. while the CPU's live
// cycle count is meaningful relative to the current slice.
static bool gInCpuSlice = false;
// The drift folded into the running slice's budget: after a JIT overshoot
// it is negative, meaning the CPU started this slice that many cycles
// ahead of event-heap time.
static double gSliceDriftIn = 0;
static void ScheduleTimerFromNow(double nanos);

static void OnTimerInterrupt()
{
	if (!IntEnable)
	{
		// The countdown ran out while the timer value is zero. Upstream's
		// countdown just goes negative and waits; remember that, and turn
		// this firing into a one-shot (FireExpired disables a zero-rearm
		// event after the handler - SetEnabled here would re-sort the heap
		// mid-iteration).
		gTimerOverdue = true;
		eventHeap.SetRearmDelta(evtTimerInterrupt, 0);
		return;
	}
	GimeAssertTimerInterupt();
	// Reload: FireExpired adds the rearm delta, which SetMasterTickCounter
	// keeps equal to the current period.
}

static void OnAudioSample()
{
	AudioEvent();
}
void SetMasterTickCounter();
void (GimeGpu::*DrawTopBoarder[4])(SystemState *)const ={ &GimeGpu::DrawTopBoarder8,&GimeGpu::DrawTopBoarder16,&GimeGpu::DrawTopBoarder24,&GimeGpu::DrawTopBoarder32 };
void (GimeGpu::*DrawBottomBoarder[4])(SystemState *)const ={ &GimeGpu::DrawBottomBoarder8,&GimeGpu::DrawBottomBoarder16,&GimeGpu::DrawBottomBoarder24,&GimeGpu::DrawBottomBoarder32 };
void (GimeGpu::*UpdateScreen[4])(SystemState *) ={ &GimeGpu::UpdateScreen8,&GimeGpu::UpdateScreen16,&GimeGpu::UpdateScreen24,&GimeGpu::UpdateScreen32 };
std::string GetClipboardText();
void HLINE();
void VSYNC(unsigned char level);
void HSYNC(unsigned char level);
std::string CvtStrToSC(std::string);

using namespace std;

_inline void CPUCycle(double nanoseconds);


void UpdateAudio();
bool PakTickDemandActive();

// ---------------------------------------------------------------------------
// Scanline bursts: when nothing software-visible needs per-line service
// (no GIME horizontal interrupt, no PIA hsync IRQ armed, no cartridge
// hsync-tick demand, and the frame's draw calls are being skipped), the
// CPU runs a whole run of lines as ONE CPUCycle call. That gives the
// block-cache dispatcher a budget of thousands of cycles instead of ~57,
// so large trace blocks run as native thunks instead of falling into
// interpreter replay at every slice seam - measured at 28% of dispatch
// entries on compiled-C workloads before this change.
//
// Correctness is preserved by observation, not by hope:
//  - Everything already event-driven (GIME timer, audio sampling) fires
//    at exact nanosecond deadlines inside CPUCycle regardless of slice
//    length.
//  - The PIA hsync flag and cartridge ticks are materialized ON DEMAND:
//    any guest access to PIA0, the GIME control registers, or a
//    cartridge port calls CoCoLineObserve(), which fires the HSYNC edge
//    triples owed up to the current cycle before the access proceeds.
//  - A mid-burst state change that creates a per-line obligation (the
//    guest arms the hsync IRQ, or a cartridge starts wanting ticks)
//    calls CoCoLineObserveAndCut(): owed edges fire, a per-line heap
//    event takes over edge delivery for the burst's remaining lines,
//    and the CPU slice is cut so the event heap deadline is honored
//    immediately (the unexecuted cycles hand back to CPUCycle's loop).
//  - Any lines still owed when the burst call returns get their edges
//    fired then, in the same order the per-line path would have used.
static struct
{
	bool   active = false;
	int    linesTotal = 0;
	int    linesFired = 0;
	double nanosDone = 0;      // completed CPU slices within the burst
	bool   eventArmed = false; // per-line heap event took over edges
} gBurst;
static int evtBurstLine = -1;

static int CPULiveCycles()
{
	return (EmuState.CpuType == 1) ? HD6309LiveCycles() : MC6809LiveCycles();
}
static void CPUCutSlice()
{
	if (EmuState.CpuType == 1) HD6309CutSlice(); else MC6809CutSlice();
}

// One line's worth of hsync edges, exactly as the per-line path fires
// them at end of line: falling edge, cartridge tick, rising edge.
static void FireLineEdges()
{
	HSYNC(0);
	PakTimer();
	HSYNC(1);
	UpdateAudio();
}

static void OnBurstLine()
{
	if (gBurst.active && gBurst.linesFired < gBurst.linesTotal)
	{
		FireLineEdges();
		++gBurst.linesFired;
	}
	if (!gBurst.active || gBurst.linesFired >= gBurst.linesTotal)
		eventHeap.SetEnabled(evtBurstLine, false);
}

// Event-heap nanoseconds elapsed in the running CPU slice: live cycles
// plus whatever overshoot the slice inherited (see gSliceDriftIn).
static double SliceLiveNanos()
{
	return ((double)CPULiveCycles() - gSliceDriftIn) / CyclesPerNano;
}

// Nanoseconds of guest time elapsed inside the current burst, including
// the live position within the CPU slice now executing.
static double BurstLiveNanos()
{
	return gBurst.nanosDone + SliceLiveNanos();
}

void CoCoLineObserve()
{
	if (!gBurst.active || gBurst.eventArmed)
		return;
	int owed = (int)(BurstLiveNanos() / NanosPerLine);
	if (owed > gBurst.linesTotal)
		owed = gBurst.linesTotal;
	while (gBurst.linesFired < owed)
	{
		FireLineEdges();
		++gBurst.linesFired;
	}
}

void CoCoLineObserveAndCut()
{
	if (!gBurst.active || gBurst.eventArmed)
		return;
	CoCoLineObserve();
	// Hand the burst's remaining lines to a per-line heap event, phase
	// aligned to the next line boundary, and end the CPU slice so the
	// event heap deadline takes effect now rather than at burst end.
	if (gBurst.linesFired < gBurst.linesTotal)
	{
		const double live = BurstLiveNanos();
		double phase = NanosPerLine - fmod(live, NanosPerLine);
		eventHeap.SetRearmDelta(evtBurstLine, NanosPerLine);
		eventHeap.SetDeadline(evtBurstLine, phase);
		eventHeap.SetEnabled(evtBurstLine, true);
		gBurst.eventArmed = true;
	}
	CPUCutSlice();
}

// Arm the GIME timer to fire `nanos` from now. Heap deadlines count from
// the start of the running CPU slice, so a mid-slice register write adds
// the time already spent in it (without that, a restart inside a long
// burst lands early by up to most of a frame), then cuts the slice so a
// deadline that falls inside it is honored on time instead of at its end.
static void ScheduleTimerFromNow(double nanos)
{
	const double elapsed = gInCpuSlice ? SliceLiveNanos() : 0.0;
	eventHeap.SetDeadline(evtTimerInterrupt, elapsed + nanos);
	eventHeap.SetEnabled(evtTimerInterrupt, true);
	if (gInCpuSlice)
		CPUCutSlice();
}

static bool CanBurstLines()
{
	// VCC_NO_BURST: kill switch for A/B measurement and refuge.
	static const bool no_burst = getenv("VCC_NO_BURST") != nullptr;
	return !no_burst && !GimeHsyncIrqArmed() && !PiaHsyncIrqArmed() &&
	       !PakTickDemandActive();
}

static void BurstLines(int count)
{
	gBurst.active = true;
	gBurst.linesTotal = count;
	gBurst.linesFired = 0;
	gBurst.nanosDone = 0;
	gBurst.eventArmed = false;
	CPUCycle(NanosPerLine * count);
	gBurst.active = false;
	if (gBurst.eventArmed)
		eventHeap.SetEnabled(evtBurstLine, false);
	// Edges still owed (bulk case: all of them) fire here, in order.
	while (gBurst.linesFired < gBurst.linesTotal)
	{
		FireLineEdges();
		++gBurst.linesFired;
	}
}

void UpdateAudio()
{
#if USE_DEBUG_AUDIOTAPE
	if (CassIndex < CassBufferSize)
	{
		uint8_t sample = LastMotorState ? CassBuffer[CassIndex] : CAS_SILENCE;
		auto& data = gAudioHistory[AudioHistorySize - 1];
		if (gAudioHistoryCount == 0)
		{
			data.inputMin = 0;
			data.inputMax = 0;
		}
		if (gAudioHistoryCount < 5)
		{
			data.inputMin += sample;
			data.inputMax += sample;
		}
		else if (gAudioHistoryCount == 5)
		{
			// begin with average sample (last 6)
			data.inputMin += sample;
			data.inputMax += sample;
			data.inputMin /= 6;
			data.inputMax /= 6;
		}
		else
		{
			// now, keep track of peak to peak volume
			if (sample > data.inputMax) data.inputMax = sample;
			if (sample < data.inputMin) data.inputMin = sample;
		}
		++gAudioHistoryCount;
	}
#endif // USE_DEBUG_AUDIOTAPE

	// keep audio system full by tiny expansion of sound
	if (AudioFreeBlockCount > 1 && (AudioIndex & 63) == 1 && AudioIndex < 16384-2)
	{
		unsigned int last = AudioBuffer[AudioIndex - 1];
		AudioBuffer[AudioIndex++] = last;
	}
}

void DebugDrawAudio()
{
#if USE_DEBUG_AUDIOTAPE
	static VCC::Pixel col(255, 255, 255);
	col.a = 240;
	for (int i = 0; i < 734; ++i)
	{
		DebugDrawLine(i, 256 - ((AudioBuffer[i] & 0xFFFF) >> 7), i + 1, 256 - ((AudioBuffer[i + 1] & 0xFFFF) >> 7), col);
		DebugDrawLine(750+i, 256 - ((AudioBuffer[i] & 0xFFFF0000) >> 23), 750+i + 1, 256 - ((AudioBuffer[i + 1] & 0xFFFF00000) >> 23), col);
	}

	auto& history = gAudioHistory[AudioHistorySize - 1];
	history.motorState = LastMotorState;
	history.audioState = GetMuxState() == PIA_MUX_CASSETTE;
	gAudioHistoryCount = 0;
	for (int i = 0; i < AudioHistorySize - 1; ++i)
	{
		auto& a = gAudioHistory[i];
		auto& b = gAudioHistory[i + 1];
		DebugDrawLine(i, 500 - a.inputMin, i + 1, 500 - a.inputMax, col);
		DebugDrawLine(i, 520 - 20 * a.motorState, i + 1, 520 - 20 * b.motorState, col);
		DebugDrawLine(i, 550 - 20 * a.audioState, i + 1, 550 - 20 * b.audioState, col);
	}
	memcpy(&gAudioHistory[0], &gAudioHistory[1], sizeof(gAudioHistory) - sizeof(*gAudioHistory));
#endif // USE_DEBUG_AUDIOTAPE
}


// Run `count` lines that need no draw calls: one burst when the burst
// predicate allows, the classic per-line path otherwise.
static void RunBlankLines(SystemState* st, int count)
{
	if (count <= 0)
		return;
	if (CanBurstLines())
	{
		BurstLines(count);
		return;
	}
	for (int i = 0; i < count; ++i)
		HLINE();
}

float RenderFrame (SystemState *RFState)
{
	static unsigned int FrameCounter=0;
	// FrameCounter was never incremented, leaving it at 0 forever -
	// (0 % FrameSkip) == 0 is always true, so FrameSkip has been
	// silently non-functional: every frame rendered regardless of the
	// setting. With FrameSkip=1 (the default) this increment changes
	// nothing; with higher values the draw calls are now actually
	// skipped (emulation timing is unaffected - HLINE still runs).
	FrameCounter++;

	// VCC_LOG_MODE: frame-stamped trace of screen-geometry changes, for
	// debugging stale-border artifacts.
	static const bool log_mode = getenv("VCC_LOG_MODE") != nullptr;
	if (log_mode)
	{
		static int last_tb = -1, last_bb = -1, last_lps = -1, last_bc = -1;
		const int bc = gGimeGpu.BoarderChange;
		if (gGimeGpu.TopBoarder != last_tb || gGimeGpu.BottomBoarder != last_bb ||
		    gGimeGpu.LinesperScreen != last_lps || bc != last_bc)
		{
			printf("MODE f=%u top=%d lps=%d bottom=%d bc=%d\n",
			       FrameCounter, gGimeGpu.TopBoarder, gGimeGpu.LinesperScreen, gGimeGpu.BottomBoarder, bc);
			last_tb = gGimeGpu.TopBoarder; last_bb = gGimeGpu.BottomBoarder;
			last_lps = gGimeGpu.LinesperScreen; last_bc = bc;
		}
	}

	// once per frame
	LastMotorState = GetMotorState();
	AudioFreeBlockCount = GetFreeBlockCount();

//********************************Start of frame Render*****************************************************

	// Blink state toggle
	if (BlinkPhase++ > RENDERS_PER_BLINK_TOGGLE && !EmuState.Debugger.IsHalted()) {
		gGimeGpu.TogBlinkState();
		BlinkPhase = 0;
	}

	// VSYNC goes Low
	VSYNC(0);

	// Four lines of blank during VSYNC low
	RunBlankLines(RFState, 4);

	// VSYNC goes High
	VSYNC(1);

	// Three lines of blank after VSYNC goes high
	RunBlankLines(RFState, 3);

	// Top Border actually begins here, but is offscreen
	RunBlankLines(RFState, gGimeGpu.TopOffScreen);

	if (!(FrameCounter % RFState->FrameSkip))
	{
		if (LockScreen())
			return 0;
	}

	// Lines with no draw calls this frame: frameskip, or the debugger has
	// the machine halted (upstream freezes the video while paused). Both
	// still run every line's timing - bursts are only a faster way to.
	const bool skipDraw = (FrameCounter % RFState->FrameSkip) != 0 ||
	                      EmuState.Debugger.IsHalted();

	// Visible Top Border begins here. (Remove 4 lines for centering)
	RFState->Debugger.TraceCaptureScreenEvent(VCC::TraceEvent::ScreenTopBorder, 0);
	if (skipDraw)
		RunBlankLines(RFState, gGimeGpu.TopBoarder);
	else
	for (RFState->LineCounter = 0; RFState->LineCounter < gGimeGpu.TopBoarder; RFState->LineCounter++)
	{
		HLINE();
		(gGimeGpu.*DrawTopBoarder[RFState->BitDepth])(RFState);
	}

	// Main Screen begins here: LPF = 192, 200 (actually 199), 225
	RFState->Debugger.TraceCaptureScreenEvent(VCC::TraceEvent::ScreenRender, 0);
	if (skipDraw)
		RunBlankLines(RFState, gGimeGpu.LinesperScreen);
	else
	for (RFState->LineCounter = 0; RFState->LineCounter < gGimeGpu.LinesperScreen; RFState->LineCounter++)		
	{
		HLINE();
		(gGimeGpu.*UpdateScreen[RFState->BitDepth])(RFState);
	}

	// Bottom Border begins here.
	RFState->Debugger.TraceCaptureScreenEvent(VCC::TraceEvent::ScreenBottomBorder, 0);
	if (skipDraw)
		RunBlankLines(RFState, gGimeGpu.BottomBoarder);
	else
	for (RFState->LineCounter=0;RFState->LineCounter < gGimeGpu.BottomBoarder;RFState->LineCounter++)
	{
		HLINE();
		(gGimeGpu.*DrawBottomBoarder[RFState->BitDepth])(RFState);
	}

	if (!(FrameCounter % RFState->FrameSkip))
	{
		if (!EmuState.Debugger.IsHalted())
			(gGimeGpu.*DrawBottomBoarder[RFState->BitDepth])(RFState);
		UnlockScreen(RFState);
		gGimeGpu.SetBoarderChange();
	}

	// Bottom Border continues but is offscreen
	RunBlankLines(RFState, gGimeGpu.BottomOffScreen);

	switch (SoundOutputMode)
	{
	case 1:
		FlushCassetteBuffer(CassBuffer,&CassIndex);
		break;
	}
	FlushAudioBuffer(AudioBuffer, AudioIndex << 2);
	AudioIndex=0;

	DebugDrawAudio();

	// Only affect frame rate if a debug window is open.
	RFState->Debugger.Update();

	static bool wasHalted = false;
	if (EmuState.Debugger.IsHalted())
	{
		wasHalted = true;
		return 0;
	}
	else if (wasHalted)
	{
		return CalculateFPS(true);
		wasHalted = false;
	}

	return CalculateFPS(false);
}

void VSYNC(unsigned char level)
{
	if (level == 0)
	{
		EmuState.Debugger.TraceCaptureScreenEvent(VCC::TraceEvent::ScreenVSYNCLow, 0);
		irq_fs(0);
		GimeAssertVertInterupt();
	}
	else
	{
		EmuState.Debugger.TraceCaptureScreenEvent(VCC::TraceEvent::ScreenVSYNCHigh, 0);
		irq_fs(1);
	}
}

void HSYNC(unsigned char level)
{
	if (level == 0)
	{
		EmuState.Debugger.TraceCaptureScreenEvent(VCC::TraceEvent::ScreenHSYNCLow, 0);
		// With the HSYNC enable bits clear in $FF92/$FF93 the assert is an
		// exact no-op (nothing latches, and LastGimeIrq/Firq always equal
		// their routed state, so no CPU line moves) - skip it: this runs
		// for every scanline, and the call was ~40% of the line-edge cost.
		if (GimeHsyncIrqArmed())
			GimeAssertHorzInterupt();
		irq_hs(0);
	}
	else
	{
		EmuState.Debugger.TraceCaptureScreenEvent(VCC::TraceEvent::ScreenHSYNCHigh, 0);
		irq_hs(1);
	}
}

void SetClockSpeed(unsigned int Cycles)
{
	OverClock=Cycles;
	CyclesPerNano = CyclesPerLine * OverClock / NanosPerLine;
	return;
}


DisplayDetails GetDisplayDetails(const int clientWidth, const int clientHeight)
{
	const float pixelsPerLine = gGimeGpu.GetDisplayedPixelsPerLine();
	const float horizontalBorderSize = gGimeGpu.GetHorizontalBorderSize();
	const float activeLines = 192.0f;	//	FIXME: Needs a symbolic

	DisplayDetails details;

	const auto extraBorderPadding = GetForcedAspectBorderPadding();

	// calcuate the complete screen size including its borders in device coords
	float deviceScreenWidth = (float)clientWidth - extraBorderPadding.x * 2;
	float deviceScreenHeight = (float)clientHeight - extraBorderPadding.y * 2;

	// calculate the content size including the borders in surface coords
	float contentWidth = pixelsPerLine + horizontalBorderSize * 2;
	float contentHeight = activeLines + gGimeGpu.TopBoarder + gGimeGpu.BottomBoarder;

	// now get scale difference between both previous equivalent boxes
	float horizontalScale = deviceScreenWidth / contentWidth;
	float verticalScale = deviceScreenHeight / contentHeight;

	// fill in details by scalling the coco screen into device coords
	details.contentRows = static_cast<int>(gGimeGpu.LinesperScreen * verticalScale);
	details.topBorderRows = static_cast<int>(gGimeGpu.TopBoarder * verticalScale) + extraBorderPadding.y;
	details.bottomBorderRows = static_cast<int>(gGimeGpu.BottomBoarder * verticalScale) + extraBorderPadding.y;

	details.contentColumns = static_cast<int>(pixelsPerLine * horizontalScale);
	details.leftBorderColumns = static_cast<int>(horizontalBorderSize * horizontalScale) + extraBorderPadding.x;
	details.rightBorderColumns = static_cast<int>(horizontalBorderSize * horizontalScale) + extraBorderPadding.x;
	
	return details;
}

_inline void HLINE()
{
	if (EmuState.Debugger.IsHalted())
		return; 

	UpdateAudio();

	// When neither hsync interrupt source is armed, nothing software-
	// visible depends on where within the line the HSYNC edges land,
	// so run the whole line as ONE CPU burst and fire the edges after.
	// This halves the CPUExec entry/exit cost, which dominates at 57
	// emulated cycles per call. The PIA HS status flag still flips and
	// PakTimer still ticks, in the same order as the split path.
	if (!GimeHsyncIrqArmed() && !PiaHsyncIrqArmed())
	{
		CPUCycle(NanosPerLine);
		HSYNC(0);
		PakTimer();
		HSYNC(1);
		return;
	}

	// First part of the line
	CPUCycle(NanosPerLine - HSYNCWidthInNanos);

	// HSYNC going low.
	HSYNC(0);
	PakTimer();

	// Run for a bit.
	CPUCycle(HSYNCWidthInNanos);

	// HSYNC goes high
	HSYNC(1);
}

_inline void CPUCycle(double NanosToRun)
{
	// CPU is in a halted state.
	if (EmuState.Debugger.IsHalted())
	{
		return;
	}

	EmuState.Debugger.TraceEmulatorCycle(VCC::TraceEvent::EmulatorCycle, 10, NanosToRun, 0, 0, 0, 0);
	NanosThisLine += NanosToRun;
	double emulationCycles = 0, emulationDrift = 0;

	// Temporary probe (VCC_CYCLE_STATS): slice/exec counts at exit.
	static std::atomic<uint64_t> stat_calls{0}, stat_slices{0}, stat_execs{0}, stat_cycles{0};
	static const bool cycle_stats = [] {
		const bool on = getenv("VCC_CYCLE_STATS") != nullptr;
		if (on)
			atexit([] {
				fprintf(stderr, "[CYC] calls=%llu slices=%llu execs=%llu cycles=%llu (%.2f sl/call, %.1f cyc/exec)\n",
				        (unsigned long long)stat_calls.load(), (unsigned long long)stat_slices.load(),
				        (unsigned long long)stat_execs.load(), (unsigned long long)stat_cycles.load(),
				        (double)stat_slices.load() / (double)(stat_calls.load() ? stat_calls.load() : 1),
				        (double)stat_cycles.load() / (double)(stat_execs.load() ? stat_execs.load() : 1));
			});
		return on;
	}();
	if (cycle_stats) ++stat_calls;

	while (NanosThisLine >= 1)
	{
		if (cycle_stats) ++stat_slices;
		// One deadline query per slice. In the common case (no event
		// due this line) the slice covers the whole remaining line and
		// no fire/rescan happens at all.
		const double nextDeadline = eventHeap.NextDeadline();
		if (nextDeadline <= 0)
		{
			// Already-overdue event: fire before running the CPU.
			eventHeap.FireExpired(0);
			continue;
		}

		const bool hitsDeadline = (nextDeadline <= NanosThisLine);
		double nanosToConsume = hitsDeadline ? nextDeadline : NanosThisLine;

		// Convert nanos to CPU cycles and execute
		CyclesThisLine = CycleDrift + nanosToConsume * CyclesPerNano;
		if (CyclesThisLine >= 1)
		{
			const double whole = floor(CyclesThisLine);
			if (cycle_stats) { ++stat_execs; stat_cycles += (uint64_t)whole; }
			gSliceDriftIn = CycleDrift;
			gInCpuSlice = true;
			CycleDrift = CPUExec((int)whole) + (CyclesThisLine - whole);
			gInCpuSlice = false;
			// A slice cut (CoCoLineObserveAndCut) returns unexecuted
			// whole cycles as positive drift. Hand them back as nanos
			// so this loop re-slices them - against the per-line event
			// deadline the cut just armed. Never true otherwise: the
			// normal exit overshoots (drift <= 0) plus a fraction.
			if (CycleDrift >= 1)
			{
				const double back = floor(CycleDrift);
				// Shrink this slice to what actually executed: the
				// remainder then stays in NanosThisLine for the next
				// loop pass, and AdvanceTime below sees only the
				// guest time that really elapsed.
				nanosToConsume -= back / CyclesPerNano;
				if (nanosToConsume < 0)
					nanosToConsume = 0;
				CycleDrift -= back;
			}
		}
		else
			CycleDrift = CyclesThisLine;

		EmuState.Debugger.TraceEmulatorCycle(VCC::TraceEvent::EmulatorCycle, 0,
			NanosThisLine, nextDeadline, 0, CyclesThisLine, CycleDrift);
		emulationCycles += CyclesThisLine;
		emulationDrift += CycleDrift;

		// Advance time; only scan for expired events when this slice
		// actually reached the nearest deadline.
		NanosThisLine -= nanosToConsume;
		if (gBurst.active)
			gBurst.nanosDone += nanosToConsume;
		eventHeap.AdvanceTime(nanosToConsume);
		if (hitsDeadline)
			eventHeap.FireExpired(0);
	}

	EmuState.Debugger.TraceEmulatorCycle(VCC::TraceEvent::EmulatorCycle, 20, 0, 0, 0, emulationCycles, emulationDrift);
}

//
// Restart with new timer value.
// 
// Note: zero is done by caller.
//
void RestartInterruptTimer(unsigned int timer)
{
	// A restart replaces any expired countdown; clear the flag first so
	// the period update below doesn't also schedule an immediate fire.
	if (timer & 0xFFF)
		gTimerOverdue = false;
	SetMasterTickCounter(timer);

	if (IntEnable)
		ScheduleTimerFromNow(MasterTickCounter);
}

//
// Setup new timer clock rate, if changed.
// 
// 0 = 63.695uS  (1/60*262)  1 scanline time
// 1 = 279.265nS (1/ColorBurst) 
//
void SetTimerClockRate(unsigned char rate)	
{											
	auto clockRate = rate ? 1 : 0;
	// only update clock rate if changed
	if (TimerClockRate == clockRate) return;

	// clock rate changed
	TimerClockRate = clockRate;
	RestartInterruptTimer(UnxlatedTickCounter);
}

//
// Setup the master time to timer interrupt based on current rate.
//
// timerOffset: depends on Gime version:
//   1 = Gime'87
//   2 = Gime'86
//
void SetMasterTickCounter(unsigned int timer)
{
	UnxlatedTickCounter = timer & 0xFFF;

	// if non-zero, update nanos to interrupt, otherwise if zero clear event.
	IntEnable = UnxlatedTickCounter > 0 ? 1 : 0;

	// Rate = { 63613.2315, 279.265 };
	double Rate[2]={NANOSECOND/(TARGETFRAMERATE*LINESPERSCREEN),NANOSECOND/COLORBURST};
	// Master count contains at least one tick. EJJ 10mar25
	const unsigned int timerOffset = 1; // 1 = Gime'87, 2 = Gime'86
	MasterTickCounter = Rate[TimerClockRate] * (UnxlatedTickCounter + timerOffset);

	// Event-heap form of upstream's countdown: the next reload uses the
	// period current at fire time (OnTimerInterrupt reads it), and a
	// countdown that ran out while the timer was zero fires as soon as a
	// nonzero value arrives - even via the non-restarting LSB write.
	eventHeap.SetRearmDelta(evtTimerInterrupt, MasterTickCounter);
	if (IntEnable && gTimerOverdue)
	{
		gTimerOverdue = false;
		ScheduleTimerFromNow(0);
	}
}

void MiscReset()
{
	MasterTimer=0;
	// Upstream never resets the countdown, so after a reset it reads as
	// already expired: the first nonzero timer value fires immediately.
	gTimerOverdue = true;
	TimerClockRate=0;
	MasterTickCounter=0;
	UnxlatedTickCounter=0;
//*************************
	SoundInterupt=0;//PICOSECOND/44100;
	NanosToSoundSample=SoundInterupt;
	NanosToAudioSample = SoundInterupt;
	CycleDrift=0;
	CyclesThisLine=0;
	NanosThisLine=0;
	IntEnable=0;
	AudioIndex=0;
	ResetAudio();

	// Initialize event heap with timer and audio events (both disabled at reset)
	eventHeap.Clear();
	evtTimerInterrupt = eventHeap.Schedule("GIMETimer", 0, 0, OnTimerInterrupt, false);
	evtAudioSample = eventHeap.Schedule("AudioSample", 0, 0, OnAudioSample, false);
	evtBurstLine = eventHeap.Schedule("BurstLine", 0, 0, OnBurstLine, false);
	return;
}

unsigned int SetAudioRate (unsigned int Rate)
{

	SndEnable=1;
	SoundInterupt=0;
	CycleDrift=0;

	if (Rate==0)
	{
		SndEnable=0;
		eventHeap.SetEnabled(evtAudioSample, false);
	}
	else
	{
		SoundInterupt=NANOSECOND/Rate;
		NanosToSoundSample=SoundInterupt;
		NanosToAudioSample = NANOSECOND/AUDIO_RATE;

		// Sync event heap: set audio sample period and enable
		eventHeap.SetRearmDelta(evtAudioSample, SoundInterupt);
		eventHeap.SetDeadline(evtAudioSample, SoundInterupt);
		eventHeap.SetEnabled(evtAudioSample, true);
	}
	SoundRate=Rate;
	return 0;
}

unsigned int GetAudioRate()
{
	return SoundRate;
}

void OutputAudio(unsigned int dac, unsigned int cas)
{
	// fade ramp state
	static unsigned int fadeTo = 0;
	static unsigned int fade = 0;

	// extract left channel
	auto getLeft = [](auto sample) { return (unsigned long)(sample & 0xFFFF); };
	// extract right channel
	auto getRight = [](auto sample) { return (unsigned long)((sample >> 16) & 0xFFFF); };
	// convert 8 bit to 16 bit stereo (like dac)
	auto monoToStereo = [](uint8_t sample) { return ((uint32_t)sample << 23) | ((uint32_t)sample << 7); };

	// mix two channels dependant on mux (2 x 16bit)
	auto casChannel = monoToStereo(cas);
	auto dacChannel = dac;

	// fade time of 16ms, note: this is slow enough to eliminate switching pop but
	// it must be quick too because some games have a nasty habit of toggling the 
	// mux on and off, such as Tuts Tomb, in order to generate clicks (footsteps).
	const int FADE_TIME = SoundRate / 64;

	// update ramp, always moving towards correct channel  
	fade = fade < fadeTo ? fade + 1 : fade > fadeTo ? fade - 1 : fade;

	// if mux changed, start transition
	fadeTo = FADE_TIME * (GetMuxState() == PIA_MUX_CASSETTE ? 1 : 0);

	// mix audio level between device channels
	auto left = (getLeft(casChannel) * fade + getLeft(dacChannel) * (FADE_TIME - fade)) / FADE_TIME;
	auto right = (getRight(casChannel) * fade + getRight(dacChannel) * (FADE_TIME - fade)) / FADE_TIME;
	auto sample = (unsigned int)(left + (right << 16));

	while (NanosToAudioSample > 0)
	{
		AudioBuffer[AudioIndex++] = sample;
		NanosToAudioSample -= NANOSECOND / AUDIO_RATE;
	}
	NanosToAudioSample += SoundInterupt;
}

void AudioOut()
{
	AudioBuffer[AudioIndex++] = GetDACSample();
}

void CassOut()
{
	if (LastMotorState && CassIndex < sizeof(CassBuffer)/sizeof(*CassBuffer))
		CassBuffer[CassIndex++]=GetCasSample();
}

//
// Reads the next byte from cassette data stream until end of the tape
// either fast mode, normal or wave data.
//
uint8_t CassInByteStream()
{
	if (CassIndex >= CassBufferSize)
	{
		LoadCassetteBuffer(CassBuffer, &CassBufferSize);
		CassIndex = 0;
	}
	return LastMotorState ? CassBuffer[CassIndex++] : CAS_SILENCE;
}

//
// In fast load mode, byte stream is two samples per bit high & low,
// the type of bit (0 or 1) depends on the period. based on this pass
// the period to basic at address $83.
//
uint8_t CassInBitStream()
{
	uint8_t nextHalfBit = CassInByteStream();
	// set counter time for one lo-hi cycle, basic checks for >18 (0bit) or <18 (1bit)
	MemWrite8(nextHalfBit & 1 ? 10 : 20, 0x83);
	return nextHalfBit >> 1;
}

void CassIn()
{
	// fade ramp state
	static unsigned int fadeTo = 0;
	static unsigned int fade = 0;

	// extract left channel
	auto getLeft = [](auto sample) { return (unsigned long)(sample & 0xFFFF); };
	// extract right channel
	auto getRight = [](auto sample) { return (unsigned long)((sample >> 16) & 0xFFFF); };
	// convert 8 bit to 16 bit stereo (like dac)
	auto monoToStereo = [](uint8_t sample) { return ((uint32_t)sample << 23) | ((uint32_t)sample << 7); };

	if (GetTapePlaybackFastLoad())
	{
		AudioBuffer[AudioIndex++] = GetMuxState() == PIA_MUX_CASSETTE ? monoToStereo(CAS_SILENCE) : GetDACSample();
	}
	else
	{
		// read next sample or same as last if motor is off
		auto casSample =  CassInByteStream();
		SetCassetteSample(casSample);

		OutputAudio(GetDACSample(), casSample);
	}
}



void SetSndOutMode(unsigned char Mode)  //0 = Speaker 1= Cassette Out 2=Cassette In
{
	static unsigned char LastMode=0;

	if (Mode == LastMode) return;

	switch (Mode)
	{
	case 0:
		if (LastMode==1)	//Send the last bits to be encoded
			FlushCassetteBuffer(CassBuffer,&CassIndex);

		AudioEvent=AudioOut;
		SetAudioRate(SoundRate);
		break;

	case 1:
		AudioEvent=CassOut;
		SetAudioRate(GetTapeRate());
		break;

	case 2:
		AudioEvent=CassIn;
		SetAudioRate(GetTapeRate());
		break;
	}

	LastMode=Mode;
	SoundOutputMode=Mode;
}

void PasteText() {
	using namespace std;
	std::string tmp;
	string cliptxt, clipparse, lines, debugout;
	int GraphicsMode = gGimeGpu.GetGraphicsMode();
	if (GraphicsMode != 0) {
		int tmp = MessageBox(nullptr, "Warning: You are not in text mode. Continue Pasting?", "Clipboard", MB_YESNO);
		if (tmp != 6) { return; }
	}

	cliptxt = GetClipboardText().c_str();
	if (PasteWithNew) { cliptxt = "NEW\n" + cliptxt; }
	char prvchr = '\0';
	for (size_t t = 0; t < cliptxt.length(); t++) {
		char tmp = cliptxt[t];
		bool eol;                   //EJJ CRLF,LFCR,CR,LF eol logic
		if (tmp == '\x0A') {        //LF
			if (prvchr == '\x0D') continue;
			eol = TRUE;
		} else if (tmp == '\x0D') { //CR
			if (prvchr == '\x0A') continue;
			eol = TRUE;
		} else {
			eol = FALSE;
		}
		prvchr = tmp;
		if (! eol) {
			lines += tmp;
		}
		else  { //...the character is a <CR>
			if (lines.length() > 249 && lines.length() < 257 && codepaste==true) {
				int b = lines.find(" ");
				string main = lines.substr(0, 249);
				string extra = lines.substr(249, lines.length() - 249);
				string spaces;
				for (int p = 1; p < 249; p++) {
					spaces.append(" ");
				}
				string linestr = lines.substr(0, b);
				lines = main+"\n\nEDIT "+linestr+"\n"+spaces+"I"+extra+"\n";
				clipparse += lines;
				lines.clear();
			}
			if(lines.length() >= 257 && codepaste==true) {
				// Line is too long to handle. Truncate.
				int b = lines.find(" ");
				string linestr = "Warning! Line "+lines.substr(0, b)+" is too long for BASIC and will be truncated.";
				
				MessageBox(nullptr, linestr.c_str(), "Clipboard", 0);
				lines = (lines.substr(0, 249));
			}
			if(lines.length() <= 249 || codepaste==false) { 
				// Just a regular line.
				clipparse += lines+"\n"; 
				lines.clear();
			}
		}
		if (t == cliptxt.length()-1) {
			clipparse += lines;
		}
	}
	PasteIntoQueue(CvtStrToSC(clipparse));
}

void QueueText(const char * text) {
	using namespace std;
	std::string str(text);
	PasteIntoQueue(CvtStrToSC(str));
}

std::string CvtStrToSC(string cliptxt)
{
	std::string out;
	char sc;
	char letter;
	bool CSHIFT;
	bool LCNTRL;
	for (size_t pp = 0; pp <= cliptxt.size(); pp++) {
		sc = 0;
		CSHIFT = FALSE;
		LCNTRL = FALSE;
		letter = cliptxt[pp];
		switch (letter)
		{
		case '@': sc = 0x03; CSHIFT = TRUE; break;
		case 'A': sc = 0x1E; CSHIFT = TRUE; break;
		case 'B': sc = 0x30; CSHIFT = TRUE; break;
		case 'C': sc = 0x2E; CSHIFT = TRUE; break;
		case 'D': sc = 0x20; CSHIFT = TRUE; break;
		case 'E': sc = 0x12; CSHIFT = TRUE; break;
		case 'F': sc = 0x21; CSHIFT = TRUE; break;
		case 'G': sc = 0x22; CSHIFT = TRUE; break;
		case 'H': sc = 0x23; CSHIFT = TRUE; break;
		case 'I': sc = 0x17; CSHIFT = TRUE; break;
		case 'J': sc = 0x24; CSHIFT = TRUE; break;
		case 'K': sc = 0x25; CSHIFT = TRUE; break;
		case 'L': sc = 0x26; CSHIFT = TRUE; break;
		case 'M': sc = 0x32; CSHIFT = TRUE; break;
		case 'N': sc = 0x31; CSHIFT = TRUE; break;
		case 'O': sc = 0x18; CSHIFT = TRUE; break;
		case 'P': sc = 0x19; CSHIFT = TRUE; break;
		case 'Q': sc = 0x10; CSHIFT = TRUE; break;
		case 'R': sc = 0x13; CSHIFT = TRUE; break;
		case 'S': sc = 0x1F; CSHIFT = TRUE; break;
		case 'T': sc = 0x14; CSHIFT = TRUE; break;
		case 'U': sc = 0x16; CSHIFT = TRUE; break;
		case 'V': sc = 0x2F; CSHIFT = TRUE; break;
		case 'W': sc = 0x11; CSHIFT = TRUE; break;
		case 'X': sc = 0x2D; CSHIFT = TRUE; break;
		case 'Y': sc = 0x15; CSHIFT = TRUE; break;
		case 'Z': sc = 0x2C; CSHIFT = TRUE; break;
		case ' ': sc = 0x39; break;
		case 'a': sc = 0x1E; break;
		case 'b': sc = 0x30; break;
		case 'c': sc = 0x2E; break;
		case 'd': sc = 0x20; break;
		case 'e': sc = 0x12; break;
		case 'f': sc = 0x21; break;
		case 'g': sc = 0x22; break;
		case 'h': sc = 0x23; break;
		case 'i': sc = 0x17; break;
		case 'j': sc = 0x24; break;
		case 'k': sc = 0x25; break;
		case 'l': sc = 0x26; break;
		case 'm': sc = 0x32; break;
		case 'n': sc = 0x31; break;
		case 'o': sc = 0x18; break;
		case 'p': sc = 0x19; break;
		case 'q': sc = 0x10; break;
		case 'r': sc = 0x13; break;
		case 's': sc = 0x1F; break;
		case 't': sc = 0x14; break;
		case 'u': sc = 0x16; break;
		case 'v': sc = 0x2F; break;
		case 'w': sc = 0x11; break;
		case 'x': sc = 0x2D; break;
		case 'y': sc = 0x15; break;
		case 'z': sc = 0x2C; break;
		case '0': sc = 0x0B; break;
		case '1': sc = 0x02; break;
		case '2': sc = 0x03; break;
		case '3': sc = 0x04; break;
		case '4': sc = 0x05; break;
		case '5': sc = 0x06; break;
		case '6': sc = 0x07; break;
		case '7': sc = 0x08; break;
		case '8': sc = 0x09; break;
		case '9': sc = 0x0A; break;
		case '!': sc = 0x02; CSHIFT = TRUE; break;
		case '#': sc = 0x04; CSHIFT = TRUE;	break;
		case '$': sc = 0x05; CSHIFT = TRUE;	break;
		case '%': sc = 0x06; CSHIFT = TRUE;	break;
		case '^': sc = 0x07; CSHIFT = TRUE;	break;
		case '&': sc = 0x08; CSHIFT = TRUE;	break;
		case '*': sc = 0x09; CSHIFT = TRUE;	break;
		case '(': sc = 0x0A; CSHIFT = TRUE;	break;
		case ')': sc = 0x0B; CSHIFT = TRUE;	break;
		case '-': sc = 0x0C; break;
		case '=': sc = 0x0D; break;
		case ';': sc = 0x27; break;
		case '\'': sc = 0x28; break;
		case '/': sc = 0x35; break;
		case '.': sc = 0x34; break;
		case ',': sc = 0x33; break;
		case '\n': sc = 0x1C; break;
		case '+': sc = 0x0D; CSHIFT = TRUE;	break;
		case ':': sc = 0x27; CSHIFT = TRUE;	break;
		case '\"': sc = 0x28; CSHIFT = TRUE; break;
		case '?': sc = 0x35; CSHIFT = TRUE; break;
		case '<': sc = 0x33; CSHIFT = TRUE; break;
		case '>': sc = 0x34; CSHIFT = TRUE; break;
		case '[': sc = 0x1A; LCNTRL = TRUE; break;
		case ']': sc = 0x1B; LCNTRL = TRUE; break;
		case '{': sc = 0x1A; CSHIFT = TRUE; break;
		case '}': sc = 0x1B; CSHIFT = TRUE; break;
		case '\\': sc = 0x2B; LCNTRL = TRUE; break;
		case '|': sc = 0x2B; CSHIFT = TRUE; break;
		case '`': sc = 0x29; break;
		case '~': sc = 0x29; CSHIFT = TRUE; break;
		case '_': sc = 0x0C; CSHIFT = TRUE; break;
		case 0x09: sc = 0x39; break; // TAB
		default: sc = -1; break;
		}
		if (CSHIFT) { out += 0x36; CSHIFT = FALSE; }
		if (LCNTRL) { out += 0x1D; LCNTRL = FALSE; }
		out += sc;
	}
	return out;
}

std::string GetClipboardText()
{
	if (!OpenClipboard(nullptr)) { MessageBox(nullptr, "Unable to open clipboard.", "Clipboard", 0); return {}; }
	HANDLE hClip = GetClipboardData(CF_TEXT);
	if (hClip == nullptr) { CloseClipboard(); MessageBox(nullptr, "No text found in clipboard.", "Clipboard", 0); return {}; }
	const char* tmp = static_cast<char*>(GlobalLock(hClip));
	if (tmp == nullptr) {
		CloseClipboard();  MessageBox(nullptr, "NULL Pointer", "Clipboard", 0); return {};
	}
	std::string out(tmp);
	GlobalUnlock(hClip);
	CloseClipboard();

	return out;
}

bool SetClipboard(const string& sendout) {
	const char* clipout = sendout.c_str();
	const size_t len = strlen(clipout) + 1;
	HGLOBAL hMem = GlobalAlloc(GMEM_MOVEABLE, len);
	memcpy(GlobalLock(hMem), clipout, len);
	GlobalUnlock(hMem);
	OpenClipboard(nullptr);
	EmptyClipboard();
	SetClipboardData(CF_TEXT, hMem);
	CloseClipboard();
	return TRUE;
}

void CopyText() {
	int idx;
	int tmp;
	int lines;
	int offset;
	int lastchar;
	int BytesPerRow = gGimeGpu.GetBytesPerRow();
	int GraphicsMode = gGimeGpu.GetGraphicsMode();
	unsigned int screenstart = gGimeGpu.GetStartOfVidram();
	if (GraphicsMode != 0) { 
		MessageBox(nullptr, "ERROR: Graphics screen can not be copied.\nCopy can ONLY use a hardware text screen.", "Clipboard", 0); 
		return;
	}
	string out;
	string tmpline;
	if (BytesPerRow == 32) { lines = 15; }
	else { lines = 23; }

	string dbug = "StartofVidram is: " + to_string(screenstart) + "\nGraphicsMode is: " + to_string(GraphicsMode)+"\n";
	OutputDebugString(dbug.c_str());
	
	// Read the lo-res text screen...
	if (BytesPerRow == 32) {
		offset = 0;
		char pcchars[] =
		{
			'@','a','b','c','d','e','f','g',
			'h','i','j','k','l','m','n','o',
			'p','q','r','s','t','u','v','w',
			'x','y','z','[','\\',']',' ',' ',
			' ','!','\"','#','$','%','&','\'',
			'(',')','*','+',',','-','.','/',
			'0','1','2','3','4','5','6','7',
			'8','9',':',';','<','=','>','?',
			'@','A','B','C','D','E','F','G',
			'H','I','J','K','L','M','N','O',
			'P','Q','R','S','T','U','V','W',
			'X','Y','Z','[','\\',']',' ',' ',
			' ','!','\"','#','$','%','&','\'',
			'(',')','*','+',',','-','.','/',
			'0','1','2','3','4','5','6','7',
			'8','9',':',';','<','=','>','?',
			'@','a','b','c','d','e','f','g',
			'h','i','j','k','l','m','n','o',
			'p','q','r','s','t','u','v','w',
			'x','y','z','[','\\',']',' ',' ',
			' ','!','\"','#','$','%','&','\'',
			'(',')','*','+',',','-','.','/',
			'0','1','2','3','4','5','6','7',
			'8','9',':',';','<','=','>','?'
		};

		for (int y = 0; y <= lines; y++) {
			lastchar = 0;
			tmpline.clear();
			tmp = 0;
			for (idx = 0; idx < BytesPerRow; idx++) {
				tmp = MemRead8(0x0400 + y * BytesPerRow + idx);
				if (tmp == 32 || tmp == 64 || tmp == 96) { tmp = 30 + offset; } 
				else { lastchar = idx + 1; }
				tmpline += pcchars[tmp - offset]; 
			}
			tmpline = tmpline.substr(0, lastchar);
			if (lastchar != 0) { out += tmpline; out += "\n"; }

		}
		if (out == "") { MessageBox(nullptr, "No text found on screen.", "Clipboard", 0); }
	}
	else if (BytesPerRow == 40 || BytesPerRow == 80) {
		offset = 32;
		int pcchars[] =
		{
			' ','!','\"','#','$','%','&','\'',
			'(',')','*','+',',','-','.','/',
			'0','1','2','3','4','5','6','7',
			'8','9',':',';','<','=','>','?',
			'@','A','B','C','D','E','F','G',
			'H','I','J','K','L','M','N','O',
			'P','Q','R','S','T','U','V','W',
			'X','Y','Z','[','\\',']',' ',' ',
			'^','a','b','c','d','e','f','g',
			'h','i','j','k','l','m','n','o',
			'p','q','r','s','t','u','v','w',
			'x','y','z','{','|','}','~','_',
			// CoCo 3 international glyphs as Windows-1252 codes, kept
			// as hex escapes: written as bare accented literals in a
			// UTF-8 source file they are multichar constants, which
			// compiled to the wrong bytes on MSVC and do not compile at
			// all on clang. (The entry before the final space was
			// already encoding-damaged in the original; it stays a
			// space.)
			'\xC7','\xFC','\xE9','\xE2','\xE4','\xE0','\xE5','\xE7',
			'\xEA','\xEB','\xE8','\xEF','\xEE','\xDF','\xC4','\xC2',
			'\xD3','\xE6','\xC6','\xF4','\xF6','\xF8','\xFB','\xF9',
			'\xD8','\xD6','\xDC','\xA7','\xA3','\xB1','\xBA',' ',
			' ',' ','!','\"','#','$','%','&',
			'\'','(',')','*','+',',','-','.',
			'/','0','1','2','3','4','5','6',
			'7','8','9',':',';','<','=','>',
			'?','@','A','B','C','D','E','F',
			'G','H','I','J','K','L','M','N',
			'O','P','Q','R','S','T','U','V',
			'W','X','Y','Z','[','\\',']',' ',
			' ','^','a','b','c','d','e','f',
			'g','h','i','j','k','l','m','n',
			'o','p','q','r','s','t','u','v',
			'w','x','y','z','{','|','}','~','_'
		};

		for (int y = 0; y <= lines; y++) {
			lastchar = 0;
			tmpline.clear();
			tmp = 0;
			for (idx = 0; idx < BytesPerRow * 2; idx += 2) {
				tmp = GetMem(screenstart + y * (BytesPerRow * 2) + idx);
				if (tmp == 32 || tmp == 64 || tmp == 96) { tmp = offset; }
				else { lastchar = idx / 2 + 1; }
				tmpline += pcchars[tmp - offset];
			}
			tmpline = tmpline.substr(0, lastchar);
			if (lastchar != 0) { out += tmpline; out += "\n"; }
		}
	}
	
	bool succ = SetClipboard(out);
}

void PasteBASIC() {
	codepaste = true;
	PasteText();
	codepaste = false;
}
void PasteBASICWithNew() {
	int tmp=MessageBox(nullptr, "Warning: This operation will erase the Coco's BASIC memory\nbefore pasting. Continue?", "Clipboard", MB_YESNO);
	if (tmp != 6) { return; }
	codepaste = true;
	PasteWithNew = true;
	PasteText();
	codepaste = false;
	PasteWithNew = false;
}

