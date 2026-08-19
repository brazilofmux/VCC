# Porting VCC off Windows (macOS / Linux)

Assessment written 2026-08-18 on Kagura (x86-64 Linux) after the last local
Windows dev box was decommissioned; revised the same day on the Macbook
after surveying the sibling DBT projects (see "Prior art" below). Windows
binaries are still produced by GitHub Actions (`BuildAll.bat`), but there
is no local machine that can run them natively. The daily driver is an
Apple Silicon Macbook, and that is the port target: **native arm64**. No
x86 translation layer (Rosetta, Wine) is part of the plan of record.

**Bottom line: this is a shell port, not a redevelopment — and the JIT is
a backend port, not a redesign.** Roughly 35–40k of the ~55k lines in root
+ `libcommon/` are portable C++, and that portable core is where all of
this fork's performance work lives (block cache, threaded interpreter,
decoded instructions, event heap, Level-1/2 JIT infrastructure, ROM
analyzer/database). The Windows-specific surface is concentrated and
enumerable, the two hardest refactors — display and cartridge-loading
abstractions — were already done upstream, and the arm64 emitter that
first read as "the one real architecture problem" turns out to be mostly
solved by donor code in sibling repos on this machine.

## What is already portable

- CPU cores: `hd6309.cpp` (no `windows.h` at all), `mc6809.cpp` (include is
  inert), plus `BlockCache.h`, `BlockDecoder.h`, `DecodedInst.h`,
  `EventHeap.h`, `OpDecoder.cpp`, `OpCodeTables.cpp`.
- Chipset: `tcc1014mmu.cpp`, `tcc1014graphics.cpp`, `tcc1014registers.cpp`,
  `mc6821.cpp`, `coco3.cpp`, `iobus.cpp`. These include `windows.h` (mostly
  via `defines.h`) but actual Win32 usage is ~9 `MessageBox` calls total
  (8 of them in `coco3.cpp`) — verified by grepping for real API calls,
  not includes.
- `libcommon/` (bus/devices/util), including `RomAnalyzer`, `RomDatabase`,
  `RomBlockStore`.
- Display seam: `IDisplay.h` abstraction with an OpenGL renderer
  (`OpenGL.cpp`, Craig Allsop, 2025) plus `DisplayNone.h` / `DisplayNull.h`
  headless stubs.
- Cartridge seam: `libcommon/src/bus/cartridge_loader.cpp` and the cartridge
  interface — `pakinterface.cpp` no longer calls `LoadLibrary` directly.

## The Windows-bound surface (~7.3k lines of shell + cart-module dialogs)

| Area | Files | Replacement |
|---|---|---|
| Message loop, menus, dialogs | `Vcc.cpp`, `config.cpp`, `Vcc.rc`, `keyboardEdit.cpp` | SDL event loop; drive config from `vcc.ini` + `CommandLine.cpp`; defer GUI config |
| Audio | `Audio.cpp` (DirectSound, ~300 lines of real API use) | SDL audio |
| Timing | `throttle.cpp` (QueryPerformanceCounter, `Sleep`) | `std::chrono` + `std::this_thread::sleep_for` — trivial |
| Threading | `_beginthreadex` in `Vcc.cpp` | `std::thread` |
| Video | `DirectDrawInterface.cpp`, `DirectX.cpp` | Drop; host the existing OpenGL renderer in an SDL window |
| Input | `keyboard.cpp` raw input, `joystickinput.cpp` | SDL keyboard/joystick events |
| Config storage | `GetPrivateProfile*` INI calls | Small shim over the same `.ini` format so existing configs carry over |
| Cart modules as DLLs | `FD502/`, `mpi/`, `HardDisk/`, `sdc/`, `becker/`, `acia/`, `GMC/`, `orch90/`, `Ramdisk/`, `SuperIDE/` (~10k lines, much of it config dialogs) | Statically link behind the existing cartridge interface (or `dlopen`); replace dialogs with ini settings |
| Build system | `Vcc.sln` / `.vcxproj` | CMake alongside; keep MSBuild files untouched so `BuildAll.bat` CI keeps working |

## The Level-1/2 JIT: retire the x86 backend, port to arm64

`BlockJit.cpp` emits x86 machine code and deliberately leans on x86 flag
semantics (V captured from `inc`/`dec`/`neg`/`shl`, C from CF after
`neg`/`shl`/`shr`/`sar`, N/Z from SF/ZF after `test al,al`). Two
corrections to the original assessment:

- **The x86 backend is ILP32-only, on every OS.** Every helper bakes
  `(uint32_t)(uintptr_t)addr` into disp32 absolute forms, and the
  `mod=00, r/m=101` encoding used throughout *changes meaning* in 64-bit
  mode (it becomes RIP-relative); the `A0`/`A2` moffs forms and
  `push imm32` __cdecl frames are 32-bit idioms too. There is no "audit
  it for 64-bit and keep it alive on Intel" option — the emitter lives
  and dies with `Win32|x86` builds. On any 64-bit target the forward
  path is a new backend.
- **The valuable parts of `BlockJit.cpp` are target-independent** and
  carry over unchanged: the cc[] liveness/DSE pass
  (`AnalyzeFlagLiveness`, `InlinedHandlerWritesMask`), PC-write
  skipping, the inline-vs-call policy in `EmitBlock`, and the stats.
  What is x86-specific is ~30 tiny byte-writer primitives and the per-op
  emitters that sequence them. Split a backend interface at that seam
  and the level-2 logic ports as-is.

Gate the tiers behind arch/platform `#if`s and make "interpreter + block
cache only" a first-class configuration. That configuration alone is
already overkill for a 2.88 MHz 6309 on an M-series core — the arm64
backend is about craft and headroom, not necessity.

## Prior art: most of the arm64 backend exists in sibling repos

Three sibling projects are built and running as native arm64 Mach-O
binaries on the Macbook, and among them they solve every hard problem the
arm64 backend faces:

| Donor | What to take |
|---|---|
| `~/z80/dbt/emit_a64.h` (1.2k lines) | Complete self-contained AArch64 encoder: MOVZ/MOVK immediate materialization, logical-immediate encoder, LDRB/STRB with imm and register offsets, ANDS/SUBS/CSET, BFI/UBFX, B/BL/BLR emission with patch-up. Plain-C static-inline, no dependencies, already "lifted from `~/riscv/dbt`" by design — it is meant to travel. Drop in verbatim; replaces every `Emit*` byte-writer. |
| `~/z80/dbt/dbt.h` (W^X block) | The macOS JIT-memory machinery as a three-liner: `MAP_JIT` mmap flags, `pthread_jit_write_protect_np` bracketing via `dbt_jit_writable_begin/end`, `#ifdef` fallback for Linux/Intel. Replaces `VirtualAlloc`. |
| `~/z80/dbt/dbt_a64.c` | Icache discipline (`__builtin___clear_cache` after emission, per-site 4-byte flushes when patching link sites — mandatory on ARM, unlike x86); register-pinning convention (X19 = CPU state, X20 = guest mem, X21 = block cache, X24 = aux tables) entered through a small emitted trampoline; `emit_alu_inline`'s `fmask`-driven flag DSE — the same liveness idea as our `flag_mask` — with the identities the 6309 will need when inlining extends to ADD/ADC/SUB/CMP: H from bit 4 of `a^b^res`, V from the sign-xor formula, C from bit 8 of the widened result. |
| `~/z80/dbt/dbt_common.c` | The `-V` shadow-verify harness: run each JIT block on the real CPU, step a shadow copy through the reference interpreter by the same instruction count, compare full register state + memory, dump a focused divergence report. The validation harness for a fresh emitter. |
| `~/riscv/dbt` | Same encoder lineage proven on a second guest ISA; performance reference. |
| `~/slow-32` (tools/dbt, selfhost/stage08) | Third DBT on the same conventions, plus an arm64 cross compiler whose generated code matches gcc's hot-path speed to three significant digits — evidence the codegen idioms are mature. |

Performance context: on this hardware the riscv DBT runs at ~9.3 BIPS and
the z80 at ~4 BIPS — the Z80's flag bits and internal state (X/Y flags,
memptr, q) are expensive to model, and the 6809/6309's much cleaner flag
model should land nearer the riscv end. Any of these numbers is thousands
of times a 2.88 MHz guest; the status-bar effective-MHz readout is the
scoreboard.

VCC is actually *easier* than the Z80 donor in one respect: cc[] is
already unpacked bytes, so there is no packed-F assembly step — each live
flag is one CSET (or computed bit) plus a STRB, and "C preserved" ops
(INC/DEC) need no flag-preservation gymnastics: just don't store cc[0].

### Prerequisite refactor (portable — do it during the headless stage)

x86-32 addressed `hd6309.cpp`'s scattered statics via disp32 absolute
forms for free; on arm64 each separate address is a 4-instruction
MOVZ/MOVK materialization. Gather the CPU state the JIT touches (PC,
A/B/D, X/Y/U/S, dp, cc[8], CycleCounter, the NatEmuCycles* bytes) into
one contiguous struct and pin a callee-saved register (x19) at its base —
every access becomes one `ldrb`/`strb`/`ldrh`/`strh [x19, #imm]`.
`BlockJit::CpuAddrs` already collects exactly these addresses at Init;
the change is making them contiguous. Pure portable C++ with no Windows
dependency, and the x86 backend reads the addresses through `CpuAddrs`
regardless of layout, so CI stays green while it lands.

### Shape of the arm64 backend

- Level-1 trampoline per instruction: STRH the baked post-insn PC,
  MOVZ/MOVK `&insn` into x0, BLR the handler. Handler calls go through
  MOVZ/MOVK + BLR because a `MAP_JIT` arena won't reliably sit within
  BL's ±128 MB of the text segment. Each block thunk gets a real frame
  (`stp x29, x30`) since it calls out; AAPCS64 args in w0/w1 replace the
  push / `add esp` dance. Days of work with the encoder dropped in, not
  a project.
- Level-2: port the ~15 inline emitters onto `emit_a64.h`, driven by the
  existing liveness masks. Where x86 gave us flags for free (`inc m8`
  setting OF on 0x7F→0x80), arm64 computes the bit explicitly or uses
  the donor identities — the DSE pass already minimizes how often that
  happens.
- Validation: adopt a z80-style shadow-verify lockstep mode alongside
  the existing smoke tests. Blocks whose handlers touch I/O need the
  same escape the donor uses (fall back to the interpreter path), which
  is an escape VCC's block model already has.
- If the app is ever signed/notarized, the hardened runtime needs the
  `com.apple.security.cs.allow-jit` entitlement; plain local builds
  (like the three donors) do not.

## Recommended staging

1. **Headless first:** get root + `libcommon` compiling with clang/CMake
   using `DisplayNull`, no audio, no carts. This flushes out `defines.h`,
   the stray `MessageBox` calls (route through a small host-services
   header), and MSVC-isms. Fold in the CPU-state struct refactor above
   while this code is being touched. Doable on the Mac itself.
2. **Carts** *(done, headless-first)*: MPI + FD502 + HardDisk build as
   dlopen-able shared libraries behind the existing LoadLibrary seam —
   truer to the DLL architecture than static linking, and MPI's slot
   loading works unchanged. `vcc-headless` boots NitrOS-9/6309 Level 2
   and Basic09 from the Hatsuhara vcc.ini via a scripted keyboard.
   Remaining modules (sdc, becker, GMC, orch90, acia, Ramdisk,
   SuperIDE) follow the same pattern; becker needs its winsock code
   ported to BSD sockets.
3. **SDL shell** *(done)*: `vcc-sdl` - an SDL2 window streaming the
   GIME renderer's 640x480 XRGB8888 surface, a character-level
   host-keyboard-to-CoCo-matrix mapping (shift synthesized or
   suppressed per the CoCo layout), SDL audio fed from the core's
   per-frame sample buffer, a 60 fps pacer with F8 turbo, and a
   self-screenshot hook (F12 / VCC_SHOT_FILE) that doubles as the
   automated visual test. Machine bootstrap is shared with
   vcc-headless in shell/machine.cpp; the same vcc.ini drives both.
4. **arm64 JIT backend:** drop in `emit_a64.h`, split the backend seam in
   `BlockJit.cpp`, emit level-1 trampolines, then port the level-2
   inline emitters; bring up shadow-verify before trusting either.
5. **Beyond** *(block linking done; Abrash arc 2026-08-18)*: profile-
   driven overhead removal roughly doubled both benchmark workloads
   (NitrOS-9 boot+idle 2513x -> ~4900x realtime, DECB busy-poll 830x ->
   ~1540x): CPUCycle slice-loop thinning + LTO, demand-driven pak
   heartbeat (PakHsyncDemand module export - the idle FDC tick chain was
   22% of wall), one CPU burst per scanline when no hsync IRQ is armed,
   an interrupt quiet-gate in the dispatch loop, block linking through a
   shared chain stub (indirect via the live cache slot, so invalidation
   severs links for free; operands live in Hd6309State so the stub runs
   base-relative), and the indexed inline family unblocked (its
   writes-masks were missing, which had also hidden a W1-clobber bug in
   the indexed store emitters and a purity-flag gap - found by rank
   bisection + VCC_VERIFY_PURE). Diagnostics added along the way:
   VCC_PAK_STATS, VCC_CYCLE_STATS, VCC_NO_LINK, chain:N in the stats
   line. Next levers: registerized block state across chains (the chain
   stub is now the hottest single PC cluster), trace formation /
   superblocks, and the CPUCycle metronome floor.

   *Registerized-state arc (same day):* DECB 1540x -> ~1950x (+27%),
   OS-9 ~5150x. (1) Stub diet: the five bail flags packed into one
   zero-padded quadword in Hd6309State (one 64-bit load), generation
   mirrored into cpu_state, CachedBlock alignas(256) so slot indexing
   is a shift, chain counter behind VCC_JIT_STATS. (2) The chain
   register convention, established by an emitted runner the
   dispatcher enters thunks through (BlockJit::GetThunkRunner; x86 and
   null backends return null): w21 = CycleCounter (every cycle-add is
   now one instruction; handler call sites spill/reload around the
   BLR), w22 = CycleFor, x23 = &cpu_state, x24 = slot base - the stub
   runs with zero materializations and register-compare budget checks,
   and thunk prologues load x19 from x23. Negative result, reverted:
   passing the exit PC in w15 to skip the stub's PC load measured as
   noise - the load hides behind branch resolution. Thunk bodies are
   now ~5% of wall; hop FREQUENCY is the cost, so the next real lever
   is superblock traces (record through predictable branches with
   guard exits), then the dispatcher-entry and metronome floors.

   *Superblock arc (same day):* DECB ~2105x, OS-9 ~5165x (+8%).
   Conditional short branches observed not-taken become mid-block
   guards; blocks stay contiguous; the arm64 emitter inlines guards
   with exit snippets that leave through the chain stub; the x86
   backend refuses guarded blocks (replay covers them); RomAnalyzer
   extends prebuilt blocks the same way; MAX_BLOCK_INSNS 14. Two
   fixes the benchmarks demanded: superblock subsumption in
   FinishRecord (a taken-guard re-record is a prefix of the cached
   superblock - keep the longer form and its thunk, else MMU-driven
   re-records flush thunks on every bias flip) and partial replay on
   budget misses (identical instruction stream and timing as the old
   single-step fallback, minus the recording machinery - without it
   longer blocks made slice tails MORE expensive and superblocks were
   a wash). Honest residue: loop-bottom branches are TAKEN, so hot
   loops still hop once per iteration - absorbing them needs
   taken-direction traces (non-contiguous blocks, multi-range
   invalidation), a full arc of its own.

   *Taken-trace arc (same day):* the full machinery landed - taken
   branches recorded via a per-block taken_mask, mask-following
   decode/validate, per-range reverse-map maintenance, guard emission
   for both directions with loop back-edge interrupt bounds - but it
   ships DEFAULT-OFF (VCC_TRACE_TAKEN=1 to experiment): loop unrolling
   measured as a net LOSS here because trace blocks overlap staggered
   entry points and the write-invalidation map holds one owner per
   address. An eager kill-the-previous-claimant invariant was correct
   but dismantled the prepopulated ROM fabric (Microsoft deliberately
   overlapped instruction decodings in the ROM to save space, so dense
   overlap is a design fact, not an accident). What the arc left
   behind PERMANENTLY: three real correctness fixes found by write-
   stream and per-instruction differential tracing - (1) ReplayBlock
   stops when the block's own generation changes mid-run (a block can
   contain a write to its own later bytes: DECB's CHRGET rewrites its
   LDA operand, latent in the fall-only design on page crossings),
   (2) the recorder ends a recording after any instruction that
   invalidated cached code, restoring the write-then-execute split,
   (3) owner-aware reverse-map bookkeeping (a trace block's secondary
   ranges were claimed under the RANGE start, not the block start,
   silently orphaning their invalidation). Residual known-theoretical
   hole: a write invalidates only the mapped owner while an unmapped
   overlapping block goes stale - closed in practice by (1)+(2) plus
   bulk MMU invalidations; a per-page epoch scheme would close it
   fully. Interrupt-latency lesson: guard loops need back-edge
   deliverable-interrupt checks or polling code observes the latency
   (DECB's keyboard debounce did). Cost of the hardening: ~2-5% vs
   the prior peak; benchmarks ~1985x DECB / ~4980x OS-9.

   *Epoch/benchmark arc (2026-08-19):* per-page trace registration
   replaced the claim-kill invariant (precise write invalidation for
   overlapping traces at zero hot-path cost), and the raw pending
   word in the chain stub / loop back-edges became ChainBreak - a
   maintained "would the dispatcher act now" byte that masked-but-
   asserted lines (FD502's CART FIRQ sits asserted forever under
   DECB) no longer trip. New tooling: tests/bench/run.sh compiles a
   C sieve with the cmoc podman container onto a DECB disk and runs
   it headless (VCC_INI config override; the headless keyboard can
   type shifted characters now, so LOADM"SIEVE works). FINAL verdict
   on taken-direction traces after full correctness: interleaved A/B
   on compiled C still measures ~19% against, so default-off stands
   on data from its best-case workload. Next levers if wanted: guard
   cost reduction (single flags-register condition instead of cc[]
   byte loads), MAX_BLOCK_INSNS growth (512-byte slots), or spending
   the effort on user-facing shell features instead.

   *The two hidden taxes (same day, one more honest shot):* guard
   cost was never the problem. Profiling the long sieve showed
   ReplayBlock itself as the top consumer - trace blocks were running
   through the INTERPRETER: (1) the exec-end path cancelled any
   recording in progress, and trace-length recordings crossed the
   ~114-cycle slice seam at coin-flip odds (32,000 cancels/run -> 5
   after BlockCache::RebaseCycles lets recordings survive seams);
   (2) large blocks failed the slice-budget check near every slice
   end and the budget-miss path replays interpretively - fixed by
   kBudgetSlack=48 in both budget checks, with CPUCycle's drift
   accounting absorbing the bounded overshoot exactly as it always
   absorbed last-instruction overshoot. Taken traces now DEFAULT ON
   (VCC_NO_TAKEN=1 to disable): interleaved A/B measures sieve +14%,
   DECB +22%, OS-9 parity - and the taxes were general, lifting
   traces-off DECB +32% by themselves. DECB record: 3203x realtime,
   ~2.9 GHz effective. Lesson for the notebook: when a feature that
   should win keeps losing, profile the LOSING configuration before
   redesigning the feature - both taxes were dispatch-policy bugs
   visible in one sample profile, not properties of the feature.

   *Platform arc (2026-08-19):* the decoration sprint. Cmd+V/Cmd+C
   through shell/text_codec (libutf tr_ascii approximation in, VDG
   semigraphics -> Unicode quadrant blocks out; VCC_SHOT_TEXT tests it
   headlessly). tools/coco-run: one command from .c/.asm/.bas to a
   running CoCo - DECB via cmoc/lwasm/writecocofile, and --os9 builds
   a NitrOS-9 module onto a ToolShed-formatted floppy riding /d1
   (VCC_DISK0-3 env overrides; the cmoc container at ~/g/cmoc now
   carries os9/decb with musl build fixes). VCC.app: double-clickable
   bundle, modules+ROMs inside, DECB boot screen as the icon.
   Joystick: joystickinput.cpp joined the portable core (_WIN32
   guards around DirectInput), shells merge vccJoystickGetScan like
   Windows keyboard.cpp, vcc-sdl maps the mouse to the stick -
   JOYSTK reads 31s at center through the real DAC comparator.
   Becker: BSD-socket half added behind the winsock names,
   COMBINE_BECKER everywhere, plus an upstream retry-flag fix (a
   failed first connect wedged dw_status forever). Loopback-verified
   DW OK. Remaining decoration ideas: SDL game-controller mapping,
   BASIC09 workflow sugar (b09 source -> pack -> run), a DriveWire
   server integration test, Cmd+C of graphics screens as PNG.

## The DriveWire file-flow arc (pyDriveWire, 2026-08)

Goal: reach files on the Mac from inside NitrOS-9 over the becker
port, both directions. Server: pyDriveWire (n6il, `develop` branch is
the Python 3 one; venv needs `pyserial` and `legacy-cgi`). It worked,
and the road there surfaced three findings worth keeping:

- **The becker ring-buffer desync (latent upstream).** A read of the
  data port ($FF42) with nothing buffered advanced `InReadPos` past
  `InWritePos`, making the ring look phantom-full of zeros forever
  after. A single stray probe read during machine reset was enough:
  every DriveWire sector transfer then failed its checksum (the
  server logs `CRC: read 0x0` - the CoCo checksummed 256 phantom
  zeros). Fix: an empty-ring data read returns junk but consumes
  nothing, which is also what the real hardware does - the FIFO lives
  server-side.
- **Wall-clock vs instruction-counted timeouts.** At ~4800x realtime
  the drivers' poll loops burn through their budgets thousands of
  times faster than any real server can answer, so empty status polls
  on a live connection now yield 50us of wall time each (capped at
  ~1s per wait so a mute server degrades to the driver's own timeout
  instead of stalling emulation).
- **Runtime `load`+`iniz` of rbdw is a dead end on this system.** The
  VHD-Emudisk NitrOS-9 keeps only ~24K of Level 2 system map free;
  runtime-loaded driver blocks fill it and everything after fails
  with E$MemFul. The modules have to ride in the bootfile - and on
  these self-booting VHDs the *real* bootfile is not the floppy's
  OS9Boot (a decoy; deleting modules from it changes nothing) but an
  image stashed in DECB virtual drive 254 at the VHD's tail, pointed
  to absolutely by LSN0's DD.BT/DD.BSZ. `tools/dw-bake` appends
  rbdw + dwio_becker + x0/x1 there in place (idempotent, reversible);
  `tools/dw-serve` starts pyDriveWire and launches vcc-sdl with
  `VCC_DW=1`; `tests/drivewire.sh` proves the round trip on a
  copy-on-write clone of the VHD. One sharp edge, same as real
  hardware: touching /x0 or /x1 with no server connected freezes the
  machine (the driver polls with interrupts masked).

Smoke tests per AGENTS.md conventions: boot path, disk attach, cartridge
load, keyboard input, debugger flow — plus OS-9 Level 2 boot and Basic09,
the status-bar effective-MHz readout to compare interpreter/JIT tiers,
and the shadow-verify lockstep once the arm64 backend exists.

## Non-goals / alternatives considered

- **Running the Win32 build under CrossOver/Wine + Rosetta 2.** Works
  today (Rosetta handles the x86 JIT's generated code), but it is
  exactly the DBT-of-x86 path this plan rejects, Apple is winding
  Rosetta down, and the native port is small enough not to need a
  stopgap. On a Linux box, plain 32-bit Wine remains a way to run the
  CI artifacts, JIT included.
- **MAME's `coco3` driver / recent XRoar** both run OS-9 Level 2 natively
  on macOS today. Fine for *using* a CoCo 3; irrelevant for continuing
  this fork's block-cache/JIT work, which is the point of the exercise.
