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
5. **Beyond:** compare/lazy-CC inlining using the donor flag identities;
   block linking à la `~/z80/dbt` if block-exit overhead ever shows up.

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
