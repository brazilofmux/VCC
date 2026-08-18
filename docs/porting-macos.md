# Porting VCC off Windows (macOS / Linux)

Assessment written 2026-08-18 on Kagura (x86-64 Linux) after the last local
Windows dev box was decommissioned. Windows binaries are still produced by
GitHub Actions (`BuildAll.bat`), but there is no local machine that can run
them natively. The daily driver is an Apple Silicon Macbook.

**Bottom line: this is a shell port, not a redevelopment.** Roughly 35–40k of
the ~55k lines in root + `libcommon/` are portable C++, and that portable core
is where all of this fork's performance work lives (block cache, threaded
interpreter, decoded instructions, event heap, Level-1/2 JIT infrastructure,
ROM analyzer/database). The Windows-specific surface is concentrated and
enumerable, and the two hardest refactors — a display abstraction and a
cartridge-loading abstraction — were already done upstream.

## What is already portable

- CPU cores: `hd6309.cpp` (no `windows.h` at all), `mc6809.cpp` (include is
  inert), plus `BlockCache.h`, `BlockDecoder.h`, `DecodedInst.h`,
  `EventHeap.h`, `OpDecoder.cpp`, `OpCodeTables.cpp`.
- Chipset: `tcc1014mmu.cpp`, `tcc1014graphics.cpp`, `tcc1014registers.cpp`,
  `mc6821.cpp`, `coco3.cpp`, `iobus.cpp`. These include `windows.h` (mostly
  via `defines.h`) but actual Win32 usage is ~10 `MessageBox` calls total —
  verified by grepping for real API calls, not includes.
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

## The one real architecture problem: the Level-2 JIT is x86-only

`BlockJit.cpp` emits x86 machine code and deliberately leans on x86 flag
semantics (V captured from `inc`/`dec`/`neg`/`shl`, C from CF after
`neg`/`shl`/`shr`/`sar`, N/Z from SF/ZF after `test al,al`). Consequences:

- **Apple Silicon:** Level-2 (and the Level-1 call-threaded trampolines, also
  emitted x86) cannot run. The threaded interpreter + block cache below them
  are plain C++ and degrade gracefully — almost certainly still overkill for
  a 2.88 MHz 6309 on an M-series core. An ARM64 emitter backend is a future
  project, not a porting prerequisite.
- **x86-64 Linux/mac (Intel):** the emitter stays live in principle, but it
  was written against a `Win32|x86` build. When porting: replace
  `VirtualAlloc`/`VirtualAlloc2` with `mmap(PROT_WRITE|PROT_EXEC)` (macOS
  needs `MAP_JIT` + `pthread_jit_write_protect_np` on arm64; not on Intel),
  and audit for 32-bit absolute-address / pointer-size assumptions before
  trusting it in a 64-bit process.

Gate the JIT tiers behind arch/platform `#if`s and make "interpreter +
block cache only" a first-class configuration.

## Recommended staging

1. **Stopgap (zero code):** run the CI-built Win32 `Vcc.exe` under
   CrossOver/Wine on the Macbook (32-bit Win32 via Rosetta 2 — Rosetta
   handles the x86 JIT's generated code fine). Caveat: Apple is winding
   Rosetta down, so this has a shelf life. On a Linux box, plain 32-bit Wine
   works, JIT included.
2. **Headless first:** get root + `libcommon` compiling with clang/CMake
   using `DisplayNull`, no audio, no carts. This flushes out `defines.h`,
   the stray `MessageBox` calls (route through a small host-services
   header), and MSVC-isms. Doable on any Linux box or the Mac itself.
3. **SDL shell:** SDL window hosting the OpenGL renderer, SDL audio,
   `std::chrono` throttle, SDL input, ini shim. Boot to BASIC, then OS-9.
4. **Carts:** statically link FD502 + HardDisk + mpi behind the cartridge
   interface, config via ini. Others follow.
5. **Later, for fun:** ARM64 backend for the Level-1/2 emitters.

Smoke tests per AGENTS.md conventions: boot path, disk attach, cartridge
load, keyboard input, debugger flow — plus OS-9 Level 2 boot and Basic09,
and the status-bar effective-MHz readout to compare interpreter/JIT tiers.

## Non-goals / alternatives considered

MAME's `coco3` driver and recent XRoar both run OS-9 Level 2 natively on
macOS today. Fine for *using* a CoCo 3; irrelevant for continuing this
fork's block-cache/JIT work, which is the point of the exercise.
