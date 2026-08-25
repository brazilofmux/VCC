# VCC on Apple Silicon — a 2.9 GHz Color Computer 3

This fork ports VCC, the Tandy Color Computer 3 emulator, to **native
arm64 macOS** — and then keeps going. The original CoCo 3 shipped in
1986 with a 0.89 MHz processor (1.78 MHz if you poked the right
register). This one boots Disk Extended BASIC at **over 3000× real
time**: an effective clock around **2.9 GHz**, executing roughly
**640 million 6309 instructions per second** on an Apple M-series
machine.

The Windows build is untouched and still lives here (see
[Upstream VCC](#upstream-vcc-on-windows) below). Everything the Mac
port adds sits beside it: a CMake build, an SDL shell, a headless
test harness, and an arm64 block-cache JIT for the 6809/6309.

## The numbers

| | |
|---|---|
| Real CoCo 3 (1986) | 0.89 MHz, ~0.4 MIPS — at one point US export rules fussed over 1 MIPS |
| This fork on an M-series Mac | ~2.9 GHz effective, ~640 MIPS, 3000–4900× real time by workload |

For calibration: this machine's sibling projects — bare-CPU dynamic
binary translators with none of a whole computer attached — hit
9.3 BIPS (RISC-V) and 4.3 BIPS (Z80). Emulating an entire computer
twists the problem into a different shape: every other instruction
may touch the GIME, the PIAs, the disk controller, or banked memory,
and each of those touches is a conversation with hardware that has
opinions. 640 MIPS with the whole machine attached is the interesting
number.

How it's done, in one breath: hot code is compiled to arm64 in a
block cache; blocks chain to each other directly and keep the
emulator's hot state in registers; traces record straight through
taken branches with cheap guards; a one-byte "would the dispatcher
act now" flag keeps interrupts honest at every back edge; and
anything cold or weird falls back to a cycle-exact interpreter. The
rule that outranks the numbers: **it has to be correct first and
always.** There is a lockstep verify mode that runs JIT and
interpreter side by side and faults on the first divergence, and a
regression suite that boots real operating systems and runs real
programs. The full engineering narrative — every arc, including the
experiments that lost and were reverted — is in
[docs/porting-macos.md](docs/porting-macos.md).

## What the Mac port does

All the emulated hardware upstream VCC provides (6309 swap, MultiPak,
FD-502 with the becker port, virtual hard drives, 8 MB RAM…) plus a
shell that behaves like a Mac app:

- **`vcc-sdl` / `VCC.app`** — SDL shell and a proper app bundle,
  with the DECB boot screen as its icon.
- **Clipboard both ways.** ⌘V pastes Mac text into the CoCo (UTF-8
  approximated to the CoCo character set via a compressed-DFA
  transliterator). ⌘C copies the text screen out as UTF-8 — including
  semigraphics blocks, which come across as Unicode quadrant
  characters. ⌘⇧C puts the screen on the clipboard as a PNG.
- **Joysticks.** The mouse drives the analog stick through the real
  PIA comparator path, or plug in any SDL-recognized game controller.
- **DriveWire, end to end.** The becker port runs on BSD sockets and
  talks to [pyDriveWire](https://github.com/n6il/pyDriveWire) on the
  Mac: files flow both directions between macOS and NitrOS-9.
  `tools/dw-serve` starts the server and the emulator in one command;
  `tools/dw-bake` bakes the DriveWire drivers into a NitrOS-9 boot
  so `/x0` and `/x1` exist at boot.
- **A real cross-development loop.** `tools/coco-run` takes a `.c`
  (CMOC), `.asm` (lwasm), `.bas`, or `.b09` file and gets it built,
  onto a disk image, and running in the emulator — targeting either
  Disk BASIC or NitrOS-9 — in one command.
- **Proof the platform is honest:** [apps/journal](apps/journal) is a
  ChaCha20-encrypted daily journal for NitrOS-9, written in C,
  compiled with CMOC, storing white-on-blue entries the way a
  certain teenage doctor would have wanted. Its crypto core is
  validated against the RFC 8439 test vector on the build host.

## Building on macOS

```sh
brew install cmake sdl2
cmake -B build
cmake --build build -j
```

Targets: `vcc-sdl` (windowed), `vcc-app` (builds `VCC.app`),
`vcc-headless` (no window, for scripting and tests).

## Building the portable shells on Windows (x64)

The same CMake build produces 64-bit `vcc-headless` and `vcc-sdl`
binaries on Windows with the **x86-64 JIT backend** (`BlockJitX64.cpp`,
the arm64 backend's sibling — same block linking, registerized state,
and superblock traces). This sits beside the classic Win32 MSBuild
build, which is untouched and still carries the original 32-bit shell.

```powershell
cmake -B build-x64 -A x64
cmake --build build-x64 --config Release
```

For `vcc-sdl`, unzip `SDL2-devel-<ver>-VC.zip` into `external/` first
(the CMake config is picked up automatically; `SDL2.dll` is copied
beside the exe). The cartridge modules build as 64-bit DLLs with their
string/dialog resources, so the same `vcc.ini` sections work; point
`VCC_INI` at a config whose MPI slots reference the x64 DLLs. You supply your
own CoCo 3 ROM; point `ExternalBasicImage` in
`~/.config/vcc/vcc.ini` at it (same ini format as the Windows build)
or pass the path on the command line. Optional extras: a libutf
checkout at `~/utf` (its compressed-DFA transliterator) enables the
full UTF-8 clipboard approximation, and a `cmoc` podman
container powers `tools/coco-run` and the test suite's guest builds.

## Headless mode and tests

`vcc-headless <rom> <frames> "<keys>"` boots the machine, types the
script (`~` is a one-second pause), and can snapshot the final screen
as text (`VCC_SHOT_TEXT`) or BMP (`VCC_SHOT_FILE`). The test suite
builds on it:

- `tests/bench/run.sh` — compiled-C prime sieve benchmark
- `tests/becker.sh` — DriveWire transport regression
- `tests/journal.sh` — the encrypted journal, end to end
- `tests/drivewire.sh` — Mac ↔ NitrOS-9 file flow over the wire
- `tests/smoke.sh` — DECB / NitrOS-9 / Basic09 boot assertions

On Windows, PowerShell twins of the suite run against the CMake x64
build: `tests/smoke.ps1`, `tests/journal.ps1`, `tests/becker.ps1`, and
`tests/bench/run.ps1`. The guest builds use the ECR cmoc image
imported as a WSL1 distro (`tools/ecr-to-wsl.ps1`), and becker's
loopback listener is a .NET `TcpListener` — no Python needed.

Useful environment knobs (tooling overrides, never required):

| Variable | Effect |
|---|---|
| `VCC_INI` | use this config file instead of `~/.config/vcc/vcc.ini` |
| `VCC_DISK0`–`3` | mount these disk images, overriding the ini |
| `VCC_DW`, `VCC_DW_PORT` | force the becker port on/off, pick the server port |
| `VCC_FRAMESKIP` | render every Nth frame (headless speed) |
| `VCC_SHOT_TEXT`, `VCC_SHOT_FILE` | write the final screen as UTF-8 text / BMP |
| `VCC_NO_JIT`, `VCC_NO_LINK`, `VCC_NO_TAKEN` | disable JIT tiers for A/B measurement |
| `VCC_VERIFY_PURE` | lockstep JIT-vs-interpreter verification |
| `VCC_JIT_STATS` | print block-cache statistics at exit |

---

# Upstream VCC on Windows

Everything below is the original project this fork tracks. The
Windows build remains intact in this tree.

## VCC - Virtual Color Computer
VCC is an emulator for the Tandy Color Computer 3, designed to run on Microsoft Windows. It aims to provide an accurate and easy-to-use emulation of the original hardware, as purchased from a Radio Shack store or Tandy Computer Center between 1986-1992.  For VCC usage, see the User Guide at [VCC Wiki](https://github.com/VCCE/VCC/wiki). The wiki also contains release notes and additional documents. Online documentation may reflect some features not available in earlier versions or that are pre-release.

VCC is licensed under the GNU General Public License v3.0. See the [LICENSE](COPYING) file for more details.

## Features
VCC emulates a stock 128k CoCo 3 and additional products, including:
- **Tandy MultiPak Interface (MPI)**: Four expansion slots.
- **Tandy FD-502 Disk Drive Controller with Becker Port**: Includes Disk Extended BASIC,four configurable virtual disk drives, and a Becker Interface for connecting to a DriveWire Server
- **Virtual Hard Drive Interface**: Allows VCC to connect two Virtual Hard Disks (VHDs).
- **SuperIDE Hard Drive Controller**: Emulates dual IDE hard drives.
- **Orchestra90cc**: A five-voice music sequencer with stereo 8-bit DACs.
- **Becker Port**: Old (previous to V2.1.9.3) VCC Interface for connecting to a DriveWire server.
- **Memory Expansions**: Up to 8192k.
- **CPU Replacement**: Swap the Motorola 6809 CPU with a Hitachi 6309.
- **SDC Simulator**: Simulates a COCO SDC floppy emulator.

## Obtaining VCC

Sources and binaries for VCC versions since v2.0.1 can be found at [VCC Releases](https://github.com/VCCE/VCC/releases). It is recommended to use the "latest" release.

VCC version numbering is somewhat erratic. Currently, the version number consists of "Vcc-" followed by four numbers separated by dots, for example: "Vcc-2.1.8.3". The first number represents a fork; the current fork is "2". The second number represents a major version, the third number represents releases that make additions or significant changes, and the fourth number represents bug fixes or minor changes.

Please be aware that the binaries provided with VCC releases, including the installers, do not contain verification certificates. It is likely you will be presented with Windows security warnings when you first run them. Alternatively, you can build the version of your choice from the sources available with the release.  Occasionally Virus detection software might flag VCC binaries due to false positives, even if you build from sources. If you encounter these issues you may be able to add an exception for VCC in the protection software you are using.  Every effort is taken to keep VCC safe to use but be aware there is no warranty that the instance of VCC you are running actually is.

## Building VCC
VCC is written in C++ and Microsoft Visual Studio 2022 Community is used to build VCC.  VS2022 is available for free download from Microsoft. The standard VCC build will run on Windows 10 and above. Optionally Visual Studio 2022 can be used to build a "legacy" VCC version using xp build tools that will run on Windows XP.

To build VCC from the command line, launch the "Developer Command Prompt for VS 2022". From there, change to the directory containing the VCC sources and type "Build" or "BuildClean".

Within Visual Studio, the "Release" and "Debug" configurations build VCC binaries that will run on current Windows versions. The "Legacy" configuration builds binaries that will run on Windows XP. "Legacy" uses the v141_xp build tools from which you need to find and install.  Maintaining a VCC version that will run on XP is becoming difficult and it is likely a future version will no longer support it.

Portions of VCC code have been modified to use features of the C++17 standard.

## Contributing to VCC
We welcome patches and code contributions that are consistent with our goals. Please comment your code well and add your name if you want credit for your work.

### How to Contribute
1. Fork the repository to your own GitHub account.
2. Clone the forked repository to your development system.
3. Create a new branch with a descriptive name.
4. Make your changes and commit them with clear messages.
5. Push your changes to your fork.
6. Submit a pull request to the `main` branch of the original repository.

Note: Do not include changes to `.sln` or `.vcxproj` files in your pull request. If you feel these project files need changes, contact the maintainers first.

We also welcome bug reports and suggestions. Please post these on the GitHub "Issues" tab. We check the issues periodically to see if there are fixes we can implement while working on VCC. The VCC Development Team is small, and we work on this project when we can, as we all have lives and families. VCC is a hobby, not a priority. It is a work of love, as we also use this software ourselves, so we try to make it as usable as possible. Sometimes progress is slow, and it looks like nothing is happening (and it may not be), but usually, there's plenty going on behind the scenes, and we have not committed our current work.
