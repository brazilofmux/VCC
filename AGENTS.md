# Repository Guidelines

## Project Structure & Module Organization
`Vcc.sln` is the top-level Visual Studio 2022 solution. The root contains the main emulator sources (`Vcc.cpp`, `config.cpp`, CPU, video, audio, debugger, and UI files). Hardware modules live in sibling directories such as `FD502/`, `HardDisk/`, `mpi/`, `SuperIDE/`, `Ramdisk/`, `acia/`, `becker/`, `GMC/`, `orch90/`, and `sdc/`. Shared code is split into `libcommon/include/vcc/` for headers and `libcommon/src/` for implementations. Static assets live in `resources/`, and reference material and sample configs live in `docs/`.

## Build, Test, and Development Commands
Run builds from a VS 2022 Developer Command Prompt.

- `Build.bat`: builds the `Release|x86` solution and writes artifacts under `__bin\Win32\Release`.
- `BuildClean.bat`: cleans and rebuilds the release configuration.
- `BuildAll.bat`: performs the standard release build, then rebuilds with `_CL_=/DUSE_LOGGING` for logging-enabled binaries.
- `msbuild Vcc.sln /m /p:Configuration=Debug /p:Platform=x86`: useful for local debugging.

GitHub Actions runs `BuildAll.bat` on Windows, so keep that path working.

## Coding Style & Naming Conventions
This is a legacy C++17 Windows codebase; prefer the existing style of the file you are editing instead of forcing a repo-wide format. Use spaces for new indentation unless the surrounding file already uses tabs. Keep includes grouped with system headers before project headers. Match existing naming: `PascalCase` for many types/files (`MemoryMap.cpp`), `snake_case` in newer `libcommon` files (`rom_cartridge.cpp`), and all-caps macros only for compile-time constants. There is no repo formatter config, so keep changes small and consistent.

## Testing Guidelines
There is no dedicated unit-test tree in this repository. Validation is build-first: at minimum, complete a `Release|x86` build and, when touching runtime behavior, do a focused smoke test in the emulator. Note manual checks in your PR, for example: boot path, disk attachment, cartridge loading, keyboard input, or debugger flow.

## Commit & Pull Request Guidelines
Recent history uses short, imperative subjects such as `Fix floppy paths` and `Rename context variables to be more meaningfull`. Follow that pattern and keep commits narrowly scoped. PRs should describe the behavioral change, list manual verification performed, and link related issues. Include screenshots only for visible UI changes. Do not submit `.sln` or `.vcxproj` edits unless a maintainer has asked for project-file changes.
