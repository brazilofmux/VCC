# CMake build for the portable core (docs/porting-macos.md stage 1).
#
# This exists ALONGSIDE Vcc.sln — the MSBuild path is untouched and CI
# (BuildAll.bat) keeps using it. This file builds the portable set:
# CPU cores, chipset, block cache, libcommon, the cartridge modules as
# dlopen-able shared libraries (mirroring the Windows DLL architecture),
# and a headless driver. The Win32 shell (Vcc.cpp, config.cpp,
# DirectX/DirectSound, dialogs) is deliberately absent; the SDL shell
# replaces it.

cmake_minimum_required(VERSION 3.20)
project(vcc-core CXX)
if(WIN32)
  enable_language(RC)   # cartridge modules carry string/dialog resources
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Windows: same preprocessor posture as vcc-base.props (the MSBuild
# projects), so <Windows.h> keeps its min/max macros out of std:: calls.
if(WIN32)
  add_compile_definitions(NOMINMAX _CRT_SECURE_NO_WARNINGS)
endif()

# Cross-TU inlining for the per-scanline hot path (HLINE -> CPUCycle ->
# irq_hs/PakTimer live in different files). Skipped for Debug builds.
include(CheckIPOSupported)
check_ipo_supported(RESULT VCC_IPO_OK OUTPUT VCC_IPO_MSG)
if(VCC_IPO_OK AND NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON)
endif()

# ---- libcommon (portable subset; DialogOps/winapi are Win32-bound,
# filesystem.cpp is deprecated). Shared by the host and, exactly like
# the Windows DLL build, statically duplicated into each cartridge
# module — the register_rom_blocks callback design depends on that.
add_library(vcc-libcommon STATIC
  libcommon/src/bus/basic_cartridge.cpp
  libcommon/src/bus/cartridge_loader.cpp
  libcommon/src/bus/cartridge_menu.cpp
  libcommon/src/bus/cpak_cartridge.cpp
  libcommon/src/bus/rom_cartridge.cpp
  libcommon/src/devices/cloud9.cpp
  libcommon/src/util/FileOps.cpp
  libcommon/src/util/filesystem.cpp
  libcommon/src/util/RomAnalyzer.cpp
  libcommon/src/util/RomBlockStore.cpp
  libcommon/src/util/RomDatabase.cpp
  libcommon/src/util/host_profile.cpp
  libcommon/src/util/fileutil.cpp
  libcommon/src/util/logger.cpp
  libcommon/src/util/settings.cpp
)

target_include_directories(vcc-libcommon PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/libcommon/include
)

# Windows: the _WIN32 paths in the cartridge modules reference the real
# dialog and winapi helpers, so they join the portable set here (they
# are inert until a config menu opens them).
if(WIN32)
  target_sources(vcc-libcommon PRIVATE
    libcommon/src/util/DialogOps.cpp
    libcommon/src/util/winapi.cpp
  )
endif()

set_target_properties(vcc-libcommon PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ---- Emulator core
# Level-1/2 JIT backend: the arm64 emitter on Apple Silicon / arm64
# Linux, the x86-64 emitter on Intel/AMD (Windows, Linux, macOS), the
# always-refuse null backend (interpreter + block cache) everywhere
# else. BlockJit.cpp (x86-32) stays MSBuild-only.
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm64|aarch64|ARM64)$")
  set(VCC_BLOCKJIT_BACKEND BlockJitA64.cpp)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64)$")
  set(VCC_BLOCKJIT_BACKEND BlockJitX64.cpp)
else()
  set(VCC_BLOCKJIT_BACKEND BlockJitNull.cpp)
endif()

add_library(vcc-core STATIC
  # CPU cores + decode/dispatch
  hd6309.cpp
  mc6809.cpp
  OpDecoder.cpp
  OpCodeTables.cpp

  ${VCC_BLOCKJIT_BACKEND}
  Debugger.cpp
  DebuggerUtils.cpp

  # Chipset + cartridge bus
  coco3.cpp
  tcc1014mmu.cpp
  tcc1014graphics.cpp
  tcc1014registers.cpp
  mc6821.cpp
  joystickinput.cpp
  iobus.cpp
  pakinterface.cpp
)

target_link_libraries(vcc-core PUBLIC vcc-libcommon)
if(WIN32)
  # joystickinput.cpp's _WIN32 half is the DirectInput scan path.
  target_link_libraries(vcc-core PUBLIC dinput8 dxguid)
endif()

# ---- Cartridge modules: shared libraries loaded at runtime through the
# LoadLibrary->dlopen shim, so the same vcc.ini module names work on
# every host. Output as plain "<name>.dylib"/"<name>.so" beside the
# executable — the shim resolves "harddisk.dll" to that.
function(vcc_cartridge_module name)
  add_library(${name} MODULE ${ARGN})
  target_link_libraries(${name} PRIVATE vcc-libcommon)
  set_target_properties(${name} PROPERTIES PREFIX "")
endfunction()

vcc_cartridge_module(harddisk
  HardDisk/harddisk.cpp
  HardDisk/cc3vhd.cpp
)
target_include_directories(harddisk PRIVATE HardDisk)
if(WIN32)
  # PakGetName/PakGetCatalogId/PakGetDescription read string resources
  # on _WIN32 (non-Windows builds return hardcoded strings instead), so
  # each module needs its .rc compiled in or the config sections it
  # reads/writes end up named "".
  target_sources(harddisk PRIVATE HardDisk/harddisk.rc)
endif()

vcc_cartridge_module(fd502
  FD502/fd502.cpp
  FD502/wd1793.cpp
  FD502/distortc.cpp
  FD502/becker.cpp
)
target_include_directories(fd502 PRIVATE FD502)
if(WIN32)
  target_link_libraries(fd502 PRIVATE ws2_32)   # becker's winsock half
  target_sources(fd502 PRIVATE FD502/fd502.rc)  # string/dialog resources
endif()

vcc_cartridge_module(mpi
  mpi/mpi.cpp
  mpi/multipak_cartridge.cpp
  mpi/multipak_configuration.cpp
  mpi/cartridge_slot.cpp
)
target_include_directories(mpi PRIVATE mpi)
if(WIN32)
  target_sources(mpi PRIVATE mpi/configuration_dialog.cpp mpi/mpi.rc)
endif()

# ---- Shells. shell/ holds the machine bootstrap and device-absent
# stubs shared by every non-Windows front end; each shell adds its own
# keyboard and audio.
add_library(vcc-shell STATIC
  shell/machine.cpp
  shell/common_stubs.cpp
  shell/text_codec.cpp
)
target_link_libraries(vcc-shell PUBLIC vcc-core)

# libutf (optional; https://github.com/brazilofmux/utf): perceptual
# UTF-8 -> ASCII approximation for clipboard paste. Without it, paste
# still works but non-ASCII clipboard text is dropped instead of
# approximated.
set(VCC_LIBUTF_DIR "$ENV{HOME}/utf" CACHE PATH "libutf checkout (optional)")
if(EXISTS "${VCC_LIBUTF_DIR}/libutf.a" AND EXISTS "${VCC_LIBUTF_DIR}/include/utf/color_ops.h")
  message(STATUS "libutf found at ${VCC_LIBUTF_DIR} - UTF-8 approximation enabled")
  target_compile_definitions(vcc-shell PRIVATE VCC_HAVE_LIBUTF)
  target_include_directories(vcc-shell PRIVATE "${VCC_LIBUTF_DIR}/include")
  target_link_libraries(vcc-shell PUBLIC "${VCC_LIBUTF_DIR}/libutf.a")
else()
  message(STATUS "libutf not found - paste drops non-ASCII")
endif()

add_executable(vcc-headless headless/main.cpp headless/stubs.cpp)
target_link_libraries(vcc-headless PRIVATE vcc-shell)
add_dependencies(vcc-headless harddisk fd502 mpi)

# SDL shell: built when SDL2 is available (brew install sdl2; on
# Windows, unzip SDL2-devel-<ver>-VC.zip into external/ and its cmake
# config is picked up from here).
if(WIN32)
  file(GLOB _vcc_sdl2_local "${CMAKE_CURRENT_SOURCE_DIR}/external/SDL2-*")
  if(_vcc_sdl2_local)
    list(GET _vcc_sdl2_local 0 _vcc_sdl2_root)
    list(APPEND CMAKE_PREFIX_PATH "${_vcc_sdl2_root}")
  endif()
endif()
find_package(SDL2 QUIET)
if(SDL2_FOUND)
  add_executable(vcc-sdl sdl/main.cpp)
  target_link_libraries(vcc-sdl PRIVATE vcc-shell SDL2::SDL2)
  if(TARGET SDL2::SDL2main)
    # Windows: SDL.h renames main to SDL_main; SDL2main supplies the
    # real entry point that calls it.
    target_link_libraries(vcc-sdl PRIVATE SDL2::SDL2main)
  endif()
  add_dependencies(vcc-sdl harddisk fd502 mpi)
  if(WIN32)
    add_custom_command(TARGET vcc-sdl POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:SDL2::SDL2> $<TARGET_FILE_DIR:vcc-sdl>)
  endif()

  # VCC.app: the same shell as a double-clickable macOS bundle. The
  # cartridge modules and any disk ROMs sitting in the build dir are
  # copied beside the binary, where the dlopen shim and fd502's
  # exedir search expect them. Config still comes from
  # ~/.config/vcc/vcc.ini (the bare-launch path), so Finder launches
  # find everything. Icon: the DECB boot screen, naturally.
  if(APPLE)
    add_executable(vcc-app MACOSX_BUNDLE sdl/main.cpp
                   ${CMAKE_CURRENT_SOURCE_DIR}/sdl/resources/vcc.icns)
    target_link_libraries(vcc-app PRIVATE vcc-shell SDL2::SDL2)
    add_dependencies(vcc-app harddisk fd502 mpi)
    set_target_properties(vcc-app PROPERTIES
      OUTPUT_NAME "VCC"
      MACOSX_BUNDLE_BUNDLE_NAME "VCC"
      MACOSX_BUNDLE_GUI_IDENTIFIER "org.tinymux.vcc"
      MACOSX_BUNDLE_BUNDLE_VERSION "2.1.9"
      MACOSX_BUNDLE_SHORT_VERSION_STRING "2.1.9"
      MACOSX_BUNDLE_ICON_FILE "vcc.icns")
    set_source_files_properties(
      ${CMAKE_CURRENT_SOURCE_DIR}/sdl/resources/vcc.icns
      PROPERTIES MACOSX_PACKAGE_LOCATION "Resources")
    add_custom_command(TARGET vcc-app POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:harddisk> $<TARGET_FILE:fd502> $<TARGET_FILE:mpi>
        $<TARGET_FILE_DIR:vcc-app>
      COMMAND sh -c "cp -f '${CMAKE_BINARY_DIR}'/*.rom \"$<TARGET_FILE_DIR:vcc-app>/\" 2>/dev/null || true"
      VERBATIM)
  endif()
else()
  message(STATUS "SDL2 not found - skipping vcc-sdl")
endif()
