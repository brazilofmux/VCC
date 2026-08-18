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

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
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

set_target_properties(vcc-libcommon PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ---- Emulator core
add_library(vcc-core STATIC
  # CPU cores + decode/dispatch
  hd6309.cpp
  mc6809.cpp
  OpDecoder.cpp
  OpCodeTables.cpp

  BlockJitNull.cpp
  Debugger.cpp
  DebuggerUtils.cpp

  # Chipset + cartridge bus
  coco3.cpp
  tcc1014mmu.cpp
  tcc1014graphics.cpp
  tcc1014registers.cpp
  mc6821.cpp
  iobus.cpp
  pakinterface.cpp
)

target_link_libraries(vcc-core PUBLIC vcc-libcommon)

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

vcc_cartridge_module(fd502
  FD502/fd502.cpp
  FD502/wd1793.cpp
  FD502/distortc.cpp
)
target_include_directories(fd502 PRIVATE FD502)

vcc_cartridge_module(mpi
  mpi/mpi.cpp
  mpi/multipak_cartridge.cpp
  mpi/multipak_configuration.cpp
  mpi/cartridge_slot.cpp
)
target_include_directories(mpi PRIVATE mpi)

# ---- Headless driver
add_executable(vcc-headless headless/main.cpp headless/stubs.cpp)
target_link_libraries(vcc-headless PRIVATE vcc-core)
add_dependencies(vcc-headless harddisk fd502 mpi)
