# CMake build for the portable core (docs/porting-macos.md stage 1).
#
# This exists ALONGSIDE Vcc.sln — the MSBuild path is untouched and CI
# (BuildAll.bat) keeps using it. This file builds only the portable set:
# CPU cores, chipset, block cache, libcommon, and a headless driver.
# The Win32 shell (Vcc.cpp, config.cpp, DirectX/DirectSound, dialogs,
# cart-module DLLs) is deliberately absent; the SDL shell replaces it.

cmake_minimum_required(VERSION 3.20)
project(vcc-core CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_library(vcc-core STATIC
  # CPU cores + decode/dispatch
  hd6309.cpp
  mc6809.cpp
  OpDecoder.cpp
  OpCodeTables.cpp

  BlockJitNull.cpp
  Debugger.cpp
  DebuggerUtils.cpp

  # Chipset
  coco3.cpp
  tcc1014mmu.cpp
  tcc1014graphics.cpp
  tcc1014registers.cpp
  mc6821.cpp
  iobus.cpp

  # libcommon (portable subset; DialogOps/winapi/FileOps are Win32-bound)
  libcommon/src/bus/basic_cartridge.cpp
  libcommon/src/bus/cartridge_loader.cpp
  libcommon/src/bus/cartridge_menu.cpp
  libcommon/src/bus/cpak_cartridge.cpp
  libcommon/src/bus/rom_cartridge.cpp
  libcommon/src/devices/cloud9.cpp
  libcommon/src/util/RomAnalyzer.cpp
  libcommon/src/util/RomBlockStore.cpp
  libcommon/src/util/RomDatabase.cpp
  libcommon/src/util/host_profile.cpp
  libcommon/src/util/fileutil.cpp
  libcommon/src/util/logger.cpp
  libcommon/src/util/settings.cpp
)

target_include_directories(vcc-core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/libcommon/include
)

add_executable(vcc-headless headless/main.cpp headless/stubs.cpp)
target_link_libraries(vcc-headless PRIVATE vcc-core)
