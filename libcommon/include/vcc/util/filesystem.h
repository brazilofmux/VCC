#pragma once
#include <string>
#include <vcc/util/host_services.h>

//TODO: depreciate this - it is used only in mpi/multipak_cartridge.cpp:
namespace VCC::Util
{
	std::string find_pak_module_path(std::string path);
}
