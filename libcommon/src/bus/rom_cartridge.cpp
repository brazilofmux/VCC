//#define USE_LOGGING
////////////////////////////////////////////////////////////////////////////////
//	Copyright 2015 by Joseph Forgione
//	This file is part of VCC (Virtual Color Computer).
//	
//	VCC (Virtual Color Computer) is free software: you can redistribute itand/or
//	modify it under the terms of the GNU General Public License as published by
//	the Free Software Foundation, either version 3 of the License, or (at your
//	option) any later version.
//	
//	VCC (Virtual Color Computer) is distributed in the hope that it will be
//	useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
//	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
//	Public License for more details.
//	
//	You should have received a copy of the GNU General Public License along with
//	VCC (Virtual Color Computer). If not, see <http://www.gnu.org/licenses/>.
////////////////////////////////////////////////////////////////////////////////
#include <vcc/bus/rom_cartridge.h>
#include <vcc/util/logger.h>
#include <vcc/util/RomDatabase.h>
#include <Windows.h>
#include <cstdio>

namespace VCC::Core
{

	rom_cartridge::rom_cartridge(
		std::unique_ptr<callbacks_type> callbacks,
		name_type name,
		catalog_id_type catalog_id,
		buffer_type buffer,
		bool enable_bank_switching)
		:
		callbacks_(move(callbacks)),
		name_(move(name)),
		catalog_id_(move(catalog_id)),
		buffer_(move(buffer)),
		enable_bank_switching_(enable_bank_switching),
		bank_offset_(0)
	{
		DLOG_C("rom_cartridge ctor type: %s cart ptr: %p buf siz: %zu\n",
			typeid(*this).name(), this, buffer_.size());

		// Identify the cartridge ROM by content fingerprint and log it.
		// Cartridge ROMs may be bank-switched (the same buffer is mapped at
		// different offsets) - the fingerprint always covers the full byte
		// sequence, so banking doesn't affect identification.
		if (!buffer_.empty())
		{
			VCC::RomInfo info = VCC::IdentifyRom(buffer_.data(), buffer_.size());
			char dbg[256];
			snprintf(dbg, sizeof(dbg),
				"[ROM] cartridge \"%s\": %s size=%u crc=0x%08X bankswitch=%d\n",
				name_.c_str(), info.name, (unsigned)info.size,
				info.fingerprint, enable_bank_switching ? 1 : 0);
			OutputDebugStringA(dbg);
		}
	}


	rom_cartridge::name_type rom_cartridge::name() const
	{
		return name_;
	}

	rom_cartridge::catalog_id_type rom_cartridge::catalog_id() const
	{
		return catalog_id_;
	}

	rom_cartridge::description_type rom_cartridge:: description() const
	{
		return {};
	}


	void rom_cartridge::reset()
	{
		bank_offset_ = 0;
	}

	void rom_cartridge::write_port(unsigned char port_id, unsigned char value)
	{
		if (enable_bank_switching_ && port_id == 0x40)
		{
			bank_offset_ = (value & 0x0f) << 14;
		}
	}

	unsigned char rom_cartridge::read_memory_byte(unsigned short memory_address)
	{
		return buffer_[((memory_address & 32767) + bank_offset_) % buffer_.size()];
	}

	void rom_cartridge::initialize_bus()
	{
		DLOG_C("rom_cartridge::initialize_bus: this=%p callbacks=%p\n",this,callbacks_.get());
		callbacks_->assert_cartridge_line(true);
	}

}
