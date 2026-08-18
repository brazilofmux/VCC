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
#pragma once
#include <mutex>

namespace VCC::Util
{

	// Formerly a Win32 CRITICAL_SECTION wrapper; std::recursive_mutex
	// keeps the same semantics (CRITICAL_SECTION is recursive) on every
	// host.
	class critical_section
	{
	public:

		critical_section() = default;

		critical_section(const critical_section&) = delete;
		critical_section& operator=(const critical_section&) = delete;

		void lock() const
		{
			mutex_.lock();
		}

		void unlock() const
		{
			mutex_.unlock();
		}


	private:

		mutable std::recursive_mutex mutex_;
	};


	class section_locker
	{
	public:

		explicit section_locker(const critical_section& section)
			: section_(section)
		{
			section_.lock();
		}

		~section_locker()
		{
			section_.unlock();
		}

		section_locker& operator=(const section_locker&) = delete;
		section_locker& operator=(section_locker&&) = delete;


	private:

		const critical_section& section_;
	};

}
