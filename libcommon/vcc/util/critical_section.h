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
#include <atomic>
#include <thread>

namespace VCC::Util
{

	// Recursive mutual exclusion (Win32 CRITICAL_SECTION semantics)
	// with an uncontended fast path of one CAS plus plain stores. The
	// pak bus takes this lock every emulated scanline and every port
	// access; as a pthread-backed mutex that traffic was ~30% of
	// CPU-bound wall time on macOS. Contention (emulation thread vs an
	// occasional config/UI operation) is rare and short-lived, so a
	// spin-then-yield wait is the right shape.
	class critical_section
	{
	public:

		critical_section() = default;

		critical_section(const critical_section&) = delete;
		critical_section& operator=(const critical_section&) = delete;

		void lock() const
		{
			// this_thread::get_id() resolves through pthread_self each
			// call; cache it per thread (visible in profiles at tens of
			// millions of locks per second).
			static const thread_local std::thread::id self = std::this_thread::get_id();
			if (owner_.load(std::memory_order_relaxed) == self)
			{
				++depth_;
				return;
			}
			std::thread::id expected {};
			int spins = 0;
			while (!owner_.compare_exchange_weak(expected, self,
			                                     std::memory_order_acquire,
			                                     std::memory_order_relaxed))
			{
				expected = {};
				if (++spins > 64)
				{
					std::this_thread::yield();
					spins = 0;
				}
			}
			depth_ = 1;
		}

		void unlock() const
		{
			if (--depth_ == 0)
				owner_.store(std::thread::id{}, std::memory_order_release);
		}


	private:

		// depth_ is only touched by the owning thread, so it needs no
		// atomicity of its own.
		mutable std::atomic<std::thread::id> owner_ {};
		mutable int depth_ = 0;
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
