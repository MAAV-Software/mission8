#ifndef _TIME_INTERFACE_HPP_
#define _TIME_INTERFACE_HPP_

#include <iostream>
#include <cstdint>
#include <utility>
#include <map>

#include <atomic>
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <memory>

#include <zcm/zcm-cpp.hpp>
#include "common/messages/sim_time_t.hpp"

class TimeInterface
{
	private:
		class Impl;
		class NormalTime;
		class SimTimeInterface;
		class HandlerSimTime;
		std::unique_ptr<Impl> pimpl;
	public:
		// type 0 and 1 are normal functions
		// type 2 uses sim_time_interface
		TimeInterface(int type);
		// For type 2 this starts the zcm thread for others nothing
		void init();
		// sleeps for duration ms in normal or duration sim ms in type 2
		void sleep(int duration);
		// Sets time to the current time in ms type 0&1 or sim in type 2
		void getTime(int64_t &time);
		// Creates the type 2 version correctly
		static void createType2(TimeInterface *&timer);
		// Destructor for this class
		~TimeInterface();
};
#endif
