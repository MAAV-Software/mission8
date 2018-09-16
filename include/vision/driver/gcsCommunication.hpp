#ifndef _GCS_COMMUNICATION_HPP_
#define _GCS_COMMUNICATION_HPP_

#include "zcm/zcm-cpp.hpp"
#include <atomic>
#include "../../../atomcore/src/zcmtypes/nav_runstate_t.hpp"
#include "../../../atomcore/src/zcmtypes/camera_disc_t.hpp"
#include <mutex>
#include <condition_variable>

class runStateHandler
{
	public:
		std::atomic<bool> active;
		runStateHandler() = delete;
		explicit runStateHandler(std::condition_variable *cvIn) : active {false} ,cv {cvIn} {}
		void handleMessage(const zcm::ReceiveBuffer *rbuf,
			const std::string &chan, const nav_runstate_t *msg)
		{
			active  = msg->running_mission;
			cv->notify_one();
		}
		std::condition_variable *cv;
		std::mutex mtx;
};

class camIdentifyHandler
{
	public:
		std::atomic<int> numFound;
		camIdentifyHandler() : numFound {-1} { }
		std::mutex mtx;
		std::condition_variable cv;
		void handleMessage(const zcm::ReceiveBuffer *rbuf,
			const std::string &chan, const camera_disc_t *msg)
		{
			numFound = (int)msg->numCameras;
			cv.notify_one();
		}
};

#endif
