#ifndef _VEHICLESTATEREPO_HPP_
#define _VEHICLESTATEREPO_HPP_

#include "zcm/zcm-cpp.hpp"
#include "common/messages/tracked_state_t.hpp"
#include "common/messages/state_tracker_t.hpp"
#include <vector>
#include <mutex>
#include <cstdint>

class VehicleStateRepo
{
	public:
		VehicleStateRepo() = default;
		void handleMessage(const zcm::ReceiveBuffer *rbuf,
			const std::string &chan, const state_tracker_t *msg);
		bool getState(std::vector<double> &input, int64_t timestamp);
	private:
		state_tracker_t stateHistory;
		std::mutex mtx;
		bool getTimeDiff(int index, int64_t timestamp, int64_t &timeDiff);
};
#endif
