#include "vision/driver/VehicleStateRepo.hpp"

void VehicleStateRepo::handleMessage(const zcm::ReceiveBuffer *rbuf, const std::string &chan,
									 const state_tracker_t *msg)
{
	std::lock_guard<std::mutex> l(mtx);
	stateHistory = *msg;
}

bool VehicleStateRepo::getState(std::vector<double> &input, int64_t timestamp)
{
	std::lock_guard<std::mutex> l(mtx);
	if (stateHistory.states.size() < 1)
	{
		return false;
	}
	int lowerBound{0};
	int upperBound{stateHistory.NUM_STATES - 1};
	int index{upperBound / 2};
	while (true)
	{
		if (!(upperBound - lowerBound))
		{
			if (timestamp > stateHistory.states[index].utime)
			{
				lowerBound = index;
				index += (upperBound - lowerBound) / 2;
			}
			else
			{
				upperBound = index;
				index -= (upperBound - lowerBound) / 2;
			}
		}
		else if (timestamp == stateHistory.states[index].utime)
		{
			double *statePtr = stateHistory.states[index].state;
			input.assign(statePtr, statePtr + 13);
			return true;
		}
		else
		{
			int64_t timeDiffMid{timestamp - stateHistory.states[index].utime};
			int64_t otherTimeDiff{0};
			if (getTimeDiff(index + 1, timestamp, otherTimeDiff) && otherTimeDiff < timeDiffMid)
			{
				double *statePtr = stateHistory.states[index + 1].state;
				input.assign(statePtr, statePtr + 13);
				return true;
			}
			else if (getTimeDiff(index - 1, timestamp, otherTimeDiff) &&
					 otherTimeDiff < timeDiffMid)
			{
				double *statePtr = stateHistory.states[index - 1].state;
				input.assign(statePtr, statePtr + 13);
				return true;
			}
		}
	}
}

bool VehicleStateRepo::getTimeDiff(int index, int64_t timestamp, int64_t &timeDiff)
{
	if (index > stateHistory.NUM_STATES || index < 0)
	{
		return false;
	}
	else
	{
		timeDiff = timestamp - stateHistory.states[index].utime;
		if (timeDiff < 0)
		{
			timeDiff *= -1;
		}
		return true;
	}
}
