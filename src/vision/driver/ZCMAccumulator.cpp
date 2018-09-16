#include "vision/driver/ZCMAccumulator.hpp"

using namespace std;

ZCMAccumulator::ZCMAccumulator() :
	roombasReady {false}, linesReady {false}, destructionReady {false} { }

bool ZCMAccumulator::ready()
{
	return roombasReady || linesReady || destructionReady;
}

void ZCMAccumulator::insertr(const int64_t timestamp)
{
	// using detached prevents blockages
	thread(insertTimer,this,timestamp).detach();
}

void ZCMAccumulator::insertl(const int64_t timestamp)
{
	// using detached prevents blockages
	thread(insertTimel,this,timestamp).detach();
}

void ZCMAccumulator::insertTimer(ZCMAccumulator* ptr, int64_t timestamp)
{
	std::lock_guard<std::mutex> l(ptr->mtx1);
	if (ptr->binCounterR.find(timestamp) == ptr->binCounterR.end())
	{
		ptr->binCounterR[timestamp] = 1;
	}
	else
	{
		++ptr->binCounterR[timestamp];
	}
	auto itr = ptr->isSingler.find(timestamp);
	if (itr != ptr->isSingler.end())
	{
		itr->second = false;
	}
	else
	{
		ptr->isSingler[timestamp] = true;
	}
	auto itr2 = ptr->receivedOner.find(timestamp);
	if (itr2 == ptr->receivedOner.end())
	{
		ptr->receivedOner[timestamp] = false;
	}
}

void ZCMAccumulator::insertTimel(ZCMAccumulator* ptr, int64_t timestamp)
{
	std::lock_guard<std::mutex> l(ptr->mtx3);
	if (ptr->binCounterL.find(timestamp) == ptr->binCounterL.end())
	{
		ptr->binCounterL[timestamp] = 1;
	}
	else
	{
		++ptr->binCounterL[timestamp];
	}
	auto itr = ptr->isSinglel.find(timestamp);
	if (itr != ptr->isSinglel.end())
	{
		itr->second = false;
	}
	else
	{
		ptr->isSinglel[timestamp] = true;
	}
	auto itr2 = ptr->receivedOnel.find(timestamp);
	if (itr2 == ptr->receivedOnel.end())
	{
		ptr->receivedOnel[timestamp] = false;
	}
}

void ZCMAccumulator::insert(const roomba_list_t &src)
{
	std::lock_guard<std::mutex> l(mtx1);
	auto itr = roombasBins.find(src.utime);
	if (itr == roombasBins.end())
	{
		roombasBins[src.utime] = src;
	}
	else
	{
		int previousNum {(*itr).second.num_roombas};
		(*itr).second.num_roombas += src.num_roombas;
		(*itr).second.roombas.resize((*itr).second.num_roombas);
		for (int i {0}; i < src.num_roombas; ++i)
		{
			(*itr).second.roombas[previousNum + i] = src.roombas[i];
		}
	}
	receivedOner[src.utime] = true;
	if (!(--binCounterR[src.utime]) && !isSingler[src.utime])
	{
		if(extractToReadyR(src.utime))
			cv.notify_all();
	}
}

void ZCMAccumulator::insert(const visual_data_t &src)
{
	std::lock_guard<std::mutex> l(mtx3);
	auto itr = linesBins.find(src.utime);
	if (itr == linesBins.end())
	{
		linesBins[src.utime] = src;
	}
	else
	{
		// Set variables holding previous size
		int prevLines {(*itr).second.num_lines};
		int prevMarks {(*itr).second.num_landmarks};
		// Set new num of lines and landmarks in msg
		(*itr).second.num_lines += src.num_lines;
		(*itr).second.num_landmarks += src.num_landmarks;
		// Resize vectors of features
		(*itr).second.lines.resize((*itr).second.num_lines);
		(*itr).second.landmarks.resize((*itr).second.num_landmarks);
		for (int i {0}; i < src.num_lines; ++i)
		{
			(*itr).second.lines[prevLines + i] = src.lines[i];
		}
		for (int i {0}; i < src.num_landmarks; ++i)
		{
			(*itr).second.landmarks[prevMarks + i] = src.landmarks[i];
		}
	}
	receivedOnel[src.utime] = true;
	if (!(--binCounterL[src.utime]) && !isSinglel[src.utime])
	{
		if (extractToReadyL(src.utime))
			cv.notify_all();
	}
}

void ZCMAccumulator::notifyR(const int64_t &timestamp)
{
	std::lock_guard<std::mutex> l(mtx1);
	receivedOner[timestamp] = true;
	if (!(--binCounterR[timestamp]) && !isSingler[timestamp])
	{
		if(extractToReadyR(timestamp))
			cv.notify_all();
	}
}

void ZCMAccumulator::notifyL(const int64_t &timestamp)
{
	std::lock_guard<std::mutex> l(mtx3);
	receivedOnel[timestamp] = true;
	if (!(--binCounterL[timestamp]) && !isSinglel[timestamp])
	{
		if (extractToReadyL(timestamp))
			cv.notify_all();
	}
}

bool ZCMAccumulator::extractToReadyR(const int64_t &timestamp)
{
	if (roombasBins.find(timestamp) == roombasBins.end())
	{
		binCounterR.erase(timestamp);
		receivedOner.erase(timestamp);
		isSingler.erase(timestamp);
		return false;
	}
	else if (roombasBins[timestamp].num_roombas)
	{
		std::lock_guard<std::mutex> l(mtx2);
		readyRoombas.push_back(roombasBins[timestamp]);
		binCounterR.erase(timestamp);
		roombasBins.erase(timestamp);
		receivedOner.erase(timestamp);
		isSingler.erase(timestamp);
		roombasReady = true;
		return true;
	}
	else
	{
		binCounterR.erase(timestamp);
		roombasBins.erase(timestamp);
		isSingler.erase(timestamp);
		receivedOner.erase(timestamp);
		return false;
	}
}

bool ZCMAccumulator::extractToReadyL(const int64_t &timestamp)
{
	if (linesBins.find(timestamp) == linesBins.end())
	{
		binCounterL.erase(timestamp);
		isSinglel.erase(timestamp);
		receivedOnel.erase(timestamp);
		return false;
	}
	else if (linesBins[timestamp].num_lines)
	{
		std::lock_guard<std::mutex> l(mtx4);
		readyLines.push_back(linesBins[timestamp]);
		binCounterL.erase(timestamp);
		linesBins.erase(timestamp);
		isSinglel.erase(timestamp);
		receivedOnel.erase(timestamp);
		linesReady = true;
		return true;
	}
	else
	{
		binCounterL.erase(timestamp);
		linesBins.erase(timestamp);
		isSinglel.erase(timestamp);
		receivedOnel.erase(timestamp);
		return false;
	}
}

bool ZCMAccumulator::extract(roomba_list_t &dst)
{
	if (roombasReady)
	{
		std::lock_guard<std::mutex> l(mtx2);
		dst = readyRoombas.front();
		readyRoombas.pop_front();
		if (!readyRoombas.size())
			roombasReady = false;
		return true;
	}
	else
	{
		return false;
	}
}

bool ZCMAccumulator::extract(visual_data_t &dst)
{
	if (linesReady)
	{
		std::lock_guard<std::mutex> l(mtx4);
		dst = readyLines.front();
		readyLines.pop_front();
		if (!readyLines.size())
			linesReady = false;
		return true;
	}
	else
	{
		return false;
	}
}

void ZCMAccumulator::forceReady()
{
	destructionReady = true;
	cv.notify_one();
}

void ZCMAccumulator::clearTime(int64_t timestamp)
{
	thread(clearTimeImpl,this,timestamp).detach();
}

void ZCMAccumulator::clearTimeImpl(ZCMAccumulator *ptr, int64_t timestamp)
{
	// Check for single roombas
	{
		std::lock_guard<std::mutex> l(ptr->mtx1);
		bool pred = ptr->isSingler.find(timestamp) != ptr->isSingler.end();
		if (pred && ptr->isSingler[timestamp] && ptr->receivedOner[timestamp])
		{
			ptr->extractToReadyR(timestamp);
		}
	}
	// Check for single lines
	{
		std::lock_guard<std::mutex> l(ptr->mtx3);
		bool pred = ptr->isSinglel.find(timestamp) != ptr->isSinglel.end();
		if (pred && ptr->isSinglel[timestamp] && ptr->receivedOnel[timestamp])
		{
			ptr->extractToReadyL(timestamp);
		}
	}
}
