#ifndef _ZCM_ACCUMULATOR_HPP_
#define _ZCM_ACCUMULATOR_HPP_

#include "common/messages/roomba_t.hpp"
#include "common/messages/roomba_list_t.hpp"
#include "common/messages/line_t.hpp"
#include "common/messages/visual_data_t.hpp"
#include "common/messages/visual_landmark_t.hpp"
#include <mutex>
#include <thread>
#include <vector>
#include <deque>
#include <condition_variable>
#include <map>
#include <atomic>
#include <cstdint>
#include <iostream>
#include <chrono>
#include <functional>

// To use properly, first insert timestamp of frame to be processed
// then insert the data after it is processed
// then extract to the outside
class ZCMAccumulator
{
	public:
		// Default contructor for this class, initializes the
		// atomic_bools as false
		ZCMAccumulator();
		// Returns true if messages are ready to be sent
		// else returns false
		bool ready();
		// Initializes the bin corresponding to timestamp with a value
		// of one, if already a bin exists corresponding to timestamp
		// then its counter is incremented
		void insertr(const int64_t timestamp);
		// Initializes the bin corresponding to timestamp with a value
		// of one, if already a bin exists corresponding to timestamp
		// then its counter is incremented
		void insertl(const int64_t timestamp);
		// Add roombas to their corresponding bin and decrement the
		// bin's counter, if counter == 0 after this, calls
		// extractToReadyR()
		void insert(const roomba_list_t &src);
		// Add lines to their corresponding bin and decrement the bin's
		// counter and adds this message's roombas to the bin
		// if counter == 0 after this, calls extractToReadyL()
		void insert(const visual_data_t &src);
		// Decrements the counter on the roombas bin corresponding to
		// timestamp and adds this messages lines to the bin if
		// counter == 0 after this, calls extractToReadyR()
		void notifyR(const int64_t &timestamp);
		// Decrements the counter on the lines bin corresponding to
		// timestamp, if counter == 0 after this, calls
		// extractToReadyL()
		void notifyL(const int64_t &timestamp);
		// If the bin corresponding to timestamp is empty, then simply
		// discards it and returns, else, will add to readyRoombas
		// deque, awaken the sender thread, and remove bin
		bool extractToReadyR(const int64_t &timestamp);
		// If the bin corresponding to timestamp is empty, then simply
		// discards it and returns, else, will add to readyLines
		// deque, awaken the sender thread, and remove bin
		bool extractToReadyL(const int64_t &timestamp);
		// If there are none to extract, does not extract and returns
		// false, else, copies from front of readyRoombas and removes
		// from the ready roombas deque and returns true
		bool extract(roomba_list_t &dst);
		// If there are none to extract does not extract and returns
		// false, else, copies from front of readyLines and removes
		// from the readyLines deque and returns true
		bool extract(visual_data_t &dst);
		// Function used to force object into ready state
		void forceReady();
		// Mutex used with the condition_variable
		std::mutex cvMtx;
		// condition variable used to control sender thread
		std::condition_variable cv;
		// Used to send singletons to ready does nothing for pairs
		void clearTime(int64_t timestamp);
	private:
		// Implementation of clearTime to prevent blocking of reader thread
		static void clearTimeImpl(ZCMAccumulator *ptr, int64_t timestamp);
		// Allow ready() to not require lock_guards
		std::atomic_bool roombasReady;
		// Allow ready() to not require lock_guards
		std::atomic_bool linesReady;
		// Used to notify zcm_acc to close properly
		std::atomic_bool destructionReady;
		// Mutex for locking just the Roombas bins
		std::mutex mtx1;
		// Mutex for locking readyRoombas
		std::mutex mtx2;
		// Mutex for locking just the visual_data_t bins
		std::mutex mtx3;
		// Mutex for locking readyLines
		std::mutex mtx4;
		// Holds whether associated bin is a single or double
		std::map<int64_t,bool> isSingler;
		// Holds whether associated bin is a single or double
		std::map<int64_t,bool> isSinglel;
		// Holds whether a message has been received with associated bin
		std::map<int64_t,bool> receivedOner;
		// Holds whether a message has been received with associated bin
		std::map<int64_t,bool> receivedOnel;
		// Holds the counter for each Roombas bin
		std::map<int64_t, unsigned int> binCounterR;
		// Holds the counter for each visual_data_t bin
		std::map<int64_t, unsigned int> binCounterL;
		// Holds the Roombas bins
		std::map<int64_t, roomba_list_t> roombasBins;
		// Holds the visual_data_t bins
		std::map<int64_t, visual_data_t> linesBins;
		// Holds ready Roombas messages
		std::deque<roomba_list_t> readyRoombas;
		// Holds ready visual_data_t messages
		std::deque<visual_data_t> readyLines;
		// Function used for debugging
		void outputBinSizes();
		// Used to prevent insert from blocking
		// pass in this and the timestamp
		static void insertTimer(ZCMAccumulator* ptr, int64_t timestamp);
		// Used to prevent insert from blocking
		// pass in this and the timestamp
		static void insertTimel(ZCMAccumulator* ptr, int64_t timestamp);
};


#endif
