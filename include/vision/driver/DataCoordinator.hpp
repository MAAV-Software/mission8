#ifndef _DATA_COORDINATOR_HPP_
#define _DATA_COORDINATOR_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "../../../atomcore/src/zcmtypes/line_t.hpp"
#include "../../../atomcore/src/zcmtypes/roomba_list_t.hpp"
#include "../../../atomcore/src/zcmtypes/roomba_t.hpp"
#include "../../../atomcore/src/zcmtypes/visual_data_t.hpp"
#include "../../../atomcore/src/zcmtypes/visual_landmark_t.hpp"
#include "zcm/zcm-cpp.hpp"

// ZCM Handler class that is used in a zcm_listener thread
class RoombaHandler
{
   public:
	std::deque<roomba_list_t> roombas;
	std::condition_variable cv;
	std::mutex mtx;
	std::mutex cvMtx;
	std::unique_lock<std::mutex> lck;
	RoombaHandler() { lck = std::unique_lock<std::mutex>(cvMtx); }
	void handleMessage(const zcm::ReceiveBuffer *rbuf, const std::string &chan,
					   const roomba_list_t *msg)
	{
		std::lock_guard<std::mutex> l(mtx);
		roombas.push_front(*msg);
		cv.notify_one();
	}
	size_t getSize()
	{
		std::lock_guard<std::mutex> l(mtx);
		return roombas.size();
	}
	roomba_list_t &getData()
	{
		std::lock_guard<std::mutex> l(mtx);
		return roombas.back();
	}
	void deleteData()
	{
		std::lock_guard<std::mutex> l(mtx);
		roombas.pop_back();
	}
};

// ZCM Handler class that is used in a zcm_listener thread
class LineHandler
{
	// accessed in listener thread to insert to data_coord
   public:
	std::deque<visual_data_t> lineInfo;
	std::condition_variable cv;
	std::mutex cvMtx;
	std::unique_lock<std::mutex> lck;
	std::mutex mtx;
	LineHandler() { lck = std::unique_lock<std::mutex>(cvMtx); }
	void handleMessage(const zcm::ReceiveBuffer *rbuf, const std::string &chan,
					   const visual_data_t *msg)
	{
		std::lock_guard<std::mutex> l(mtx);
		lineInfo.push_front(*msg);
		cv.notify_one();
	}
	size_t getSize()
	{
		std::lock_guard<std::mutex> l(mtx);
		return lineInfo.size();
	}
	visual_data_t &getData()
	{
		std::lock_guard<std::mutex> l(mtx);
		return lineInfo.back();
	}
	void deleteData()
	{
		std::lock_guard<std::mutex> l(mtx);
		lineInfo.pop_back();
	}
};

class DataCoordinator
{
   public:
	// Constructor for this object, pass in the timeout delay
	DataCoordinator(int readerDelayIn, int bidxCont, bool recordIn, int lineThicknessIn,
					int roombaSizeIn, bool trackTimeIn, int typeIn);
	// Creates a bin for this frame based on timestamp and camID
	// Any roombas or lines found will be added to this bin for this
	// camID and timestamp, also spawns a detached thread that will
	// write this frame to the video file upon timeout
	void insert(std::shared_ptr<cv::Mat> &framePtr, int64_t timestamp, int camID,
				std::shared_ptr<std::shared_mutex> &sharedMtx);
	// Draws the found roombas on the frames in the correct bins
	void insert(roomba_list_t &src);
	// Draws the found lines on the frames in the correct bins
	void insert(visual_data_t &src);
	// zcm listener thread for roomba_list_t messages
	// recieves "Roomba" messages and writes data to frame
	static void listenerThreadR(DataCoordinator *dataCoord, RoombaHandler *roombaHandler,
								std::atomic<bool> *isRunning, zcm::ZCM *zcmPtr);
	// zcm listener thread for visual_data_t messages
	// recieves "Lines" messages and writes data to frame
	static void listenerThreadL(DataCoordinator *dataCoord, LineHandler *lineHandler,
								std::atomic<bool> *isRunning, zcm::ZCM *zcmPtr);
	// Creates the zcm_listener threads and detaches them
	void startZCM(zcm::ZCM *zcmPtr);
	// Destructor for this class stops zcm threads and closes videos
	~DataCoordinator();

   private:
	void drawRoombas(std::shared_ptr<cv::Mat> &frame, roomba_list_t &src, int camID);
	// Called to draw roombas on a frame
	void drawLines(std::shared_ptr<cv::Mat> &frame, visual_data_t &src, int camID);
	// Called to draw lines and landmarks on a frame
	void write(std::shared_ptr<cv::Mat> &framePtr, int64_t &timestamp, int camID);
	// Writes frame to file or video
	std::atomic<bool> isRunning;
	// Turns off the threads in the destructor
	bool record;
	// Controls whether roombas and lines are drawn on the frames
	int lineThickness;
	// Controls how thick the drawn lines are on the frames
	int roombaSize;
	// Contorls how large the circle representing roomba centroids is
	bool trackTime;
	// Controls whether timestamps of each frame are kept track of
	int type;
	// Controls whether videos are written or many images to the dst
	int counter[5];
	// Controls the naming of image files for each camera
	std::map<int64_t, visual_data_t> visualData;
	// Holds the currently unwritten visual_data_t messages received
	std::map<int64_t, roomba_list_t> roombaData;
	// Holds the currently unwritten roomba_list_t messages received
	cv::VideoWriter writer[5];
	// Videowriters, need to be opened in the constructor
	std::ofstream fout[5];
	// filestreams where timestamps are written for each frame
	// index corresponds to VideoWriter index
	RoombaHandler roombaHandler;
	// ZCM Handler class for roomba_list_t messages
	LineHandler lineHandler;
	// ZCM Handler class for visual_data_t messages
	std::vector<std::thread> threads;
	// Holds threads for later in the destructor
	std::mutex mtx1;
	// Acquired for altering data in object
};
#endif
