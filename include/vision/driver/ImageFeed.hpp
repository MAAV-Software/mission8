#ifndef _Image_Feed_HPP_
#define _Image_Feed_HPP_

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <thread>
#include <vector>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "DataCoordinator.hpp"

// Helper class of vision_driver, holds one image and timestamp
// Can be used with multiple calls retrieving data from it.
class ImageFeed
{
   public:
	// Constructor for Image Feed requires min size to write to file
	// and min size for delete from deque as arguments
	ImageFeed(size_t writeSizeIn, size_t deleteSizeIn, int IDIn);
	// Constructor for ImageFeed that is used when using DataCoordinator
	// will spawn writer thread as well when called
	ImageFeed(size_t writeSizeIn, size_t deleteSizeIn, std::shared_ptr<DataCoordinator> dataCoordIn,
			  int IDIn);
	// Stores a frame and timestamp in the Image_Feed adding it to the front of
	// the deque and removing from the back if the deque exceeds the max size
	void storeFrame(std::shared_ptr<cv::Mat> &input, int64_t inTime);
	// Attempts to get a new frame from Image_Feed, if frame is the
	// same as the last time this was called, then
	// false is returned and a new frame is not obtained, otherwise
	// returns false
	bool getFrame(std::shared_ptr<cv::Mat> &input, int64_t &inTime,
				  std::shared_ptr<std::shared_mutex> &outMtx);
	// Returns whether a new frame is ready based on the timestamp if so,
	// returns true, else returns false
	bool ready(int64_t inTime);
	// forces the object into a ready state allowing threads to finish
	void forceReady();
	// Conditional variable used to control the activation of
	// threads that use this instance of Image_Feed
	std::condition_variable cv;
	// Conditional variable used to control the activation of an
	// image writer thread that would pop from the back of the deque
	std::condition_variable cvWriter;
	// Mutexes used with the conditional variable above to control
	// the activation of threads that use this instance of
	// Image_Feed, one for each worker thread that will be working
	// off of it.
	std::mutex cvMtxR;
	std::mutex cvMtxL;
	// Thread that is used to run DataCoordinator when is test == true
	// is controlled by size of deque and maximum deque size for write mode
	// Will finish writing in destructor and possibly start earlier if max
	// size of deque is reached
	static void dataCoordThread(std::shared_ptr<DataCoordinator> dataCoord, ImageFeed *imgFeed);
	// Gets the size of the frames dequei and returns it
	size_t getFramesSize();
	// Sets the associated cameraID any time after construction
	void setID(int newID);
	// Gets the value of the cameraID associated to this ImageFeed
	int getID();
	// Destructor, will either write frames at destruction or delete depending on
	// the hasDataCoord bool
	~ImageFeed();

   private:
	// Deletes the frame and timestamp at the back of the deque
	void deleteFrame();
	// Holds the storeFrameImpl thread that was last executed
	std::thread currentThread;
	// Holds whether currentThread has something that can potentially be joined
	std::atomic<bool> joinable;
	// Extracts shared pointer to frame and timestamp of frame in back of deque
	// Use with data coordinator to write to video or image
	void extractToWrite(std::shared_ptr<cv::Mat> &output, int64_t &timestamp,
						std::shared_ptr<std::shared_mutex> &sharedMtxPtr);
	// stores last frame from camera
	std::deque<std::shared_ptr<cv::Mat>> frames;
	// stores last timestamp from camera
	std::deque<int64_t> timestamps;
	// Stores shared mutexes for each frame in frames and
	std::deque<std::shared_ptr<std::shared_mutex>> sharedMutexes;
	// Shared mutex to write and read from the deque
	std::shared_mutex sharedMtx;
	// Called by insert to prevent blocking upon insertion
	static void storeFrameImpl(std::shared_ptr<cv::Mat> input, int64_t inTime, ImageFeed *imgFeed);
	// Stores the size required before pre destruction writting begins
	size_t writeSize;
	// Stores the size required before pre destruction deletion of frames
	size_t deleteSize;
	// Is set to true when destructor runs so that the thread will continue until the deque
	// is completely empty (exit while loop basically)
	std::atomic<bool> isDestructing;
	// Is true if there is an instance of DataCoordinator associated with this object
	// False if there isn't
	std::atomic<bool> hasDataCoord;
	// Holds the camera ID associated to this Image Feed
	int ImageFeedID;
	// Holds the DataCoord thread if there is one
	std::thread dataCoordThreadHolder;
};

#endif
