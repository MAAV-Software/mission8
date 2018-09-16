#include "vision/driver/ImageFeed.hpp"
#include <thread>

using namespace std;
using namespace cv;

ImageFeed::ImageFeed(size_t writeSizeIn, size_t deleteSizeIn, int IDIn)
	: joinable {false}, writeSize {writeSizeIn}, deleteSize {deleteSizeIn},
	isDestructing {false} , hasDataCoord {false}, ImageFeedID {IDIn} { }

ImageFeed::ImageFeed(size_t writeSizeIn, size_t deleteSizeIn, std::shared_ptr<DataCoordinator>
	dataCoordIn, int cameraID) : joinable {false}, writeSize {writeSizeIn},
	deleteSize {deleteSizeIn}, isDestructing {false}, hasDataCoord {false},
	ImageFeedID {cameraID}
{
	dataCoordThreadHolder = thread(dataCoordThread,dataCoordIn,this);
}

void ImageFeed::setID(int newID)
{
	std::lock_guard<std::shared_mutex> l(sharedMtx);
	ImageFeedID = newID;
}

void ImageFeed::storeFrame(shared_ptr<cv::Mat> &input, int64_t inTime)
{
	if (joinable)
	{
		currentThread.join();
		joinable = false;
	}
	currentThread = thread(storeFrameImpl,input,inTime,this);
	joinable = true;
}

void ImageFeed::storeFrameImpl(shared_ptr<cv::Mat> input, int64_t inTime,ImageFeed* imgFeed)
{
	std::lock_guard <std::shared_mutex> l(imgFeed->sharedMtx);
	imgFeed->frames.push_front(input);
	imgFeed->timestamps.push_front(inTime);
	imgFeed->sharedMutexes.push_front(make_shared<shared_mutex>());
	if (imgFeed->hasDataCoord && imgFeed->frames.size() > imgFeed->writeSize)
	{
		imgFeed->cvWriter.notify_one();
	}
	else if (imgFeed->frames.size() > imgFeed->deleteSize)
	{
		imgFeed->deleteFrame();
	}
	imgFeed->cv.notify_one();
	imgFeed->cv.notify_one();
}

size_t ImageFeed::getFramesSize()
{
	shared_lock<shared_mutex> l(sharedMtx);
	return frames.size();
}

void ImageFeed::dataCoordThread(shared_ptr<DataCoordinator> dataCoord, ImageFeed *imgFeed)
{
	imgFeed->hasDataCoord = true;
	// Mutex used only for condition_variable in this thread
	std::mutex mtx;
	std::unique_lock<std::mutex> lck(mtx);
	// Initialize variables for storing current frame to be written
	// and associated information
	int64_t timestamp;
	std::shared_ptr<cv::Mat> framePtr;
	std::shared_ptr<std::shared_mutex> sharedMtxSharedPtr;
	// Waits until writing is necessary
	while (!imgFeed->isDestructing && imgFeed->getFramesSize() < imgFeed->writeSize)
	{
		imgFeed->cvWriter.wait(lck);
	}
	std::cout << "ImageFeed " << imgFeed->getID() << " is begining to write.\n";
	// Writes when necessary
	while (!imgFeed->isDestructing)
	{
		while (!imgFeed->isDestructing && imgFeed->getFramesSize() < imgFeed->writeSize)
		{
			imgFeed->cvWriter.wait(lck);
		}
		imgFeed->extractToWrite(framePtr,timestamp,sharedMtxSharedPtr);
		dataCoord->insert(framePtr,timestamp,imgFeed->getID(),sharedMtxSharedPtr);
	}
	int counter {0};
	int last {0};
	while (imgFeed->getFramesSize())
	{
		imgFeed->extractToWrite(framePtr,timestamp,sharedMtxSharedPtr);
		dataCoord->insert(framePtr,timestamp,imgFeed->getID(),sharedMtxSharedPtr);
		if (++counter - last > 49)
		{
			last = counter;
			cout << "Writting frames " << counter << " and greater...\n";
		}
	}
	cout << "Data Coord Thread " <<  imgFeed->getID() << " finished...\n";
}

int ImageFeed::getID()
{
	std::shared_lock<std::shared_mutex> lck(sharedMtx);
	return ImageFeedID;
}

void ImageFeed::extractToWrite(shared_ptr<Mat> &framePtr, int64_t &timeOut,
	shared_ptr<shared_mutex> &sharedMtxPtr)
{
	std::lock_guard<std::shared_mutex> l(sharedMtx);
	framePtr = frames.back();
	frames.pop_back();
	timeOut = timestamps.back();
	timestamps.pop_back();
	sharedMtxPtr = sharedMutexes.back();
	sharedMutexes.pop_back();
}

void ImageFeed::deleteFrame()
{
	std::lock_guard<std::shared_mutex> l(sharedMtx);
	frames.pop_back();
	timestamps.pop_back();
	sharedMutexes.pop_back();
}

void ImageFeed::forceReady()
{
	std::lock_guard <std::shared_mutex> l(sharedMtx);
	shared_ptr<cv::Mat> tempPtr = make_shared<cv::Mat>();
	*tempPtr = Mat::ones(480,640,CV_8UC3);
	frames.push_front(tempPtr);
	sharedMutexes.push_front(make_shared<shared_mutex>());
	timestamps.push_front(999);
	cv.notify_one();
	cv.notify_one();
}

bool ImageFeed::getFrame(shared_ptr<Mat> &input, int64_t &inTime,
	shared_ptr<shared_mutex> &outMtx)
{
	std::shared_lock<std::shared_mutex> lck(sharedMtx);
	if (frames.size() && inTime != timestamps.front())
	{
		input = frames.front();
		inTime = timestamps.front();
		outMtx = sharedMutexes.front();
		return true;
	}
	else
	{
		return false;
	}
}

bool ImageFeed::ready(int64_t inTime)
{
	std::shared_lock<std::shared_mutex> lck(sharedMtx);
	return frames.size() && inTime != timestamps.front();
}

ImageFeed::~ImageFeed()
{
	if (joinable)
	{
		currentThread.join();
	}
	isDestructing = true;
	cvWriter.notify_one();
	if (hasDataCoord)
	{
		dataCoordThreadHolder.join();
	}
}
