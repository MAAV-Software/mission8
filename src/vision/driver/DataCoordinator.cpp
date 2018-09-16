#include "vision/driver/DataCoordinator.hpp"
#include "common/messages/MsgChannels.hpp"

using cv::VideoWriter;
using std::shared_ptr;
using std::shared_mutex;
using cv::Mat;

DataCoordinator::DataCoordinator(int readerDelayIn, int bidxCont, bool recordIn,
	int lineThicknessIn, int roombaSizeIn, bool trackTimeIn, int typeIn)
	: isRunning {true}, record {recordIn},
	lineThickness {lineThicknessIn}, roombaSize {roombaSizeIn},
	trackTime {trackTimeIn}, type {typeIn}, counter {0,0,0,0,0}
{
	int fourcc = VideoWriter::fourcc('M','J','P','G');
	double fps = (1000 / readerDelayIn) / 4;
	if (type == 0)
	{
		writer[0].open("Output_File_0.avi",fourcc,fps * (4 / bidxCont),
			cv::Size(640,480));
		for (int i = 0; i < 5; ++i)
		{
			std::string name = "Output_File_";
			name += std::to_string(i);
			name += ".avi";
			writer[i].open(name,fourcc,fps,cv::Size(640,480));
		}
	}
	if (trackTime)
	{
		for (int i = 0; i < 5; ++i)
		{
			std::string name = "OutTimestamps";
			name += std::to_string(i);
			name += ".txt";
			fout[i].open(name.c_str());
		}
	}
}

void DataCoordinator::insert(shared_ptr<Mat> &framePtr, int64_t timestamp,
	int camID, shared_ptr<shared_mutex> &sharedMtx)
{
	std::lock_guard<std::mutex> l(mtx1);
	std::lock_guard<shared_mutex> l2(*sharedMtx);
	if (record)
	{
		// Finds the data needed and creates itrs to use when accessing it
		auto rItr = roombaData.find(timestamp);
		auto lItr = visualData.find(timestamp);
		if (rItr != roombaData.end())
		{
			drawRoombas(framePtr, rItr->second, camID);
		}
		if (lItr != visualData.end())
		{
			drawLines(framePtr, lItr->second, camID);
		}
	}
	write(framePtr,timestamp,camID);
}

void DataCoordinator::drawRoombas(shared_ptr<Mat> &frame, roomba_list_t &src, int camID)
{
	for (int i {0}; i < src.num_roombas; ++i)
	{
		if (src.roombas[i].camera == camID)
		{
			cv::circle(*frame,cv::Point(src.roombas[i].x,
				src.roombas[i].y),1,cv::Scalar(255,0,0),roombaSize);
		}
	}
}

void DataCoordinator::drawLines(shared_ptr<Mat> &frame, visual_data_t &src, int camID)
{
	for (int i {0}; i < src.num_lines; ++i)
	{
		if (src.lines[i].cameraId == camID)
		{
			cv::line(*frame,cv::Point(src.lines[i].
				line_coords[0],src.lines[i].line_coords[1]),cv::
				Point(src.lines[i].line_coords[2],src.lines[i].
				line_coords[3]),cv::Scalar(255,255,0),lineThickness);
		}
	}
	if (src.num_landmarks > 0)
	{
		for (int i {0}; i < src.num_landmarks; ++i)
		{
			if (src.landmarks[i].cameraId == camID)
			{
				cv::circle(*frame,cv::Point(src.landmarks[i].px,
					src.landmarks[i].py),1,cv::Scalar(255,0,255),roombaSize);
			}
		}
	}
}

void DataCoordinator::write(shared_ptr<Mat> &framePtr, int64_t &timestamp, int camID)
{
	if (framePtr->empty())
	{
		return;
	}
	if (trackTime)
	{
		fout[camID] << timestamp << '\n';
	}
	if (type == 0)
	{
		writer[camID].write(*framePtr);
	}
	else
	{
		std::string fileName("DataCoordImages/");
		fileName += std::to_string(camID);
		fileName += "/";
		fileName += std::to_string(++counter[camID]);
		fileName += ".png";
		imwrite(fileName,*framePtr);
	}
}

void DataCoordinator::listenerThreadR(DataCoordinator* dataCoord,
	RoombaHandler* handler, std::atomic<bool> *isRunning, zcm::ZCM *zcmPtr)
{
	zcmPtr->subscribe(maav::VISION_ROOMBAS_CHANNEL, &RoombaHandler::handleMessage, handler);
	while (*isRunning)
	{
		while (*isRunning && !handler->getSize())
		{
			handler->cv.wait(handler->lck);
		}
		if (!(*isRunning))
		{
			return;
		}
		dataCoord->insert(handler->getData());
		handler->deleteData();
	}
}

void DataCoordinator::listenerThreadL(DataCoordinator* dataCoord,
	LineHandler* handler, std::atomic<bool> *isRunning, zcm::ZCM *zcmPtr)
{
	zcmPtr->subscribe(maav::VISION_LINES_CHANNEL,&LineHandler::handleMessage,handler);
	while (*isRunning)
	{
		while (*isRunning && !handler->getSize())
		{
			handler->cv.wait(handler->lck);
		}
		if (!(*isRunning))
		{
			return;
		}
		dataCoord->insert(handler->getData());
		handler->deleteData();
	}
}

void DataCoordinator::startZCM(zcm::ZCM *zcmPtr)
{
	if (record)
	{
		threads.emplace_back(listenerThreadR,this,&roombaHandler,&isRunning,zcmPtr);
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
		threads.emplace_back(listenerThreadL,this,&lineHandler,&isRunning,zcmPtr);
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}
}

void DataCoordinator::insert(roomba_list_t &src)
{
	std::lock_guard<std::mutex> l(mtx1);
	roombaData[src.utime] = src;
}

void DataCoordinator::insert(visual_data_t &src)
{
	std::lock_guard<std::mutex> l(mtx1);
	visualData[src.utime] = src;
}

DataCoordinator::~DataCoordinator()
{
	isRunning = false;
	roombaHandler.cv.notify_one();
	lineHandler.cv.notify_one();
	for (size_t i {0}; i < threads.size(); ++i)
	{
		threads[i].join();
	}
	std::cout << "DataCoordinator closed successfully" << std::endl;
}
