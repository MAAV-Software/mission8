// Use -c to set config file path, if not set
// will use config/vision-config.yaml

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <ctime>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <thread>
#include <vector>

#ifdef BENCHMARKING
#include <fstream>
#endif

#include "../line-detector/LineDetector.hpp"
#include "../roomba-detector/RoombaDetector.hpp"

#include "vision/driver/CameraIdentifier.hpp"
#include "vision/driver/DataCoordinator.hpp"
#include "vision/driver/ImageFeed.hpp"
#include "vision/driver/TimeInterface.hpp"
#include "vision/driver/VehicleStateRepo.hpp"
#include "vision/driver/VideoInterface.hpp"
#include "vision/driver/ZCMAccumulator.hpp"
#include "vision/driver/gcsCommunication.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <eigen3/Eigen/Dense>
#include <librealsense/rs.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "yaml-cpp/yaml.h"

#include <zcm/zcm-cpp.hpp>
#include "common/messages/MsgChannels.hpp"
#include "common/messages/nav_runstate_t.hpp"
#include "common/messages/roomba_list_t.hpp"
#include "common/messages/roomba_t.hpp"
#include "common/messages/visual_data_t.hpp"
#include "common/utils/GetOpt.hpp"

using namespace std;
using namespace cv;

// config that will be obtained from config.yaml in main()
YAML::Node config;
// Boolean used for deactivating threads
std::atomic<bool> run(true);
// Dynamic class that controls delays in reader thread
// Either normal sleeps and times, or sim_time sleeps and times
TimeInterface* timer;
// Holds whether KILL is true
sig_atomic_t KILL = 0;
// Mutex controlling termination of program
mutex mainMtx;
// Condition Variable controlling termination of program
condition_variable mainCv;

void sigHandler(int)
{
	KILL = 1;
	mainCv.notify_one();
}

// subscribe outside, handles inside
void Listener(zcm::ZCM* zcm)
{
	std::condition_variable cv;
	runStateHandler tempListener(&cv);
	zcm->subscribe("END_VISION", &runStateHandler::handleMessage, &tempListener);
	while (!KILL)
	{
		zcm->handle();
	}
	std::cerr << "Listener thread finished.\n";
}

void testVersionTerminator()
{
	char dump;
	cin >> dump;
	KILL = 1;
	mainCv.notify_one();
}

void statusSender(zcm::ZCM* zcmPtr)
{
	nav_runstate_t state;
	zcm::ZCM& zcm = *zcmPtr;
	state.running_mission = true;
	while (!KILL && zcm.good())
	{
		zcm.publish(maav::VISION_STAT, &state);
		this_thread::sleep_for(chrono::milliseconds(250));
	}
}

// Pass one zcm_accumulator
// Reworked to new design: COMPLETE
void zcm_thread(ZCMAccumulator* zcm_acc, zcm::ZCM* zcmPtr)
{
	cout << "Sender Thread Start" << endl;
	// initialize message variables and zcm
	roomba_list_t roombas;
	visual_data_t visualData;
	// Creates alias for zcm object
	zcm::ZCM& sender = *zcmPtr;
	// Initialize condition variable stuff
	std::unique_lock<std::mutex> lck(zcm_acc->cvMtx);
	while (run)
	{
		while (!zcm_acc->ready())
		{
			zcm_acc->cv.wait(lck);
		}
		if (zcm_acc->extract(roombas) && sender.good())
		{
			sender.publish(maav::VISION_ROOMBAS_CHANNEL, &roombas);
			std::this_thread::sleep_for(std::chrono::milliseconds(2));
		}
		if (zcm_acc->extract(visualData) && sender.good())
		{
			sender.publish(maav::VISION_LINES_CHANNEL, &visualData);
			std::this_thread::sleep_for(std::chrono::milliseconds(2));
		}
	}
	std::cerr << "Exitting zcmThread\n";
}

// Uses round robin scheme with gap in config file  between grabs
// Pass in array of 5 image feeds and one ZCMAccumulator shared between
// all relevant threads
// Reworked to new design: COMPLETE
void reader_thread(vector<ImageFeed*>* img_feed, ZCMAccumulator* zcm_acc, zcm::ZCM* zcmPtr,
				   condition_variable* imgFeedWaitCvPtr, atomic<bool>* imgFeedReadyPtr)
{
	std::cout << "reader thread start" << endl;
	// Create zcm object alias
	zcm::ZCM& zcm = *zcmPtr;
	// Init variables
	std::vector<VideoInterface*> cap(5);
	std::shared_ptr<Mat> frame;
	// Used with ignoring VideoInterfaces when less than
	// 5 cameras are in use
	std::vector<bool> activeCams(5);
	for (int i{0}; i < config["vision_driver"]["activeCams"].as<int>(); ++i)
	{
		activeCams[i] = true;
	}
	for (int i{4}; i > config["vision_driver"]["activeCams"].as<int>() - 1; --i)
	{
		activeCams[i] = false;
	}
	// Holds the data coordinator object pointer
	// Used for communicating with gcs
	std::condition_variable cv;
	runStateHandler runHandler(&cv);
	camIdentifyHandler identifyHandler;
	// Controls the pulling of the cameras
	unsigned int idx{1};
	int bidx{3};
	// Controls the delay between each camera pull
	int64_t this_time;
	int64_t end_time;
	int timeToSleep{0};
	bool bottom_controller{false};
	// controls whether camera identification or pause before start
	// relying upon messages from GCS will occur
	bool useGCS = config["vision_driver"]["useGCS"].as<bool>();
	// Controls whether camera identification is performed
	bool identifyCams = config["vision_driver"]["identifyCams"].as<bool>();
	// How many times cameras fire for the bottom camera to fire
	int bidx_control = config["vision_driver"]["bidx_control"].as<int>();
	// The delay between each image pull of the reader thread
	int reader_delay = config["vision_driver"]["reader_delay"].as<int>();
	if (config["vision_driver"]["type"].as<int>() == 2)
	{
		VideoInterface::spawnT2Caps(cap);
		TimeInterface::createType2(timer);
	}
	else
	{
		timer = new TimeInterface(0);
	}
	// initialize video captures
	// First finds how many cameras are intended to be used
	int numAvailableCams{0};
	for (int i{0}; i < 5; ++i)
	{
		if (activeCams[i]) ++numAvailableCams;
	}
	rs::context* context;
	for (int i{0}; i < numAvailableCams; ++i)
	{
		if (config["vision_driver"]["type"].as<int>() != 2)
		{
			cap[i] = new VideoInterface(config["vision_driver"]["type"].as<int>());
		}
		if (config["vision_driver"]["type"].as<int>() == 4)
		{
			if (i == 0)
			{
				cap[i]->open(i);
				context = cap[i]->getContext();
			}
			else
			{
				std::cout << context << "\n";
				cap[i]->setContext(context);
				cap[i]->open(i);
			}
		}
		else if (config["vision_driver"]["type"].as<int>() == 1)
		{
			cap[i]->open(config["vision_driver"]["filenames"][i].as<string>());
		}
		else
		{
			cap[i]->open(i);
		}
		cap[i]->set(cv::CAP_PROP_FRAME_WIDTH, 640);
		cap[i]->set(cv::CAP_PROP_FRAME_HEIGHT, 480);
		// Sleeps to ensure propper camera initialization
		std::this_thread::sleep_for(std::chrono::milliseconds(3));
		// Checks to ensure that cameras have opened properly
		try
		{
			if (!cap[i]->isOpened()) throw i;
		}
		catch (int cam)
		{
			cout << "Camera " << cam << " failed to open... exit.";
			cout << endl;
			throw "camera error";
		}
		// Notifies of camera opening success
		cout << "Camera " << i << " good." << endl;
	}
	if (useGCS)
	{
		zcm.subscribe(maav::CAMERA_DISC_CMD, &camIdentifyHandler::handleMessage, &identifyHandler);
		zcm.subscribe(maav::NAV_RUNSTATE_CMD, &runStateHandler::handleMessage, &runHandler);
		// Prime zcm channels for gcs communication
		camera_disc_t cameraDiscMsg;
		cameraDiscMsg.numCameras = 5;
		if (zcm.good()) zcm.publish(maav::CAMERA_DISC_STAT, &cameraDiscMsg);
		// Done priming zcm channels that NEED to be primed
	}
	// Identify cameras
	for (bool ready{false}; !ready && useGCS && identifyCams;)
	{
		CameraIdentifier camIdentifier(config["vision_driver"]["cameraIdThresh"].as<int>());
		camera_disc_t msg;
		cout << "Wating for ready message from GCS\n";
		while (identifyHandler.numFound == -1)
		{
			this_thread::sleep_for(chrono::milliseconds(1));
		}
		// If gcs sends the number 15 then will skip identification
		if (identifyHandler.numFound == 15)
		{
			ready = true;
			continue;
		}
		msg.numCameras =
			camIdentifier.identify(cap, activeCams, *img_feed, useGCS, zcmPtr, &identifyHandler);
		if (msg.numCameras == 5)
		{
			ready = true;
		}
		else
		{
			identifyHandler.numFound = -1;
		}
		if (zcm.good()) zcm.publish(maav::CAMERA_DISC_STAT, &msg);
	}
	if (identifyCams && !useGCS)
	{
		cout << "Identifying cameras\n";
		CameraIdentifier camIdentifier(config["vision_driver"]["cameraIdThresh"].as<int>());
		cout << camIdentifier.identify(cap, activeCams, *img_feed, useGCS, zcmPtr, &identifyHandler)
			 << " cameras found\n";
	}
	for (int i{0}; i < 5; ++i)
	{
		img_feed[0][i]->setID(i);
	}
	shared_ptr<runStateHandler> tempRunHandler;
	*imgFeedReadyPtr = true;
	imgFeedWaitCvPtr->notify_one();
	if (useGCS)
	{
		tempRunHandler = make_shared<runStateHandler>(&cv);
		zcm.subscribe(maav::START_VISION, &runStateHandler::handleMessage, tempRunHandler.get());
		cout << "Waiting for start mission message from GCS\n";
		while (!runHandler.active && !tempRunHandler->active)
		{
			std::unique_lock<std::mutex> lck(runHandler.mtx);
			runHandler.cv->wait(lck);
		}
	}
// Initialize video capture controller loop
#ifdef BENCHMARKING
	ofstream fout;
	fout.open("Outreader.txt");
	int64_t benchTime{0};
	int64_t benchLast{0};
#endif
	int64_t lastZcmTime{0};
	while (run)
	{
		// Gets starting time to control delay
		timer->getTime(this_time);
		// Gets frame of idx
		bottom_controller = (++bidx == bidx_control && activeCams[0] && cap[0]->grab());
		if (activeCams[idx] && cap[idx]->grab())
		{
			timer->getTime(end_time);
			frame = make_shared<Mat>();
			cap[idx]->retrieve(*frame);
			// alerts worker threads and stores image
			(*img_feed)[idx]->storeFrame(frame, end_time);
// Increases bin size if is front cam
#ifdef BENCHMARKING
			benchLast = benchTime;
			timer->getTime(benchTime);
			fout << benchTime << '\n';
			fout << "diff: " << (benchTime - benchLast) / 1000 << '\n';
#endif
		}
		// Gets frame of bottom camera on bidx 3
		if (bottom_controller)
		{
			bidx = 0;
			frame = make_shared<Mat>();
			cap[0]->retrieve(*frame);
			// alerts worker threads and stores image
			(*img_feed)[0]->storeFrame(frame, end_time);
			bottom_controller = false;
			zcm_acc->clearTime(lastZcmTime);
			lastZcmTime = end_time;
#ifdef BENCHMARKING
			benchLast = benchTime;
			timer->getTime(benchTime);
			fout << benchTime << "bottom\n";
			fout << "diff: " << (benchTime - benchLast) / 1000 << '\n';
#endif
		}
		// Either sets idx to 1 or increments it
		if (++idx == 5) idx = 1;
		if (bidx == bidx_control) bidx_control = 0;
		// Gets end_time of frame capture
		timer->getTime(end_time);
		timeToSleep = reader_delay - ((end_time - this_time) / 1000);
		// Delays until correct offset is obtained
		if (timeToSleep > 0) timer->sleep(timeToSleep);
	}
	std::cerr << "Deleting VideoInterfaces \n";
	for (int i{0}; i < 5; ++i)
	{
		if (activeCams[i]) delete cap[i];
	}
	std::cerr << "Reader Thread finished \n";
}

// Function that runs in the Roomba detection threads
// Pass in the appropriate image feed, boolean indicating if this is the bottom
// camera, the cameraID, and the ZCMAccumulator shared across the entire driver
// Reworked to new design: COMPLETE
void RoombaDetection(ImageFeed* img_feed, bool is_bottom, int camera, ZCMAccumulator* zcm_acc,
					 zcm::ZCM* zcmPtr, VehicleStateRepo* stateHandlerPtr)
{
	// Notify that thread has begun working
	cout << "Roomba Detector " << camera << " start" << endl;
// If in benchmarking mode will record time it takes
// to run detection to roombaTimes<camera>.txt
#ifdef BENCHMARKING
	ofstream fout;
	string fileName = "OutroombaTimes";
	fileName += to_string(camera);
	fileName += ".txt";
	fout.open(fileName.c_str());
	int64_t startTime;
	int64_t endTime;
	TimeInterface timer(1);
#endif

	YAML::Node roombaConfig = config["RoombaDetection"];
	// Initialize Roomba Detector
	RoombaDetector detectObject(roombaConfig, camera);
	// Initialized zcm message stuff
	roomba_list_t roombas;
	// Initialize data storage
	shared_ptr<Mat> framePtr;
	shared_ptr<shared_mutex> sharedMtxPtr;
	int64_t timestamp{0};
	// Initialize condition variable stuff
	std::unique_lock<std::mutex> lck(img_feed->cvMtxR);
	while (run)
	{
		// Wait until a frame is ready to be retrieved
		while (!img_feed->ready(timestamp)) img_feed->cv.wait(lck);
		// Get frame and info
		img_feed->getFrame(framePtr, timestamp, sharedMtxPtr);
		// Add to size of zcm_acc bin if either front or bottom cam
		if (!camera || camera == 1)
		{
			zcm_acc->insertr(timestamp);
		}
		// Run roomba_t detection, if an exception occurs, will
		// notify zcm_acc allowing it to continue proper function
		try
		{
#ifdef BENCHMARKING
			timer.getTime(startTime);
#endif
			{
				pcl::PointCloud<pcl::PointXYZ> cloud;
				Eigen::MatrixXf fakeGroundPlane;
				shared_lock<shared_mutex> lck(*sharedMtxPtr);
				detectObject.combineDetect(*framePtr, cloud, fakeGroundPlane, roombas, camera,
										   timestamp);
			}
#ifdef BENCHMARKING
			timer.getTime(endTime);
			fout << (endTime - startTime) / 1000 << '\n';
#endif
			// Check if any roombas found, if not, throw exception
			if (!detectObject.locations.size()) throw camera;
			// add message to zcm accumulator
			if (camera == 1 || camera == 0)
				zcm_acc->insert(roombas);
			else if (zcmPtr->good())
				zcmPtr->publish(maav::VISION_ROOMBAS_CHANNEL, &roombas);
		}
		// notify zcm_acc of empty message or exception
		catch (...)
		{
			if (camera == 1 || camera == 0) zcm_acc->notifyR(timestamp);
		}
	}
	std::cerr << "Roomba Thread " << camera << " done.\n";
}

// Function runs in the line detection threads
// Pass in the appropriate ImageFeed, id used for the image feed, and
// ZCMAccumulator shared throughout the entire vision_driver
// Reworked to new design: COMPLETE
void LineDetection(ImageFeed* img_feed, int camera, ZCMAccumulator* zcm_acc, zcm::ZCM* zcmPtr,
				   VehicleStateRepo* stateHandlerPtr)
{
	cout << "Line Detector " << camera << " start" << endl;
// Initialize line detector
// If in BENCHMARKING mode will write time it takes to
// detect lines in ms in lineTimes<camera>.txt
#ifdef BENCHMARKING
	ofstream fout;
	string fileName = "OutlineTimes";
	fileName += to_string(camera);
	fileName += ".txt";
	fout.open(fileName.c_str());
	int64_t startTime;
	int64_t endTime;
	TimeInterface timer(1);
#endif
	YAML::Node lineConfig = config["LineDetection"];
	LineDetector linedetector(lineConfig);
	// Initialize zcm stuff
	visual_data_t visualData;
	// Initialize data storage
	shared_ptr<Mat> framePtr;
	shared_ptr<shared_mutex> sharedMtxPtr;
	int64_t timestamp{0};
	// Initialize condition variable stuff
	std::unique_lock<std::mutex> lck(img_feed->cvMtxL);
	while (run)
	{
		// Wait until a frame is ready
		while (!img_feed->ready(timestamp))
		{
			img_feed->cv.wait(lck);
		}
		// Get the frame and associated data
		img_feed->getFrame(framePtr, timestamp, sharedMtxPtr);
		visualData.utime = timestamp;
		// Adds to size of zcm_acc bins
		if (!camera || camera == 1)
		{
			zcm_acc->insertl(timestamp);
		}
		// Run Line detection, if an exception occurs, will
		// notify zcm_acc allowing it to continue proper function
		try
		{
// Run detect lines
#ifdef BENCHMARKING
			timer.getTime(startTime);
#endif
			{
				shared_lock<shared_mutex> lck(*sharedMtxPtr);
				// Check if any lines detected, if not, throw exception
				if (!linedetector.detect_lines(*framePtr, visualData, camera, stateHandlerPtr))
				{
					throw camera;
				}
			}
#ifdef BENCHMARKING
			timer.getTime(endTime);
			fout << (endTime - startTime) / 1000 << '\n';
#endif
			// Add message to ZCMAccumulator if needed
			if (camera == 1 || camera == 0)
			{
				zcm_acc->insert(visualData);
			}
			else if (zcmPtr->good())
			{
				zcmPtr->publish(maav::VISION_LINES_CHANNEL, &visualData);
			}
		}
		// notify zcm_acc of empty messages or an exception
		catch (...)
		{
			if (camera == 1 || camera == 0) zcm_acc->notifyL(timestamp);
		}
	}
	std::cerr << "Line Thread " << camera << " finished.\n";
}

int main(int argc, char** argv)
{
	// Constructs the zcm object and subscribes to state channel
	// as well as constructs the object that stores state history
	zcm::ZCM zcm{"ipc"};
	VehicleStateRepo stateHandler;
	zcm.subscribe(maav::TRACKED_STATE, &VehicleStateRepo::handleMessage, &stateHandler);
// Sets up the signal handling
#ifndef NOVISIONSIGHANDLERS
	signal(SIGINT, sigHandler);
	signal(SIGSEGV, sigHandler);
	signal(SIGABRT, sigHandler);
#endif
	GetOpt getOptObject;
	getOptObject.addString('c', "config_path", "config/vision-config.yaml", "path to config");
	getOptObject.parse(argc, argv, true);
	// Creates a thread to handle updating the status of
	// roomba (whether it is running or not)
	thread statusHandler(statusSender, &zcm);
	// Parse config file into the config node
	config = YAML::LoadFile(getOptObject.getString("config_path"));
	// Initialize image feeds and zcm accumulator
	vector<ImageFeed*> img_feeds;
	shared_ptr<DataCoordinator> dataCoord;
	// If config specified it, creates a DataCoordinator on the heap
	if (config["vision_driver"]["is_test"].as<bool>())
	{
		dataCoord =
			std::make_shared<DataCoordinator>(config["vision_driver"]["reader_delay"].as<int>(),
											  config["vision_driver"]["bidx_control"].as<int>(),
											  config["vision_driver"]["draw_results"].as<bool>(),
											  config["vision_driver"]["line_thickness"].as<int>(),
											  config["vision_driver"]["roomba_size"].as<int>(),
											  config["vision_driver"]["track_time"].as<bool>(),
											  config["vision_driver"]["DataCoordType"].as<int>());
		dataCoord->startZCM(&zcm);
		for (int i{0}; i < 5; ++i)
		{
			img_feeds.push_back(new ImageFeed(config["vision_driver"]["writeSize"].as<int>(),
											  config["vision_driver"]["deleteSize"].as<int>(),
											  dataCoord, i));
		}
	}
	else
	{
		for (int i{0}; i < 5; ++i)
		{
			img_feeds.push_back(new ImageFeed(config["vision_driver"]["writeSize"].as<int>(),
											  config["vision_driver"]["deleteSize"].as<int>(), i));
		}
	}
	ZCMAccumulator zcm_acc;
	// Initialize thread storage
	vector<std::thread> roomba_threads;
	vector<std::thread> line_detection_threads;
	// Create thread that listens for messages
	thread listenerThread(Listener, &zcm);
	// Ensure that listener has listened for awhile before others begin
	std::this_thread::sleep_for(std::chrono::milliseconds(50));
	// Create Reader Thread and controls for continuing main()
	std::mutex imgFeedWaitMtx;
	std::unique_lock<std::mutex> imgFeedWaitLck(imgFeedWaitMtx);
	std::condition_variable imgFeedWaitCv;
	std::atomic<bool> imgFeedReady{false};

	std::thread Reader_Thread(reader_thread, &img_feeds, &zcm_acc, &zcm, &imgFeedWaitCv,
							  &imgFeedReady);
	// Waits until camera identification has been called and finished
	// before notifying the cv and allowing the other threads to be created
	while (!imgFeedReady)
	{
		imgFeedWaitCv.wait(imgFeedWaitLck);
	}
	// Create Roomba Threads
	if (config["vision_driver"]["run_roombas"].as<bool>())
	{
		for (int i{0}; i < 5; ++i)
		{
			roomba_threads.emplace_back(std::thread(RoombaDetection, img_feeds[i], !(bool)i, i,
													&zcm_acc, &zcm, &stateHandler));
			std::this_thread::sleep_for(std::chrono::milliseconds(3));
		}
	}
	// Create Line Threads
	if (config["vision_driver"]["run_lines"].as<bool>())
	{
		for (int i{0}; i < 5; ++i)
		{
			line_detection_threads.emplace_back(
				std::thread(LineDetection, img_feeds[i], i, &zcm_acc, &zcm, &stateHandler));
			std::this_thread::sleep_for(std::chrono::milliseconds(3));
		}
	}
	// Create zcm sender thread
	std::thread sender_thread(zcm_thread, &zcm_acc, &zcm);
	// If test version, will spawn a thread that stops program if a
	// character is entered
	if (config["vision_driver"]["is_test"].as<bool>()) std::thread(testVersionTerminator).detach();
	// This Thread Might as well just sleep now until termination
	std::unique_lock<std::mutex> lck(mainMtx);
	while (!KILL) mainCv.wait(lck);
	run = false;
	// Ensures that zcmThread will finish
	zcm_acc.forceReady();
	std::cerr << "Shutting down threads, please wait... \n";
	std::cerr << "Shutting down sender \n";
	sender_thread.join();
	// Shuts down the listener thread before others
	std::cerr << "Ending Listener Thread \n";
	nav_runstate_t tempMsg;
	tempMsg.running_mission = true;
	zcm.publish("END_VISION", &tempMsg);
	this_thread::sleep_for(chrono::milliseconds(200));
	zcm.publish("END_VISION", &tempMsg);
	this_thread::sleep_for(chrono::milliseconds(200));
	zcm.publish("END_VISION", &tempMsg);
	listenerThread.join();
	for (int i{0}; i < 5; ++i)
	{
		img_feeds[i]->forceReady();
	}
	if (config["vision_driver"]["run_lines"].as<bool>())
	{
		std::cerr << "Shutting down line detection \n";
		for (size_t i{0}; i < line_detection_threads.size(); ++i)
		{
			line_detection_threads[i].join();
		}
	}
	if (config["vision_driver"]["run_roombas"].as<bool>())
	{
		std::cerr << "Shutting down roomba detection \n";
		for (size_t i{0}; i < roomba_threads.size(); ++i)
		{
			roomba_threads[i].join();
		}
	}
	std::cerr << "Shutting down Reader Thread \n";
	Reader_Thread.join();
	std::cerr << "Deleting timer\n";
	delete timer;
	std::cerr << "Ending statusHandler\n";
	statusHandler.join();
	std::cerr << "Destructing Image Feeds and possible recording to file.\n";
	for (int i{0}; i < 5; ++i)
	{
		std::cerr << "Destructing Image Feed " << i << ".\n";
		delete img_feeds[i];
	}
	std::cerr << "exitting... \n";
}
