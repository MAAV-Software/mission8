#include <sys/stat.h>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <librealsense/rs.hpp>
#include <librealsense2/rs.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "vision/depth-utils/CameraInput.hpp"
#include "vision/depth-utils/CameraInputBase.hpp"
#include "vision/depth-utils/NewCameraInput.hpp"

using std::vector;
using std::mutex;
using std::condition_variable;
using std::thread;
using std::atomic;

atomic<bool> terminate = false;
std::mutex mtx;
std::condition_variable condVar;
std::string directory;

void sigHandler(int);
void camera_thread(CameraInputBase *cam);
void createDirectories();
void createDirectories(std::string);

int main(int argc, char **argv)
{
	std::cout << "Usage: data-log <# rs1 cams> <# rs2 cams> <directory (optional)>\n";
	if (argc == 4)
	{
		directory = argv[3];
		if (directory.back() == '/') directory.pop_back();
		createDirectories(directory);
	}
	else
	{
		createDirectories();
		directory = "ImageData";
	}

	int oldCams = std::stoi(argv[1]);
	int newCams = std::stoi(argv[2]);

	// Initialize cameras
	vector<CameraInputBase *> cameras;
	cameras.reserve(oldCams + newCams);

	if (oldCams > 0)
	{
		CameraInput *cam1 = new CameraInput(0);
		cameras.push_back(cam1);
		std::cout << "First rs1 camera made" << std::endl;
		// Retrieve rs::context
		rs::context *ctx = cam1->getContext();
		// Continue creating cameras
		for (unsigned i = 1; i < oldCams; ++i)
		{
			CameraInput *nextCam = new CameraInput(i, ctx);
			cameras.push_back(nextCam);
		}
		std::cout << "rs1 Cameras created" << std::endl;
	}

	if (newCams > 0)
	{
		rs2::context ctx;
		auto list = ctx.query_devices();
		std::cout << list.size() << " rs2 devices connected.\n";
		for (int i = 0; i < newCams; ++i)
		{
			NewCameraInput *nextCam =
				new NewCameraInput(list[i].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
			cameras.push_back(nextCam);
		}
		std::cout << "rs2 Cameras created" << std::endl;
	}

	// drop first 10 frames
	for (int i = 0; i < 10; ++i)
		for (auto cam : cameras) cam->loadNext();

	// cameras now contains CameraInput instances for each of the connected cameras
	// Initialize variables used for thread control
	vector<thread> threads;
	// Set up sighandlers
	signal(SIGINT, sigHandler);
	signal(SIGSEGV, sigHandler);
	signal(SIGABRT, sigHandler);
	std::cout << "Got to thread creation" << std::endl;
	// Create threads
	for (unsigned i = 0; i < oldCams + newCams; ++i)
	{
		threads.emplace_back(camera_thread, cameras[i]);
	}
	// Wait until awoken by sig handler
	std::unique_lock<std::mutex> lck(mtx);
	while (terminate == false)
	{
		condVar.wait(lck);
	}
	// Join the threads back to main
	for (unsigned i = 0; i < oldCams + newCams; ++i)
	{
		threads[i].join();
	}

	for (CameraInputBase *cam : cameras) delete cam;
	return 0;
}

void sigHandler(int)
{
	// Set terminate to true
	terminate = true;
	condVar.notify_one();
	condVar.notify_one();
}

void camera_thread(CameraInputBase *cam)
{
	cv::Mat rgb;
	cv::Mat depth;
	std::string fname = directory;
	fname += "/Timestamps/time";
	fname += std::to_string(cam->getTag());
	fname += ".txt";
	std::ofstream timestamps(fname);
	for (int frame = 0; !(terminate); ++frame)
	{
		int64_t startTime = std::chrono::duration_cast<std::chrono::milliseconds>(
								std::chrono::system_clock::now().time_since_epoch())
								.count();

		cam->getRGB(rgb);
		cam->getDepth(depth);

		std::string fname = directory;
		fname += "/RGB";
		fname += std::to_string(cam->getTag());
		fname += "/";
		fname += std::to_string(frame);
		fname += ".bin";
		std::ofstream outfileRGB(fname, std::ios::out | std::ios::binary);
		std::cout << rgb.elemSize() << " " << rgb.total() << "\n";
		outfileRGB.write((char *)rgb.data, rgb.elemSize() * rgb.total());
		outfileRGB.close();

		fname = directory;
		fname += "/Depth";
		fname += std::to_string(cam->getTag());
		fname += "/";
		fname += std::to_string(frame);
		fname += ".bin";
		std::ofstream outfileDepth(fname, std::ios::out | std::ios::binary);
		std::cout << depth.elemSize() << " " << depth.total() << "\n";
		outfileDepth.write((char *)depth.data, depth.elemSize() * depth.total());
		outfileDepth.close();

		timestamps << startTime << '\n';

		cam->loadNext();
	}
}

void createDirectories(std::string dirName)
{
	std::string command = "rm -rf ";
	command += dirName;
	system(command.c_str());
	command = "mkdir ";
	command += dirName;
	system(command.c_str());
	command += "/Timestamps";
	system(command.c_str());

	for (int i = 0; i < 5; ++i)
	{
		std::string command = "mkdir ";
		command += dirName;
		command += "/RGB";
		command += std::to_string(i);
		system(command.c_str());
		command = "mkdir ";
		command += dirName;
		command += "/Depth";
		command += std::to_string(i);
		system(command.c_str());
	}
}

void createDirectories()
{
	struct stat statStruct;
	stat("ImageData", &statStruct);

	// if the ImageData directory exists, tar the existing data
	if (S_ISDIR(statStruct.st_mode))
	{
		int i = 0;
		std::string filename = "ImageData0.tar.gz";
		std::ifstream file(filename);
		while (file)
		{
			++i;
			filename = "ImageData";
			filename += std::to_string(i);
			filename += ".tar.gz";
			file = std::ifstream(filename);
		}
		std::string command = "tar -czf ";
		command += filename;
		command += " ImageData/*";
		std::cout << command << std::endl;
		system(command.c_str());
		system("rm -rf ImageData");
	}

	// create the new directories for storage
	system("mkdir ImageData");
	system("mkdir ImageData/Timestamps");

	for (int i = 0; i < 5; ++i)
	{
		std::string command = "mkdir ImageData/RGB";
		command += std::to_string(i);
		system(command.c_str());
		command = "mkdir ImageData/Depth";
		command += std::to_string(i);
		system(command.c_str());
	}
}
