#include <getopt.h>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <ctime>
#include <fstream>
#include <iostream>
#include <mutex>
#include <shared_mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <librealsense/rs.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cstdio>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>

#include "vision/depth-utils/getoptSetup.hpp"

using std::cout;
using std::cerr;
using std::string;
using std::vector;
using cv::Mat;
using std::thread;
using std::atomic;
using cv::imwrite;
using std::mutex;
using std::shared_mutex;
using std::shared_lock;
using std::lock_guard;
using std::unique_lock;

shared_mutex globalMtx;
int camNumGlobal{0};

void toggleOn(atomic<bool>* currentlyOn)
{
	char dump;
	std::cin >> dump;
	*currentlyOn = false;
}

void storeDepth(const uint16_t* depthImage, int counter, int camNum)
{
	shared_lock<shared_mutex> lck(globalMtx);
	Mat depthMat(cv::Size(640, 480), CV_16UC1, (void*)depthImage, Mat::AUTO_STEP);
	// Depth image output
	string imgName = "RGBD_DATA/DepthImage";
	imgName += std::to_string(camNum);
	imgName += "/";
	imgName += std::to_string(counter);
	imgName += ".png";
	imwrite(imgName, depthMat);
}

void storeRGB(const uint8_t* rgbImage, int counter, int camNum)
{
	shared_lock<shared_mutex> lck(globalMtx);
	// RGB image output
	Mat color(cv::Size(640, 480), CV_8UC3, (void*)rgbImage, Mat::AUTO_STEP);
	string imgName = "RGBD_DATA/RGB";
	imgName += std::to_string(camNum);
	imgName += "/";
	imgName += std::to_string(counter);
	imgName += ".png";
	imwrite(imgName, color);
}

void storeCombined(const uint8_t* rgbImage, const uint16_t* depthImage, int counter, int camNum)
{
	shared_lock<shared_mutex> lck(globalMtx);
	// Combine the images and store CV8U_C1
	// with dimensions 640 x 1920, to reverse, just use cv::resize()
	Mat color(cv::Size(640, 480), CV_8UC3, (void*)rgbImage, Mat::AUTO_STEP);
	Mat depthMat(cv::Size(640, 480), CV_16UC1, (void*)depthImage, Mat::AUTO_STEP);
	cv::Mat tempDepthMat;
	depthMat.convertTo(tempDepthMat, CV_8UC1);
	cv::Mat splitMats[4];
	cv::split(color, splitMats);
	cv::split(tempDepthMat, splitMats + 3);
	cv::Mat combinedMat;
	cv::merge(splitMats, 4, combinedMat);
	combinedMat.reshape(1, 1920);
	string imgName = "RGBD_DATA/Combined";
	imgName += std::to_string(camNum);
	imgName += "/";
	imgName += std::to_string(counter);
	imgName += ".png";
	imwrite(imgName, combinedMat);
}

int main(int argc, char** argv)
{
	// Begins logging to the console
	rs::log_to_console(rs::log_severity::warn);
	rs::context ctx;
	printf("There are %d connected realsense devices.\n", ctx.get_device_count());
	if (ctx.get_device_count() < 1)
	{
		throw string("");
	}
	rs::device* devicePtr = ctx.get_device(0);
	devicePtr->enable_stream(rs::stream::depth, 640, 480, rs::format::z16, 60);
	devicePtr->enable_stream(rs::stream::color, 640, 480, rs::format::bgr8, 60);
	devicePtr->start();
	// Stores the data obtained from the cameras
	Mat ColorFrame;
	Mat depthFrame;
	// Point cloud that hold the points from the depthImage
	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::PointCloud<pcl::PointXYZRGBA> cloudXYZRGBA;
	// Reads data from the camera into the cv::Mats
	// And stores the points that are found in the cloud
	// Writes the points to a file and then gets the next frame for 300 frames
	// Testing to see if the first frame is an outlier in terms of how long it takes to get
	// (it very likely is, and was for the previous cameras)
	// Run until a valid frame is acquired
	atomic<bool> currentlyOn = true;
	thread toggleOnThread(toggleOn, &currentlyOn);
	// Get the intrinsic and extrinsic parameters of the cameras and store them
	// Open txt file to dump timestamps into
	// as well as the string stream to expedite it
	std::ofstream fout("RGBD_DATA/timestamps.txt");
	std::stringstream foutHelper;
	// Controls dumping into ofstream from stringstream
	int foutCounter{0};
	// Used for naming files
	int counter{0};
	// Writer that writes PCD files
	pcl::PCDWriter pcdwriter;
	// Used to create files for pcl to write to
	std::ofstream foutPcd;
	// Create config bools and set them using getopt
	bool writeRGB;
	bool writeDepth;
	bool writeCombined;
	bool writeCloud;
	setVariables(writeCombined, writeRGB, writeDepth, writeCloud, argc, argv);
	// Loop until request to end is processed
	while (currentlyOn)
	{
		// Remove points from previous iteration
		cloud.clear();
		cloudXYZRGBA.clear();
		// Wait for frames
		unique_lock<shared_mutex> lck(globalMtx);  // lock shared mutex
		devicePtr->wait_for_frames();
		// Get timestamp of this frame
		int64_t startTime = std::chrono::duration_cast<std::chrono::milliseconds>(
								std::chrono::system_clock::now().time_since_epoch())
								.count();
		// Get frame data
		const uint16_t* depthImage = (const uint16_t*)devicePtr->get_frame_data(rs::stream::depth);
		const uint8_t* colorImage = (const uint8_t*)devicePtr->get_frame_data(rs::stream::color);
		rs::intrinsics depthIntrinsics = devicePtr->get_stream_intrinsics(rs::stream::depth);
		rs::extrinsics depth_to_color =
			devicePtr->get_extrinsics(rs::stream::depth, rs::stream::color);
		rs::intrinsics color_intrin = devicePtr->get_stream_intrinsics(rs::stream::color);
		lck.unlock();  // unlock shared mutex
		shared_lock<shared_mutex> slck(globalMtx);
		// If specified, store rgb image
		std::vector<std::thread> threads;
		if (writeRGB)
		{
			threads.emplace_back(storeRGB, colorImage, counter, camNumGlobal);
		}
		// If specified store depth image
		if (writeDepth)
		{
			threads.emplace_back(storeDepth, depthImage, counter, camNumGlobal);
		}
		// If specified, store combined image
		if (writeCombined)
		{
			threads.emplace_back(storeCombined, colorImage, depthImage, counter, camNumGlobal);
		}
		// If specified, store point cloud
		if (writeCloud)
		{
			float scale = devicePtr->get_depth_scale();
			// Fill the point cloud
			for (int dy{0}; dy < depthIntrinsics.height; ++dy)
			{
				for (int dx{0}; dx < depthIntrinsics.width; ++dx)
				{
					// Retrieve depth value and map it to more "real" coordinates
					uint16_t depthValue = depthImage[dy * depthIntrinsics.width + dx];
					float depthInMeters = depthValue * scale;
					// Skip over values with a depth of zero (not found depth)
					if (depthValue == 0) continue;
					// For mapping color to depth
					// Map from pixel coordinates in the depth image to pixel coordinates in the
					// color image
					rs::float2 depth_pixel = {(float)dx, (float)dy};
					// Projects the depth value into 3-D space
					rs::float3 depthPoint = depthIntrinsics.deproject(depth_pixel, depthInMeters);
					// Creates a corresponding color points in 3-D space (maybe)
					rs::float3 color_point = depth_to_color.transform(depthPoint);
					// Projects the 3-D color point into image space (2-D)
					rs::float2 color_pixel = color_intrin.project(color_point);
					const int cx = (int)std::round(color_pixel.x),
							  cy = (int)std::round(color_pixel.y);
					// int colorIndex = cy * color_intrin.width + cx;
					int colorIndex = (cy * color_intrin.width + cx) * 3;
					// int channelSize = color_intrin.width * color_intrin.height;
					pcl::PointXYZRGBA xyzrgbaPoint;
					if (cy > color_intrin.height || cx > color_intrin.width || cy < 0 || cx < 0)
					{
						xyzrgbaPoint.r = 255;
						xyzrgbaPoint.g = 255;
						xyzrgbaPoint.b = 255;
						xyzrgbaPoint.a = 255;
					}
					else
					{
						xyzrgbaPoint.r = colorImage[colorIndex];
						xyzrgbaPoint.g = colorImage[colorIndex + 1];
						xyzrgbaPoint.b = colorImage[colorIndex + 2];
						xyzrgbaPoint.a = 255;
					}
					xyzrgbaPoint.x = depthPoint.x;
					xyzrgbaPoint.y = depthPoint.y;
					xyzrgbaPoint.z = depthPoint.z;
					cloudXYZRGBA.push_back(xyzrgbaPoint);
					// Push back new point into cloud
					cloud.push_back(pcl::PointXYZ(depthPoint.x, depthPoint.y, depthPoint.z));
				}
			}
			// Store point cloud
			string imgName = "RGBD_DATA/Cloud";
			imgName += std::to_string(camNumGlobal);
			imgName += "/";
			imgName += std::to_string(counter);
			imgName += ".pcd";
			foutPcd.open(imgName.c_str(), std::ios::out);
			foutPcd.close();
			pcdwriter.writeASCII(imgName, cloud);
			pcl::io::savePCDFileASCII(imgName, cloud);
			// Store other point cloud
			imgName = "RGBD_DATA/RCloud";
			imgName += std::to_string(camNumGlobal);
			imgName += "/";
			imgName += std::to_string(counter);
			imgName += ".pcd";
			foutPcd.open(imgName.c_str(), std::ios::out);
			foutPcd.close();
			pcdwriter.writeASCII(imgName, cloudXYZRGBA);
			pcl::io::savePCDFileASCII(imgName, cloudXYZRGBA);
		}
		// Join all created threads back
		while (!threads.empty())
		{
			threads.back().join();
			threads.pop_back();
		}
		// Timestamp output
		foutHelper << startTime << '\n';
		if (++foutCounter == 30)
		{
			foutCounter = 0;
			fout << foutHelper.str();
		}
		++counter;
	}
	fout << foutHelper.str() << '\n';
	toggleOnThread.join();
}
