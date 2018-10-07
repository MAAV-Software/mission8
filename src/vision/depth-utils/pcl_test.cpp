#include <chrono>
#include <cstdint>
#include <ctime>
#include <iostream>
#include <string>
#include <vector>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cstdio>
#include <librealsense/rs.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

using std::cout;
using std::cerr;
using std::string;
using std::vector;
using cv::Mat;

int main()
{
	rs::log_to_console(rs::log_severity::warn);

	rs::context ctx;
	printf("There are %d connected realsense devices.\n", ctx.get_device_count());
	if (ctx.get_device_count() < 1)
	{
		throw string("");
	}
	rs::device *devicePtr = ctx.get_device(0);
	devicePtr->enable_stream(rs::stream::depth, rs::preset::best_quality);
	devicePtr->enable_stream(rs::stream::color, rs::preset::best_quality);
	devicePtr->start();
	// Stores the data obtained from the cameras
	Mat ColorFrame;
	Mat depthFrame;
	// Vector of Point clouds that hold the points from the depthImage
	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;
	clouds.emplace_back(new pcl::PointCloud<pcl::PointXYZ>);
	clouds.emplace_back(new pcl::PointCloud<pcl::PointXYZ>);
	// cloud->width = 640;
	// cloud->height = 480;
	// Pointers to the memory where the frame data is stored
	const uint16_t *depthImage;
	// const uint8_t *colorImage;
	// Reads data from the camera into the cv::Mats
	// And stores the points that are found in the cloud
	// Writes the points to a file and then gets the next frame for 300 frames
	int numIterations{30};
	std::vector<int64_t> times;
	times.reserve(numIterations * 4);
	int64_t endTime;
	// Set up planar segmentation
	std::vector<int64_t> times2;
	times2.reserve(numIterations * 4);
	// Create vectors of inliers and coefficients
	vector<pcl::ModelCoefficients::Ptr> coefficients;
	coefficients.emplace_back(new pcl::ModelCoefficients);
	vector<pcl::PointIndices::Ptr> inliers;
	inliers.emplace_back(new pcl::PointIndices);
	// Create the segmentation object and store it in the vector
	std::vector<pcl::SACSegmentation<pcl::PointXYZ>> segs(1);
	// Optional
	segs[0].setOptimizeCoefficients(true);
	// Mandatory
	segs[0].setModelType(pcl::SACMODEL_PLANE);
	segs[0].setMethodType(pcl::SAC_RANSAC);
	segs[0].setDistanceThreshold(0.3);

	segs[0].setInputCloud(clouds[0]);
	// Testing to see if the first frame is an outlier in terms of how long it takes to get
	// (it very likely is, and was for the previous cameras)
	// Run until a valid frame is acquired
	for (int i{0}; (i < numIterations && clouds[0]->size() == 0) || i < 20; ++i)
	{
		for (size_t i{0}; i < clouds.size(); ++i)
		{
			clouds[i]->clear();
		}
		int64_t startTime = std::chrono::duration_cast<std::chrono::milliseconds>(
								std::chrono::system_clock::now().time_since_epoch())
								.count();
		// Wait for frames
		devicePtr->wait_for_frames();
		// Get frame data
		depthImage = (const uint16_t *)devicePtr->get_frame_data(rs::stream::depth);
		// colorImage = (const uint8_t*)devicePtr->get_frame_data(rs::stream::color);
		// Get the intrinsic and extrinsic parameters of the cameras and store them
		rs::intrinsics depthIntrinsics = devicePtr->get_stream_intrinsics(rs::stream::depth);
		// rs::extrinsics depthToColor = devicePtr->get_extrinsics(rs::stream::depth,
		// rs::stream::color);
		// rs::intrinsics colorIntrinsics = devicePtr->get_stream_intrinsics(rs::stream::color);
		float scale = devicePtr->get_depth_scale();
		// Create point cloud from depth image
		std::cout << "Filling point cloud\n";
		for (int dy{0}; dy < depthIntrinsics.height; ++dy)
		{
			for (int dx{0}; dx < depthIntrinsics.width; ++dx)
			{
				// Retrieve depth value and map it to more "real" coordinates
				uint16_t depthValue = depthImage[dy * depthIntrinsics.width + dx];
				float depthInMeters = depthValue * scale;

				// Skip over values with a depth of zero (not found depth)
				if (depthValue == 0) continue;

				rs::float2 depthPixel = {(float)dx, (float)dy};
				rs::float3 depthPoint = depthIntrinsics.deproject(depthPixel, depthInMeters);

				clouds[0]->push_back(pcl::PointXYZ(depthPoint.x, depthPoint.y, depthPoint.z));
			}
		}
		endTime = std::chrono::duration_cast<std::chrono::milliseconds>(
					  std::chrono::system_clock::now().time_since_epoch())
					  .count();
		times.push_back(endTime - startTime);
		// Loops until there are no more planes to be found
		for (size_t i{0}, boolVar{1}; boolVar == 1; ++i)
		{
			if (segs.size() < (i + 1))
			{
				clouds.emplace_back(new pcl::PointCloud<pcl::PointXYZ>);
				segs.emplace_back();
				// Optional
				segs[i].setOptimizeCoefficients(true);
				// Mandatory
				segs[i].setModelType(pcl::SACMODEL_PLANE);
				segs[i].setMethodType(pcl::SAC_RANSAC);
				segs[i].setDistanceThreshold(0.3);
				// Set the cloud to use for plane fitting
				segs[i].setInputCloud(clouds[i]);
				// Emplace back new coefficients and inliers
				coefficients.emplace_back(new pcl::ModelCoefficients);
				inliers.emplace_back(new pcl::PointIndices);
			}
			if (clouds[i]->size())
			{
				// Beginning planar segmentation
				startTime = std::chrono::duration_cast<std::chrono::milliseconds>(
								std::chrono::system_clock::now().time_since_epoch())
								.count();
				segs[i].segment(*inliers[i], *coefficients[i]);
				// Check to see if a model was found
				if (inliers[i]->indices.size() == 0)
				{
					std::cerr << "Could not estimate a planar model for the given dataset.\n";
					std::cerr << "Iteration number: " << i << '\n';
					boolVar = 0;
				}
				endTime = std::chrono::duration_cast<std::chrono::milliseconds>(
							  std::chrono::system_clock::now().time_since_epoch())
							  .count();
				times2.push_back(endTime - startTime);
				endTime = std::chrono::duration_cast<std::chrono::milliseconds>(
							  std::chrono::system_clock::now().time_since_epoch())
							  .count();
			}
			else
			{
				boolVar = 0;
			}
			size_t inliersCounter{0};
			for (size_t ii{0}; ii < clouds[i]->size(); ++ii)
			{
				if (inliersCounter == inliers[i]->indices.size() ||
					ii != inliers[i]->indices[inliersCounter])
				{
					clouds[i + 1]->push_back(clouds[i]->at(ii));
				}
				else
				{
					++inliersCounter;
				}
			}
		}
	}
	for (size_t i{0}; i < coefficients.size(); ++i)
	{
		if (coefficients[i]->values.size())
		{
			std::cerr << "Model coefficients: " << coefficients[i]->values[0] << " "
					  << coefficients[i]->values[1] << " " << coefficients[i]->values[2] << " "
					  << coefficients[i]->values[3] << std::endl;

			std::cerr << "Model inliers: " << inliers[i]->indices.size() << std::endl;
		}
	}
	std::cerr << "Time it took to get image and store points:\n";
	for (size_t i{0}; i < times.size(); ++i)
	{
		std::cerr << times[i] << ", ";
	}
	std::cerr << std::endl;
	std::cerr << "Time it took to run plane fitting:\n";
	for (size_t i{0}; i < times2.size(); ++i)
	{
		std::cerr << times2[i] << ", ";
	}
	std::cerr << std::endl;
	// Prints out each bloody inlier
	/*
	for (size_t i = 0; i < inliers->indices.size (); ++i)
	{
		std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
		<< cloud->points[inliers->indices[i]].y << " "
		<< cloud->points[inliers->indices[i]].z << std::endl;
	}
	*/
}
