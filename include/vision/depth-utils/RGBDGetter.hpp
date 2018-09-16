
#ifndef RGBDGETTER_H
#define RGBDGETTER_H

#include <fstream>
#include <cstdint>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <librealsense/rs.hpp>
#include <cstdio>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/pcd_io.h>

#include "Point3f.hpp"


// An iterator-like class for the purposes of extracting
// RGBD data from the filesystem and providing it in various forms
class RGBDGetter {
	public:

		// Creates a new RGBDGetter with the given numbered directory
		//
		// _num : the number of the directory to pull RGBD data from
		explicit RGBDGetter(int _num);

		// an RGBDGetter cannot be created without a specified directory
		RGBDGetter() = delete;

		// pulls the RGB data for the frame at the present counter position
		// and provides it in a cv::Mat
		//
		// img : a cv::Mat reference to hold the returned RGB data
		void getRGB(cv::Mat &img);

		// pulls the Depth data for the frame at the present counter position
		// and provides it in a cv::Mat
		//
		// img : a cv::Mat reference to hold the returned depth data
		void getDepth(cv::Mat &img);

		// pulls the combined RGB & Depth (RGBD) data for the frame at the
		// present counter position and provides it in a cv::Mat
		//
		// img : a cv::Mat reference to hold the returned RGBD data
		void getCombined(cv::Mat &img);

		// pulls the Point Cloud data for the frame at the present counter
		// position and provides it in a pcl::PointCloud
		// -- gives the XYZ data for each point
		//
		// cloud : a pcl::PointCloud reference to hold the returned data
		void getCloud(pcl::PointCloud<pcl::PointXYZ> &cloud);

		// pulls the Point Cloud data for the frame at the present counter
		// position and provides it in a vector of Points
		//
		// cloud : a vector of Points to hold the returned data
		void getCloud(std::vector<pf::Point3f> & cloud);

		// pulls the Point Cloud data for the frame at the present counter
		// position and provides it in a pcl::PointCloud
		// -- gives the XYZRGBA data for each point
		//
		// cloud : a pcl::PointCloud reference to hold the returned data
		void getCloudXYZRGBA(pcl::PointCloud<pcl::PointXYZRGBA> &cloud);

		// increments the counter to the next frame
		// all the get methods now pull from the following frame
		// the timestamp is updated to the time of the next frame
		RGBDGetter & operator++();

	private:
		int counter = 0;
		int64_t timestamp = 0;
		std::ifstream timestamps;
		int dir_num;
		void openMatFromDirectory(std::string, cv::Mat&);

};

#endif

