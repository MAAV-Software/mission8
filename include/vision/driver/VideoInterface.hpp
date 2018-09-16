#ifndef _VIDEO_INTERFACE_HPP_
#define _VIDEO_INTERFACE_HPP_

#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>
#include <string>
#include <cstdint>
#include <memory>
#include "zcm/zcm-cpp.hpp"
#include <librealsense/rs.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/pcd_io.h>

#include "../depth-utils/Point3f.hpp"



class VideoInterface
{
private:
	class HandlerFrame;
	struct VisionSimZCMData;
	class Impl;
	class cvVideoCapture;
	class RGBDGetterInterface;
	class CameraInputInterface;
	class PhysSimInterface;
	std::unique_ptr<Impl> pimpl;
	// Constructor for sim interface without new zcm object
	// must pass in a pointer to an already existing object
	// within another instance of this class
	VideoInterface(VisionSimZCMData *zcmDataIn);
	// commands impl to initialize zcm threads and zcm within this
	// instance of impl and returns the struct
	void init(VisionSimZCMData *&zcmDataOut);
public:
	// Constructor of video capture object
	// type = 0 is standard cv::VideoCapture (from camera)
	// type = 1 is standard cv::VideoCapture (from file)
	// type = 2 is sim VideoCapture
	explicit VideoInterface(int type); // TODO
	// Function that accepts an std::vector of size 5 and fills it
	// with VideoInterface type 2 pointers, the de facto
	// constructor for type 2 VideoInterface (Phys_Sim_Intrface)
	static void spawnT2Caps(std::vector<VideoInterface*> &cap);
	// Opens the indicated video input
	bool open(int index);
	bool open(std::string filename);
	// Wrapper for cv::VideoCapture::isOpened()
	bool isOpened();
	// Wrapper for cv::VideoCapture::release()
	void release();
	// Wrapper for cv::VideoCapture::grab()
	bool grab();
	// Wrapper for cv::VideoCapture::retrieve(cv::Mat&,int)
	bool retrieve(cv::Mat &image, int channel=0);
	// Wrapper for cv::VideoCapture::read(cv::Mat &image)
	bool read(cv::Mat &image);

	bool retrieve(pcl::PointCloud<pcl::PointXYZ> &cloud, int channel);
	bool read(pcl::PointCloud<pcl::PointXYZ> &cloud);
	bool retrieve(std::vector<pf::Point3f> & cloud, int channel);
	bool read(std::vector<pf::Point3f> & cloud);


	// Wrapper for cv::VideoCapture::get(int)
	double get(int propID);
	// Wrapper for cv::VideoCapture::set(int,double)
	bool set(int propID, double value);

	void setContext(rs::context * context);
	rs::context * getContext();
	// Destructor for VideoInterface
	~VideoInterface();
};



#endif
