
#ifndef CAMERA_INPUT_H
#define CAMERA_INPUT_H

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cstdio>
#include <librealsense/rs.hpp>
#include <vector>
#include "CameraInputBase.hpp"
#include "Point3f.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

// pulls data frame-by-frame directly from the camera
// and provides the RGBD data in various forms
class CameraInput : public CameraInputBase
{
   public:
	// Creates a new CameraInput instance that pulls
	// frames from the camera associated with the provided
	// camera id (creates a new rs::context to find the device)
	//
	// _id : the camera id for the wanted camera
	explicit CameraInput(int _id);

	// Creates a new CameraInput instance that pulls
	// frames from the camera associated with the provided
	// camera id (using the given rs::context to find the device)
	//
	// _id : the camera id for the wanted camera
	// ctx_in : the rs::context for the camera
	// sourceIn : whether or not this instance is a camera source
	// 		(whether it "owns" the rs::context)
	CameraInput(int _id, rs::context *ctx_in, bool sourceIn);

	// Creates a new CameraInput instance that pulls
	// frames from the camera associated with the provided
	// camera id (using the given rs::context to find the device)
	//
	// _id : the camera id for the wanted camera
	// ctx_in : the rs::context for the camera
	CameraInput(int _id, rs::context *ctx_in);

	// A CameraInput instance cannot be created without
	// at least a given camera id
	CameraInput() = delete;

	// Deletes the rs::context if the CameraInput instance
	// is a source
	~CameraInput();

	// Returns the rs::context
	rs::context *getContext();

	// pulls the RGB data for the current camera frame
	// and provides it in a cv::Mat
	//
	// img : a cv::Mat reference to hold the returned RGB data
	virtual void getRGB(cv::Mat &img) const override;

	// pulls the Depth data for the current camera frame
	// and provides it in a cv::Mat
	//
	// img : a cv::Mat reference to hold the returned depth data
	virtual void getDepth(cv::Mat &img) const override;

	// pulls the combined RGB & Depth (RGBD) data for the
	// current camera frame and provides it in a cv::Mat
	//
	// img : a cv::Mat reference to hold the returned RGBD data
	void getCombined(cv::Mat &img);

	// pulls the Point Cloud data for the current camera
	// frame and provides it in a pcl::PointCloud
	// -- gives the XYZ data for each point
	//
	// cloud : a pcl::PointCloud reference to hold the returned data
	void getCloud(pcl::PointCloud<pcl::PointXYZ> &cloud);

	// pulls the Point CLoud data for the current camera
	// frame and provides it in a vector of Points
	//
	// cloud : a vector of Points to hold the returned data
	void getCloud(std::vector<pf::Point3f> &cloud);

	// pulls the Point Cloud data for the current camera
	// frame and provides it in a pcl::PointCloud
	// -- gives the XYZRGB data for each point
	//
	// cloud : a pcl::PointCloud reference to hold the returned data
	void getCloudXYZRGB(pcl::PointCloud<pcl::PointXYZRGB> &cloud);

	// pulls the Point Cloud data for the current camera
	// frame and provides it in a pcl::PointCloud
	// -- gives the XYZRGBA data for each point
	//
	// cloud : a pcl::PointCloud reference to hold the returned data
	void getCloudXYZRGBA(pcl::PointCloud<pcl::PointXYZRGBA> &cloud);

	// increments the camera to the next available frame
	// all the get methods now pull data from the most recent frame
	virtual void loadNext() override;

	// increments to the next frame using loadNext
	// Returns the CameraInput instance following the increment
	virtual CameraInput &operator++() override;

	// returns the id associated with the camera
	int getCamID() const;

	// These two not currently implemented
	virtual void getPointCloudBasic(pcl::PointCloud<pcl::PointXYZ> &cloud) const override;

	virtual void getMappedPointCloud(pcl::PointCloud<pcl::PointXYZ> &cloud) const override;

   private:
	int camera_id;
	bool isSource;

	rs::device *devicePtr;
	rs::context *ctx;
	const uint16_t *depthImage;
	const uint8_t *colorImage;

	float scale;
	rs::intrinsics depthIntrinsics;
	rs::extrinsics depth_to_color;
	rs::intrinsics color_intrin;
};

#endif
