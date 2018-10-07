#include "vision/depth-utils/PlaneFitterPCL.hpp"

#include <chrono>
#include <cmath>
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

#include "vision/depth-utils/Point3f.hpp"

using std::cout;
using std::cerr;
using std::string;
using std::vector;
using cv::Mat;

PlaneFitterPCL::PlaneFitterPCL(float inlierThreshold) : inlierThresh{inlierThreshold}, lastHeight{0}
{
}

void PlaneFitterPCL::convertCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
								  std::vector<pf::Point3f> &points)
{
	for (auto &point : points)
	{
		cloud->push_back(pcl::PointXYZ(point.x, point.y, point.z));
	}
}

Eigen::MatrixXf PlaneFitterPCL::fitPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> segs;
	// Optional
	segs.setOptimizeCoefficients(true);
	// Mandatory
	segs.setModelType(pcl::SACMODEL_PLANE);
	segs.setMethodType(pcl::SAC_RANSAC);
	segs.setDistanceThreshold(inlierThresh);
	// Old distanceThreshold at 0.3

	segs.setInputCloud(cloud);
	segs.setOptimizeCoefficients(true);
	// Set the cloud to use for plane fitting
	segs.setInputCloud(cloud);
	if (cloud->size())
	{
		// Beginning planar segmentation
		segs.segment(*inliers, *coefficients);
		// Check to see if a model was found
		if (inliers->indices.size() == 0)
		{
			return Eigen::MatrixXf(0, 0);
		}
	}
	else
	{
		return Eigen::MatrixXf(0, 0);
	}
	if (coefficients->values.size())
	{
		Eigen::MatrixXf coefficientsMatrix(4, 1);
		for (unsigned i = 0; i < 4; ++i)
		{
			coefficientsMatrix(i, 0) = coefficients->values[i];
		}
		return coefficientsMatrix;
	}
	return Eigen::MatrixXf(0,0);
}

bool PlaneFitterPCL::runPlaneFitting(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
	float &zdot, float &zdepth ,float &roll, float &pitch)
{
	return getPlaneInfo(cloud, zdot, zdepth, roll, pitch, junkMatrix);
}

bool PlaneFitterPCL::getPlaneInfo(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
	float &zdot, float &zdepth, float &roll, float & pitch,
	Eigen::MatrixXf& coefs)
{
	coefs = fitPlane(cloud);
	if (coefs.size() == 0)
	{
		return false;
	}
	// Sets the cosAlpha to coefs(2)
	float cosAlpha = coefs(2);
	if (coefs(2) == 0.f)
	{
		return false;
	}
	zdepth = abs(coefs(3) / coefs(2));
	float height = abs(cosAlpha * zdepth);
	zdot = height - lastHeight;
	lastHeight = height;
	// TODO Implement the plane fitting stuff here
	// assuming quad plane normal is [0,0,1]
	float xq = 0, yq = 0, zq = 1;
	float xg = coefs(0);
	float yg = coefs(1);
	float zg = coefs(2);
	roll = acos((xg * xq + zg *zq) / sqrt(xg * xg + zg *zg));
	pitch = acos((yg *yq + zg *zq) / sqrt(yg * yg + zg * zg));
	if (yg < 0)
	{
		pitch *= -1;
	}

	if (xg < 0)
	{
		roll *= -1;
	}
	return true;
}

float PlaneFitterPCL::getLastHeight() const { return lastHeight; }
