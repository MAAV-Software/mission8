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

#include "Point3f.hpp"

class PlaneFitterPCL
{
   public:
	PlaneFitterPCL(float inlierThreshold);
	// Used mainly for testing converts from dz point
	// cloud to pcl point cloud
	void convertCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::vector<pf::Point3f> &points);
	// Provide a point cloud, will provide plane coefficients
	// or zero dimension matrix if failure
	Eigen::MatrixXf fitPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
	// Calculate the zdepth and the zdot
	bool runPlaneFitting(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, float &zdot, float &zdepth);
	// Obtain all the plane information needed by driver
	bool getPlaneInfo(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, float &zdot, float &zdepth,
					  Eigen::MatrixXf &coefs);
	// Get the last height
	float getLastHeight() const;

   private:
	float inlierThresh;
	float lastHeight;
	Eigen::MatrixXf junkMatrix;
};
