#include <iostream>
#include <vector>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <librealsense2/rs.hpp>

#include "vision/depth-utils/NewCameraInput.hpp"
#include "vision/depth-utils/PlaneFitterPCL.hpp"
#include "vision/depth-utils/Point3f.hpp"

using std::cout;
using std::endl;
using std::vector;

int main()
{
	rs2::context ctx;
	auto list = ctx.query_devices();
	std::cout << list.size() << " rs2 devices connected.\n";
	NewCameraInput cam(list[0].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));

	PlaneFitterPCL planeFitter(0.05);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZ>());
	float zdot = 0;
	float zdepth = 0;
	float roll = 0;
	float pitch = 0;
	while (true)
	{
		cam.loadNext();
		cam.getPointCloudBasic(*cloudPtr);
		if (planeFitter.runPlaneFitting(cloudPtr, zdot, zdepth, roll, pitch))
		{
			cout << planeFitter.getLastHeight() << '\t' <<
				zdot << '\t' << zdepth << '\t' << roll << '\t' << pitch << endl;
		}
		else
		{
			cout << "FAILED: " << cloudPtr->size() << endl;
		}
	}
}
