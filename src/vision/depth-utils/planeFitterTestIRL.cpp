#include <iostream>
#include <vector>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <common/utils/GetOpt.hpp>
#include <librealsense2/rs.hpp>

#include "vision/core/D400CameraInterface.hpp"
#include "vision/core/PlaneFitter.hpp"

using maav::vision::D400CameraInterface;
using pcl::PointCloud;
using pcl::PointXYZ;
using std::cout;
using std::endl;
using std::vector;

int main(int argc, char** argv)
{
    GetOpt gopt;
    gopt.addBool('h', "help", false, "This message");
    gopt.addString('c', "config", "../config/imu-config.yaml", "Path to config.");

    if (!gopt.parse(argc, argv, 1) || gopt.getBool("help"))
    {
        std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
        gopt.printHelp();
        return 1;
    }

    YAML::Node config = YAML::LoadFile(gopt.getString("config"));

    D400CameraInterface cam(config["downward"]);

    maav::vision::PlaneFitter planeFitter(0.05f);
    PointCloud<PointXYZ>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZ>());
    vector1_t zdot;
    vector1_t zdepth;
    vector1_t roll;
    vector1_t pitch;
    unsigned long long utime = 0;
    while (true)
    {
        cam.loadNext();
        cloudPtr = cam.getPointCloudBasic();
        if (planeFitter.runPlaneFitting(cloudPtr, zdot, zdepth, roll, pitch, utime))
        {
            cout << planeFitter.getLastHeight() << '\t' << zdot.data[0] << '\t' << zdepth.data[0]
                 << '\t' << roll.data[0] << '\t' << pitch.data[0] << endl;
        }
        else
        {
            cout << "FAILED: " << cloudPtr->size() << endl;
        }
    }
}
