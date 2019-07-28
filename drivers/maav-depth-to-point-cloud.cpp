#include <librealsense2/rsutil.h>
#include <librealsense2/rs.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <opencv2/imgproc/imgproc.hpp>

#include <vector>
#include <string>
#include <yaml-cpp/yaml.h>

#include <iostream>

#include <vision/core/D400CameraInterface.hpp>

int main(int argc, char** argv)
{
    std::string config_path = "../config/camera-config.yaml";
    YAML::Node config = YAML::LoadFile(config_path);
    maav::vision::D400CameraInterface camera(config["forward"]);
    for (size_t i = 1; i < (size_t)argc; ++i)
    {
        cv::FileStorage fs;
        fs.open(argv[i], cv::FileStorage::READ);

        if (!fs.isOpened()) 
        {
            continue;
        }

        cv::Mat depth_img;
        fs["depth"] >> depth_img; 

        std::string pcd_name = argv[i];
        pcd_name += ".pcd";

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = camera.convertToCloud(depth_img);
        pcl::io::savePCDFileASCII(pcd_name.c_str(), *cloud);
    }
}