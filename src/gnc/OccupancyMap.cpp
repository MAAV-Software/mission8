#include <gnc/OccupancyMap.hpp>

#include <string>
#include <limits>
#include <iostream>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

using std::string;
using std::vector;
using std::make_shared;
using std::numeric_limits;
using Eigen::Matrix4d;
using Eigen::Matrix4f;
using Eigen::Vector3d;


namespace maav
{
namespace gnc
{

OccupancyMap::OccupancyMap(YAML::Node& config, YAML::Node& camera_config, zcm::ZCM &zcm)
  : map_res_(config["map_res"].as<double>()),
    prob_hit_(config["prob_hit"].as<double>()),
    prob_miss_(config["prob_miss"].as<double>()),
     thresh_min_(config["thresh_min"].as<double>()),
     thresh_max_(config["thresh_max"].as<double>()),
    max_range_(config["max_range"].as<double>()),
     point_cloud_min_x_(config["point_cloud_min_x"].as<double>()),
     point_cloud_max_x_(config["point_cloud_max_x"].as<double>()),
     point_cloud_min_y_(config["point_cloud_min_y"].as<double>()),
     point_cloud_max_y_(config["point_cloud_max_y"].as<double>()),
     point_cloud_min_z_(config["point_cloud_min_z"].as<double>()),
     point_cloud_max_z_(config["point_cloud_max_z"].as<double>()),
    compress_map_(config["compress"].as<bool>()),
    point_mapper_{PointMapper(camera_config, zcm, config["simulator"].as<bool>())}
{
    octree_ = make_shared<octomap::OcTree>(map_res_);
    // octree_->setProbHit(prob_hit_);
    // octree_->setProbMiss(prob_miss_);
    // octree_->setClampingThresMin(thresh_max_);
    // octree_->setClampingThresMax(thresh_min_);
    std::cout << "ProbHit : " << octree_->getProbHit() << std::endl;
    std::cout << "ProbMiss : " << octree_->getProbMiss() << std::endl;
    std::cout << "ClampingThreshMin : " << octree_->getClampingThresMin() << std::endl;
    std::cout << "ClampingThreshMax : " << octree_->getClampingThresMax() << std::endl;
    cloud_filters_.resize(3);
    cloud_filters_[0].setFilterFieldName("x");
    cloud_filters_[0].setFilterLimits(point_cloud_min_x_, point_cloud_max_x_);
    cloud_filters_[1].setFilterFieldName("y");
    cloud_filters_[1].setFilterLimits(point_cloud_min_y_, point_cloud_max_y_);
    cloud_filters_[2].setFilterFieldName("z");
    cloud_filters_[2].setFilterLimits(point_cloud_min_z_, point_cloud_max_z_);
}

void OccupancyMap::update(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, uint64_t utime)
{
    for (auto& cf : cloud_filters_)
    {
        cf.setInputCloud(cloud);
        cf.filter(*cloud);
    }
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr world_cloud = point_mapper_.transformCloud(cloud, utime);
    Vector3d camera_origin = point_mapper_.getCameraOrigin(utime);
    

    // octomap pointcloud type
    octomap::Pointcloud pc;
    for (unsigned i = 0; i < world_cloud->size(); ++i)
    {
        const auto pt = (*world_cloud)[i];
        pc.push_back(pt.x, pt.y, pt.z);
    }
    // octomap point type
    octomap::point3d sensor_origin(camera_origin(0), camera_origin(1), camera_origin(2));

    // octomap point cloud insertion
    octree_->insertPointCloud(pc, sensor_origin, max_range_, true, true);
    //octree_->insertPointCloud(pc, sensor_origin, frame_origin, max_range_, false, true);
    octree_->updateInnerOccupancy();
    if (compress_map_) octree_->prune();
}
/*
* TODO:
* Optimize code with a concurrent filter - concurrent filter doesn't exist in PCL. Maybe we can template
*/

}
}
