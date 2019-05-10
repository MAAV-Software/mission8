#ifndef __MAAV_OCCUPANCY_MAP__HPP__
#define __MAAV_OCCUPANCY_MAP__HPP__

#include <vector>
#include <memory>
#include <yaml-cpp/yaml.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <zcm/zcm-cpp.hpp>
#include <gnc/PointMapper.hpp>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

namespace maav
{
namespace gnc
{
/**
 * @brief Takes in Point Cloud measurements to update and build an occupancy grid 
 *		  using octomap library
 *
 * @details wrapper around octomap OctTree class and the PointMapper class. Takes in 
 * point cloud measurements, maps the points to the global reference frame, and updates
 * Octree. The member pointmapper_ subscribes to the STATE channel automatically so a 
 * zcm instance must be passed in.
 *
 */

class OccupancyMap
{
public:
    /**
     * @param config This yaml node is used to tune the octomap
	 * @param camera_config yaml node for the camera extrinsics and intrinsics
     */
    explicit OccupancyMap(YAML::Node& config, YAML::Node& camera_config, zcm::ZCM &zcm);

    void update(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, uint64_t utime);

    std::shared_ptr<const octomap::OcTree> map() const { return octree_; }

	// Unblocks the point mapper if waiting if program is being killed
    void kill() { point_mapper_.kill(); };

private:
	double map_res_;
	double prob_hit_;
	double prob_miss_;
	double thresh_min_;
	double thresh_max_;
	double max_range_;
	double point_cloud_min_x_;
	double point_cloud_max_x_;
	double point_cloud_min_y_;
	double point_cloud_max_y_;
	double point_cloud_min_z_;
	double point_cloud_max_z_;
	bool compress_map_;
	bool simulator;
	PointMapper point_mapper_;
	std::shared_ptr<octomap::OcTree> octree_;
	std::vector<pcl::PassThrough<pcl::PointXYZ>> cloud_filters_;
};

}  // namespace gnc
}  // namespace maav

#endif
