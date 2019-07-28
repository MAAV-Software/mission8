#ifndef ___NAIVE_OBSTACLE__HPP___
#define ___NAIVE_OBSTACLE__HPP___

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>
#include <Eigen/Core>

/* Using the depth image:
 * 1.1. Threshold the image to only include the pixels that represent points
 * close enough to the vehicle to be considered potentially part of a
 * threatening obstacle.
 * 1.2. Only proceed to the segmentation phase if there are enough points close
 * to the vehicle to potentially be part of a threat to the vehicle (probably
 * tune this using the config.)
 * 1.3. Segment the depth image into regions based on continuous not changing to
 * only slightly changing depth.
 * Segmentation can be done by computing the derivative of the image, and then
 * thresholding to only include areas of smooth transition, thus splitting apart
 * the regions. Then each one could be labeled and turned into point clouds.
 * 2. Compute the location and size of each nearby region by modeling it as a
 * sphere.
 * 3. Based upon the configuration passed in, return the detected obstacles that
 * pose a threat to the vehicle.
 * 0. Use the direction that the vehicle is traveling in to filter out obstacles
 * that otherwise would be considered a threat.
 */

/* Using the point cloud:
 * 1. Sort the points in the point cloud by distance from (0, 0, 0).
 * 2. If there are enough points within some distance that can be set from
 * the config, then run kmeans (or some other clustering) to sort the points
 * into clusters that can be represented as gaussian distributions. Then return
 * these distributions that represent threatening obstacles.
 * 0. Instead of kmeans, "gaussian growing" can be used, in which points are
 * iteratively added to a cluster if it is near it
 */

/* Extra naive point cloud:
 * 1. Count the number of points that are near the vehicle. If that number is
 * higher than the configured highest safe value, then move to step 2.
 * 2. Compute the centroid of the threatening points, and return that as the
 * threatening obstacle.
 */
namespace maav
{
namespace vision
{
class NaiveObstacle
{
public:
    std::vector<Eigen::Vector3d> detectObstacles(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
};
}
}

#endif