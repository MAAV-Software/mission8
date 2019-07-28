#include <vision/obstacle/NaiveObstacle.hpp>
#include <pcl/common/distances.h>
#include <pcl/common/centroid.h>

#include <memory>

using std::vector;
using std::shared_ptr;
using Eigen::Vector3d;
using pcl::PointCloud;
using pcl::PointXYZ;
using pcl::euclideanDistance;
using pcl::CentroidPoint;

constexpr float maxPointDist = 1.5; // Maximum distance of points from zero point
constexpr float maxObstacleDist = 0.5; // Maximum distance of points from main obstacle point
constexpr size_t minPointsInObstacle = 50;

// Simple class used to bin points into obstacles
// and then retrieve the list of obstacles compiled here
class ObstaclesHolder
{
public:
    void addPoint(PointXYZ& point)
    {
        // Try to fit into new obstacles
        for (size_t i = 0; i < obstacles_.size(); ++i)
        {
            if (euclideanDistance<PointXYZ, PointXYZ>(obstacles_[i], point) < maxObstacleDist)
            {
                centroids_[i]->add(point);
                return;
            }
        }
        // Could not fit into any already present obstacles, put here
        obstacles_.push_back(point);
        centroids_.emplace_back();
        centroids_.back()->add(point);
    }
    vector<Vector3d> getObstacles()
    {
        vector<Vector3d> eigenObstacles;
        eigenObstacles.reserve(obstacles_.size());
        for (auto& centroid : centroids_)
        {
            if (centroid->getSize() >= minPointsInObstacle)
            {
                PointXYZ center;
                centroid->get(center);
                eigenObstacles.emplace_back(center.x, center.y, center.z);
            }
        }
        return eigenObstacles;
    }
private:
    vector<PointXYZ> obstacles_;
    vector<shared_ptr<CentroidPoint<PointXYZ>>> centroids_;
};

vector<Vector3d> maav::vision::NaiveObstacle::detectObstacles(PointCloud<PointXYZ>::Ptr cloud)
{
    // Traverse the point cloud keeping only points that are within 1.5m of the camera
    PointXYZ zeroPoint(0, 0, 0);
    PointCloud<PointXYZ>::Ptr filteredCloud = PointCloud<PointXYZ>::Ptr(new PointCloud<PointXYZ>());
    for (auto& point : *cloud)
    {
        if (euclideanDistance<PointXYZ, PointXYZ>(zeroPoint, point) < maxPointDist)
        {
            filteredCloud->push_back(point);
        }
    }
    ObstaclesHolder obstacles;
    for (auto& point : *filteredCloud)
    {
        obstacles.addPoint(point);
    }
    return obstacles.getObstacles();
}