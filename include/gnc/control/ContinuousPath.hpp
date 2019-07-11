#pragma once

#include <common/messages/path_t.hpp>

#include <Eigen/Eigen>

#include <utility>
#include <vector>

namespace maav
{
namespace gnc
{
/**
 * Abstract class for interpolating between waypoints provided by the path planner
 */
class ContinuousPath
{
public:
    struct Waypoint
    {
        Waypoint() = default;
        Waypoint(const waypoint_t& waypoint);
        Waypoint(const Waypoint&) = default;
        Eigen::Vector3d position;
        double heading;
    };

    ContinuousPath(const path_t& path);

    /**
     * Replace the current path with a new one
     */
    virtual void updatePath(const path_t& path);

    /**
     * Sample the trajectory at a time
     */
    virtual Waypoint sample(uint64_t time) const = 0;

    /**
     * Get the minimum time allowed to be sampled
     */
    uint64_t getStartTime() const;

protected:
    uint64_t starting_time_;
    std::vector<Waypoint> path_;
};
}  // namespace gnc
}  // namespace maav
