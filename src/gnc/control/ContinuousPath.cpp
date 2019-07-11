#include <gnc/control/ContinuousPath.hpp>

namespace maav
{
namespace gnc
{
ContinuousPath::Waypoint::Waypoint(const waypoint_t& waypoint)
    : position{waypoint.pose[0], waypoint.pose[1], waypoint.pose[2]}, heading(waypoint.pose[3])
{
}

ContinuousPath::ContinuousPath(const path_t& path) { updatePath(path); }

uint64_t ContinuousPath::getStartTime() const { return starting_time_; }

void ContinuousPath::updatePath(const path_t& path)
{
    starting_time_ = path.utime;

    path_.clear();
    path_.reserve(path.waypoints.size());
    for (const waypoint_t& waypoint : path.waypoints)
    {
        path_.emplace_back(waypoint);
    }
}

}  // namespace gnc
}  // namespace maav