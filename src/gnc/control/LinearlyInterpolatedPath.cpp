#include <gnc/Constants.hpp>
#include <gnc/control/LinearlyInterpolatedPath.hpp>

#include <iostream>

#include <stdexcept>

namespace maav
{
namespace gnc
{
LinearlyInterpolatedPath::LinearlyInterpolatedPath(const path_t& path, double speed)
    : ContinuousPath(path), speed_(speed)
{
    updatePath(path);
}

void LinearlyInterpolatedPath::updatePath(const path_t& path)
{
    ContinuousPath::updatePath(path);

    waypoint_dictionary_.clear();
    double time = 0;
    waypoint_dictionary_.insert({time, 0});
    for (size_t i = 1; i < path_.size(); i++)
    {
        const auto& prev_waypoint = path_[i - 1];
        const auto& curr_waypoint = path_[i];

        const Eigen::Vector3d difference = curr_waypoint.position - prev_waypoint.position;
        const double distance = difference.norm();
        const double segment_duration = distance / speed_;

        if (segment_duration > 0)
        {
            time += segment_duration;
            waypoint_dictionary_.insert({time, i});
        }
    }
}

ContinuousPath::Waypoint LinearlyInterpolatedPath::sample(uint64_t time) const
{
    const uint64_t diff = time - getStartTime();
    const double t = static_cast<double>(diff) * constants::USEC_TO_SEC;
    if (t < 0)
    {
        throw std::runtime_error("You must sample a spline after the starting control point.");
    }

    auto lower_bound = waypoint_dictionary_.lower_bound(t);
    if (lower_bound->first != t)
    {
        lower_bound--;
    }
    const size_t prev_waypoint_index = lower_bound->second;
    const Waypoint& previous_waypoint = path_[prev_waypoint_index];

    auto upper_bound = waypoint_dictionary_.upper_bound(t);

    // If we reach the end of the path, just try return the last waypoint
    if (upper_bound == waypoint_dictionary_.end())
    {
        return previous_waypoint;
    }

    const size_t next_waypoint_index = upper_bound->second;
    const Waypoint& next_waypoint = path_[next_waypoint_index];

    const double prev_time = lower_bound->first;
    const double next_time = upper_bound->first;
    const double segment_duration = next_time - prev_time;

    const double time_passed = t - prev_time;
    const double segment_completion_ratio = time_passed / segment_duration;

    const Eigen::Vector3d difference = next_waypoint.position - previous_waypoint.position;
    const double heading_difference = next_waypoint.heading - previous_waypoint.heading;

    // Linearly interpolate between the two waypoints
    Waypoint interpolated_waypoint;
    interpolated_waypoint.position =
        previous_waypoint.position + difference * segment_completion_ratio;

    interpolated_waypoint.heading =
        previous_waypoint.heading + heading_difference * segment_completion_ratio;

    return interpolated_waypoint;
}
}  // namespace gnc
}  // namespace maav