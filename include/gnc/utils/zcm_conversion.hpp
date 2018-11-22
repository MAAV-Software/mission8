#pragma once

#include <Eigen/Eigen>
#include <common/messages/lidar_t.hpp>
#include <common/messages/state_t.hpp>
#include <common/messages/waypoint_t.hpp>
#include <gnc/State.hpp>
#include <gnc/measurements/Lidar.hpp>
#include <gnc/measurements/Waypoint.hpp>
namespace maav
{
namespace gnc
{
state_t convert_state(const State& state);
State convert_state(const state_t& state);
Waypoint convert_waypoint(const waypoint_t& zcm_waypoint);

measurements::LidarMeasurement convertLidar(const lidar_t& zcm_lidar);
lidar_t convertLidar(const measurements::LidarMeasurement& lidar);
}
}
