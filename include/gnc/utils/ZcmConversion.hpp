#pragma once

#include <memory>

#include <Eigen/Eigen>

#include <common/messages/global_update_t.hpp>
#include <common/messages/imu_t.hpp>
#include <common/messages/lidar_t.hpp>
#include <common/messages/plane_fit_t.hpp>
#include <common/messages/state_t.hpp>
#include <common/messages/waypoint_t.hpp>
#include <gnc/State.hpp>
#include <gnc/measurements/GlobalUpdateMeasurement.hpp>
#include <gnc/measurements/ImuMeasurement.hpp>
#include <gnc/measurements/LidarMeasurement.hpp>
#include <gnc/measurements/PlaneFitMeasurement.hpp>
#include <gnc/measurements/Waypoint.hpp>
namespace maav
{
namespace gnc
{
state_t ConvertState(const State& state);
State ConvertState(const state_t& state);

Waypoint ConvertWaypoint(const waypoint_t& zcm_waypoint);

std::shared_ptr<measurements::LidarMeasurement> convertLidar(const lidar_t& zcm_lidar);
// lidar_t convertLidar(const measurements::LidarMeasurement& lidar);

std::shared_ptr<measurements::ImuMeasurement> convertImu(const imu_t& zcm_imu);
// imu_t convertImu(const measurements::ImuMeasurement& imu);

std::shared_ptr<measurements::PlaneFitMeasurement> convertPlaneFit(
    const plane_fit_t& zcm_plane_fit);
// plane_fit_t convertPlaneFit(const measurements::PlaneFitMeasurement& plane_fit);

std::shared_ptr<measurements::GlobalUpdateMeasurement> convertGlobalUpdate(
    const global_update_t& zcm_global);
// global_update_t convertGlobalUpdate(const measurements::GlobalUpdateMeasurement& global);
}
}
