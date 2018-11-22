#include <gnc/utils/zcm_conversion.hpp>

namespace maav
{
namespace gnc
{
state_t convert_state(const State& state)
{
    state_t zcm_state;
    zcm_state.utime = state.timeUSec();
    const Eigen::Quaterniond& q = state.attitude().unit_quaternion();
    zcm_state.attitude[0] = q.w();
    zcm_state.attitude[1] = q.x();
    zcm_state.attitude[2] = q.y();
    zcm_state.attitude[3] = q.z();

    zcm_state.position[0] = state.position().x();
    zcm_state.position[1] = state.position().y();
    zcm_state.position[2] = state.position().z();

    zcm_state.velocity[0] = state.velocity().x();
    zcm_state.velocity[1] = state.velocity().y();
    zcm_state.velocity[2] = state.velocity().z();

    zcm_state.angular_velocity[0] = state.angularVelocity().x();
    zcm_state.angular_velocity[1] = state.angularVelocity().y();
    zcm_state.angular_velocity[2] = state.angularVelocity().z();

    zcm_state.acceleration[0] = state.acceleration().x();
    zcm_state.acceleration[1] = state.acceleration().y();
    zcm_state.acceleration[2] = state.acceleration().z();

    // TODO: add covariance, gyro_biases, accel_biases

    return zcm_state;
}

State convert_state(const state_t& zcm_state)
{
    // Convert to State
    State state(zcm_state.utime);
    Eigen::Vector3d av(zcm_state.angular_velocity[0], zcm_state.angular_velocity[1],
        zcm_state.angular_velocity[2]);
    Eigen::Vector3d pos(zcm_state.position[0], zcm_state.position[1], zcm_state.position[2]);
    Eigen::Vector3d vel(zcm_state.velocity[0], zcm_state.velocity[1], zcm_state.velocity[2]);
    Eigen::Vector3d accel(
        zcm_state.acceleration[0], zcm_state.acceleration[1], zcm_state.acceleration[2]);
    state.angularVelocity() = av;
    state.position() = pos;
    state.velocity() = vel;
    state.acceleration() = accel;
    state.attitude() = Sophus::SO3d(Eigen::Quaterniond(zcm_state.attitude[0], zcm_state.attitude[1],
        zcm_state.attitude[2], zcm_state.attitude[3]));

    return state;
}

Waypoint convert_waypoint(const waypoint_t& zcm_waypoint)
{
    Waypoint waypoint;
    waypoint.position =
        Eigen::Vector3d(zcm_waypoint.pose[0], zcm_waypoint.pose[1], zcm_waypoint.pose[2]);
    waypoint.velocity =
        Eigen::Vector3d(zcm_waypoint.rate[0], zcm_waypoint.rate[1], zcm_waypoint.rate[2]);
    waypoint.yaw = zcm_waypoint.pose[3];

    return waypoint;
}

measurements::LidarMeasurement convertLidar(const lidar_t& zcm_lidar)
{
    measurements::LidarMeasurement lidar;
    lidar.distance()(0) = zcm_lidar.distance;
    lidar.setTime(zcm_lidar.utime);
    return lidar;
}
lidar_t convertLidar(const measurements::LidarMeasurement& lidar)
{
    lidar_t zcm_lidar;
    zcm_lidar.distance = lidar.distance()(0);
    zcm_lidar.utime = lidar.timeUSec();
    return zcm_lidar;
}
}
}
