#include <gnc/utils/ZcmConversion.hpp>

using maav::gnc::measurements::ImuMeasurement;
using maav::gnc::measurements::LidarMeasurement;
using maav::gnc::measurements::PlaneFitMeasurement;
using maav::gnc::measurements::GlobalUpdateMeasurement;

namespace maav
{
namespace gnc
{
state_t ConvertState(const State& state)
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

    zcm_state.magnetic_field[0] = state.magneticFieldVector().x();
    zcm_state.magnetic_field[1] = state.magneticFieldVector().y();
    zcm_state.magnetic_field[2] = state.magneticFieldVector().z();

    zcm_state.gravity[0] = state.gravity().x();
    zcm_state.gravity[1] = state.gravity().y();
    zcm_state.gravity[2] = state.gravity().z();

    zcm_state.gyro_biases[0] = state.gyroBias().x();
    zcm_state.gyro_biases[1] = state.gyroBias().y();
    zcm_state.gyro_biases[2] = state.gyroBias().z();

    zcm_state.accel_biases[0] = state.accelBias().x();
    zcm_state.accel_biases[1] = state.accelBias().y();
    zcm_state.accel_biases[2] = state.accelBias().z();

    for (size_t i = 0; i < State::DoF; i++)
    {
        for (size_t j = 0; j < State::DoF; j++)
        {
            zcm_state.covariance[i][j] = state.covariance()(i, j);
        }
    }

    return zcm_state;
}

State ConvertState(const state_t& zcm_state)
{
    State state(zcm_state.utime);

    state.attitude().setQuaternion({zcm_state.attitude[0], zcm_state.attitude[1],
        zcm_state.attitude[2], zcm_state.attitude[3]});
    state.velocity() = {zcm_state.velocity[0], zcm_state.velocity[1], zcm_state.velocity[2]};
    state.position() = {zcm_state.position[0], zcm_state.position[1], zcm_state.position[2]};

    state.gyroBias() = {
        zcm_state.gyro_biases[0], zcm_state.gyro_biases[1], zcm_state.gyro_biases[2]};
    state.accelBias() = {
        zcm_state.accel_biases[0], zcm_state.accel_biases[1], zcm_state.accel_biases[2]};

    state.angularVelocity() = {zcm_state.angular_velocity[0], zcm_state.angular_velocity[1],
        zcm_state.angular_velocity[2]};
    state.acceleration() = {
        zcm_state.acceleration[0], zcm_state.acceleration[1], zcm_state.acceleration[2]};
    state.magneticFieldVector() = {
        zcm_state.magnetic_field[0], zcm_state.magnetic_field[1], zcm_state.magnetic_field[2]};
    state.gravity() = {zcm_state.gravity[0], zcm_state.gravity[1], zcm_state.gravity[2]};

    for (size_t i = 0; i < State::DoF; i++)
    {
        for (size_t j = 0; j < State::DoF; j++)
        {
            state.covariance()(i, j) = zcm_state.covariance[i][j];
        }
    }

    return state;
}

Waypoint ConvertWaypoint(const waypoint_t& zcm_waypoint)
{
    Waypoint waypoint;
    waypoint.position =
        Eigen::Vector3d(zcm_waypoint.pose[0], zcm_waypoint.pose[1], zcm_waypoint.pose[2]);
    waypoint.velocity =
        Eigen::Vector3d(zcm_waypoint.rate[0], zcm_waypoint.rate[1], zcm_waypoint.rate[2]);
    waypoint.yaw = zcm_waypoint.pose[3];

    return waypoint;
}

std::shared_ptr<LidarMeasurement> convertLidar(const lidar_t& zcm_lidar)
{
    std::shared_ptr<LidarMeasurement> lidar(new LidarMeasurement());

    lidar->distance()(0) = zcm_lidar.distance;
    lidar->setTime(zcm_lidar.utime);

    return lidar;
}

// lidar_t convertLidar(const measurements::LidarMeasurement& lidar)
// {
//     lidar_t zcm_lidar;
//     zcm_lidar.distance = lidar.distance()(0);
//     zcm_lidar.utime = lidar.timeUSec();
//     return zcm_lidar;
// }

std::shared_ptr<ImuMeasurement> convertImu(const imu_t& zcm_imu)
{
    std::shared_ptr<ImuMeasurement> imu(new ImuMeasurement);

    imu->time_usec = zcm_imu.utime;

    imu->acceleration = {zcm_imu.acceleration[0], zcm_imu.acceleration[1], zcm_imu.acceleration[2]};
    imu->angular_rates = {
        zcm_imu.angular_rates[0], zcm_imu.angular_rates[1], zcm_imu.angular_rates[2]};
    imu->magnetometer = {zcm_imu.magnetometer[0], zcm_imu.magnetometer[1], zcm_imu.magnetometer[2]};

    return imu;
}

std::shared_ptr<PlaneFitMeasurement> convertPlaneFit(const plane_fit_t& zcm_plane_fit)
{
    std::shared_ptr<PlaneFitMeasurement> plane_fit(new PlaneFitMeasurement());

    plane_fit->time_usec = zcm_plane_fit.utime;

    plane_fit->height = zcm_plane_fit.z;
    plane_fit->vertical_speed = zcm_plane_fit.z_dot;

    plane_fit->roll = zcm_plane_fit.roll;
    plane_fit->pitch = zcm_plane_fit.pitch;

    return plane_fit;
}

std::shared_ptr<GlobalUpdateMeasurement> convertGlobalUpdate(const global_update_t& zcm_global)
{
    std::shared_ptr<GlobalUpdateMeasurement> global_update(new GlobalUpdateMeasurement());

    global_update->setTime(zcm_global.utime);

    global_update->position() = {
        zcm_global.position[0], zcm_global.position[1], zcm_global.position[2]};
    global_update->attitude().setQuaternion({zcm_global.attitude[0], zcm_global.attitude[1],
        zcm_global.attitude[2], zcm_global.attitude[3]});

    return global_update;
}
}
}
