#include <chrono>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <common/math/angle_functions.hpp>
#include <common/messages/MsgChannels.hpp>
#include <gnc/Constants.hpp>
#include <gnc/control/Controller.hpp>
#include <gnc/utils/LoadParameters.hpp>
#include <gnc/utils/ZcmConversion.hpp>

#include <sophus/so2.hpp>

using maav::gnc::utils::LoadParametersFromYAML;
using maav::mavlink::InnerLoopSetpoint;
using std::cout;
using std::pair;
using std::string;
using std::vector;
using namespace std::chrono;

namespace maav
{
namespace gnc
{
namespace control
{
/*
 *      Helper to saturate values
 */
static double bounded(double value, const pair<double, double>& limits)
{
    assert(limits.first > limits.second);  // first is upper limit

    if (value > limits.first)
        return limits.first;
    else if (value < limits.second)
        return limits.second;
    else
        return value;
}

Controller::Controller(const YAML::Node& control_config, float starting_yaw)
    : control_config_(control_config),
      internal_yaw_(starting_yaw),
      time_initialized_{false},
      zcm("ipc")
{
    set_control_params(LoadParametersFromYAML(control_config));
    zcm.publish(maav::PID_ERROR_CHANNEL, &pid_error_msg);
    std::cout << std::fixed << std::setprecision(3) << std::showpos;
}

Controller::~Controller() {}

/*
 *      Control parameters set from message and vehicle
 *      parameters set from struct.  Vehicle parametes should
 *      not change once the vehicle is in flight but control
 *      params can be tuned.
 */
void Controller::set_control_params(const Controller::Parameters& params)
{
    veh_params = params;
    x_position_pid.setGains(params.pos_gains[Parameters::X][Parameters::P],
        params.pos_gains[Parameters::X][Parameters::I],
        params.pos_gains[Parameters::X][Parameters::D]);
    y_position_pid.setGains(params.pos_gains[Parameters::Y][Parameters::P],
        params.pos_gains[Parameters::Y][Parameters::I],
        params.pos_gains[Parameters::Y][Parameters::D]);
    z_position_pid.setGains(params.pos_gains[Parameters::Z][Parameters::P],
        params.pos_gains[Parameters::Z][Parameters::I],
        params.pos_gains[Parameters::Z][Parameters::D]);

    x_velocity_pid.setGains(params.rate_gains[Parameters::DX][Parameters::P],
        params.rate_gains[Parameters::DX][Parameters::I],
        params.rate_gains[Parameters::DX][Parameters::D]);
    y_velocity_pid.setGains(params.rate_gains[Parameters::DY][Parameters::P],
        params.rate_gains[Parameters::DY][Parameters::I],
        params.rate_gains[Parameters::DY][Parameters::D]);
    z_velocity_pid.setGains(params.rate_gains[Parameters::DZ][Parameters::P],
        params.rate_gains[Parameters::DZ][Parameters::I],
        params.rate_gains[Parameters::DZ][Parameters::D]);

    yaw_velocity_pid.setGains(params.pos_gains[Parameters::DYAW][Parameters::P],
        params.pos_gains[Parameters::DYAW][Parameters::I],
        params.pos_gains[Parameters::DYAW][Parameters::D]);
}

void Controller::set_control_params(const ctrl_params_t& ctrl_params)
{
    set_control_params(convertControlParams(ctrl_params));
}

/*
 *      Runs flight mode and controller mode
 */
InnerLoopSetpoint Controller::operator()(
    const State& current_state, const ContinuousPath::Waypoint& target)
{
    InnerLoopSetpoint new_setpoint;

    const Eigen::Vector3d& target_position = target.position;

    const Eigen::Vector3d& position = current_state.position();
    const Eigen::Vector3d& velocity = current_state.velocity();

    const Eigen::Vector3d position_error = target_position - position;
    const Eigen::Vector2d lateral_error = position_error.topRows<2>();
    const Eigen::Vector2d lateral_velocity = velocity.topRows<2>();

    const Sophus::SO2d rotation(current_state.attitude().angleZ());
    const Eigen::Vector2d body_frame_lateral_error = rotation.inverse() * lateral_error;
    const Eigen::Vector2d body_frame_lateral_velocity = rotation.inverse() * lateral_velocity;

    const Eigen::Vector3d target_acceleration = {
        x_position_pid.run(body_frame_lateral_error.x(), -body_frame_lateral_velocity.x()),
        y_position_pid.run(body_frame_lateral_error.y(), -body_frame_lateral_velocity.y()),
        z_position_pid.run(position_error.z(), -velocity.z())};

    float roll = static_cast<float>(target_acceleration.y());
    float pitch = -static_cast<float>(target_acceleration.x());

    double yaw_error = eecs467::angle_diff(target.heading, current_state.attitude().angleZ());

    double yaw_velocity = current_state.angularVelocity().z();
    double yaw_rate = yaw_velocity_pid.run(yaw_error, yaw_velocity);
    yaw_rate = bounded(yaw_rate, veh_params.rate_limits[3]);

    // TODO: don't make this constant
    if (!time_initialized_)
    {
        previous_time_ = current_state.timeSec();
        time_initialized_ = true;
    }
    const double dt = current_state.timeSec() - previous_time_;
    previous_time_ = current_state.timeSec();

    internal_yaw_ = eecs467::wrap_to_pi(internal_yaw_ + yaw_rate * dt);

    roll = bounded(roll, veh_params.angle_limits[0]);
    pitch = bounded(pitch, veh_params.angle_limits[1]);
    new_setpoint.thrust =
        bounded(-target_acceleration.z() + veh_params.ff_thrust, veh_params.thrust_limits);

    Eigen::Quaternionf q_roll(Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()));
    Eigen::Quaternionf q_pitch(Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()));
    Eigen::Quaternionf q_yaw(Eigen::AngleAxisf(internal_yaw_, Eigen::Vector3f::UnitZ()));

    new_setpoint.roll = roll;
    new_setpoint.pitch = pitch;
    new_setpoint.yaw = internal_yaw_;
    new_setpoint.q = q_yaw * q_pitch * q_roll;

    pid_error_msg.pos_error = convertVector3d(position_error);
    pid_error_msg.vel_error = convertVector3d({0, 0, 0});
    pid_error_msg.yaw_error = convertVector1d(yaw_error);
    pid_error_msg.roll = convertVector1d(roll);
    pid_error_msg.pitch = convertVector1d(pitch);
    pid_error_msg.thrust = convertVector1d(new_setpoint.thrust);
    pid_error_msg.utime = current_state.timeUSec();
    zcm.publish(maav::PID_ERROR_CHANNEL, &pid_error_msg);

    return new_setpoint;
}

}  // namespace control
}  // namespace gnc
}  // namespace maav
