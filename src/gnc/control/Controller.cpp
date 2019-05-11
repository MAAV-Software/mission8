#include <chrono>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <common/messages/MsgChannels.hpp>
#include <gnc/Constants.hpp>
#include <gnc/control/Controller.hpp>
#include <gnc/utils/LoadParameters.hpp>
#include <gnc/utils/ZcmConversion.hpp>

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

/*
 *      Helper to get heading out of state
 */
static double get_heading(const State& state)
{
    double q0 = state.attitude().unit_quaternion().w();
    double q1 = state.attitude().unit_quaternion().x();
    double q2 = state.attitude().unit_quaternion().y();
    double q3 = state.attitude().unit_quaternion().z();
    return atan2((q1 * q2) + (q0 * q3), 0.5 - (q2 * q2) - (q3 * q3));
}

Controller::Controller(const YAML::Node& control_config)
    : control_config_(control_config),
      current_state(0),
      current_target({Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0), 0.}),
      zcm("ipc"),
      landing_speed_(control_config["landing-speed"].as<double>())
{
    set_control_params(LoadParametersFromYAML(control_config));
    zcm.publish(maav::PID_ERROR_CHANNEL, &pid_error_msg);
    current_state.zero(0);
    std::cout << std::fixed << std::setprecision(3) << std::showpos;
}

Controller::~Controller() {}

/*
 *      Sets current path and sets up class to move
 *      through path
 */
void Controller::set_path(const path_t& _path)
{
    converged_on_waypoint = true;  // so that path will choose first waypoint in flight()
    path = _path;
    path_counter = 0;
}

void Controller::set_yaw_north() { yaw_north = get_heading(current_state); }

void Controller::set_current_target(const Waypoint& new_target) { current_target = new_target; }

/*
 *      Control parameters set from message and vehicle
 *      parameters set from struct.  Vehicle parametes should
 *      not change once the vehicle is in flight but control
 *      params can be tuned.
 */
void Controller::set_control_params(const Controller::Parameters& params)
{
    veh_params = params;
    x_position_pid.setGains(params.pos_gains[0][0], params.pos_gains[0][1], params.pos_gains[0][2]);
    y_position_pid.setGains(params.pos_gains[1][0], params.pos_gains[1][1], params.pos_gains[1][2]);
    z_position_pid.setGains(params.pos_gains[2][0], params.pos_gains[2][1], params.pos_gains[2][2]);
    x_velocity_pid.setGains(
        params.rate_gains[0][0], params.rate_gains[0][1], params.rate_gains[0][2]);
    y_velocity_pid.setGains(
        params.rate_gains[1][0], params.rate_gains[1][1], params.rate_gains[1][2]);
    z_velocity_pid.setGains(
        params.rate_gains[2][0], params.rate_gains[2][1], params.rate_gains[2][2]);
    emz_z_velocity_pid.setGains(
        params.rate_gains[3][0], params.rate_gains[3][1], params.rate_gains[3][2]);
}

void Controller::set_control_params(const ctrl_params_t& ctrl_params)
{
    x_position_pid.setGains(ctrl_params.value[0].p, ctrl_params.value[0].i, ctrl_params.value[0].d);
    y_position_pid.setGains(ctrl_params.value[1].p, ctrl_params.value[1].i, ctrl_params.value[1].d);
    z_position_pid.setGains(ctrl_params.value[2].p, ctrl_params.value[2].i, ctrl_params.value[2].d);
    x_velocity_pid.setGains(ctrl_params.rate[0].p, ctrl_params.rate[0].i, ctrl_params.rate[0].d);
    y_velocity_pid.setGains(ctrl_params.rate[1].p, ctrl_params.rate[1].i, ctrl_params.rate[1].d);
    z_velocity_pid.setGains(ctrl_params.rate[2].p, ctrl_params.rate[2].i, ctrl_params.rate[2].d);
    emz_z_velocity_pid.setGains(
        ctrl_params.rate[3].p, ctrl_params.rate[3].i, ctrl_params.rate[3].d);
}

/*
 *      State
 */
void Controller::add_state(const State& state) { current_state = state; }

/*
 *      Ems state is z position based on barometer
 *      readings from pixhawk, trust no other state variables
 */
void Controller::add_ems_state(const State& state)
{
    if (ems_state.timeUSec() == state.timeUSec()) return;
    ems_dt = state.timeSec() - ems_state.timeSec();
    ems_state = state;
}

/*
 *      Runs flight mode and controller mode
 */
InnerLoopSetpoint Controller::flight()
{
    if (!path.waypoints.empty() && converged_on_waypoint && path_counter < path.NUM_WAYPOINTS)
    {
        set_current_target(ConvertWaypoint(path.waypoints[path_counter++]));
        converged_on_waypoint = false;

        total_distance_to_target = (current_target.position - current_state.position()).norm();
        origin_yaw = get_heading(current_state);
    }

    return move_to_current_target();
}

/*
 *      Run method that controlls vehicle with xbox controller input
 *      to use run "maav-controller -x"
 */
InnerLoopSetpoint Controller::run_xbox(const XboxController& xbox_controller)
{
    InnerLoopSetpoint setpoint;
    controller_run_loop_1 = controller_run_loop_2;
    controller_run_loop_2 = high_resolution_clock::now();
    auto dt = duration_cast<duration<double>>(controller_run_loop_2 - controller_run_loop_1);

    setpoint.thrust = (static_cast<float>(xbox_controller.left_joystick_y) / 32767 + 1) / 2;
    float y_acc = (static_cast<float>(xbox_controller.right_joystick_x) / 32767) * M_PI * 20 / 180;
    float x_acc = (-static_cast<float>(xbox_controller.right_joystick_y) / 32767) * M_PI * 20 / 180;
    if (xbox_controller.left_trigger > 50)
    {
        xbox_yaw = xbox_yaw - dt.count() * veh_params.rate_limits[3].first;
        if (xbox_yaw < 0) xbox_yaw = xbox_yaw + 2 * M_PI;
    }
    if (xbox_controller.right_trigger > 50)
    {
        xbox_yaw = xbox_yaw + dt.count() * veh_params.rate_limits[3].first;
        if (xbox_yaw > 2 * M_PI) xbox_yaw = xbox_yaw - 2 * M_PI;
    }

    std::cout << xbox_yaw << '\n';

    // Todo: add pid controls, very hard to use as is (but it werks tho?!)

    Eigen::Quaterniond q_roll = {
        cos(y_acc / 2), cos(xbox_yaw) * sin(y_acc / 2), sin(xbox_yaw) * sin(y_acc / 2), 0};
    Eigen::Quaterniond q_pitch = {
        cos(x_acc / 2), -sin(xbox_yaw) * sin(x_acc / 2), cos(xbox_yaw) * sin(x_acc / 2), 0};
    Eigen::Quaterniond q_yaw = {cos(xbox_yaw / 2), 0, 0, sin(xbox_yaw / 2)};

    setpoint.q = static_cast<Eigen::Quaternion<float>>(q_roll * q_pitch * q_yaw);
    return setpoint;
}

/*
 *      Main control function
 *      Takes current position and current
 *      target and provides inner loop setpoint
 *      to move to that target
 */
mavlink::InnerLoopSetpoint Controller::move_to_current_target()
{
    InnerLoopSetpoint new_setpoint;

    const Eigen::Vector3d& target_position = current_target.position;

    const Eigen::Vector3d& position = current_state.position();
    const Eigen::Vector3d& velocity = current_state.velocity();

    Eigen::Vector3d position_error = target_position - position;
    Eigen::Vector3d target_acceleration = {x_position_pid.run(position_error.x(), -velocity.x()),
        y_position_pid.run(position_error.y(), -velocity.y()),
        z_position_pid.run(position_error.z(), -velocity.z())};
    // TODO: add rate limits

    float roll = static_cast<float>(target_acceleration.y());
    float pitch = -static_cast<float>(target_acceleration.x());
    float yaw;
    if (position_error.norm() < 1 ||
        (total_distance_to_target != 0 && position_error.norm() / total_distance_to_target < 0.85))
    {
        yaw = static_cast<float>(current_target.yaw * constants::DEG_TO_RAD + yaw_north);
    }
    else
    {
        yaw = origin_yaw;
    }

    roll = bounded(roll, veh_params.angle_limits[0]);
    pitch = bounded(pitch, veh_params.angle_limits[1]);
    new_setpoint.thrust =
        bounded(-target_acceleration.z() + veh_params.ff_thrust, veh_params.thrust_limits);

    Eigen::Quaternionf q_roll(Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()));
    Eigen::Quaternionf q_pitch(Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()));
    Eigen::Quaternionf q_yaw(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));

    new_setpoint.q = q_roll * q_pitch * q_yaw;

    if (position_error.norm() < veh_params.setpoint_tol)
    {
        converged_on_waypoint = true;
    }

    pid_error_msg.pos_error[0] = position_error.x();
    pid_error_msg.pos_error[1] = position_error.y();
    pid_error_msg.pos_error[2] = position_error.z();
    pid_error_msg.vel_error[0] = 0;
    pid_error_msg.vel_error[1] = 0;
    pid_error_msg.vel_error[2] = 0;
    pid_error_msg.roll = roll;
    pid_error_msg.pitch = pitch;
    pid_error_msg.thrust = new_setpoint.thrust;
    pid_error_msg.utime = current_state.timeUSec();
    zcm.publish(maav::PID_ERROR_CHANNEL, &pid_error_msg);

    return new_setpoint;
}

InnerLoopSetpoint Controller::takeoff(const double takeoff_alt)
{
    takeoff_alt_setpoint = -takeoff_alt;  // saved for reference by other methods
    current_target.position.z() = takeoff_alt_setpoint;
    return move_to_current_target();
}

InnerLoopSetpoint Controller::land()
{
    using namespace std::chrono;

    if (lt_dt_ == seconds(0))
    {
        lt_last_time_ = std::chrono::system_clock::now();
    }

    lt_dt_ = duration_cast<duration<double>>(system_clock::now() - lt_last_time_);
    lt_last_time_ = std::chrono::system_clock::now();

    double dh = landing_speed_ * lt_dt_.count();
    if (dh > 0.2)
    {
        dh = 0;  // stops bad first try
    }

    current_target.position.z() = current_target.position.z() + dh;  // [+] <-- increasing z is down

    return move_to_current_target();
}

InnerLoopSetpoint Controller::ems_land()
{
    double vel_error = 1 - ems_state.velocity().z();
    double thrust = -emz_z_velocity_pid.runDiscrete(vel_error, ems_dt);

    thrust = bounded(thrust, veh_params.thrust_limits);
    cout << thrust << '\n';

    // return {{1, 0, 0, 0}, static_cast<float>(thrust), 0, 0, 0};
    return InnerLoopSetpoint::zero();  // not implemented
}

bool Controller::at_takeoff_alt()
{
    return fabs(takeoff_alt_setpoint - current_state.position()(2)) < convergence_tolerance;
}

}  // namespace control
}  // namespace gnc
}  // namespace maav
