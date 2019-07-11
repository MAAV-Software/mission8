#pragma once

#include <array>
#include <atomic>
#include <list>
#include <memory>
#include <string>
#include <utility>

#include <yaml-cpp/yaml.h>
#include <zcm/zcm-cpp.hpp>

#include <common/mavlink/AutopilotInterface.hpp>
#include <common/messages/ctrl_params_t.hpp>
#include <common/messages/path_t.hpp>
#include <common/messages/pid_error_t.hpp>
#include <gnc/State.hpp>
#include <gnc/control/ContinuousPath.hpp>
#include <gnc/control/Pid.hpp>
#include <gnc/measurements/Waypoint.hpp>

namespace maav
{
namespace gnc
{
namespace control
{
/**
 * The controller class computes inner loop setpoints given a state and a target. It is only meant
 * to be used for this purpose. Other higher level operations should go in the StateMachine
 */
class Controller
{
public:
    struct Parameters
    {
        // Parameter indicies
        constexpr static size_t P = 0;
        constexpr static size_t I = 1;
        constexpr static size_t D = 2;

        constexpr static size_t X = 0;
        constexpr static size_t Y = 1;
        constexpr static size_t Z = 2;

        constexpr static size_t DX = 0;
        constexpr static size_t DY = 1;
        constexpr static size_t DZ = 2;
        constexpr static size_t DYAW = 3;

        std::array<std::array<double, 3>, 4> pos_gains;         // x, y, z, yaw
        std::array<std::array<double, 3>, 4> rate_gains;        // x, y, z, yaw
        std::array<std::pair<double, double>, 4> rate_limits;   // x, y, z, yaw
        std::array<std::pair<double, double>, 3> accel_limits;  // x, y, z
        std::array<std::pair<double, double>, 2> angle_limits;  // roll, pitch
        std::pair<double, double> thrust_limits{1, 0};
        double takeoff_alt;
        double setpoint_tol;  //< convergence tolerance for achieving setpoints
        std::string zcm_url;
        double ff_thrust;
    };

    Controller(const YAML::Node& control_config, float starting_yaw);
    ~Controller();

    /*
     *      Main control function
     *      Takes current position and current
     *      target and provides inner loop setpoint
     *      to move to that target
     */
    maav::mavlink::InnerLoopSetpoint operator()(
        const State& current_state, const ContinuousPath::Waypoint& target);

    /**
     * Control params can be updated on the fly
     */
    void set_control_params(const Parameters& params);
    void set_control_params(const ctrl_params_t& ctrl_params);

private:
    maav::mavlink::InnerLoopSetpoint move_to_current_target();

    const YAML::Node control_config_;

    float origin_yaw;
    float yaw_north;
    float internal_yaw_;
    bool time_initialized_;
    double previous_time_;

    control::Pid x_position_pid;
    control::Pid x_velocity_pid;
    control::Pid y_position_pid;
    control::Pid y_velocity_pid;
    control::Pid z_position_pid;
    control::Pid z_velocity_pid;
    control::Pid yaw_velocity_pid;
    control::Pid emz_z_velocity_pid;
    Parameters veh_params;

    zcm::ZCM zcm;
    pid_error_t pid_error_msg;
};

}  // namespace control
}  // namespace gnc
}  // namespace maav