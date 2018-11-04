#pragma once

#include <array>
#include <atomic>
#include <list>
#include <utility>

#include <common/mavlink/offboard_control.hpp>
#include <common/messages/ctrl_params_t.hpp>
#include <common/messages/path_t.hpp>
#include <gnc/control/pid.hpp>
#include <gnc/measurements/Waypoint.hpp>
#include <gnc/state.hpp>

namespace maav
{
namespace gnc
{
enum class ControlState
{
    STANDBY = 0,
    HOLD_ALT,
    TEST_WAYPOINT,
    TEST_PATH  // migrate to PLANNER
};

class Controller
{
   public:
    struct Parameters
    {
        double mass;          //< vehicle mass
        double setpoint_tol;  //< convergence tolerance for achieving setpoints
        double min_F_norm;    //< minimum allowed force L2-norm						///<---remove?
        std::array<std::pair<double, double>, 4> rate_limits;
        std::array<std::pair<double, double>, 3> accel_limits;
        std::array<std::pair<double, double>, 2> angle_limits;  // radians, problaby
        std::pair<double, double> thrust_limits{1, 0};
    };

    Controller();
    ~Controller();
    void set_path(const path_t& _path);
    mavlink::InnerLoopSetpoint run(const State& state);
    void set_control_params(const ctrl_params_t&);
    void set_control_params(const ctrl_params_t&, const Parameters&);
    ControlState get_control_state() const;
    bool set_control_state(const ControlState new_control_state);

   private:
    mavlink::InnerLoopSetpoint move_to_current_target();
    double calculate_thrust(const double height_setpoint);

    mavlink::InnerLoopSetpoint hold_altitude(const Waypoint& hold_alt_wpt);

    ControlState current_control_state;

    State current_state;
    State previous_state;
    double dt;  // Difference in time between current and previous state

    path_t path;
    int16_t path_counter;
    Waypoint current_target;
    const double convergence_tolerance = 0.5;  // set with veh params
    bool converged_on_waypoint;

    // Combine pids into array?
    control::Pid z_position_pid;
    control::Pid z_rate_pid;
    control::Pid pitch_pid;
    control::Pid yaw_pid;
    Parameters veh_params;

    Waypoint hold_altitude_setpoint;
    std::chrono::time_point<std::chrono::system_clock> pause_timer;
    bool set_pause;
};

// global altitude for testing
// This needs to become "hold_altitude" a member variable in controller class
Waypoint SETPOINT = Waypoint{Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0), 0};
std::atomic<bool> CONTROL_STATE_SETPOINT{true};

}  // namespace gnc
}  // namespace maav