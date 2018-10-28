#pragma once

#include <array>
#include <atomic>
#include <list>
#include <utility>

#include <common/mavlink/offboard_control.hpp>
#include <common/messages/ctrl_params_t.hpp>
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
	TAKEOFF,
	LAND,
	HOLD_ALT,
	PLANNER,
	TEST_ASCEND_AT_RATE,
	TEST_HOLD_ALTITUDE,
	TEST_DIRECT_THRUST
};

class Controller
{
   public:
	struct Parameters
	{
		double mass;		  //< vehicle mass
		double setpoint_tol;  //< convergence tolerance for achieving setpoints
		double min_F_norm;	//< minimum allowed force L2-norm
		std::array<std::pair<double, double>, 3>
			rate_limits;  //< pair of (upper, lower) limits on [x, y, z, yaw] rates
		std::array<std::pair<double, double>, 3>
			accel_limits;  //< pair of (upper, lower) limits on [x, y, z] accel
		std::array<std::pair<double, double>, 3>
			angle_limits;  //< pair of (upper, lower) limits on [roll, pitch]
		std::pair<double, double> thrust_limits{1, 0};
	};

	Controller();
	~Controller();
	void set_target(const Waypoint& waypoint);
	mavlink::InnerLoopSetpoint run(const State& state);
	void set_control_params(const ctrl_params_t&);
	void set_control_params(const ctrl_params_t&, const Parameters&);
	ControlState get_control_state() const;
	bool set_control_state(const ControlState new_control_state);
	double calculate_thrust(const double height_setpoint);

   private:
	mavlink::InnerLoopSetpoint move_to_waypoint(const Waypoint& waypoint);
	mavlink::InnerLoopSetpoint hold_altitude(const double altitude);
	mavlink::InnerLoopSetpoint takeoff(const double takeoff_altitude);
	mavlink::InnerLoopSetpoint land();
	mavlink::InnerLoopSetpoint yaw_to_heading(const double heading);

	ControlState current_control_state;

	State current_state;
	State previous_state;
	double dt;  // Difference in time between current and previous state

	double takeoff_altitude = -5;		// meters
	double hold_altitude_setpoint = 0;  // Default to zero so nothing crazy happens on accident
	std::chrono::time_point<std::chrono::system_clock> takeoff_delay;
	std::chrono::time_point<std::chrono::system_clock> landing_timer;
	bool landing_sequence_init;

	control::Pid z_position_pid;
	control::Pid z_rate_pid;
	Parameters veh_params;
};

// global altitude for testing
// This needs to become "hold_altitude" a member variable in controller class
std::atomic<double> SETPOINT = 0;

}  // namespace gnc
}  // namespace maav