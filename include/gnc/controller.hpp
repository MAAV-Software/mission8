#pragma once

#include <atomic>

#include <common/mavlink/offboard_control.hpp>
#include <gnc/control/pid.hpp>
#include <gnc/measurements/waypoint.hpp>
#include <gnc/state.hpp>
#include <common/messages/ctrl_params_t.hpp>

namespace maav
{
namespace gnc
{
enum ControlState
{
	CONTROL_STATE_TAKEOFF = 1,
	CONTROL_STATE_LAND,
	CONTROL_STATE_HOLD_ALT
};

class Controller
{
   public:
	Controller(double p, double i, double d);
	~Controller();

	// TODO: create target struct
	void set_target(const Waypoint& waypoint);

<<<<<<< b916e82788727fb6a9298039298b7021c94584cf
	// TODO: return actuator controls
	void add_state(const State& state);

	void run();
=======
	mavlink::InnerLoopSetpoint run(const State& state);
>>>>>>> PID cmd line arguments for maav-controller

<<<<<<< 364c87a6c879c5bac4c600186a91a7d669f469ce
	void set_control_params(const ctrl_params_t& _params);
	
	mavlink::InnerLoopSetpoint hold_altitude(const double altitude);

   private:
    ctrl_params_t control_params;
=======
	ControlState get_control_state() const;
	bool set_control_state(const ControlState new_control_state);

   private:
	mavlink::InnerLoopSetpoint hold_altitude(const double altitude);

	// Ascend at a particular rate
	mavlink::InnerLoopSetpoint ascend_at_rate(const double rate);

	// Takeoff to takeoff_altitude at ascent_rate m/s
	void takeoff(const double takeoff_altitude, const double ascent_rate = 1);

	// Land at current location at descent_rate m/s
	void land(const double descent_rate = 1);

	ControlState current_control_state;

>>>>>>> Added interface for state machine and ascent/descent rate control
	State current_state;
	State previous_state;
	double dt;  // Difference in time between current and previous state

	float thrust_0 = 0.59;  // Equilibrium Thrust (TODO: get real thrust data)
	control::Pid thrust_pid;
	double height_error_prev;
};

// global altitude for testing
std::atomic<double> ALTITUDE = -0.1;

}  // namespace gnc
}  // namespace maav