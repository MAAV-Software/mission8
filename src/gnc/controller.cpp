#include "gnc/controller.hpp"
#include <iostream>

using std::cout;
using maav::mavlink::InnerLoopSetpoint;

namespace maav
{
namespace gnc
{
Controller::Controller() : current_state(0), previous_state(0), thrust_pid(0.1, 0, 0.1)
{
	current_state.zero(0);
	previous_state.zero(0);
}

Controller::~Controller() {}
void Controller::set_target(const Waypoint& waypoint) {}
void Controller::set_control_params(const ctrl_params_t& _params) { control_params = _params; }
void Controller::add_state(const State& state) 
{
	previous_state = current_state;
	current_state = state;
	dt = current_state.time_sec() - previous_state.time_sec();
}

InnerLoopSetpoint Controller::run() { return hold_altitude(1); }
InnerLoopSetpoint Controller::hold_altitude(const double altitude)
{
	assert(dt != 0);
	double height_setpoint = altitude;
	double height_error = height_setpoint - current_state.position()(2);
	double height_error_dot = (height_error - height_error_prev) / dt;
	double thrust = thrust_pid.run(height_error, height_error_dot) + thrust_0;

	// Set values in proper range
	if (thrust > 1)
		thrust = 1;
	else if (thrust < 0)
		thrust = 0;

	InnerLoopSetpoint new_setpoint;					   // default initialize everything to zero
	new_setpoint.thrust = static_cast<float>(thrust);  // checking above assures good downcast

	height_error_prev = height_error;
	return new_setpoint;
}
}  // namespace gnc
}  // namespace maav
