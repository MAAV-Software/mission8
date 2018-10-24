#include "gnc/controller.hpp"
#include <iostream>

using std::cout;
using maav::mavlink::InnerLoopSetpoint;

namespace maav
{
namespace gnc
{
Controller::Controller()
	: current_control_state(CONTROL_STATE_STANDBY), current_state(0), previous_state(0)
{
	current_state.zero(0);
	previous_state.zero(0);
}

Controller::~Controller() {}
void Controller::set_target(const Waypoint& waypoint) {}
void Controller::set_control_params(const ctrl_params_t& ctrl_params)
{
	thrust_pid.reset();
	thrust_pid.setGains(ctrl_params.value[2].p, ctrl_params.value[2].i, ctrl_params.value[2].d);
}

InnerLoopSetpoint Controller::run(const State& state)
{
	previous_state = current_state;
	current_state = state;
	dt = current_state.time_sec() - previous_state.time_sec();

	switch (current_control_state)
	{
		case CONTROL_STATE_TAKEOFF:
			if (fabs(current_state.position().z() - takeoff_altitude) < takeoff_error)
			{
				ALTITUDE = takeoff_altitude;
				current_control_state = CONTROL_STATE_HOLD_ALT;
				cout << "\rTakeoff altitude +/-" << takeoff_error << " reached\n";
				cout << "Control mode switched to hold altitude\n";
				return hold_altitude(takeoff_altitude);
			}
			return takeoff(takeoff_altitude);

		case CONTROL_STATE_HOLD_ALT:
			return hold_altitude(ALTITUDE);

		case CONTROL_STATE_LAND:
			assert(false);

		case CONTROL_STATE_STANDBY:
			return InnerLoopSetpoint();

		default:
			assert(false);
	}

	assert(false);
	return hold_altitude(0);
}

ControlState Controller::get_control_state() const { return current_control_state; }
bool Controller::set_control_state(const ControlState new_control_state)
{
	current_control_state = new_control_state;
	return true;
}

InnerLoopSetpoint Controller::hold_altitude(const double height_setpoint)
{
	assert(dt != 0);
	double height_error = height_setpoint - current_state.position().z();
	double height_error_dot = -current_state.velocity().z();
	double thrust = thrust_pid.run(height_error, height_error_dot) + thrust_0;

	// Set values in proper range
	if (thrust > 1)
		thrust = 1;
	else if (thrust < 0)
		thrust = 0;

	InnerLoopSetpoint new_setpoint;					   // default initialize everything to zero
	new_setpoint.thrust = static_cast<float>(thrust);  // checking above assures good downcast

	return new_setpoint;
}

InnerLoopSetpoint Controller::ascend_at_rate(const double rate)
{
	assert(false);
	return InnerLoopSetpoint();
}

// Now: same as hold altitude, Later: steady ascent rate with rate controls
mavlink::InnerLoopSetpoint Controller::takeoff(const double takeoff_altitude,
											   const double ascent_rate)
{
	return hold_altitude(takeoff_altitude);
}

mavlink::InnerLoopSetpoint Controller::land(const double descent_rate)
{
	assert(false);
	return InnerLoopSetpoint();
}

}  // namespace gnc
}  // namespace maav
