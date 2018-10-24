#include "gnc/controller.hpp"
#include <iostream>

using std::cout;
using maav::mavlink::InnerLoopSetpoint;

namespace maav
{
namespace gnc
{
Controller::Controller()
	: current_control_state(ControlState::STANDBY), current_state(0), previous_state(0)
{
	current_state.zero(0);
	previous_state.zero(0);
}

Controller::~Controller() {}
void Controller::set_target(const Waypoint& waypoint) {}
void Controller::set_control_params(const ctrl_params_t& ctrl_params)
{
	z_position_pid.reset();
	z_position_pid.setGains(ctrl_params.value[2].p, ctrl_params.value[2].i, ctrl_params.value[2].d);

	z_velocity_pid.reset();
	z_velocity_pid.setGains(ctrl_params.rate[2].p, ctrl_params.rate[2].i, ctrl_params.rate[2].d);
}

InnerLoopSetpoint Controller::run(const State& state)
{
	previous_state = current_state;
	current_state = state;
	dt = current_state.time_sec() - previous_state.time_sec();

	switch (current_control_state)
	{
		case ControlState::TAKEOFF:
			if (fabs(current_state.position().z() - takeoff_altitude) < takeoff_error)
			{
				hold_altitude_setpoint = takeoff_altitude;
				current_control_state = ControlState::HOLD_ALT;
				cout << "\rTakeoff altitude +/-" << takeoff_error << " reached\n";
				cout << "Control mode switched to HOLD_ALT\n";
				return hold_altitude(takeoff_altitude);
			}
			return takeoff(takeoff_altitude, takeoff_rate);

		case ControlState::HOLD_ALT:
			return hold_altitude(hold_altitude_setpoint);

		case ControlState::LAND:
			if (fabs(current_state.velocity().z()) <= 0.1 &&
				fabs(current_state.position().z()) <= 0.2)
			{
				current_control_state = ControlState::STANDBY;
				cout << "\rLanding detecteed\n";
				cout << "Control mode switched to STANDBY\n";
				return InnerLoopSetpoint();
			}
			return land();

		case ControlState::STANDBY:
			return InnerLoopSetpoint();

		case ControlState::TEST_ASCEND_AT_RATE:
			return ascend_at_rate(SETPOINT);

		case ControlState::TEST_HOLD_ALTITUDE:
			return hold_altitude(SETPOINT);

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
	double thrust = z_position_pid.run(height_error, height_error_dot) + thrust_0;

	// Set values in proper range
	if (thrust > 1)
		thrust = 1;
	else if (thrust < 0)
		thrust = 0;

	InnerLoopSetpoint new_setpoint;					   // default initialize everything to zero
	new_setpoint.thrust = static_cast<float>(thrust);  // checking above assures good downcast

	return new_setpoint;
}

InnerLoopSetpoint Controller::ascend_at_rate(const double velocity_setpoint)
{
	double velocity_error = velocity_setpoint - current_state.velocity().z();
	double velocity_error_dot = -current_state.acceleration().z();
	double thrust = z_velocity_pid.run(velocity_error, velocity_error_dot) + thrust_0;

	// Set values in proper range
	if (thrust > 1)
		thrust = 1;
	else if (thrust < 0)
		thrust = 0;

	InnerLoopSetpoint new_setpoint;					   // default initialize everything to zero
	new_setpoint.thrust = static_cast<float>(thrust);  // checking above assures good downcast

	return new_setpoint;
}

// Ascend at a constant rate until hold altitude takes over
mavlink::InnerLoopSetpoint Controller::takeoff(const double takeoff_altitude,
											   const double ascent_rate)
{
	InnerLoopSetpoint rate_setpoint = ascend_at_rate(ascent_rate);
	InnerLoopSetpoint height_setpoint = hold_altitude(takeoff_altitude);

	InnerLoopSetpoint new_setpoint;
	if (rate_setpoint.thrust < height_setpoint.thrust)
		new_setpoint.thrust = rate_setpoint.thrust;
	else
		new_setpoint.thrust = height_setpoint.thrust;

	// dummy check
	assert(new_setpoint.thrust <= 1 && new_setpoint.thrust >= 0);

	return new_setpoint;
}

mavlink::InnerLoopSetpoint Controller::land(const double descent_rate)
{
	if (current_state.position().z() > 1)
		return ascend_at_rate(-descent_rate);
	else
		return ascend_at_rate(-0.2);
}

}  // namespace gnc
}  // namespace maav
