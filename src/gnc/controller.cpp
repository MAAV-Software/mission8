#include "gnc/controller.hpp"
#include <iostream>

using std::cout;
using maav::mavlink::InnerLoopSetpoint;
using std::pair;
using std::chrono::system_clock;
using std::chrono::seconds;

namespace maav
{
namespace gnc
{
static double bounded(double value, const pair<double, double>& limits)
{
	assert(limits.first > limits.second);

	if (value > limits.first)
		return limits.first;
	else if (value < limits.second)
		return limits.second;
	else
		return value;
}

Controller::Controller() : current_state(0), previous_state(0)
{
	set_control_state(ControlState::STANDBY);
	current_state.zero(0);
	previous_state.zero(0);
}

Controller::~Controller() {}
void Controller::set_target(const Waypoint& waypoint) {}
void Controller::set_control_params(const ctrl_params_t& ctrl_params)
{
	z_position_pid.setGains(ctrl_params.value[2].p, ctrl_params.value[2].i, ctrl_params.value[2].d);
	z_rate_pid.setGains(ctrl_params.rate[2].p, ctrl_params.rate[2].i, ctrl_params.rate[2].d);
	pitch_pid.setGains(ctrl_params.value[3].p, ctrl_params.value[3].i, ctrl_params.value[3].d);
}
void Controller::set_control_params(const ctrl_params_t& ctrl_params, const Parameters& _veh_params)
{
	set_control_params(ctrl_params);
	veh_params = _veh_params;
}

InnerLoopSetpoint Controller::run(const State& state)
{
	previous_state = current_state;
	current_state = state;
	dt = current_state.time_sec() - previous_state.time_sec();
	InnerLoopSetpoint s;  // For tests

	switch (current_control_state)
	{
		case ControlState::TAKEOFF:
			if (std::chrono::system_clock::now() < takeoff_delay)
			{
				return hold_altitude(0);
			}
			if (fabs(current_state.position().z() - takeoff_altitude) < 0.25)
			{
				hold_altitude_setpoint = takeoff_altitude;
				set_control_state(ControlState::HOLD_ALT);
				return hold_altitude(takeoff_altitude);
			}
			return takeoff(takeoff_altitude);
		case ControlState::HOLD_ALT:
			return hold_altitude(hold_altitude_setpoint);
		case ControlState::LAND:
			if (fabs(current_state.velocity().z()) <= 0.1 &&
				fabs(current_state.position().z()) <= 0.1)
			{
				set_control_state(ControlState::STANDBY);
				cout << "Landing detected\n";
				return InnerLoopSetpoint();
			}
			return land();
		case ControlState::STANDBY:
			return InnerLoopSetpoint();
		case ControlState::TEST_HOLD_ALTITUDE:
			return hold_altitude(SETPOINT);
		case ControlState::TEST_DIRECT_THRUST:
			s.thrust = SETPOINT;
			return s;
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

	switch (current_control_state)
	{
		case ControlState::TAKEOFF:
			cout << "Control mode switched to TAKEOFF\n";
			takeoff_delay = system_clock::now() + seconds(2);
			break;
		case ControlState::HOLD_ALT:
			cout << "Control mode switched to HOLD_ALT\n";
			break;
		case ControlState::LAND:
			cout << "Control mode switched to LAND\n";
			break;
		case ControlState::STANDBY:
			cout << "Control mode switched to STANDBY\n";
			break;
		case ControlState::TEST_HOLD_ALTITUDE:
			cout << "Control mode switched to TEST_HOLD_ALTITUDE\n";
			break;
		case ControlState::TEST_DIRECT_THRUST:
			cout << "Control mode switched to TEST_DIRECT_THRUST\n";
			break;
		default:
			assert(false);
	}

	return true;
}

mavlink::InnerLoopSetpoint move_to_waypoint(const Waypoint& waypoint)
{
	assert(false);
	return InnerLoopSetpoint();
}

InnerLoopSetpoint Controller::hold_altitude(const double height_setpoint)
{
	static double xy_disp_last_norm;
	InnerLoopSetpoint new_setpoint;

	Eigen::Vector2d xy_disp = {current_state.position().x(), current_state.position().y()};
	double xy_disp_dot = (xy_disp.norm() - xy_disp_last_norm) / dt;
	xy_disp_last_norm = xy_disp.norm();
	double angle = pitch_pid.run(xy_disp.norm(), xy_disp_dot);
	Eigen::Vector2d rot = {-xy_disp.normalized().y(), xy_disp.normalized().x()};

	bounded(angle, veh_params.angle_limits[0]);

	new_setpoint.thrust = calculate_thrust(height_setpoint);
	new_setpoint.q.w() = cos(angle / 2);
	new_setpoint.q.x() = rot.x() * sin(angle / 2);
	new_setpoint.q.y() = rot.y() * sin(angle / 2);
	new_setpoint.q.z() = 0;
	new_setpoint.yaw_rate = 0;
	return new_setpoint;
}

double Controller::calculate_thrust(const double height_setpoint)
{
	double height_error = height_setpoint - current_state.position().z();
	double vel = z_position_pid.run(height_error, current_state.velocity().z());

	vel = bounded(vel, veh_params.rate_limits[2]);

	double vel_error = vel - current_state.velocity().z();
	double thrust =
		-z_rate_pid.run(vel_error, current_state.acceleration().z());  // TODO: derivative control?

	thrust = bounded(thrust, veh_params.thrust_limits);

	return static_cast<float>(thrust);
}

mavlink::InnerLoopSetpoint Controller::takeoff(const double alt) { return hold_altitude(alt); }
mavlink::InnerLoopSetpoint Controller::land()
{
	if (current_state.position().z() < -0.55)
	{
		if (landing_sequence_init) landing_sequence_init = false;
		return hold_altitude(-0.5);
	}
	else if (landing_sequence_init && system_clock::now() < landing_timer + seconds(3))
	{
		return hold_altitude(-0.1);
	}
	else if (landing_sequence_init && system_clock::now() > landing_timer + seconds(3))
	{
		return hold_altitude(0);
	}
	else if (!landing_sequence_init)
	{
		landing_sequence_init = true;
		landing_timer = system_clock::now();
		return hold_altitude(-0.5);
	}

	assert(false);
	return hold_altitude(-0.1);
}

}  // namespace gnc
}  // namespace maav
