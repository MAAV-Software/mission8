#pragma once

#include <common/mavlink/offboard_control.hpp>
#include <gnc/control/pid.hpp>
#include <gnc/measurements/waypoint.hpp>
#include <gnc/state.hpp>
#include <common/messages/ctrl_params_t.hpp>

namespace maav
{
namespace gnc
{
class Controller
{
   public:
	Controller();
	~Controller();

	// TODO: create target struct
	void set_target(const Waypoint& waypoint);

	// TODO: return actuator controls
	void add_state(const State& state);

	void run();

	void set_control_params(const ctrl_params_t& _params);
	
	mavlink::InnerLoopSetpoint hold_altitude(const double altitude);

   private:
    ctrl_params_t control_params;
	State current_state;
	State previous_state;
	double dt;
	float thrust_0 = 0.59;
	control::Pid thrust_pid;
	double height_error_prev;
};

}  // namespace gnc
}  // namespace maav