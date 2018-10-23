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

	void set_control_params(const ctrl_params_t& _params);
	
	mavlink::InnerLoopSetpoint hold_altitude(const double altitude);

   private:
    ctrl_params_t control_params;
	State current_state;
	State previous_state;
	double dt;  // Difference in time between current and previous state

	float thrust_0 = 0.59;  // Equilibrium Thrust (TODO: get real thrust data)
	control::Pid thrust_pid;
	double height_error_prev;
};

std::atomic<double> ALTITUDE = -0.1;

}  // namespace gnc
}  // namespace maav