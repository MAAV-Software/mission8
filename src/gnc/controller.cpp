#include "gnc/controller.hpp"

//Please direct complaints about class OffboardControl to Glen Haggin

namespace maav
{
namespace gnc
{
Controller::Controller() { offboard_control.set_thrust(0.58); }
Controller::~Controller() {}
void Controller::set_target(const Waypoint& waypoint) {}
void Controller::add_state(const State& state) {}
void Controller::take_off() {
    offboard_control.zero_rates(); 
    offboard_control.set_thrust(0.58);
}
void Controller::yaw_right(){
    offboard_control.set_yaw_rate(-1);
}
void Controller::yaw_left(){
    offboard_control.set_yaw_rate(1);
}
void Controller::roll_left(){
    offboard_control.set_yaw_rate(0);
    offboard_control.set_pitch_rate(0);
    offboard_control.set_roll_rate(-0.5);
    offboard_control.set_thrust(0.6);
}
void Controller::roll_right(){
    offboard_control.set_yaw_rate(0);
    offboard_control.set_pitch_rate(0);
    offboard_control.set_roll_rate(0.5);
    offboard_control.set_thrust(0.6);
}
void Controller::pitch_forward(){
    offboard_control.set_roll_rate(0);
    offboard_control.set_yaw_rate(0);
    offboard_control.set_pitch_rate(-0.5);
    offboard_control.set_thrust(0.6);
}
void Controller::pitch_back(){
    offboard_control.set_roll_rate(0);
    offboard_control.set_yaw_rate(0);
    offboard_control.set_pitch_rate(0.5);
    offboard_control.set_thrust(0.6);
}
void Controller::land(){
    offboard_control.zero_rates();
    offboard_control.set_thrust(0.55);
}
}  // namespace gnc
}  // namespace maav



/* Inteface to offboard control
 * ==============================
 * 
 * void set_zero_attitude()
 * --Sets level attitude, zero angle rates, zero thrust
 * 
 * void set_thrust(const float thrust);
 * --sets normalized thrust value 0<=thrust<=1
 * 
 * void set_yaw_rate(const float yaw_rate);
 * void set_roll_rate(const float roll_rate);
 * void set_pitch_rate(const float pitch_rate);
 * --sets yaw/pitch/roll rate in rad/s
 * 
 * void set_attitutde(const Eigen::Quaternion<float>& q)
 * --set attitude
 * 
 * NOTE: Offboard control will set current setpoints and
 * use them until changed (this interface might suck,
 * please provide Glen Haggin with feedback)
 */