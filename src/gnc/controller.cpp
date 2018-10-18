#include "gnc/controller.hpp"

//Please direct complaints about class OffboardControl to Glen Haggin

namespace maav
{
namespace gnc
{
Controller::Controller() {}
Controller::~Controller() {}
void Controller::set_target(const Waypoint& waypoint) {}
void Controller::add_state(const State& state) {}
void Controller::run(){
    //just sets zero attitude right now to keep offboard control
    offboard_control.set_zero_attitude();
}
}  // namespace gnc
}  // namespace maav
