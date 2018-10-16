#include "gnc/controller.hpp"

namespace maav
{
namespace gnc
{
Controller::Controller() {}
Controller::~Controller() {}
void Controller::set_target(const Waypoint& waypoint) {}
void Controller::add_state(const State& state) {}
void Controller::run() { offboard_control.set_zero_attitude();}
}  // namespace gnc
}  // namespace maav