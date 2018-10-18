#include "gnc/controller.hpp"

// Please direct complaints about class OffboardControl to Glen Haggin

namespace maav
{
namespace gnc
{
Controller::Controller() {}
Controller::~Controller() {}
void Controller::set_target(const Waypoint& waypoint) {}
void Controller::add_state(const State& state) {}
void Controller::run() {}
}  // namespace gnc
}  // namespace maav
