#include <gnc/control/LandDetector.hpp>

using namespace std::chrono_literals;

namespace maav
{
namespace gnc
{
namespace control
{
LandDetector::LandDetector(YAML::Node config)
    : current_land_state_(LandedState::Landed),
      thrust_(0),
      z_vel_max_(config["z_vel_max"].as<float>()),
      xy_vel_max_(config["xy_vel_max"].as<float>()),
      rot_max_(config["rot_max"].as<float>()),
      throttle_min_(config["throttle_min"].as<float>()),
      throttle_hover_(config["throttle_hover"].as<float>()),
      thrust_thresh_1_(throttle_min_ + (throttle_hover_ - throttle_min_) * 0.3),
      thrust_thresh_2_(throttle_min_ + (throttle_hover_ - throttle_min_) * 0.1)
{
    resetClock();
}

void LandDetector::detectLanding()
{
    auto now = std::chrono::system_clock::now();
    auto dt = now - land_state_start_;

    constexpr auto GC_TIME = 0.35s;
    constexpr auto ML_TIME = 0.25s;
    constexpr auto L_TIME = 0.3s;

    switch (current_land_state_)
    {
        case LandedState::Air:
            if (detectGroundContact())
            {
                if (dt > GC_TIME)
                {
                    setLandedState(LandedState::GroundContact);
                }
            }
            else
            {
                resetClock();
            }
            break;
        case LandedState::GroundContact:
            if (detectMaybeLanded())
            {
                if (dt > ML_TIME)
                {
                    setLandedState(LandedState::MaybeLanded);
                }
            }
            else
            {
                setLandedState(LandedState::Air);
            }
            break;
        case LandedState::MaybeLanded:
            if (detectLanded())
            {
                if (dt > L_TIME)
                {
                    setLandedState(LandedState::Landed);
                }
            }
            else
            {
                setLandedState(LandedState::Air);
            }
            break;
        case LandedState::Landed:
            break;
    }  // namespace control
}  // namespace control

bool LandDetector::detectGroundContact()
{
    bool vertical_movement = std::abs(state_.velocity().z()) < z_vel_max_;
    bool horizontal_movement = state_.velocity().bottomRows<2>().norm() < z_vel_max_;
    bool throttle = thrust_ < thrust_thresh_1_;

    return vertical_movement && horizontal_movement && throttle;
}

bool LandDetector::detectMaybeLanded()
{
    bool rotation = state_.angularVelocity().norm() < rot_max_;
    bool throttle = thrust_ < thrust_thresh_2_;

    return rotation && throttle && detectGroundContact();
}

bool LandDetector::detectLanded() { return detectMaybeLanded(); }

LandDetector::LandedState LandDetector::getLandedState() { return current_land_state_; }

void LandDetector::setState(const State& state) { state_ = state; }

void LandDetector::setThrust(float thrust) {}

void LandDetector::resetClock() { land_state_start_ = std::chrono::system_clock::now(); }

void LandDetector::setLandedState(LandedState state)
{
    current_land_state_ = state;
    resetClock();
    switch (state)
    {
        case LandedState::Air:
            break;
        case LandedState::GroundContact:
            std::cout << "GROUND CONTACT DETECTED" << std::endl;
            break;
        case LandedState::MaybeLanded:
            std::cout << "MAYBE LANDED" << std::endl;
            break;
        case LandedState::Landed:
            std::cout << "LANDED" << std::endl;
            break;
    }
}

void LandDetector::setAir() { setLandedState(LandedState::Air); }

}  // namespace control
}  // namespace gnc
}  // namespace maav