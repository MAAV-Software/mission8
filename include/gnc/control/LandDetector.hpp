#ifndef LAND_DETECTOR_HPP
#define LAND_DETECTOR_HPP

#include <chrono>

#include <yaml-cpp/yaml.h>

#include <gnc/State.hpp>

namespace maav
{
namespace gnc
{
namespace control
{
class LandDetector
{
public:
    enum class LandedState
    {
        Air = 0,
        GroundContact,
        MaybeLanded,
        Landed
    };

public:
    LandDetector(YAML::Node config);

    void detectLanding();
    LandedState getLandedState();
    void setState(const State& state);
    void setThrust(float thrust);
    void setAir();

private:
    void resetClock();
    void setLandedState(LandedState state);

    bool detectGroundContact();
    bool detectMaybeLanded();
    bool detectLanded();

    LandedState current_land_state_;
    State state_;
    float thrust_;

    bool in_state_{false};
    std::chrono::system_clock::time_point land_state_start_;
    // Landing detector
    float z_vel_max_;
    float xy_vel_max_;
    float rot_max_;
    float throttle_min_;
    float throttle_hover_;

    float thrust_thresh_1_;
    float thrust_thresh_2_;
};
}  // namespace control
}  // namespace gnc
}  // namespace maav

#endif