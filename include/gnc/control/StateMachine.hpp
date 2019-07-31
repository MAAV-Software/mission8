#pragma once

#include <atomic>
#include <chrono>
#include <list>
#include <queue>
#include <string>

#include <yaml-cpp/yaml.h>
#include <zcm/zcm-cpp.hpp>

#include <common/mavlink/AutopilotInterface.hpp>
#include <common/messages/MsgChannels.hpp>
#include <common/messages/control_commands_t.hpp>
#include <common/messages/groundtruth_inertial_t.hpp>
#include <common/messages/killswitch_t.hpp>
#include <common/messages/path_t.hpp>
#include <common/messages/state_t.hpp>
#include <common/utils/ZCMHandler.hpp>
#include <gnc/control/Controller.hpp>
#include <gnc/control/LandDetector.hpp>

namespace maav
{
namespace gnc
{
namespace control
{
class StateMachine
{
private:
    enum class ControlState
    {
        STANDBY = 0,
        TAKEOFF,
        LAND,
        FLIGHT,
        ARMING,
        SOFT_ARM,
        DISARMING,
        KILLSWITCH
    };

    enum class ControlCommands
    {
        TAKEOFF = 0,
        LAND,
        ARM,
        DISARM,
        SETGAINS
    };

    enum class LandedState
    {
        Air = 0,
        GroundContact,
        MaybeLanded,
        Landed
    };

public:
    StateMachine(const std::string& control_config_file,
        maav::mavlink::AutopilotInterface* const autopilot_interface);
    void run(const std::atomic<bool>& kill);

private:
    void readZcm();
    void initializeRun(const std::atomic<bool>& kill);
    void setControlState(const ControlState new_control_state);
    float armedThrust(float thrust);
    void detectLanding();
    bool checkCommand(const ControlCommands command);

    maav::mavlink::InnerLoopSetpoint runStandby();
    maav::mavlink::InnerLoopSetpoint runTakeoff();
    maav::mavlink::InnerLoopSetpoint runLand();
    maav::mavlink::InnerLoopSetpoint runFlight();
    maav::mavlink::InnerLoopSetpoint runArming();
    maav::mavlink::InnerLoopSetpoint runDisarming();
    maav::mavlink::InnerLoopSetpoint runKillswitch();
    maav::mavlink::InnerLoopSetpoint runSoftArm();
    bool arm();

    const std::string control_config_file_;
    const YAML::Node control_config_;

    maav::mavlink::AutopilotInterface* const autopilot_interface_;
    Controller controller_;
    LandDetector land_detector_;
    ControlState current_control_state_;

    std::queue<ControlCommands> commands_;

    zcm::ZCM zcm_;
    ZCMHandler<path_t> path_handler_;
    ZCMHandler<state_t> state_handler_;
    ZCMHandler<groundtruth_inertial_t> sim_state_handler_;
    ZCMHandler<control_commands_t> command_handler_;
    ZCMHandler<killswitch_t> killswitch_handler_;

    bool sim_state_;
    bool soft_arm_ = false;
};
}  // namespace control
}  // namespace gnc
}  // namespace maav