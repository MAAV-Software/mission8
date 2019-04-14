#pragma once

#include <atomic>
#include <queue>
#include <string>

#include <yaml-cpp/yaml.h>
#include <zcm/zcm-cpp.hpp>

#include <common/mavlink/OffboardControl.hpp>
#include <common/messages/MsgChannels.hpp>
#include <common/messages/control_commands_t.hpp>
#include <common/messages/groundtruth_inertial_t.hpp>
#include <common/messages/killswitch_t.hpp>
#include <common/messages/path_t.hpp>
#include <common/messages/state_t.hpp>
#include <common/utils/ZCMHandler.hpp>
#include <gnc/control/Controller.hpp>

namespace maav
{
namespace gnc
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

public:
    StateMachine(const std::string& control_config_file);
    void run(const std::atomic<bool>& kill);

private:
    void readZcm();
    void initializeRun(const std::atomic<bool>& kill);
    void setControlState(const ControlState new_control_state);

    maav::mavlink::InnerLoopSetpoint runStandby();
    maav::mavlink::InnerLoopSetpoint runTakeoff();
    maav::mavlink::InnerLoopSetpoint runLand();
    maav::mavlink::InnerLoopSetpoint runFlight();
    maav::mavlink::InnerLoopSetpoint runArming();
    maav::mavlink::InnerLoopSetpoint runDisarming();
    maav::mavlink::InnerLoopSetpoint runKillswitch();

    const std::string control_config_file_;
    const YAML::Node control_config_;

    maav::mavlink::OffboardControl offboard_control_;
    maav::gnc::Controller controller_;
    ControlState current_control_state_;

    std::queue<ControlCommands> commands_;

    zcm::ZCM zcm_;
    ZCMHandler<path_t> path_handler_;
    ZCMHandler<state_t> state_handler_;
    ZCMHandler<groundtruth_inertial_t> sim_state_handler_;
    ZCMHandler<control_commands_t> command_handler_;
    ZCMHandler<killswitch_t> killswitch_handler_;

    bool sim_state_;
};
}
}