#include <algorithm>
#include <chrono>
#include <iostream>

#include <gnc/control/StateMachine.hpp>
#include <gnc/utils/LoadParameters.hpp>
#include <gnc/utils/ZcmConversion.hpp>

using maav::gnc::utils::LoadParametersFromYAML;
using maav::mavlink::AutopilotInterface;
using maav::mavlink::InnerLoopSetpoint;

using namespace std::chrono_literals;

namespace maav
{
namespace gnc
{
namespace control
{
StateMachine::StateMachine(
    const std::string& control_config_file, AutopilotInterface* const autopilot_interface)
    : control_config_file_(control_config_file),
      control_config_(YAML::LoadFile(control_config_file_)),
      autopilot_interface_(autopilot_interface),
      controller_(control_config_, autopilot_interface->initial_position.yaw),
      land_detector_(control_config_),
      current_control_state_(ControlState::STANDBY),
      zcm_(control_config_["zcm-url"].as<std::string>()),
      sim_state_(control_config_["sim-state"].as<bool>())

{
    /*
     *  ZCM initialize
     */
    if (sim_state_)
    {
        zcm_.subscribe(maav::GT_INERTIAL_CHANNEL, &ZCMHandler<groundtruth_inertial_t>::recv,
            &sim_state_handler_);
    }
    else
    {
        zcm_.subscribe(maav::STATE_CHANNEL, &ZCMHandler<state_t>::recv, &state_handler_);
    }
    zcm_.subscribe(maav::PATH_CHANNEL, &ZCMHandler<path_t>::recv, &path_handler_);
    zcm_.subscribe(maav::KILLSWITCH_CHANNEL, &ZCMHandler<killswitch_t>::recv, &killswitch_handler_);
    zcm_.subscribe(
        maav::CONTROL_COMMANDS_CHANNEL, &ZCMHandler<control_commands_t>::recv, &command_handler_);
    zcm_.start();
}

void StateMachine::run(const std::atomic<bool>& kill)
{
    /*
     *  Check vehicle active and connect to offboard control,
     *  perform this prior to trying to control the vehicle
     */
    initializeRun(kill);

    InnerLoopSetpoint inner_loop_setpoint = InnerLoopSetpoint::zero();

    while (!kill)
    {
        /*
         *  Read in all new messages and handle commands
         */
        readZcm();

        switch (current_control_state_)
        {
            case ControlState::STANDBY:
                inner_loop_setpoint = runStandby();
                break;
            case ControlState::TAKEOFF:
                inner_loop_setpoint = runTakeoff();
                break;
            case ControlState::LAND:
                inner_loop_setpoint = runLand();
                break;
            case ControlState::FLIGHT:
                inner_loop_setpoint = runFlight();
                break;
            case ControlState::ARMING:
                inner_loop_setpoint = runArming();
                break;
            case ControlState::SOFT_ARM:
                inner_loop_setpoint = runSoftArm();
                break;
            case ControlState::DISARMING:
                inner_loop_setpoint = runDisarming();
                break;
            case ControlState::KILLSWITCH:
                inner_loop_setpoint = runKillswitch();
                break;
            default:
                assert(false);
                break;
        }

        if (current_control_state_ != ControlState::STANDBY &&
            current_control_state_ != ControlState::ARMING &&
            current_control_state_ != ControlState::DISARMING)
        {
            inner_loop_setpoint.thrust = armedThrust(inner_loop_setpoint.thrust);
        }

        autopilot_interface_->update_setpoint(inner_loop_setpoint);

        while (!commands_.empty())
        {
            commands_.pop();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void StateMachine::initializeRun(const std::atomic<bool>& kill)
{
    /*
     *      Establish the initial state of the vehicle
     */
    std::cout << "Establishing initial state..." << std::endl;
    int counter = 0;
    auto timeout = std::chrono::system_clock::now() + std::chrono::seconds(10);
    auto zero_setpoint = InnerLoopSetpoint::zero();
    while (!kill && counter < 10 && std::chrono::system_clock::now() < timeout)
    {
        autopilot_interface_->update_setpoint(zero_setpoint);
        if (!sim_state_ && state_handler_.ready())
        {
            controller_.add_state(ConvertState(state_handler_.msg()));
            state_handler_.pop();
            ++counter;
        }
        if (sim_state_ && sim_state_handler_.ready())
        {
            controller_.add_state(ConvertGroundTruthState(sim_state_handler_.msg()));
            sim_state_handler_.pop();
            ++counter;
        }
    }
    if (counter > 1)
    {
        std::cout << "Initial state established" << std::endl;
    }
    else
    {
        std::cout << "Unable to establish state of vehicle" << std::endl;
    }
    return;
}

void StateMachine::readZcm()
{
    while (path_handler_.ready())
    {
        if (!path_handler_.msg().waypoints.empty())
        {
            std::cout << "Path received\n";
            controller_.set_path(path_handler_.msg());
            if (current_control_state_ == ControlState::STANDBY)
            {
                setControlState(ControlState::ARMING);
            }
        }
        path_handler_.pop();
    }

    while (!sim_state_ && state_handler_.ready())
    {
        controller_.add_state(ConvertState(state_handler_.msg()));
        land_detector_.setState(ConvertState(state_handler_.msg()));
        state_handler_.pop();
    }

    while (sim_state_ && sim_state_handler_.ready())
    {
        controller_.add_state(ConvertGroundTruthState(sim_state_handler_.msg()));
        land_detector_.setState(ConvertGroundTruthState(sim_state_handler_.msg()));
        sim_state_handler_.pop();
    }

    while (command_handler_.ready())
    {
        std::cout << "Command received\n";

        if (command_handler_.msg().takeoff)
        {
            commands_.push(ControlCommands::TAKEOFF);
        }
        if (command_handler_.msg().land)
        {
            commands_.push(ControlCommands::LAND);
        }
        if (command_handler_.msg().disarm)
        {
            commands_.push(ControlCommands::DISARM);
        }
        if (command_handler_.msg().arm)
        {
            commands_.push(ControlCommands::ARM);
        }
        if (command_handler_.msg().gains)
        {
            try
            {
                controller_.set_control_params(
                    LoadParametersFromYAML(YAML::LoadFile(control_config_file_)));
            }
            catch (...)
            {
                std::cout << "Could not find config file\nPlease provide command line "
                             "option \"-c <path-to-config>\"\n";
            }
        }
        command_handler_.pop();
    }

    while (killswitch_handler_.ready())
    {
        killswitch_handler_.pop();
    }
}

bool StateMachine::checkCommand(const ControlCommands command)
{
    if (!commands_.empty() && commands_.front() == command)
    {
        commands_.pop();
        return true;
    }
    return false;
}

InnerLoopSetpoint StateMachine::runStandby()
{
    if (!autopilot_interface_->offboardMode())
    {
        autopilot_interface_->enable_offboard_control();
        std::this_thread::sleep_for(250ms);
    }
    else
    {
        if (checkCommand(ControlCommands::TAKEOFF))
        {
            setControlState(ControlState::ARMING);
        }

        if (checkCommand(ControlCommands::ARM))
        {
            setControlState(ControlState::ARMING);
            soft_arm_ = true;
        }
    }
    return InnerLoopSetpoint::zero();
}

InnerLoopSetpoint StateMachine::runTakeoff()
{
    // variable to track printing takeoff message
    static bool print_takeoff = true;

    if (checkCommand(ControlCommands::LAND))
    {
        setControlState(ControlState::LAND);
    }

    if (controller_.at_takeoff_alt())
    {
        setControlState(ControlState::FLIGHT);
        std::cout << "switching to flight in takeoff" << std::endl;
        print_takeoff = true;
    }

    if (print_takeoff)
    {
        std::cout << "Taking off to alt: " << control_config_["takeoff-alt"].as<double>()
                  << std::endl;
        print_takeoff = false;
    }
    return controller_.takeoff(control_config_["takeoff-alt"].as<double>());
}

InnerLoopSetpoint StateMachine::runLand()
{
    if (checkCommand(ControlCommands::DISARM))
    {
        setControlState(ControlState::DISARMING);
    }

    land_detector_.detectLanding();
    if (land_detector_.getLandedState() == LandDetector::LandedState::Landed)
    {
        setControlState(ControlState::DISARMING);
    }

    auto setpoint = controller_.land();
    land_detector_.setThrust(setpoint.thrust);

    return setpoint;
}

InnerLoopSetpoint StateMachine::runFlight()
{
    if (checkCommand(ControlCommands::LAND))
    {
        setControlState(ControlState::LAND);
    }
    return controller_.flight();
}

InnerLoopSetpoint StateMachine::runArming()
{
    static bool print_message = true;

    // Print message on first arm attempt
    if (print_message)
    {
        std::cout << "Pixhawk arming...\n";
        print_message = false;
    }

    if (!autopilot_interface_->armed())
    {
        autopilot_interface_->arm();
        std::this_thread::sleep_for(500ms);
    }
    else
    {
        // ARMED!
        // arm command pushed if soft are desired
        if (soft_arm_)
        {
            setControlState(ControlState::SOFT_ARM);
            soft_arm_ = false;
        }
        else
        {
            setControlState(ControlState::TAKEOFF);
        }
        print_message = true;  // print message on next disarm
    }

    // If arming has not occured return zero thrust
    return InnerLoopSetpoint::zero();
}

InnerLoopSetpoint StateMachine::runSoftArm()
{
    if (checkCommand(ControlCommands::TAKEOFF))
    {
        setControlState(ControlState::TAKEOFF);
    }
    if (checkCommand(ControlCommands::DISARM))
    {
        setControlState(ControlState::DISARMING);
    }

    return InnerLoopSetpoint::zero();
}

InnerLoopSetpoint StateMachine::runDisarming()
{
    static bool print_message = true;

    if (print_message)
    {
        std::cout << "Pixhawk disarming...\n";
        print_message = false;
    }

    if (autopilot_interface_->armed())
    {
        autopilot_interface_->disarm();
        std::this_thread::sleep_for(500ms);
    }
    else
    {
        setControlState(ControlState::STANDBY);
        print_message = true;  // print message on next disarm
    }

    return InnerLoopSetpoint::zero();
}

InnerLoopSetpoint StateMachine::runKillswitch()
{
    assert(false);
    return InnerLoopSetpoint::zero();
}

void StateMachine::setControlState(const ControlState new_control_state)
{
    std::cout << "Control state switched to ";
    switch (new_control_state)
    {
        case ControlState::ARMING:
            std::cout << "ARMING";
            break;
        case ControlState::SOFT_ARM:
            std::cout << "SOFT ARMING";
            break;
        case ControlState::DISARMING:
            std::cout << "DISARMING";
            break;
        case ControlState::TAKEOFF:
            std::cout << "TAKEOFF";
            break;
        case ControlState::LAND:
        {
            // Clear path
            path_t empty_path;
            empty_path.NUM_WAYPOINTS = 0;
            empty_path.waypoints.clear();
            controller_.set_path(empty_path);

            // Set landing state
            land_detector_.setAir();
            std::cout << "LAND";
            break;
        }
        case ControlState::STANDBY:
            std::cout << "STANDBY";
            break;
        case ControlState::KILLSWITCH:
            std::cout << "KILLSWITCH";
            break;
        case ControlState::FLIGHT:
            std::cout << "FLIGHT";
            break;
        default:
            assert(false);
            break;
    }

    std::cout << std::endl;
    current_control_state_ = new_control_state;
}

float StateMachine::armedThrust(float thrust)
{
    constexpr float min_thrust = 0.12;
    return std::max(thrust, min_thrust);
}

}  // namespace control
}  // namespace gnc
}  // namespace maav
