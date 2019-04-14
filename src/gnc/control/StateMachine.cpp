#include <chrono>
#include <iostream>

#include <gnc/control/StateMachine.hpp>
#include <gnc/utils/LoadParameters.hpp>
#include <gnc/utils/ZcmConversion.hpp>

using maav::gnc::utils::LoadParametersFromYAML;
using maav::mavlink::CommunicationType;
using maav::mavlink::InnerLoopSetpoint;

namespace maav
{
namespace gnc
{
StateMachine::StateMachine(const std::string& control_config_file)
    : control_config_file_(control_config_file),
      control_config_(YAML::LoadFile(control_config_file_)),
      offboard_control_(
          control_config_["sim"].as<bool>() ? CommunicationType::UDP : CommunicationType::UART,
          control_config_["uart-path"].as<std::string>()),
      controller_(control_config_),
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

        offboard_control_.set_attitude_target(inner_loop_setpoint);

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void StateMachine::initializeRun(const std::atomic<bool>& kill)
{
    std::cout << "Checking for heartbeat..." << std::endl;
    while (!kill && !offboard_control_.readMessage())
    {
    }
    std::cout << "Heartbeat received" << std::endl;

    offboard_control_.holdZeroAttitude(std::chrono::seconds(1));

    // Connect to offboard control
    std::cout << "Establishing offboard control..." << std::endl;
    while (!kill && !offboard_control_.activate_offboard_control())
    {
        offboard_control_.set_attitude_target(InnerLoopSetpoint::zero());
    }

    // Print success/failer of offboard connect
    if (offboard_control_.offboardControlActive())
    {
        std::cout << "Offboard control established on pixhawk" << std::endl;
    }

    /*
     *      Establish the initial state of the vehicle
     */
    std::cout << "Establishing initial state..." << std::endl;
    int counter = 0;
    auto timeout = std::chrono::system_clock::now() + std::chrono::seconds(10);
    while (!kill && counter < 10 && std::chrono::system_clock::now() < timeout)
    {
        offboard_control_.set_attitude_target(InnerLoopSetpoint::zero());
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
        state_handler_.pop();
    }

    while (sim_state_ && sim_state_handler_.ready())
    {
        controller_.add_state(ConvertGroundTruthState(sim_state_handler_.msg()));
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

InnerLoopSetpoint StateMachine::runStandby()
{
    if (commands_.front() == ControlCommands::TAKEOFF)
    {
        setControlState(ControlState::ARMING);
        commands_.pop();
    }
    return InnerLoopSetpoint::zero();
}

InnerLoopSetpoint StateMachine::runTakeoff()
{
    if (commands_.front() == ControlCommands::LAND)
    {
        setControlState(ControlState::LAND);
        commands_.pop();
    }

    if (controller_.at_takeoff_alt())
    {
        setControlState(ControlState::FLIGHT);
        std::cout << "switching to flight in takeoff" << std::endl;
    }

    std::cout << "Taking off to alt: " << control_config_["takeoff-alt"].as<double>() << std::endl;
    return controller_.takeoff(control_config_["takeoff-alt"].as<double>());
}

InnerLoopSetpoint StateMachine::runLand()
{
    if (controller_.landing_detected())
    {
        setControlState(ControlState::DISARMING);
    }

    return controller_.land();
}

InnerLoopSetpoint StateMachine::runFlight()
{
    if (commands_.front() == ControlCommands::LAND)
    {
        setControlState(ControlState::LAND);
    }
    return controller_.flight();
}

InnerLoopSetpoint StateMachine::runArming()
{
    static bool print_message = true;
    static std::chrono::time_point<std::chrono::system_clock, std::chrono::duration<double>>
        arming_timeout;
    static auto arming_delay =
        std::chrono::duration<double>(control_config_["arming-delay"].as<double>());

    // Print message on first arm attempt
    if (print_message)
    {
        std::cout << "Pixhawk arming...\n";
        print_message = false;
    }

    if (!offboard_control_.is_armed())
    {
        // Try to arm, if success set arming delay timeout
        if (offboard_control_.arm())
        {
            arming_timeout = std::chrono::system_clock::now() + arming_delay;
        }
    }
    else
    {
        if (std::chrono::system_clock::now() > arming_timeout)
        {
            std::cout << "Pixhawk armed\n";
            setControlState(ControlState::TAKEOFF);
            print_message = true;  // print on next arm}
        }

        // During the arming delay, return 12% thrust
        return InnerLoopSetpoint{{1, 0, 0, 0}, 0.12, 0, 0, 0};
    }

    // If arming has not occured return zero thrust
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

    if (offboard_control_.is_armed())
    {
        offboard_control_.disarm();
    }
    else
    {
        setControlState(ControlState::STANDBY);
        std::cout << "Pixhawk disarmed\n";
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
        case ControlState::DISARMING:
            std::cout << "DISARMING";
            break;
        case ControlState::TAKEOFF:
            std::cout << "TAKEOFF";
            break;
        case ControlState::LAND:
            std::cout << "LAND";
            break;
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
}
}
