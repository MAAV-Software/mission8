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

namespace
{
path_t get_empty_path()
{
    path_t empty_path;
    empty_path.NUM_WAYPOINTS = 0;
    empty_path.utime = std::numeric_limits<int64_t>::infinity();
    return empty_path;
}

}  // namespace

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
      convergence_tolerance_(control_config_["tol"].as<double>()),
      takeoff_altitude_(-control_config_["takeoff-alt"].as<double>()),
      takeoff_speed_(control_config_["takeoff-speed"].as<double>()),
      landing_speed_(control_config_["landing-speed"].as<double>()),
      flight_speed_(control_config_["flight-speed"].as<double>()),
      valid_path(false),
      takeoff_path_(new LinearlyInterpolatedPath(get_empty_path(), takeoff_speed_)),
      landing_path_(new LinearlyInterpolatedPath(get_empty_path(), landing_speed_)),
      path_(new LinearlyInterpolatedPath(get_empty_path(), flight_speed_)),
      reset_path_time_(control_config_["reset-path-starts"].as<bool>()),
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
    initializeRun(kill);

    InnerLoopSetpoint inner_loop_setpoint = InnerLoopSetpoint::zero();

    while (!kill)
    {
        /*
         *  Read in all new messages and handle commands
         */
        if (readZcm())
        {
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
        }

        while (!commands_.empty())
        {
            commands_.pop();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void StateMachine::initializeRun(const std::atomic<bool>& kill)
{
    std::cout << "Establishing initial state..." << std::endl;
    int counter = 0;
    auto timeout = std::chrono::system_clock::now() + std::chrono::seconds(10);
    auto zero_setpoint = InnerLoopSetpoint::zero();
    while (!kill && counter < 10 && std::chrono::system_clock::now() < timeout)
    {
        autopilot_interface_->update_setpoint(zero_setpoint);
        if (!sim_state_ && state_handler_.ready())
        {
            current_state_ = ConvertState(state_handler_.msg());
            state_handler_.pop();
            ++counter;
        }
        if (sim_state_ && sim_state_handler_.ready())
        {
            current_state_ = ConvertGroundTruthState(sim_state_handler_.msg());
            sim_state_handler_.pop();
            ++counter;
        }
    }
    if (counter > 1)
    {
        std::cout << "Initial state established" << std::endl;
        if (sim_state_)
        {
            std::cout << "Using SIM STATE" << std::endl;
        }
        else
        {
            std::cout << "Using KALMAN FILTER STATE" << std::endl;
        }
    }
    else
    {
        std::cout << "Unable to establish state of vehicle" << std::endl;
    }
    return;
}

bool StateMachine::readZcm()
{
    bool read_zcm = false;

    while (path_handler_.ready())
    {
        if (!path_handler_.msg().waypoints.empty())
        {
            path_t path_msg = path_handler_.msg();

            if (reset_path_time_)
            {
                path_msg.utime = current_state_.timeUSec();
            }

            if (!path_msg.waypoints.empty())
            {
                valid_path = true;
                std::cout << "Path received\n";
                path_->updatePath(path_msg);
                if (current_control_state_ == ControlState::STANDBY)
                {
                    setControlState(ControlState::ARMING);
                }
            }
        }
        path_handler_.pop();
        read_zcm = true;
    }

    while (!sim_state_ && state_handler_.ready())
    {
        current_state_ = ConvertState(state_handler_.msg());
        land_detector_.setState(ConvertState(state_handler_.msg()));
        state_handler_.pop();
        read_zcm = true;
    }

    while (sim_state_ && sim_state_handler_.ready())
    {
        current_state_ = ConvertGroundTruthState(sim_state_handler_.msg());
        land_detector_.setState(ConvertGroundTruthState(sim_state_handler_.msg()));
        sim_state_handler_.pop();
        read_zcm = true;
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
        read_zcm = true;
    }

    while (killswitch_handler_.ready())
    {
        killswitch_handler_.pop();
        read_zcm = true;
    }

    return read_zcm;
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
            path_t takeoff_path;
            takeoff_path.NUM_WAYPOINTS = 1;
            takeoff_path.utime = current_state_.timeUSec();
            takeoff_path.waypoints.reserve(1);

            waypoint_t takeoff_waypoint;
            takeoff_waypoint.pose[0] = current_state_.position().x();
            takeoff_waypoint.pose[1] = current_state_.position().y();
            takeoff_waypoint.pose[2] = takeoff_altitude_;
            takeoff_waypoint.pose[3] = current_state_.attitude().angleZ();
            takeoff_path.waypoints.push_back(takeoff_waypoint);

            valid_path = true;
            path_->updatePath(takeoff_path);

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

    if (atTakeoffAltitude())
    {
        setControlState(ControlState::FLIGHT);
        std::cout << "switching to flight in takeoff" << std::endl;
        print_takeoff = true;
    }

    if (print_takeoff)
    {
        print_takeoff = false;
    }

    const uint64_t time = current_state_.timeUSec();
    ContinuousPath::Waypoint target = takeoff_path_->sample(time);
    // std::cout << "Takeoff path start time: " << takeoff_path_->getStartTime() << std::endl;
    return controller_(current_state_, target);
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

    const uint64_t time = current_state_.timeUSec();
    ContinuousPath::Waypoint target = landing_path_->sample(time);

    return controller_(current_state_, target);
}

InnerLoopSetpoint StateMachine::runFlight()
{
    if (checkCommand(ControlCommands::LAND))
    {
        setControlState(ControlState::LAND);
    }

    const uint64_t time = current_state_.timeUSec();
    ContinuousPath::Waypoint target = path_->sample(time);

    return controller_(current_state_, target);
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
            if (valid_path)
            {
                setControlState(ControlState::TAKEOFF);
            }
            else
            {
                std::cout << "No valid path to fly" << std::endl;
                setControlState(ControlState::DISARMING);
            }
        }
        print_message = true;  // print message on next disarm
    }

    // If arming has not occured return zero thrust
    return InnerLoopSetpoint::zero();
}

InnerLoopSetpoint StateMachine::runSoftArm()
{
    if (valid_path)
    {
        if (checkCommand(ControlCommands::TAKEOFF))
        {
            setControlState(ControlState::TAKEOFF);
        }
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
        {
            std::cout << "TAKEOFF";
            std::cout << std::endl;
            std::cout << "Taking off to alt: " << -takeoff_altitude_ << "m.";

            path_t takeoff_path;
            takeoff_path.NUM_WAYPOINTS = 2;
            takeoff_path.utime = current_state_.timeUSec();
            takeoff_path.waypoints.reserve(2);

            waypoint_t current_waypoint;
            current_waypoint.pose[0] = current_state_.position().x();
            current_waypoint.pose[1] = current_state_.position().y();
            current_waypoint.pose[2] = current_state_.position().z();
            current_waypoint.pose[3] = current_state_.attitude().angleZ();
            takeoff_path.waypoints.push_back(current_waypoint);

            waypoint_t takeoff_waypoint;
            takeoff_waypoint.pose[0] = current_state_.position().x();
            takeoff_waypoint.pose[1] = current_state_.position().y();
            takeoff_waypoint.pose[2] = takeoff_altitude_;
            current_waypoint.pose[3] = current_state_.attitude().angleZ();
            takeoff_path.waypoints.push_back(takeoff_waypoint);

            takeoff_path_->updatePath(takeoff_path);

            break;
        }
        case ControlState::LAND:
        {
            path_t landing_path;
            landing_path.NUM_WAYPOINTS = 2;
            landing_path.utime = current_state_.timeUSec();
            landing_path.waypoints.reserve(2);

            waypoint_t current_waypoint;
            current_waypoint.pose[0] = current_state_.position().x();
            current_waypoint.pose[1] = current_state_.position().y();
            current_waypoint.pose[2] = current_state_.position().z();
            current_waypoint.pose[3] = current_state_.attitude().angleZ();
            landing_path.waypoints.push_back(current_waypoint);

            waypoint_t landed_waypoint;
            landed_waypoint.pose[0] = current_state_.position().x();
            landed_waypoint.pose[1] = current_state_.position().y();
            landed_waypoint.pose[2] = 1;
            landed_waypoint.pose[3] = current_state_.attitude().angleZ();
            landing_path.waypoints.push_back(landed_waypoint);

            landing_path_->updatePath(landing_path);

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

bool StateMachine::atTakeoffAltitude() const
{
    bool low_velocity = current_state_.velocity().norm() <= convergence_tolerance_;
    bool at_altitude =
        std::abs(takeoff_altitude_ - current_state_.position().z()) <= convergence_tolerance_;
    return low_velocity && at_altitude;
}

}  // namespace control
}  // namespace gnc
}  // namespace maav
