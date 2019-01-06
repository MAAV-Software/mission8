#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

#include <yaml-cpp/yaml.h>
#include <zcm/zcm-cpp.hpp>

#include <gnc/xbox-controller/gamepad.h>
#include <common/mavlink/offboard_control.hpp>
#include <common/messages/MsgChannels.hpp>
#include <common/messages/control_commands_t.hpp>
#include <common/messages/ctrl_params_t.hpp>
#include <common/messages/killswitch_t.hpp>
#include <common/messages/localization_status_t.hpp>
#include <common/messages/path_t.hpp>
#include <common/messages/state_t.hpp>
#include <common/utils/GetOpt.hpp>
#include <common/utils/ZCMHandler.hpp>
#include <gnc/State.hpp>
#include <gnc/State.hpp>
#include <gnc/controller.hpp>
#include <gnc/utils/LoadParameters.hpp>
#include <gnc/utils/ZcmConversion.hpp>

using maav::STATE_CHANNEL;
using maav::SIM_STATE_CHANNEL;
using maav::PATH_CHANNEL;
using maav::CTRL_PARAMS_CHANNEL;
using maav::gnc::Controller;
using maav::gnc::XboxController;
using maav::gnc::ConvertState;
using maav::gnc::ConvertWaypoint;
using maav::mavlink::OffboardControl;
using maav::mavlink::InnerLoopSetpoint;
using std::this_thread::sleep_for;
using namespace std::chrono;
using std::cout;
using maav::mavlink::CommunicationType;
using std::cin;
using std::thread;
using maav::gnc::ControlState;
using YAML::Node;
using std::make_pair;
using std::string;
using maav::gnc::Waypoint;
using std::to_string;
using std::this_thread::sleep_for;
using maav::gnc::State;
using maav::gnc::utils::LoadParametersFromYAML;

XboxController read_controller_input();
State ems_state(const OffboardControl& offboard_control);

/*
 *      Signal handling variable/function
 */
std::atomic<bool> KILL{false};
void sig_handler(int) { KILL = true; }
struct Commands
{
    bool gains = false;
    bool takeoff = false;
    bool land = false;
};

int main(int argc, char** argv)
{
    cout << "Controller Driver" << std::endl;

    /*
     *      Signal handlers
     */
    signal(SIGINT, sig_handler);
    signal(SIGABRT, sig_handler);
    signal(SIGSEGV, sig_handler);
    signal(SIGTERM, sig_handler);

    /*
     *      Command line arguments
     */
    GetOpt gopt;
    gopt.addBool('h', "help", false, "This message");
    gopt.addString(
        'c', "config", "../config/gnc/control-config.yaml", "Path to YAML control config");
    gopt.addBool('x', "xbox360", false, "Use xbox controller for control");
    if (!gopt.parse(argc, argv, 1) || gopt.getBool("help"))
    {
        std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
        gopt.printHelp();
        return 1;
    }

    YAML::Node control_config;
    try
    {
        control_config = YAML::LoadFile(gopt.getString("config"));
    }
    catch (...)
    {
        cout << "Could not find config file\nPlease provide command line option \"-c "
                "<path-to-config>\"\n";
        return 2;
    }

    /*
     *      Start zcm and subscribe to proper channels
     */
    zcm::ZCM zcm{"ipc"};
    zcm.start();
    ZCMHandler<path_t> path_handler;
    ZCMHandler<state_t> state_handler;
    ZCMHandler<ctrl_params_t> gains_handler;
    ZCMHandler<control_commands_t> command_handler;
    ZCMHandler<killswitch_t> killswitch_handler;
    ZCMHandler<localization_status_t> localizer_status_handler;

    if (control_config["sim-state"].as<bool>())
    {
        zcm.subscribe(SIM_STATE_CHANNEL, &ZCMHandler<state_t>::recv, &state_handler);
        cout << "Reading state from simulator\n";
    }
    else
    {
        zcm.subscribe(STATE_CHANNEL, &ZCMHandler<state_t>::recv, &state_handler);
        cout << "Reading state from kalman filter\n";
    }
    zcm.subscribe(PATH_CHANNEL, &ZCMHandler<path_t>::recv, &path_handler);
    zcm.subscribe(CTRL_PARAMS_CHANNEL, &ZCMHandler<ctrl_params_t>::recv, &gains_handler);
    zcm.subscribe(
        maav::CONTROL_COMMANDS_CHANNEL, &ZCMHandler<control_commands_t>::recv, &command_handler);
    zcm.subscribe(maav::LOCALIZATION_STATUS_CHANNEL, &ZCMHandler<localization_status_t>::recv,
        &localizer_status_handler);
    zcm.subscribe(maav::KILLSWITCH_CHANNEL, &ZCMHandler<killswitch_t>::recv, &killswitch_handler);

    /*
     *      Initialize controller class and offboard control
     *      construct offboard controller for sim or irl
     */
    Controller controller(
        control_config["zcm-url"].as<std::string>());  // sets control state to standby
    controller.set_control_params(LoadParametersFromYAML(control_config));
    CommunicationType com_type;
    if (control_config["sim"].as<bool>())
    {
        com_type = CommunicationType::UDP;
        cout << "Connecting to px4 through UDP (simulator)\n";
    }
    else
    {
        com_type = CommunicationType::UART;
        cout << "Connecting to px4 through USB fd: " << control_config["uart-path"].as<string>()
             << '\n';
    }
    OffboardControl offboard_control(
        control_config["zcm-url"].as<string>(), com_type, control_config["uart-path"].as<string>());
    try
    {
        offboard_control.init(KILL);  // pass kill variable into init so loops can exit on signal
    }
    catch (...)
    {
        // Sig handlers in offboard control throw
        KILL = true;
    }
    InnerLoopSetpoint inner_loop_setpoint;

    /*
     *      Establish the initial state of the vehicle
     */
    cout << "Establishing initial state...\n";
    int counter = 0;
    auto timeout = system_clock::now() + 10s;
    while (!KILL && counter < 2 && system_clock::now() < timeout)
    {
        offboard_control.set_attitude_target(InnerLoopSetpoint::zero());
        if (state_handler.ready())
        {
            const auto msg = state_handler.msg();
            state_handler.pop();
            ++counter;
        }
    }
    if (counter > 1)
        cout << "Initial state established\n";
    else
    {
        cout << "Unable to establish state of vehicle\n";
        return 3;
    }

    /*
     *      Set control to xbox controller
     */
    const bool xbox_input = gopt.getBool("xbox360");
    if (xbox_input)
    {
        GamepadInit();
        controller.set_control_state(ControlState::XBOX_CONTROL);
    }

    /*
     *      Main run loop
     *      Runs until receives proper signal
     *      Checks for messages, runs controller as appropriate
     *      and updates inner loop setpoint on px4
     */
    State current_state;
    Commands commands;
    State ems_state;
    while (!KILL)
    {
        // Killswitch
        if (killswitch_handler.ready())
        {
            if (killswitch_handler.msg().kill)
            {
                controller.set_control_state(ControlState::KILLSWITCH);
            }
            else
            {
                killswitch_handler.pop();  // pop false message
            }
        }

        // Gains
        if (gains_handler.ready())
        {
            const auto msg = gains_handler.msg();
            gains_handler.pop();
            controller.set_control_params(msg);
        }

        // Path
        if (path_handler.ready())
        {
            if (!path_handler.msg().waypoints.empty())
            {
                cout << "Path received\n";
                controller.set_path(path_handler.msg());
                if (controller.get_control_state() == ControlState::STANDBY)
                {
                    controller.set_control_state(ControlState::ARMING);
                }
            }
            path_handler.pop();
        }

        // State
        if (state_handler.ready())
        {
            const auto msg = state_handler.msg();
            state_handler.pop();
            controller.add_state(ConvertState(msg));
        }

        // Commands, more commands to be added
        if (command_handler.ready())
        {
            cout << "Command received\n";
            if (command_handler.msg().takeoff)
            {
                commands.takeoff = true;
            }
            if (command_handler.msg().land)
            {
                commands.land = true;
            }
            if (command_handler.msg().gains)
            {
                try
                {
                    control_config = YAML::LoadFile(gopt.getString("config"));
                    controller.set_control_params(LoadParametersFromYAML(control_config));
                }
                catch (...)
                {
                    cout << "Could not find config file\nPlease provide command line option \"-c "
                            "<path-to-config>\"\n";
                }
            }

            command_handler.pop();
        }

        switch (controller.get_control_state())
        {
            case ControlState::STANDBY:
                inner_loop_setpoint = InnerLoopSetpoint::zero();
                if (commands.takeoff)
                {
                    commands.takeoff = false;
                    controller.set_control_state(ControlState::ARMING);
                    continue;
                }
                break;

            case ControlState::XBOX_CONTROL:
                assert(false && "xbox control not implemented");
                break;

            case ControlState::TAKEOFF:
                inner_loop_setpoint =
                    controller.takeoff(control_config["takeoff-alt"].as<double>());
                if (localizer_status_handler.ready() && !localizer_status_handler.msg().localized)
                {
                    controller.set_control_state(ControlState::EMS_LAND);
                    continue;
                }
                if (commands.land)
                {
                    commands.land = false;
                    controller.set_control_state(ControlState::LAND);
                    continue;
                }
                if (controller.at_takeoff_alt())
                {
                    controller.set_control_state(ControlState::FLIGHT);
                    continue;
                }
                break;

            case ControlState::LAND:
                if (controller.landing_detected())
                {
                    controller.set_control_state(ControlState::DISARMING);
                    continue;
                }

                if (localizer_status_handler.ready() && !localizer_status_handler.msg().localized)
                {
                    controller.set_control_state(ControlState::EMS_LAND);
                    continue;
                }

                inner_loop_setpoint = controller.land();

                break;

            case ControlState::EMS_LAND:
                ems_state.velocity()(2) = offboard_control.get_ems_state().z_velocity;
                ems_state.setTime(offboard_control.get_ems_state().usec);
                controller.add_ems_state(ems_state);
                inner_loop_setpoint = controller.ems_land();
                if (controller.landing_detected())
                {
                    controller.set_control_state(ControlState::DISARMING);
                }
                break;

            case ControlState::FLIGHT:
                // If localization is lost, set EMS_LAND
                if (localizer_status_handler.ready() && !localizer_status_handler.msg().localized)
                {
                    controller.set_control_state(ControlState::EMS_LAND);
                    continue;
                }
                else if (commands.land)
                {
                    commands.land = false;
                    controller.set_control_state(ControlState::LAND);
                    continue;
                }
                else
                {
                    inner_loop_setpoint = controller.flight();
                }
                break;

            case ControlState::ARMING:
                cout << "Pixhawk arming...\n";
                while (!KILL && !offboard_control.is_armed())
                {
                    offboard_control.arm();
                    offboard_control.set_attitude_target(InnerLoopSetpoint::zero());
                }
                cout << "Pixhawk armed\n";
                controller.set_control_state(ControlState::TAKEOFF);
                break;

            case ControlState::DISARMING:
                cout << "Pixhawk disarming...\n";
                while (!KILL && offboard_control.is_armed())
                {
                    offboard_control.disarm();
                    offboard_control.set_attitude_target(InnerLoopSetpoint::zero());
                }
                cout << "Pixhawk disarmed\n";
                controller.set_control_state(ControlState::STANDBY);
                break;

            case ControlState::KILLSWITCH:
                inner_loop_setpoint = InnerLoopSetpoint::zero();
                offboard_control.set_attitude_target(inner_loop_setpoint);
                sleep_for(1s);
                while (offboard_control.is_armed() && !KILL)
                {
                    offboard_control.disarm();
                    offboard_control.set_attitude_target(InnerLoopSetpoint::zero());
                }
                KILL = true;
                continue;

            default:
                assert(false);  // make sure all states get handled
        }

        // Dont perform actions iff kill set (a goto would probably be more useful than KILL)
        if (!KILL)
        {
            offboard_control.set_attitude_target(inner_loop_setpoint);
            sleep_for(1ms);
        }
    }

    zcm.stop();
}

/*
 *      Reads xbox controller is options is selected from cli
 */
XboxController read_controller_input()
{
    XboxController xbox_controller;
    GamepadUpdate();
    if (GamepadIsConnected(GAMEPAD_0))
    {
        GamepadStickXY(GAMEPAD_0, STICK_LEFT, &(xbox_controller.left_joystick_x),
            &(xbox_controller.left_joystick_y));
        GamepadStickXY(GAMEPAD_0, STICK_RIGHT, &(xbox_controller.right_joystick_x),
            &(xbox_controller.right_joystick_y));
        xbox_controller.left_trigger = GamepadTriggerValue(GAMEPAD_0, TRIGGER_LEFT);
        xbox_controller.right_trigger = GamepadTriggerValue(GAMEPAD_0, TRIGGER_RIGHT);
    }
    else
    {
        cout << "Controller connection error!";
    }

    return xbox_controller;
}

State ems_state(const OffboardControl& offboard_control)
{
    assert(false);
    return State();
}