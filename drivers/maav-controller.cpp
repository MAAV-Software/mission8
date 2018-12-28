#include <signal.h>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

#include <zcm/zcm-cpp.hpp>

#include <gnc/xbox-controller/gamepad.h>
#include <yaml-cpp/yaml.h>
#include <common/mavlink/offboard_control.hpp>
#include <common/messages/MsgChannels.hpp>
#include <common/messages/ctrl_params_t.hpp>
#include <common/messages/path_t.hpp>
#include <common/messages/state_t.hpp>
#include <common/utils/GetOpt.hpp>
#include <common/utils/ZCMHandler.hpp>
#include <gnc/controller.hpp>
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

/*
 * Temporary Operating Procedure (to start work on controller)
 * ================================================================
 * 1. Start simulator and wait for px4 to initialize
 * 2. run "./maav-controller" - the controller will try to
 * 	  establish offboard control.  If it fails, restart it and
 * 	  it should work.
 * Optional: provide flag "-x" to use xbox controller instead of
 *    manually typing in waypoints.
 * 3. The program will indicate that offboard control has been
 *    established and armed.
 * 4. If the pixhawk does not receive setpoint commands (thrust,
 *    attitude, angle rates) at a rate of >2 Hz it will enter failsafe
 *    mode and switch out of offboard control.  It is currently the
 *    responsibility of the controller class to provide these inputs
 *    at a sufficient rate.
*/

ctrl_params_t load_gains_from_yaml(const YAML::Node& config_file);
Controller::Parameters load_vehicle_params(const Node& config);
XboxController read_controller_input();

std::atomic<bool> KILL{false};
void sig_handler(int) { KILL = true; }
Waypoint SETPOINT = Waypoint{Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0), 0};
std::atomic<bool> MANUAL_SETPOINT_INPUT{true};
void get_setpoint(Controller*);
path_t create_test_path(const YAML::Node& path_file);

int main(int argc, char** argv)
{
    std::cout << "Controller Driver" << std::endl;

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
    gopt.addBool(
        's', "command-line-input", true, "Manually input waypoints using command line interface");
    gopt.addString('p', "test-path", "../config/gnc/test-path.yaml", "Read custom test path");
    if (!gopt.parse(argc, argv, 1) || gopt.getBool("help"))
    {
        std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
        gopt.printHelp();
        return 1;
    }

    YAML::Node gains_config = YAML::LoadFile(gopt.getString("config"));
    YAML::Node path_file = YAML::LoadFile(gopt.getString("test-path"));

    /*
     *      Start zcm and subscribe to proper channels
     */
    zcm::ZCM zcm{"ipc"};
    zcm.start();
    ZCMHandler<path_t> path_handler;
    ZCMHandler<state_t> state_handler;
    ZCMHandler<ctrl_params_t> gains_handler;

    if (gains_config["sim-state"].as<bool>())
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

    /*
     *      Initialize controller class and offboard control
     */
    Controller controller;
    controller.set_control_params(
        load_gains_from_yaml(gains_config), load_vehicle_params(gains_config));
    OffboardControl offboard_control(CommunicationType::UDP);
    InnerLoopSetpoint inner_loop_setpoint;

    /*
     *      Establish the initial state of the vehicle
     */
    cout << "Establishing initial state...\n";
    int counter = 0;
    auto timeout = system_clock::now() + 10s;
    while (!KILL && counter < 2 && system_clock::now() < timeout)
    {
        if (state_handler.ready())
        {
            const auto msg = state_handler.msg();
            state_handler.pop();
            controller.run(ConvertState(msg));
            ++counter;
        }
    }
    cout << "Initial state established\n";

    /*
     *      Read command line control input
     *      and set control as either through cli
     *      or xbox controller
     */
    path_t test_path = create_test_path(path_file);
    ;
    thread tid;
    const bool xbox_input = gopt.getBool("xbox360");
    const bool setpoint_input = gopt.getBool("command-line-input");
    if (xbox_input)
    {
        GamepadInit();
    }
    else if (setpoint_input)
    {
        controller.set_control_state(ControlState::TEST_WAYPOINT);
        tid = thread(std::bind(get_setpoint, &controller));
    }

    /*
     *      Main run loop
     *      Runs until receives proper signal
     *      Checks for messages, runs controller as appropriate
     *      and updates inner loop setpoint on px4
     */
    while (!KILL)
    {
        // Command line input logic (with global variables...#sorrynotsorry)
        if (MANUAL_SETPOINT_INPUT)
        {
            if (controller.get_control_state() != ControlState::TEST_WAYPOINT)
            {
                controller.set_control_state(ControlState::TEST_WAYPOINT);
            }
        }
        else
        {
            if (controller.get_control_state() != ControlState::TEST_PATH)
            {
                controller.set_control_state(ControlState::TEST_PATH);
                controller.set_path(test_path);
            }
        }

        if (gains_handler.ready())
        {
            const auto msg = gains_handler.msg();
            gains_handler.pop();
            controller.set_control_params(msg);
        }

        if (path_handler.ready())
        {
            controller.set_path(path_handler.msg());
            path_handler.pop();
            // TODO: save path, pass waypoints sequentially to set_target when waiting for new
            // path!!!
        }

        if (state_handler.ready())
        {
            // What happens when states come in faster than this loop runs?
            // What happens when states DONT come in at all?
            const auto msg = state_handler.msg();
            state_handler.pop();
            if (xbox_input)
                inner_loop_setpoint = controller.run(read_controller_input(), ConvertState(msg));
            else
                inner_loop_setpoint = controller.run(ConvertState(msg));
        }

        // Continues setting the same inner loop setpoint even if there is no input from the
        // controller
        offboard_control.set_attitude_target(inner_loop_setpoint);
        std::this_thread::sleep_for(1ms);
    }

    if (setpoint_input) tid.join();
    zcm.stop();
}

/*
 *      Reads vehicle and control params from yaml
 */
ctrl_params_t load_gains_from_yaml(const YAML::Node& config_file)
{
    ctrl_params_t gains;

    const YAML::Node& posGains = config_file["pid-gains"]["pos-ctrl"];

    gains.value[0].p = posGains["x"][0].as<double>();
    gains.value[0].i = posGains["x"][1].as<double>();
    gains.value[0].d = posGains["x"][2].as<double>();

    gains.value[1].p = posGains["y"][0].as<double>();
    gains.value[1].i = posGains["y"][1].as<double>();
    gains.value[1].d = posGains["y"][2].as<double>();

    gains.value[2].p = posGains["z"][0].as<double>();
    gains.value[2].i = posGains["z"][1].as<double>();
    gains.value[2].d = posGains["z"][2].as<double>();

    gains.value[3].p = posGains["yaw"][0].as<double>();
    gains.value[3].i = posGains["yaw"][1].as<double>();
    gains.value[3].d = posGains["yaw"][2].as<double>();

    const YAML::Node& rateGains = config_file["pid-gains"]["rate-ctrl"];
    gains.rate[0].p = rateGains["x"][0].as<double>();
    gains.rate[0].i = rateGains["x"][1].as<double>();
    gains.rate[0].d = rateGains["x"][2].as<double>();

    gains.rate[1].p = rateGains["y"][0].as<double>();
    gains.rate[1].i = rateGains["y"][1].as<double>();
    gains.rate[1].d = rateGains["y"][2].as<double>();

    gains.rate[2].p = rateGains["z"][0].as<double>();
    gains.rate[2].i = rateGains["z"][1].as<double>();
    gains.rate[2].d = rateGains["z"][2].as<double>();

    return gains;
}

Controller::Parameters load_vehicle_params(const Node& config)
{
    Controller::Parameters params;

    params.mass = config["mass"].as<double>();
    params.setpoint_tol = config["tol"].as<double>();
    params.min_F_norm = config["min-fnorm"].as<double>();
    params.thrust_limits = make_pair<double, double>(1, config["min-thrust"].as<double>());

    constexpr double deg2rad = M_PI / 180.0;

    const Node& rateNode = config["limits"]["rate"];
    params.rate_limits[0] = make_pair(rateNode["x"][0].as<double>(), rateNode["x"][1].as<double>());
    params.rate_limits[1] = make_pair(rateNode["y"][0].as<double>(), rateNode["y"][1].as<double>());
    params.rate_limits[2] = make_pair(rateNode["z"][0].as<double>(), rateNode["z"][1].as<double>());
    params.rate_limits[3] =
        make_pair(rateNode["yaw"][0].as<double>(), rateNode["yaw"][1].as<double>());

    const Node& accelNode = config["limits"]["accel"];
    params.accel_limits[0] =
        make_pair(accelNode["x"][0].as<double>(), accelNode["x"][1].as<double>());
    params.accel_limits[1] =
        make_pair(accelNode["y"][0].as<double>(), accelNode["y"][1].as<double>());
    params.accel_limits[2] =
        make_pair(accelNode["z"][0].as<double>(), accelNode["z"][1].as<double>());

    const Node& angNode = config["limits"]["angle"];
    params.angle_limits[0] = make_pair(
        angNode["roll"][0].as<double>() * deg2rad, angNode["roll"][1].as<double>() * deg2rad);
    params.angle_limits[1] = make_pair(
        angNode["pitch"][0].as<double>() * deg2rad, angNode["pitch"][1].as<double>() * deg2rad);

    return params;
}

/*
 *	    Test path for testing tests
 *      Reads from yaml file (path can be provided)
 *      Make sure that the waypoints in yaml are named
 *      sequentially with integer values starting at 0
 */
path_t create_test_path(const Node& path_file)
{
    path_t path;
    waypoint_t wpt;
    int waypoint_count = 0;
    string waypoint_key;

    const Node& path_node = path_file["path"];
    for (auto it = path_node.begin(); it != path_file["path"].end(); ++it)
    {
        waypoint_key = to_string(waypoint_count);
        wpt.pose[0] = path_node[waypoint_key][0].as<double>();
        wpt.pose[1] = path_node[waypoint_key][1].as<double>();
        wpt.pose[2] = path_node[waypoint_key][2].as<double>();
        wpt.pose[3] = path_node[waypoint_key][3].as<double>();
        path.waypoints.push_back(wpt);
        ++waypoint_count;
    }

    path.NUM_WAYPOINTS = waypoint_count;

    return path;
}

// Function for testing altitude controller, intent
// is to remove when no longer needed for testing.
void get_setpoint(Controller* controller)
{
    cout << "Enter \">> test\" to run test path\n";
    cout << "  <or>\n";
    cout << "Enter wayoints (x,y,z,yaw)\n";
    double a, b, c, d;
    string command;
    while (!KILL)
    {
        cout << ">> ";
        cin >> command;
        if (command == "test")
        {
            MANUAL_SETPOINT_INPUT = false;
        }
        else
        {
            try
            {
                a = std::stof(command);
            }
            catch (...)
            {
                cout << "INVALID INPUT\n";
                continue;
            }

            MANUAL_SETPOINT_INPUT = true;
            cin >> b >> c >> d;
            controller->set_current_target({Eigen::Vector3d(a, b, c), Eigen::Vector3d(0, 0, 0), d});
        }
    }
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