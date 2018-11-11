#include <signal.h>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

#include <zcm/zcm-cpp.hpp>

#include <yaml-cpp/yaml.h>
#include <common/mavlink/offboard_control.hpp>
#include <common/messages/MsgChannels.hpp>
#include <common/messages/ctrl_params_t.hpp>
#include <common/messages/path_t.hpp>
#include <common/messages/state_t.hpp>
#include <common/utils/GetOpt.hpp>
#include <common/utils/ZCMHandler.hpp>
#include <gnc/controller.hpp>
#include <gnc/utils/zcm_conversion.hpp>

using maav::STATE_CHANNEL;
using maav::SIM_STATE_CHANNEL;
using maav::PATH_CHANNEL;
using maav::CTRL_PARAMS_CHANNEL;
using maav::gnc::Controller;
using maav::gnc::convert_state;
using maav::gnc::convert_waypoint;
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
using maav::gnc::CONTROL_STATE_SETPOINT;

/*
 * Temporary Operating Procedure (to start work on controller)
 * ================================================================
 * 1. Start simulator and wait for px4 to initialize
 * 2. run "./maav-controller" - the controller will try to
 * 	  establish offboard control.  If it fails, restart it and
 * 	  it should work.
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

std::atomic<bool> KILL{false};
void sig_handler(int) { KILL = true; }
void get_setpoint();
path_t create_test_path();

int main(int argc, char** argv)
{
    std::cout << "Controller Driver" << std::endl;

    signal(SIGINT, sig_handler);
    signal(SIGABRT, sig_handler);
    signal(SIGSEGV, sig_handler);
    signal(SIGTERM, sig_handler);

    GetOpt gopt;
    gopt.addBool('h', "help", false, "This message");
    gopt.addString('c', "config", "", "Path to YAML control config");
    // TODO: Add getopt arguments as necessary

    if (!gopt.parse(argc, argv, 1) || gopt.getBool("help"))
    {
        std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
        gopt.printHelp();
        return 1;
    }

    YAML::Node gains_config = YAML::LoadFile(gopt.getString("config"));

    zcm::ZCM zcm{"ipc"};
    zcm.start();

    ZCMHandler<path_t> path_handler;
    ZCMHandler<state_t> state_handler;
    ZCMHandler<ctrl_params_t> gains_handler;

    if (gains_config["sim-state"].as<bool>())
    {
        zcm.subscribe(SIM_STATE_CHANNEL, &ZCMHandler<state_t>::recv, &state_handler);
    }
    else
    {
        zcm.subscribe(STATE_CHANNEL, &ZCMHandler<state_t>::recv, &state_handler);
    }
    zcm.subscribe(PATH_CHANNEL, &ZCMHandler<path_t>::recv, &path_handler);
    zcm.subscribe(CTRL_PARAMS_CHANNEL, &ZCMHandler<ctrl_params_t>::recv, &gains_handler);

    Controller controller;
    controller.set_control_params(
        load_gains_from_yaml(gains_config), load_vehicle_params(gains_config));
    OffboardControl offboard_control(CommunicationType::UDP);
    InnerLoopSetpoint inner_loop_setpoint;

    // establish state
    cout << "Establishing initial state...\n";
    int counter = 0;
    auto timeout = system_clock::now() + 10s;
    while (!KILL && counter < 2 && system_clock::now() < timeout)
    {
        if (state_handler.ready())
        {
            const auto msg = state_handler.msg();
            state_handler.pop();
            controller.run(convert_state(msg));
            ++counter;
        }
    }
    cout << "Initial state established\n";

    // Command line input for tests
    thread tid(get_setpoint);

    // Create a test path;
    path_t test_path = create_test_path();

    while (!KILL)
    {
        // Command line input logic (with global variables...sorry)
        if (CONTROL_STATE_SETPOINT)
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
            inner_loop_setpoint = controller.run(convert_state(msg));
        }

        // Continues setting the same inner loop setpoint even if there is no input from the
        // controller
        offboard_control.set_attitude_target(inner_loop_setpoint);
    }

    tid.join();
    zcm.stop();
}

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
 *	Test path for testing tests
*/
path_t create_test_path()
{
    path_t path;

    waypoint_t wpt;
    wpt.pose[0] = 0;
    wpt.pose[1] = 0;
    wpt.pose[2] = -2;
    wpt.pose[3] = 0;
    path.waypoints.push_back(wpt);

    wpt.pose[0] = 1;
    wpt.pose[1] = 1;
    wpt.pose[2] = -3;
    wpt.pose[3] = 0;
    path.waypoints.push_back(wpt);

    wpt.pose[0] = 1;
    wpt.pose[1] = -1;
    wpt.pose[2] = -4;
    wpt.pose[3] = 0;
    path.waypoints.push_back(wpt);

    wpt.pose[0] = -1;
    wpt.pose[1] = -1;
    wpt.pose[2] = -3;
    wpt.pose[3] = 0;
    path.waypoints.push_back(wpt);

    wpt.pose[0] = -1;
    wpt.pose[1] = 1;
    wpt.pose[2] = -2;
    wpt.pose[3] = 0;
    path.waypoints.push_back(wpt);

    wpt.pose[0] = 0;
    wpt.pose[1] = 0;
    wpt.pose[2] = -1;
    wpt.pose[3] = 0;
    path.waypoints.push_back(wpt);

    wpt.pose[0] = 0;
    wpt.pose[1] = 0;
    wpt.pose[2] = 0;
    wpt.pose[3] = 0;
    path.waypoints.push_back(wpt);

    path.NUM_WAYPOINTS = 7;

    return path;
}

// Function for testing altitude controller, intent
// is to remove when no longer needed for testing.
void get_setpoint()
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
            CONTROL_STATE_SETPOINT = false;
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

            CONTROL_STATE_SETPOINT = true;
            cin >> b >> c >> d;
            maav::gnc::SETPOINT = {Eigen::Vector3d(a, b, c), Eigen::Vector3d(0, 0, 0), d};
        }
    }
}