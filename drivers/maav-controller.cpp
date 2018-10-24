#include <signal.h>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <iostream>
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

std::atomic<bool> KILL{false};
void sig_handler(int) { KILL = true; }
// Function for testing altitude controller, intent
// is to remove when no longer needed for testing.
void get_altitude()
{
	double _altitude;
	while (!KILL)
	{
		cout << "Enter altitude >> ";
		cin >> _altitude;
		maav::gnc::SETPOINT = _altitude;
	}
}

int main(int argc, char** argv)
{
	signal(SIGINT, sig_handler);
	signal(SIGABRT, sig_handler);
	signal(SIGSEGV, sig_handler);
	signal(SIGTERM, sig_handler);

	std::cout << "Controller Driver" << std::endl;

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

	zcm::ZCM zcm{"ipc"};
	zcm.start();

	ZCMHandler<path_t> path_handler;
	ZCMHandler<state_t> state_handler;
	ZCMHandler<ctrl_params_t> gains_handler;

	zcm.subscribe(PATH_CHANNEL, &ZCMHandler<path_t>::recv, &path_handler);
	zcm.subscribe(STATE_CHANNEL, &ZCMHandler<state_t>::recv, &state_handler);
	zcm.subscribe(CTRL_PARAMS_CHANNEL, &ZCMHandler<ctrl_params_t>::recv, &gains_handler);

	YAML::Node config = YAML::LoadFile(gopt.getString("config"));
	Controller controller;
	OffboardControl offboard_control(CommunicationType::UART,
									 config["uart-path"].as<std::string>());
	InnerLoopSetpoint inner_loop_setpoint;

	// establish state
	cout << "Establishing initial state...\n";
	int counter = 0;
	auto timeout = system_clock::now() + 10s;
	while (counter < 10 && system_clock::now() < timeout)
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

	// Load default gains from YAML

	controller.set_control_params(load_gains_from_yaml(config));

	controller.set_control_state(ControlState::TAKEOFF);

	auto land_timer = system_clock::now() + 10s;
	while (!KILL)
	{
		if (gains_handler.ready())
		{
			const auto msg = gains_handler.msg();
			gains_handler.pop();
			controller.set_control_params(msg);
		}

		if (path_handler.ready())
		{
			const auto msg = path_handler.msg();
			path_handler.pop();
			controller.set_target(convert_waypoint(msg.waypoints[0]));
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

		if (system_clock::now() > land_timer && controller.get_control_state() == ControlState::HOLD_ALT) 
			controller.set_control_state(ControlState::LAND);

		// pixhawk needs attitude/thrust setpoint commands at
		// rate >2 Hz otherwise it will go into failsafe
		// ***make sure at some point controller is
		// sending commands at a sufficient rate***
		offboard_control.set_attitude_target(inner_loop_setpoint);
	}

	// tid.join();
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
