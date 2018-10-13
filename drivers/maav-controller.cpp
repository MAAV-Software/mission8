#include <iostream>

#include <zcm/zcm-cpp.hpp>

#include <common/messages/MsgChannels.hpp>
#include <common/messages/path_t.hpp>
#include <common/messages/state_t.hpp>
#include <common/utils/GetOpt.hpp>
#include <common/utils/ZCMHandler.hpp>
#include <gnc/controller.hpp>
#include <gnc/utils/zcm_conversion.hpp>

using maav::STATE_CHANNEL;
using maav::PATH_CHANNEL;
using maav::gnc::Controller;
using maav::gnc::convert_state;
using maav::gnc::convert_waypoint;

int main(int argc, char** argv)
{
	std::cout << "Controller Driver" << std::endl;

	GetOpt gopt;
	gopt.addBool('h', "help", false, "This message");
	gopt.addString('c', "config", "", "Path to config.");
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

	zcm.subscribe(PATH_CHANNEL, &ZCMHandler<path_t>::recv, &path_handler);
	zcm.subscribe(STATE_CHANNEL, &ZCMHandler<state_t>::recv, &state_handler);

	Controller controller;

	bool kill = false;
	while (!kill)
	{
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
			const auto msg = state_handler.msg();
			state_handler.pop();

			controller.add_state(convert_state(msg));
		}

		// TODO: Get output from planner
		//        path_t path = localizer.get_path();
		//        zcm.publish(PATH_CHANNEL, &path);
	}
	zcm.stop();
}