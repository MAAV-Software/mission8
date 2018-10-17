#include <signal.h>
#include <atomic>
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

std::atomic<bool> KILL{false};
void sig_handler(int) { KILL = true; }


/*
 * Temporary Operating Procedure (to start work on controller)
 * ================================================================
 * 1. Start simulator
 * 2. Wait for px4 to initialize. When "pxh >> commander status" shows
 *	  the px4 in main mode: 4 it is probably time
 * 3. run "./maav-controller" - the controller will try to 
 * 	  establish offboard control.  It will timeout after 10 sec.
 * 	  If it doesnt work for some reason just kil the program and try again.
 * 	  It usually works after one or two tries (this will be fixed to be more 
 *    robust when state machine is implemented)
 * 4. The program will indicate that offboard control has bee
 *    established.  The quadcopter is now in the controllers hands....
 * 5. If the pixhawk does not receive setpoint commands (thrust,
 *    attitude, angle rates) at a rate of >2 Hz it will enter failsafe
 *    mode and switch out of offboard control.  It is currently the
 *    responsibility of the controller class to provide these inputs
 *    at a sufficient rate.
 * 6. If offboard control is lost in flight, order the pixhawk to land
 * 	  with "pxh >> commander land" and rerun maav-controller.  If the 
 *    pixhawk rejects offboard control, try setting auto loiter with
 *    "pxh >> commander mode auto:loiter" then rerunning maav-controller
*/


int main(int argc, char** argv)
{
	signal(SIGINT, sig_handler);
	signal(SIGABRT, sig_handler);
	signal(SIGSEGV, sig_handler);
	signal(SIGTERM, sig_handler);

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

	uint64_t t = time(NULL);
	uint64_t et = 0;
	while (!KILL)
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

		// pixhawk needs attitude/thrust setpoint commands at
		// rate >2 Hz otherwise it will go into failsafe
		// ***make sure at some point controller is
		// sending commands at a sufficient rate***
		et = (time(NULL) - t);
		if(et < 10) controller.take_off();
		else if(et < 11) controller.yaw_right();
		else if(et < 13) controller.yaw_left();
		else if(et < 14) controller.yaw_right();
		else if(et < 15) controller.pitch_forward();
		else if(et < 17) controller.pitch_back();
		else if(et < 18) controller.pitch_forward();
		else if(et < 19) controller.roll_right();
		else if(et < 21) controller.roll_left();
		else if(et < 22) controller.roll_right();
		else controller.land();
	}

	zcm.stop();
}