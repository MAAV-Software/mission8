#include <iostream>
#include <thread>
#include <atomic>
#include <signal.h>
#include <functional>
#include <algorithm>
#include <chrono>
#include <stdexcept>

#include <zcm/zcm-cpp.hpp>
#include "common/utils/ZCMHandler.hpp"

#include "tanfan/lcmlite.h"
#include "tanfan/nav/PhysicalController.hpp"
#include "tanfan/nav/msg/feedback_t.h"
#include "tanfan/nav/msg/gains_t.h"
#include "tanfan/nav/msg/setpt_t.h"
#include "tanfan/messaging/dji_t.h"

namespace zcm
{
#include "common/messages/feedback_t.hpp"
#include "common/messages/setpt_t.hpp"
#include "common/messages/gains_t.hpp"
#include "common/messages/lidar_t.hpp"
#include "common/messages/imu_t.hpp"
#include "common/messages/dji_t.hpp"
}

using maav::PhysicalController;
using zcm::ZCM;
using std::cerr;
using std::cout;
using std::endl;
using std::thread;
using std::ref;

void tivaToAtomLoop(ZCM &zcm, PhysicalController &physicalController);
void sendToTivaLoop(PhysicalController &physicalController,
		ZCMHandler<zcm::setpt_t> &setpointZcmHandler,
	   	ZCMHandler<zcm::gains_t> &gainsZcmHandler,
	   	ZCMHandler<zcm::dji_t> &djiZcmHandler);
void recvLcmLoop(ZCM &zcm);
void sendGarbage(ZCM &zcm);
void transmitPlaceholder(const uint8_t*, uint32_t);

// Kill Signal Handler
void gracefulKill(int);

// Global flag for kill signal
sig_atomic_t isAlive = 1;

/*
 * @brief TANFAN's main loop
 *
 * @details TANFAN (Tiva-Atom Network Forwarding ApplicatioN) sits between the
 * Tiva and the Atom. It forwards messages sent from the Tiva to the main
 * network on the Atom. The messaging is done via ZCM and lcm-lite.
 */
int main()
{
	maav::Log::init("maav.log", maav::Log::Level::info);

	// on any of the following signals, quit
	signal(SIGINT, gracefulKill);
	signal(SIGQUIT, gracefulKill);
	signal(SIGTERM, gracefulKill);

	// ZCM Message handling queues (thread safe)

	// ZCMHandler<zcm::feedback_t> feedbackLcmHandler;

	// These handlers are from Atom to Tiva, passed into sendToTiva
	ZCMHandler<zcm::setpt_t> setpointZcmHandler;
	ZCMHandler<zcm::gains_t> gainsZcmHandler;
	ZCMHandler<zcm::dji_t> djiZcmHandler;

	// Instantiate ZCM
	const char *URL = "ipc";
	ZCM zcm{URL};

	// subscribe to the "POS" (position) channel. Whenever we get a position
	// update, send send it to LCMHandleruartlcm
	zcm.subscribe("SET", &ZCMHandler<zcm::setpt_t>::recv, &setpointZcmHandler);
	zcm.subscribe("GNS", &ZCMHandler<zcm::gains_t>::recv, &gainsZcmHandler);
	zcm.subscribe("DJI", &ZCMHandler<zcm::dji_t>::recv, &djiZcmHandler);

	if (!zcm.good())
	{
		cerr << "LCM Error\n";
		return -1;
	}

	PhysicalController physicalController;

	physicalController.connect("/dev/ttyUSB0");

	thread sendToTivaThread(sendToTivaLoop, ref(physicalController),
		   	ref(setpointZcmHandler), ref(gainsZcmHandler), ref(djiZcmHandler));
	thread recvLcmThread(recvLcmLoop, ref(zcm));

	tivaToAtomLoop(ref(zcm), ref(physicalController));

	// the above functions block, so this is just cleanup for when the program
	// exits.
	//sendToTivaThread.join();
	//recvLcmThread.join();

	physicalController.disconnect();

	return 0;
}

/*
 * @brief This loop handles getting the messages sent from the Tiva to the Atom
 *
 * @details The Tiva communicates with the Atom by sending messages over UART.
 * The Atom writes the messages to a tty file. The communication is wrapped by
 * LCM.
 *
 * @param zcm
 *
 * @param zcmHandler an ZCMHandler which is called on updates
 */
void tivaToAtomLoop(ZCM &zcm, PhysicalController &physicalController)
{
	while (isAlive)
	{
		void (*placeholder)(const uint8_t*, uint32_t);
		placeholder = &transmitPlaceholder;
		DataLink dlink(placeholder, &zcm);

		physicalController.process(dlink);
	}

	physicalController.stop();
}

/*
 * @brief Takes queued updates and sends them to the Tiva
 *
 * @details The ZCMHandlers queue up messages that need to be sent to the Tiva.
 * This loop takes those messages and send them using LCM-lite to the Tiva
 *
 * @param physicalController This function uses physicalController's methods
 * for sending to the Tiva
 *
 * @param setpointLcmHandler queue handler for setpoints
 *
 * @param gainsLcmHandler queue handler for gains
 *
 * @param djiZcmHandler queue handler for dji
 *
 */
void sendToTivaLoop(PhysicalController &physicalController,
		ZCMHandler<zcm::setpt_t> &setpointZcmHandler,
	   	ZCMHandler<zcm::gains_t> &gainsZcmHandler,
	   	ZCMHandler<zcm::dji_t> &djiZcmHandler)
{
	while (isAlive)
	{
		if (setpointZcmHandler.ready())
		{
			// note that sendSetpoint sends utime
			zcm::setpt_t msg = setpointZcmHandler.msg();

			cout << msg.x << endl;

			// send and pop
			physicalController.sendSetpoint(msg.x, msg.y, msg.z, msg.yaw, msg.flags);
			setpointZcmHandler.pop();
		}

		if (gainsZcmHandler.ready())
		{
			// get the next message
			zcm::gains_t msg = gainsZcmHandler.msg();

			gains_t liteMsg;

			// copy all of the data from the zcm type to the lite type
			std::copy(msg.xGains, msg.xGains + 6, liteMsg.xGains);
			std::copy(msg.yGains, msg.yGains + 6, liteMsg.yGains);
			std::copy(msg.zGains, msg.zGains + 6, liteMsg.zGains);
			std::copy(msg.yawGains, msg.yawGains + 3, liteMsg.yawGains);
			liteMsg.utime = msg.utime;

			// send and pop
			physicalController.sendGains(&liteMsg);
			gainsZcmHandler.pop();
		}

		if (djiZcmHandler.ready())
		{
			// get the next message
			zcm::dji_t msg = djiZcmHandler.msg();

			// send and pop
			physicalController.sendDji(msg.roll, msg.pitch, msg.yawRate, msg.thrust);
			djiZcmHandler.pop();
		}

	}
}

/*
 * @brief ZCM Handle loop
 *
 * @details Allows ZCM to recieve messages subscribed to in main
 * Timeout allows for non-blocking, so we check the kill signal
 * lcm.handleTimeout reads from the lcm buffer and sends the processed data to their
 * corresponding subscriber functions
 *
 * @param zcm ZCM object, needed for the handle timeout
 */
void recvLcmLoop(ZCM &zcm)
{
	while (isAlive) zcm.handle();
}

/*
 * @brief Stops the program.
 *
 * @details Since we're using threads, we need to quit gracefully. We use a
 * threadsafe isAlive atomic type to store the state, and join the threads and
 * exit on quit.
 *
 */
void gracefulKill(int)
{
	isAlive = false;
}

void transmitPlaceholder(const uint8_t* buffer, uint32_t size) {

	throw std::runtime_error("This function should not have been called. This DataLink object exists solely to process data from the Tiva. PhysicalController should be used to receive and send data from the Tiva");

}
