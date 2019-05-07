#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <string>

#include <zcm/zcm-cpp.hpp>
#include "common/utils/ZCMHandler.hpp"
#include <common/utils/GetOpt.hpp>

#include "tanfan/lcmlite.h"
#include "tanfan/messaging/dji_t.h"
#include "tanfan/nav/PhysicalController.hpp"
#include "tanfan/nav/msg/feedback_t.h"
#include "tanfan/nav/msg/gains_t.h"
#include "tanfan/nav/msg/setpt_t.h"

namespace zcm
{
#include "common/messages/imu_t.hpp"
#include "common/messages/lidar_t.hpp"
}

using maav::PhysicalController;
using zcm::ZCM;
using std::cerr;
using std::cout;
using std::endl;
using std::thread;
using std::ref;
using std::string;
using std::atomic_bool;
using std::runtime_error;

void tivaToNucLoop(ZCM &zcm, PhysicalController &physicalController);
void recvLcmLoop(ZCM &zcm);
void sendGarbage(ZCM &zcm);
void transmitPlaceholder(const uint8_t *, uint32_t);

atomic_bool is_running = true;

void sigHandler(int) { is_running = false; }

/*
 * @brief TANFAN's main loop
 *
 * @details TANFAN (TivA-nuc Network Forwarding ApplicatioN) sits between the
 * Tiva and the Nuc. It forwards messages sent from the Tiva to the main
 * network on the Nuc. The messaging is done via ZCM and lcm-lite.
 */
int main(int argc, char** argv)
{
    GetOpt gopt;
    gopt.addBool('h', "help", false, "This message");
    gopt.addBool('v', "verbose", false, "Prints out message content from Tiva.");
	gopt.addString('p', "port", "/dev/ttyUSB0", "Serial port for Tiva connection.");

    if (!gopt.parse(argc, argv, 1) || gopt.getBool("help"))
    {
        cout << "Usage: " << argv[0] << " [options]" << endl;
        gopt.printHelp();
        return 0;
    }
    
	maav::Log::init("maav.log", 
		gopt.getBool("verbose") ? maav::Log::Level::debug : maav::Log::Level::info);

    // on any of the following signals, quit
    signal(SIGINT, sigHandler);
    signal(SIGABRT, sigHandler);
    signal(SIGSEGV, sigHandler);
    signal(SIGTERM, sigHandler);
    signal(SIGQUIT, sigHandler);

    // Instantiate ZCM
    ZCM zcm{"ipc"};
    if (!zcm.good()) throw runtime_error("tanfan error: Bad ZCM");
	
    zcm.start();

    PhysicalController physicalController;
    physicalController.connect(string(gopt.getString("port")));

    tivaToNucLoop(ref(zcm), ref(physicalController));

    physicalController.disconnect();

    zcm.stop();

    return 0;
}

/*
 * @brief This loop handles getting the messages sent from the Tiva to the Nuc
 *
 * @details The Tiva communicates with the Nuc by sending messages over UART.
 * The Nuc writes the messages to a tty file. The communication is wrapped by
 * LCM.
 *
 * @param zcm
 *
 * @param zcmHandler an ZCMHandler which is called on updates
 */
void tivaToNucLoop(ZCM &zcm, PhysicalController &physicalController)
{
    while (is_running)
    {
        DataLink dlink(&transmitPlaceholder, &zcm);
        physicalController.process(dlink);
    }

    physicalController.stop();
}

void transmitPlaceholder(const uint8_t *buffer, uint32_t size)
{
    throw runtime_error(
        "This function should not have been called. This DataLink object exists solely to process "
        "data from the Tiva. PhysicalController should be used to receive and send data from the "
        "Tiva");
}
