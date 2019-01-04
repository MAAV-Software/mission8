#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <thread>

#include <zcm/zcm-cpp.hpp>
#include "common/utils/ZCMHandler.hpp"

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

void tivaToNucLoop(ZCM &zcm, ZCM &zcm_udp, PhysicalController &physicalController);
void recvLcmLoop(ZCM &zcm);
void sendGarbage(ZCM &zcm);
void transmitPlaceholder(const uint8_t *, uint32_t);

std::atomic_bool is_running = true;

void sigHandler(int) { is_running = false; }
/*
 * @brief TANFAN's main loop
 *
 * @details TANFAN (TivA-nuc Network Forwarding ApplicatioN) sits between the
 * Tiva and the Nuc. It forwards messages sent from the Tiva to the main
 * network on the Nuc. The messaging is done via ZCM and lcm-lite.
 */
int main()
{
    maav::Log::init("maav.log", maav::Log::Level::info);

    // on any of the following signals, quit
    signal(SIGINT, sigHandler);
    signal(SIGABRT, sigHandler);
    signal(SIGSEGV, sigHandler);
    signal(SIGTERM, sigHandler);
    signal(SIGQUIT, sigHandler);

    // ZCM Message handling queues (thread safe)

    // Instantiate ZCM
    ZCM zcm{"ipc"};
    ZCM zcm_udp{"udpm://239.255.76.67:7667?ttl=1"};

    if (!zcm.good())
    {
        throw "Bad ZCM";
    }
    if (!zcm_udp.good())
    {
        throw "Bad ZCM UDP";
    }

    zcm.start();

    PhysicalController physicalController;

    physicalController.connect("/dev/ttyUSB0");

    tivaToNucLoop(ref(zcm), ref(zcm_udp), ref(physicalController));

    // the above functions block, so this is just cleanup for when the program
    // exits.
    // sendToTivaThread.join();
    // recvLcmThread.join();

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
void tivaToNucLoop(ZCM &zcm, ZCM &zcm_udp, PhysicalController &physicalController)
{
    while (is_running)
    {
        void (*placeholder)(const uint8_t *, uint32_t);
        placeholder = &transmitPlaceholder;
        DataLink dlink(placeholder, &zcm, &zcm_udp);

        physicalController.process(dlink);
    }

    physicalController.stop();
}

void transmitPlaceholder(const uint8_t *buffer, uint32_t size)
{
    throw std::runtime_error(
        "This function should not have been called. This DataLink object exists solely to process "
        "data from the Tiva. PhysicalController should be used to receive and send data from the "
        "Tiva");
}
