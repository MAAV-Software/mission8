/*
 * DataLink is the Transport Network Layer Protocol used over the serial
 * link between Navigation and Controls on the quadrotor. Consider the typical
 * LCM link stack of LCM, UDP, IP, Ethernet. The stack for LCM on the quadrotor
 * would be LCM, DataLink, UART.
 *
 *  Created on: May 20, 2015
 *      Author: Clark Zhang, Sasawat Prankprakma
 */

#include "tanfan/messaging/DataLink.hpp"
#include <stdint.h>
#include <cstring>
#include <zcm/zcm-cpp.hpp>
#include "tanfan/lcmlite.h"
#include "tanfan/messaging/DataLinkDefines.hpp"
#include "tanfan/messaging/Decoder.hpp"
#include "tanfan/messaging/MessageHandler.hpp"
#include "tanfan/messaging/TransmitHandler.hpp"
#include "tanfan/messaging/emergency_t.h"
#include "tanfan/messaging/imu_t.h"
#include "tanfan/messaging/lidar_t.h"

// LCM Sender Address (we don't use UDP, it just has to be something)
static const uint64_t ATOM_ADDR = 9001;

using namespace std;

DataLink::DataLink(void (*f)(const uint8_t*, uint32_t), zcm::ZCM* zcm, zcm::ZCM* zcm_udp)
    : msgHandler(zcm, zcm_udp)
{
    // Initialize LCM and handlers
    msgSender = TransmitHandler(f);
    lcmlite_init(&lcm, transmitPacket, &msgSender);

    // Initialize LCM subscriptions
    lidarSub.callback = callback;
    lidarSub.channel = strdup("LID");
    lidarSub.user = &msgHandler;
    imuSub.callback = callback;
    imuSub.channel = strdup("IMU");
    imuSub.user = &msgHandler;
    emsSub.callback = callback;
    emsSub.channel = strdup("EMS");
    emsSub.user = &msgHandler;

    // Subscribe LCM subscriptions
    lcmlite_subscribe(&lcm, &lidarSub);
    lcmlite_subscribe(&lcm, &imuSub);
    lcmlite_subscribe(&lcm, &emsSub);
}

void DataLink::processRecv(const uint8_t raw)
{
    d.push(raw);

    if (d.isError()) d.reset();  // check for error

    if (d.isDone())  // check for done
    {
        lcmlite_receive_packet(&lcm, d.packetData(), d.packetDataSize(), ATOM_ADDR);
        d.reset();
    }
}

DataLink::~DataLink()
{
    free(lidarSub.channel);
    free(imuSub.channel);
    free(emsSub.channel);
}
