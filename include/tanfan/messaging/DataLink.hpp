/**
 * @brief Allows communication with the atom
 *
 * @details DataLink is the communications protocol, combining LCM with a custom Transport Layer,
 * used over the serial link between Navigation and Controls on the quadrotor. '
 * Consider the typical LCM link stack of LCM, UDP, IP, Ethernet.
 * The stack for LCM on the quadrotor would be DataLink/LCM, UART driver, UART.
 *
 * A Data link layer packet will look at like
 * <start delimiter> - 1 byte
 * <length of message> - 2 to 4 bytes (depending if they have to be escaped) (big endian)
 * <msg> - msg with escaped characters.
 * <checksum> - 1 to 2 bytes (depending if it needs to be escaped)
 *
 * checksum is the sum of all the data bytes (unescaped), take only the least significant byte, and
 * subtract from 0xFF.
 * NOTE: probably should include the length bytes in the checksum
 *
 * As our RingBuffer will be 256 for reading in, that means the maximum size of a
 * decoded/pre-encoded message is 120 bytes.
 *
 *  @date May 20, 2015
 *  @author Clark Zhang, Sasawat Prankprakma
 */

#ifndef DATALINK_HPP_
#define DATALINK_HPP_

#include <stdint.h>

#include <zcm/zcm-cpp.hpp>
#include "../lcmlite.h"
#include "Decoder.hpp"
#include "MessageHandler.hpp"
#include "RingBuffer.hpp"
#include "TransmitHandler.hpp"
#include "emergency_t.h"
#include "imu_t.h"
#include "lidar_t.h"

class DataLink
{
    public:
    DataLink(void (*f)(const uint8_t*, uint32_t), zcm::ZCM* zcm);
    ~DataLink();

    // void recv(uint8_t datum) { rb->push(datum); }
    void processRecv(const uint8_t raw);

    // Getters for received messages
    lidar_t getLidarMsg() const { return msgHandler.lidar; }
    imu_t getImuMsg() const { return msgHandler.imu; }
    emergency_t getEmergencyMsg() const { return msgHandler.ems; }
    private:
    lcmlite_t lcm;  // lcmlite struct

    // subscription types for the messages received
    lcmlite_subscription_t lidarSub;
    lcmlite_subscription_t imuSub;
    lcmlite_subscription_t emsSub;

    // Lcm Message Handler for recieved messages
    MessageHandler msgHandler;

    // Handler for sending lcm messages
    TransmitHandler msgSender;

    // uint8_t rbBack[256]; // data allocated for ringbuffer
    // RingBuffer<256> *rb; // ringbuffer for incoming bytes

    // Decoder for Transport Layer
    Decoder d;
};

#endif /* DATALINK_HPP_ */
