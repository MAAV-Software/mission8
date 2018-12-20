#pragma once

#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <thread>

#include <libusb-1.0/libusb.h>

#include <common/messages/MsgChannels.hpp>
#include <imu/ImuDevice.hpp>

namespace maav
{
class MicrostrainImu : public ImuDevice
{
public:
    MicrostrainImu();

    ~MicrostrainImu();

    void read(imu_t &msg);

private:
    static constexpr size_t MAX_MS_CMD_LENGTH = 15;
    static constexpr int MEASUREMENT_DATA_LENGTH = 79;
    static constexpr int ACCEL_BIAS_DATA_LENGTH = 19;
    static constexpr int GYRO_BIAS_DATA_LENGTH = 19;
    static constexpr size_t NUM_M_VAL = 9;

    static constexpr unsigned char MEASUREMENT_CMD = 0xCC;
    static constexpr unsigned char ACCEL_CALIB_CMD = 0xC9;
    static constexpr unsigned char GYRO_CALIB_CMD = 0xCD;

    struct MicroStrainCmd
    {
        unsigned char buf[MAX_MS_CMD_LENGTH];
        uint32_t length;
    };

    typedef union u16union {
        uint16_t number;
        unsigned char buf[sizeof(uint16_t)];
    } BytesU16;

    typedef union u32union {
        uint32_t number;
        unsigned char buf[sizeof(uint32_t)];
    } BytesU32;

    typedef union f32union {
        float number;
        unsigned char buf[sizeof(float)];
    } BytesF32;

    MicroStrainCmd formatStopContMode();
    MicroStrainCmd formatSoftResetCmd();
    MicroStrainCmd formatMeasCmd();
    MicroStrainCmd formatGyroBiasCmd(const uint16_t samplingTime);
    MicroStrainCmd formatAccelBiasCmd(
        const float accXBias, const float accYBias, const float accZBias);

    void serialInit();

    void connect(uint16_t vendor_id, uint16_t product_id);

    void checkKernelDriver();

    void claimInterface();

    void sendData(unsigned char, unsigned char *);

    int receiveData(unsigned char);

    void parseData(unsigned char *, imu_t &imu);

    float bytesToFloat(unsigned char *, unsigned int);

    int bytesToInt(unsigned char *, unsigned int);

    bool goodChecksum(const unsigned char *data, uint32_t size);

    void floatToBytes(unsigned char *dest, uint32_t idx, float num);

    void u16ToBytes(unsigned char *dest, uint32_t idx, uint16_t num);

    // pointer to pointer of device, used to retrieve a list of devices
    libusb_device **devs;

    // a device handle
    libusb_device_handle *dev_handle;

    // a libusb session
    libusb_context *ctx = NULL;

    // holding number of devices in list
    ssize_t cnt;

    // The actual length should be 79.
    unsigned char outputBytes[128];
};
}