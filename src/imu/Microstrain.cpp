#include <imu/Microstrain.hpp>

using namespace std;
using namespace std::chrono;
using namespace std::chrono_literals;
using std::chrono::microseconds;

namespace maav
{
MicrostrainImu::MicrostrainImu()
{
    serialInit();

    // These are the vendor_id and product_id on my laptop, checking using "lsusb"
    constexpr uint16_t vendor_id = 0x199b;
    constexpr uint16_t product_id = 0x3065;
    connect(vendor_id, product_id);

    // free the list, unref the devices in it
    libusb_free_device_list(devs, 1);

    checkKernelDriver();

    claimInterface();
}

MicrostrainImu::~MicrostrainImu()
{
    cout << "Exiting MAAV IMU Driver" << endl;

    libusb_close(dev_handle);
    libusb_exit(ctx);
}

void MicrostrainImu::read(imu_t &msg)
{
    // The endpoint could be got by using "lsusb -v"
    sendData(0x03, formatMeasCmd().buf);
    int data_length = receiveData(0x81);

    if ((data_length == MEASUREMENT_DATA_LENGTH) && goodChecksum(outputBytes, data_length))
        parseData(outputBytes, msg);
}

void MicrostrainImu::floatToBytes(unsigned char *dest, uint32_t idx, float num)
{
    BytesF32 data;
    data.number = num;

    dest[idx] = data.buf[3];
    dest[idx + 1] = data.buf[2];
    dest[idx + 2] = data.buf[1];
    dest[idx + 3] = data.buf[0];
}

void MicrostrainImu::u16ToBytes(unsigned char *dest, uint32_t idx, uint16_t num)
{
    BytesU16 data;
    data.number = num;

    dest[idx] = data.buf[1];
    dest[idx + 1] = data.buf[0];
}

MicrostrainImu::MicroStrainCmd MicrostrainImu::formatStopContMode()
{
    MicroStrainCmd m;
    m.length = 3;
    m.buf[0] = 0xFA;
    m.buf[1] = 0x75;
    m.buf[2] = 0xB4;
    return m;
}

MicrostrainImu::MicroStrainCmd MicrostrainImu::formatSoftResetCmd()
{
    MicroStrainCmd m;
    m.length = 3;
    m.buf[0] = 0xFE;
    m.buf[1] = 0x9E;
    m.buf[2] = 0x3A;
    return m;
}

MicrostrainImu::MicroStrainCmd MicrostrainImu::formatMeasCmd()
{
    MicroStrainCmd m;
    m.buf[0] = MEASUREMENT_CMD;
    m.length = 1;
    return m;
}

MicrostrainImu::MicroStrainCmd MicrostrainImu::formatGyroBiasCmd(const uint16_t samplingTime)
{
    MicroStrainCmd m;
    m.length = 5;
    m.buf[0] = GYRO_CALIB_CMD;
    m.buf[1] = 0xC1;
    m.buf[2] = 0x29;
    u16ToBytes(m.buf, 3, samplingTime);
    return m;
}

MicrostrainImu::MicroStrainCmd MicrostrainImu::formatAccelBiasCmd(
    const float accXBias, const float accYBias, const float accZBias)
{
    MicroStrainCmd m;
    m.length = 15;
    m.buf[0] = ACCEL_CALIB_CMD;
    m.buf[1] = 0xB7;
    m.buf[2] = 0x44;
    floatToBytes(m.buf, 3, accXBias);
    floatToBytes(m.buf, 7, accYBias);
    floatToBytes(m.buf, 11, accZBias);
    return m;
}

bool MicrostrainImu::goodChecksum(const unsigned char *data, uint32_t size)
{
    int16_t checksum = 0;
    for (uint32_t i = 0; i < size - 2; ++i) checksum += data[i];

    int16_t responseChecksum = ((uint16_t)data[size - 2] << 8) | data[size - 1];

    return checksum == responseChecksum;
}

void MicrostrainImu::serialInit(void)
{
    // initialize the library for the session we just declared
    int r = libusb_init(&ctx);

    if (r < 0)
    {
        cout << "Init Error " << r << endl;
    }

    // set verbosity level,suggested in the documentation
    libusb_set_debug(ctx, LIBUSB_LOG_LEVEL_WARNING);

    // get the list of devices
    cnt = libusb_get_device_list(ctx, &devs);

    if (cnt < 0)
    {
        cout << "Get Device Error" << endl;
    }

    cout << cnt << ": Devices in list." << endl;
}

void MicrostrainImu::connect(uint16_t vendor_id, uint16_t product_id)
{
    // vendorID and productID
    dev_handle = libusb_open_device_with_vid_pid(ctx, vendor_id, product_id);

    if (dev_handle == NULL)
    {
        std::cout << "Cannot open device. Is it plugged in and recognized with 'lsusb'? " << endl;

        // needs to be called to end the
        libusb_exit(ctx);
        exit(EXIT_FAILURE);
    }
    else
        std::cout << "Device Opened" << endl;
}

void MicrostrainImu::checkKernelDriver(void)
{
    // find out if kernel driver is attached
    if (libusb_kernel_driver_active(dev_handle, 0) == 1)
    {
        std::cout << "Kernel Driver Active" << endl;

        // detach it
        if (libusb_detach_kernel_driver(dev_handle, 0) == 0)
            std::cout << "Kernel Driver Detached!" << endl;
    }
}

void MicrostrainImu::claimInterface(void)
{
    // claim interface 0 (the first) of device
    int r = libusb_claim_interface(dev_handle, 0);

    if (r < 0)
        cout << "Cannot Claim Interface" << endl;
    else
        cout << "Claimed Interface" << endl;
}

void MicrostrainImu::sendData(unsigned char endpoint, unsigned char *address)
{
    // used to find out how many bytes were written
    int actual;

    int r = libusb_bulk_transfer(dev_handle, endpoint, address, 1, &actual, 0);

    // successfully
    if (r == 0)
    {
        // cout<<"Sending Successfully!"<<endl;
    }
    else
    {
        cout << "Send Error" << endl;

        // release the claimed interface
        r = libusb_release_interface(dev_handle, 0);

        if (r != 0)
        {
            cout << "Cannot Release Interface" << endl;
        }
        else
        {
            cout << "Released Interface" << endl;
        }
        // close the device we opened
        libusb_close(dev_handle);
        // needs to be called to end the
        libusb_exit(ctx);
    }
}

int MicrostrainImu::receiveData(unsigned char endpoint)
{
    // used to find out how many bytes were received
    int actual;

    int r =
        libusb_bulk_transfer(dev_handle, endpoint, outputBytes, sizeof(outputBytes), &actual, 0);

    if (r == 0)
    {
        // cout<<"Receiving Successfully!"<<endl<<"What i get is :\n"<<outputBytes<<endl;
        // cout<<"Received Length: "<<actual<<endl;
        return actual;
    }
    else
    {
        cout << "Receive Error" << endl;

        r = libusb_release_interface(dev_handle, 0);  // release the claimed interface

        if (r != 0)
        {
            cout << "Cannot Release Interface" << endl;
        }

        cout << "Released Interface" << endl;

        // close the device we opened
        libusb_close(dev_handle);
        // needs to be called to end the
        libusb_exit(ctx);
        return 0;
    }
}

void MicrostrainImu::parseData(unsigned char *data, imu_t &imu)
{
    // TODO This was miliseconds during the logs. I just recently changed them to usec.
    uint64_t utime = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
    imu.utime = utime;

    // Big-Endian?
    // char FirstByte = data[0];
    constexpr double STANDARD_GRAVITY = 9.80665;

    imu.acceleration[0] = bytesToFloat(data, 1) * STANDARD_GRAVITY;
    imu.acceleration[1] = bytesToFloat(data, 5) * STANDARD_GRAVITY;
    imu.acceleration[2] = bytesToFloat(data, 9) * STANDARD_GRAVITY;
    imu.angular_rates[0] = bytesToFloat(data, 13);
    imu.angular_rates[1] = bytesToFloat(data, 17);
    imu.angular_rates[2] = bytesToFloat(data, 21);
    imu.magnetometer[0] = bytesToFloat(data, 25);
    imu.magnetometer[1] = bytesToFloat(data, 29);
    imu.magnetometer[2] = bytesToFloat(data, 33);
}

float MicrostrainImu::bytesToFloat(unsigned char *raw, unsigned int i)
{
    union B2F {
        unsigned char buf[4];
        float number;
    } data;

    data.buf[0] = raw[i + 3];
    data.buf[1] = raw[i + 2];
    data.buf[2] = raw[i + 1];
    data.buf[3] = raw[i + 0];

    return data.number;
}

int MicrostrainImu::bytesToInt(unsigned char *raw, unsigned int i)
{
    union B2I {
        unsigned char buf[4];
        int number;
    } data;

    data.buf[0] = raw[i + 3];
    data.buf[1] = raw[i + 2];
    data.buf[2] = raw[i + 1];
    data.buf[3] = raw[i + 0];

    return data.number;
}
}