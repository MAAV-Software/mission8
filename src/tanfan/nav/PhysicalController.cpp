#include "tanfan/nav/PhysicalController.hpp"
#include <ctime>
#include "tanfan/messaging/dji_t.h"
#include "tanfan/nav/msg/setpt_t.h"

using maav::PhysicalController;
using maav::SerialTTY;
using std::string;
using std::function;

#define MAGIC_LCM2 0x4c433032

/* from lcmlite */
static inline void encode_u32(uint8_t *p, uint32_t v)
{
    p[3] = v & 0xff;
    v >>= 8;
    p[2] = v & 0xff;
    v >>= 8;
    p[1] = v & 0xff;
    v >>= 8;
    p[0] = v & 0xff;
}

PhysicalController::PhysicalController() : log{"PhysicalController"} { running = true; }
PhysicalController::~PhysicalController() {}
void PhysicalController::connect(const string &portPath) { port.connect(portPath.c_str()); }
void PhysicalController::move(double dx, double dy, double dz)
{
    sendSetpoint(dx, dy, dz, 0, SETPT_T_POSE);
}

void PhysicalController::sendGains(const gains_t *g)
{
    unsigned char *lcmBuf{nullptr};
    unsigned char *serBuf{nullptr};
    int lcmLen;
    char *channel = strdup("GNS");
    int extraLen;

    lcmLen = gains_t_encoded_size(g);
    extraLen = 8 + strlen(channel) + 1;

    lcmBuf = new unsigned char[lcmLen + extraLen];
    memset(lcmBuf, 0, lcmLen + extraLen);

    encode_u32(lcmBuf, MAGIC_LCM2);
    encode_u32(lcmBuf + 4, 0);
    memcpy(lcmBuf + 8, channel, strlen(channel));

    if (gains_t_encode(lcmBuf + extraLen, 0, lcmLen, g) != lcmLen)
    {
        log.warn("failed to encode gains");
    }
    else
    {
        int serLen = (2 * (lcmLen + extraLen)) + 3;
        serBuf = new unsigned char[serLen];

        serLen = data_link_assemble_packet(lcmBuf, serBuf, lcmLen + extraLen);

        log.info("sending gains message");
        port.send((char *)serBuf, serLen);
    }

    delete[] lcmBuf;
    delete[] serBuf;
    free(channel);
}

void PhysicalController::sendSetpoint(float x, float y, float z, float h, int8_t flags)
{
    setpt_t pt;
    unsigned char *lcmBuf{nullptr};
    unsigned char *serBuf{nullptr};
    int lcmLen;
    char *setptChannel = strdup("SET");
    int extraLen;

    pt.x = x;
    pt.y = y;
    pt.z = z;
    pt.yaw = h;
    pt.flags = flags;
    pt.utime = time(nullptr);

    lcmLen = setpt_t_encoded_size(&pt);
    extraLen = 8 + strlen(setptChannel) + 1;

    lcmBuf = new unsigned char[lcmLen + extraLen];
    memset(lcmBuf, 0, lcmLen + extraLen);

    encode_u32(lcmBuf, MAGIC_LCM2);
    encode_u32(lcmBuf + 4, 0);
    memcpy(lcmBuf + 8, setptChannel, strlen(setptChannel));

    if (setpt_t_encode(lcmBuf + extraLen, 0, lcmLen, &pt) != lcmLen)
    {
        log.warn("failed to encode setpoint");
    }
    else
    {
        int serLen = (2 * (lcmLen + extraLen)) + 3;
        serBuf = new unsigned char[serLen];

        serLen = data_link_assemble_packet(lcmBuf, serBuf, lcmLen + extraLen);

        log.info("sending setpoint message");
        port.send((char *)serBuf, serLen);
    }

    delete[] lcmBuf;
    delete[] serBuf;
    free(setptChannel);
}

void PhysicalController::sendDji(float roll, float pitch, float yaw, float thrust)
{
    dji_t dji;
    unsigned char *lcmBuf{nullptr};
    unsigned char *serBuf{nullptr};
    int lcmLen;
    char *djiChannel = strdup("DJI");
    int extraLen;

    dji.roll = roll;
    dji.pitch = pitch;
    dji.yaw = yaw;
    dji.thrust = thrust;

    lcmLen = dji_t_encoded_size(&dji);
    extraLen = 8 + strlen(djiChannel) + 1;

    lcmBuf = new unsigned char[lcmLen + extraLen];
    memset(lcmBuf, 0, lcmLen + extraLen);

    encode_u32(lcmBuf, MAGIC_LCM2);
    encode_u32(lcmBuf + 4, 0);
    memcpy(lcmBuf + 8, djiChannel, strlen(djiChannel));

    if (dji_t_encode(lcmBuf + extraLen, 0, lcmLen, &dji) != lcmLen)
    {
        log.warn("failed to encode dji");
    }
    else
    {
        int serLen = (2 * (lcmLen + extraLen)) + 3;
        serBuf = new unsigned char[serLen];

        serLen = data_link_assemble_packet(lcmBuf, serBuf, lcmLen + extraLen);

        log.info("sending dji message");
        port.send((char *)serBuf, serLen);
    }

    delete[] lcmBuf;
    delete[] serBuf;
    free(djiChannel);
}

void PhysicalController::rotate(double dr, double dp, double dy)
{
    log.warn() << "this function is *not* implemented, so I'm not "
               << "applying the following RPY tuple: (" << dr << "," << dp << "," << dy << ")"
               << commit;
}

void PhysicalController::takeoff() { sendSetpoint(0, 0, 0, 0, SETPT_T_TAKEOFF); }
void PhysicalController::land() { sendSetpoint(0, 0, 0, 0, SETPT_T_LAND); }
void PhysicalController::process(DataLink &dlink)
{
    //	data_frame_t *frame;

    //	frame = data_frame_create(512);

    while (running)
    {
        unsigned char *serBuf{nullptr};
        int serLen = 0;

        serBuf = new unsigned char[512];
        try
        {
            serLen = static_cast<int>(port.receive((char *)serBuf, 512));
        }
        catch (std::system_error e)
        {
            log.error() << "Reading from serial port failed " << e.what() << commit;
            running = false;
            break;
        }

        for (int i = 0; i < serLen; i++)
        {
            dlink.processRecv(serBuf[i]);
        }

        delete[] serBuf;
    }
}

void PhysicalController::setFeedbackHandler(function<void(const feedback_t *)> func)
{
    handler = func;
}

void PhysicalController::stop()
{
    port.stop();
    running = false;
}

void PhysicalController::disconnect() noexcept { port.disconnect(); }
