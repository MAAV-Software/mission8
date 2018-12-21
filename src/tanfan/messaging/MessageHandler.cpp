#include "tanfan/messaging/MessageHandler.hpp"
#include <chrono>
#include <cstring>
#include <iostream>

#include <zcm/zcm-cpp.hpp>
#include "common/utils/TimeSync.hpp"
#include "common/utils/ZCMHandler.hpp"

namespace zcm
{
// #include "common/messages/emergency_t.hpp"
#include "common/messages/imu_t.hpp"
#include "common/messages/lidar_t.hpp"
}

#include "common/messages/MsgChannels.hpp"

using zcm::ZCM;
using std::strncmp;

using namespace std::chrono;
using namespace std::chrono_literals;

MessageHandler::MessageHandler(zcm::ZCM *zcm_in, double alpha1, double alpha2)
    : zcm(zcm_in), ts(alpha1, alpha2)
{
    lidar.dist = 0;
    lidar.vel = 0;
    lidar.time = 0;

    imu.refYaw = 0;
    imu.gAccX = 0;
    imu.gAccY = 0;
    imu.gAccZ = 0;
    imu.AccX = 0;
    imu.AccY = 0;
    imu.AccZ = 0;
    imu.AngRateX = 0;
    imu.AngRateY = 0;
    imu.AngRateZ = 0;
    imu.MagX = 0;
    imu.MagY = 0;
    imu.MagZ = 0;
    for (int i = 0; i < 9; ++i)
    {
        imu.M[i] = 0;
    }
    imu.time = 0;
    imu.Timer = 0;
    imu.GyroBiasX = 0;
    imu.GyroBiasY = 0;
    imu.GyroBiasZ = 0;
    imu.AccBiasX = 0;
    imu.AccBiasY = 0;
    imu.AccBiasZ = 0;

    ems.status = 0;
}

void callback(lcmlite_t *lcm, const char *channel, const void *buf, int buf_len, void *user)
{
    MessageHandler *mh = (MessageHandler *)user;
    if (strncmp(channel, "LID", 3) == 0)
    {
        lidar_t_decode(buf, 0, buf_len, &(mh->lidar));
        zcm::lidar_t zcmLidar;

        zcmLidar.distance = mh->lidar.dist;

        std::cout << "LIdar distance: " << zcmLidar.distance << std::endl;

        zcmLidar.utime =
            duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();

        mh->zcm->publish("LID", &zcmLidar);
    }
    else if (strncmp(channel, "IMU", 3) == 0)
    {
        imu_t_decode(buf, 0, buf_len, &(mh->imu));
        zcm::imu_t zcmImu;

        zcmImu.acceleration[0] = mh->imu.AccX;
        zcmImu.acceleration[1] = mh->imu.AccY;
        zcmImu.acceleration[2] = mh->imu.AccZ;
        zcmImu.angular_rates[0] = mh->imu.AngRateX;
        zcmImu.angular_rates[1] = mh->imu.AngRateY;
        zcmImu.angular_rates[2] = mh->imu.AngRateZ;
        zcmImu.magnetometer[0] = mh->imu.MagX;
        zcmImu.magnetometer[1] = mh->imu.MagY;
        zcmImu.magnetometer[2] = mh->imu.MagZ;

        zcmImu.utime = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();

        mh->zcm->publish("IMU", &zcmImu);
    }
    else if (strncmp(channel, "EMS", 3) == 0)
    {
        // TODO ()
        emergency_t_decode(buf, 0, buf_len, &(mh->ems));
        // zcm::emergency_t zcmEms;
        // zcmEms.status = mh->ems.status;
        // zcmEms.time =
        // std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        // zcmEms.time = mh->ts.reclock(mh->ems.time, recvTimeStamp);

        // mh->zcm->publish("EMS", &zcmEms);
    }
}
