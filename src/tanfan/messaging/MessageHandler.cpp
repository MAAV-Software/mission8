#include "tanfan/messaging/MessageHandler.hpp"
#include <cstring>
#include <iostream>

#include <zcm/zcm-cpp.hpp>
#include "common/utils/TimeSync.hpp"
#include "common/utils/ZCMHandler.hpp"

namespace zcm
{
#include "common/messages/emergency_t.hpp"
#include "common/messages/imu_t.hpp"
#include "common/messages/lidar_t.hpp"
}

#include "common/messages/MsgChannels.hpp"

using zcm::ZCM;
using std::strncmp;

MessageHandler::MessageHandler(zcm::ZCM *zcm_in, double alpha1, double alpha2)
    : zcm(zcm_in), ts(alpha1, alpha2)
{
    offset = 0;

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
    int64_t recvTimeStamp = std::chrono::duration_cast<std::chrono::microseconds>(
                                std::chrono::system_clock::now().time_since_epoch())
                                .count();

    MessageHandler *mh = (MessageHandler *)user;
    if (strncmp(channel, "LID", 3) == 0)
    {
        lidar_t_decode(buf, 0, buf_len, &(mh->lidar));
        zcm::lidar_t zcmLidar;

        zcmLidar.dist = mh->lidar.dist;
        zcmLidar.vel = mh->lidar.vel;
        // Add time sync system!
        // zcmLidar.time = mh->lidar.time + mh->offset;
        // zcmLidar.time =
        // std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        zcmLidar.time = mh->ts.reclock(mh->lidar.time, recvTimeStamp);

        mh->zcm->publish("LID", &zcmLidar);
    }
    else if (strncmp(channel, "IMU", 3) == 0)
    {
        imu_t_decode(buf, 0, buf_len, &(mh->imu));
        zcm::imu_t zcmImu;

        zcmImu.refYaw = mh->imu.refYaw;
        zcmImu.gAccX = mh->imu.gAccX;
        zcmImu.gAccY = mh->imu.gAccZ;
        zcmImu.gAccZ = mh->imu.gAccZ;
        zcmImu.AccX = mh->imu.AccX;
        zcmImu.AccY = mh->imu.AccY;
        zcmImu.AccZ = mh->imu.AccZ;
        zcmImu.AngRateX = mh->imu.AngRateX;
        zcmImu.AngRateY = mh->imu.AngRateY;
        zcmImu.AngRateZ = mh->imu.AngRateZ;
        zcmImu.MagX = mh->imu.MagX;
        zcmImu.MagY = mh->imu.MagY;
        zcmImu.MagZ = mh->imu.MagZ;
        for (int i = 0; i < 9; ++i)
        {
            zcmImu.M[i] = mh->imu.M[i];
        }
        // zcmImu.time = mh->imu.time + mh->offset;
        zcmImu.time = mh->ts.reclock(mh->imu.time, recvTimeStamp);
        zcmImu.Timer = mh->imu.Timer;
        zcmImu.GyroBiasX = mh->imu.GyroBiasX;
        zcmImu.GyroBiasY = mh->imu.GyroBiasY;
        zcmImu.GyroBiasZ = mh->imu.GyroBiasZ;
        zcmImu.AccBiasX = mh->imu.AccBiasX;
        zcmImu.AccBiasY = mh->imu.AccBiasY;
        zcmImu.AccBiasZ = mh->imu.AccBiasZ;

        mh->zcm->publish("IMU", &zcmImu);
    }
    else if (strncmp(channel, "EMS", 3) == 0)
    {
        emergency_t_decode(buf, 0, buf_len, &(mh->ems));
        zcm::emergency_t zcmEms;
        zcmEms.status = mh->ems.status;
        // zcmEms.time =
        // std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        zcmEms.time = mh->ts.reclock(mh->ems.time, recvTimeStamp);

        mh->zcm->publish("EMS", &zcmEms);
    }
}
