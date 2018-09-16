#ifndef MESSAGEHANDLER_HPP_
#define MESSAGEHANDLER_HPP_

#include "../lcmlite.h"
#include "lidar_t.h"
#include "imu_t.h"
#include "emergency_t.h"
#include <zcm/zcm-cpp.hpp>

#include "common/utils/TimeSync.hpp"

#define DEFAULT_ALPHA1 0.0006
#define DEFAULT_ALPHA2 0.0006

struct MessageHandler
{
	lidar_t lidar;
	imu_t imu;
	emergency_t ems;
	int64_t offset;

	zcm::ZCM * zcm;
	TimeSync ts;

	MessageHandler(zcm::ZCM * zcm_in, double alpha1 = DEFAULT_ALPHA1, double alpha2 = DEFAULT_ALPHA2);
};

void callback(lcmlite_t *lcm, const char *channel, const void *buf,
			  int buf_len, void *user);

#endif // MessageHandler.hpp
