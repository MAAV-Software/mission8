#define BOOST_TEST_MODULE LidarTest
/**
 * Tests for both BaseState and Kalman State
 */

#include <cmath>
#include <vector>

#include <Eigen/Eigen>
#include <boost/test/unit_test.hpp>
#include <sophus/so3.hpp>

#include <gnc/constants.hpp>
#include <gnc/kalman/updates/lidar_update.hpp>

using namespace boost::unit_test;
using namespace Eigen;

using namespace maav::gnc::state;
using namespace maav::gnc::kalman;
using namespace maav::gnc;

BOOST_AUTO_TEST_CASE(RunTest)
{
	KalmanState state(1000);
	state.attitude() = Sophus::SO3d(Quaterniond{1, 0, 0, 0});
	state.position() = {0, 1, -1};
	state.velocity() = Vector3d::Zero();
	state.angular_velocity() = Vector3d::Zero();
	state.acceleration() = Vector3d::Zero();

	measurements::Measurement measurement;
	measurements::ImuMeasurement imu;
	imu.time_usec = 1000;
	imu.acceleration = {0, 0, -maav::gnc::constants::STANDARD_GRAVITY};
	imu.angular_rates = Vector3d::Zero();
	imu.magnetometer = {0, 0, 1};

	measurement.imu = std::make_shared<measurements::ImuMeasurement>(imu);

	measurements::LidarMeasurement lidar;
	lidar.distance = 1;
	lidar.time_usec = 1000;
	measurement.lidar = std::make_shared<measurements::LidarMeasurement>(lidar);

	History::Snapshot snapshot(state, measurement);

	LidarUpdate update;
	update(snapshot);
}
