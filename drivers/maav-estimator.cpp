#include <Eigen/Eigen>
#include <common/messages/MsgChannels.hpp>
#include <common/messages/imu_t.hpp>
#include <common/messages/lidar_t.hpp>
#include <common/messages/plane_fit_t.hpp>
#include <common/messages/state_t.hpp>
#include <common/utils/GetOpt.hpp>
#include <common/utils/ZCMHandler.hpp>
#include <gnc/utils/zcm_conversion.hpp>
#include <gnc/estimator.hpp>
#include <gnc/measurements/Measurement.hpp>
#include <gnc/measurements/Imu.hpp>
#include <csignal>
#include <zcm/zcm-cpp.hpp>
#include <Eigen/Core>

using maav::HEIGHT_LIDAR_CHANNEL;
using maav::IMU_CHANNEL;
using maav::PLANE_FIT_CHANNEL;
using maav::STATE_CHANNEL;
using maav::gnc::Estimator;
using maav::gnc::measurements::MeasurementSet;
using maav::gnc::measurements::ImuMeasurement;
using maav::gnc::measurements::LidarMeasurement;
using maav::gnc::measurements::PlaneFitMeasurement;
using maav::gnc::State;
using maav::gnc::convert_state;

using namespace std;

sig_atomic_t KILL = 0;


void sighandler(int sig) {
	KILL = sig;
}

int main(int argc, char** argv)
{
	signal(SIGINT, sighandler);
	signal(SIGABRT, sighandler);

	GetOpt gopt;
	gopt.addBool('h', "help", false, "This message");
	gopt.addString('c', "config", "", "Path to config.");

	if (!gopt.parse(argc, argv, 1) || gopt.getBool("help")) {
		cout << "Usage: " << argv[0] << " [options]" << endl;
		gopt.printHelp();
		return 1;
	}

	// Init ZCM
	zcm::ZCM zcm{"ipc"};

	ZCMHandler<imu_t> imu_handler;
	ZCMHandler<lidar_t> lidar_handler;
	ZCMHandler<plane_fit_t> plane_fit_handler;

	zcm.subscribe(IMU_CHANNEL, &ZCMHandler<imu_t>::recv, &imu_handler);
	zcm.subscribe(HEIGHT_LIDAR_CHANNEL, &ZCMHandler<lidar_t>::recv, &lidar_handler);
	zcm.subscribe(PLANE_FIT_CHANNEL, &ZCMHandler<plane_fit_t>::recv, &plane_fit_handler);
	// The Kalman filter also updates with data from itself, but this is taken care of
	// within the Kalman filter.

	//KalmanInitializer init;
	//init.config_file = gopt.getString("config");
	Estimator estimator;

	MeasurementSet set;

	// Main Loop
	KILL = false;
	while (!KILL) {

		// Kalman Update Rate = IMU's Update rate,
		// so we ONLY update the Kalman Filter when the IMU Updates.
		if (imu_handler.ready()) {
			const imu_t msg = imu_handler.msg();
			imu_handler.pop();

			ImuMeasurement imu;
			imu.acceleration = Eigen::Vector3d(msg.acceleration[0], msg.acceleration[1], msg.acceleration[2]);
			imu.magnetometer = Eigen::Vector3d(msg.angular_rates[0], msg.angular_rates[1], msg.angular_rates[2]);
			imu.time_usec = msg.utime;
			set.imu = std::make_shared<ImuMeasurement>(imu); 
			
			const State& state = estimator.add_measurement_set(set);
			state_t zcm_state = convert_state(state);
        	zcm.publish(STATE_CHANNEL, &zcm_state);
		}
		if (lidar_handler.ready()) {
			const lidar_t msg = lidar_handler.msg();
			lidar_handler.pop();

			LidarMeasurement lidar;
			lidar.distance = msg.distance;
			lidar.time_usec = msg.utime;

			set.lidar = std::make_shared<LidarMeasurement>(lidar);
		}
		if (plane_fit_handler.ready()) {
			const plane_fit_t msg = plane_fit_handler.msg();
			plane_fit_handler.pop();

			PlaneFitMeasurement planeFit;
			planeFit.height = msg.z;
			planeFit.vertical_speed = msg.z_dot;
			planeFit.roll = msg.roll;
			planeFit.pitch = msg.pitch;
			planeFit.time_usec = msg.utime;
			
			set.plane_fit = std::make_shared<PlaneFitMeasurement>(planeFit);
		}
	}

	return 0;
}
