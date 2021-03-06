// clang-format off
/**
 * 	AUTOGENERATED FILE!! DO NOT EDIT!!!!
 */


#include "ZcmLoop.hpp"
#include "AbstractData.hpp"
#include "DataDict.hpp"
#include "ZcmConversion.hpp"
#include "GaussianData.hpp"
#include <chrono>
#include <thread>
#include <memory>
#include <Eigen/Dense>
#include <common/messages/MsgChannels.hpp>
#include <common/utils/ZCMHandler.hpp>
#include <common/messages/state_t.hpp>
#include <common/messages/imu_t.hpp>
#include <common/messages/imu_t.hpp>
#include <common/messages/lidar_t.hpp>
#include <common/messages/plane_fit_t.hpp>
#include <common/messages/global_update_t.hpp>
#include <common/messages/groundtruth_inertial_t.hpp>
#include <common/messages/groundtruth_imu_t.hpp>
#include <common/messages/groundtruth_world_t.hpp>
#include <common/messages/groundtruth_slamdrift_t.hpp>
#include <common/messages/pid_error_t.hpp>

using namespace std::chrono;

void ZcmLoop::run()
{
	ZCMHandler<pid_error_t> PID_ERROR_handler;
	zcm.subscribe(maav::PID_ERROR_CHANNEL, &ZCMHandler<pid_error_t>::recv, &PID_ERROR_handler);
	ZCMHandler<imu_t> SIM_IMU_handler;
	zcm.subscribe(maav::SIM_IMU_CHANNEL, &ZCMHandler<imu_t>::recv, &SIM_IMU_handler);
	ZCMHandler<groundtruth_slamdrift_t> GT_SLAMDRIFT_handler;
	zcm.subscribe(maav::GT_SLAMDRIFT_CHANNEL, &ZCMHandler<groundtruth_slamdrift_t>::recv, &GT_SLAMDRIFT_handler);
	ZCMHandler<groundtruth_world_t> GT_WORLD_handler;
	zcm.subscribe(maav::GT_WORLD_CHANNEL, &ZCMHandler<groundtruth_world_t>::recv, &GT_WORLD_handler);
	ZCMHandler<imu_t> IMU_handler;
	zcm.subscribe(maav::IMU_CHANNEL, &ZCMHandler<imu_t>::recv, &IMU_handler);
	ZCMHandler<lidar_t> HEIGHT_LIDAR_handler;
	zcm.subscribe(maav::HEIGHT_LIDAR_CHANNEL, &ZCMHandler<lidar_t>::recv, &HEIGHT_LIDAR_handler);
	ZCMHandler<global_update_t> GLOBAL_UPDATE_handler;
	zcm.subscribe(maav::GLOBAL_UPDATE_CHANNEL, &ZCMHandler<global_update_t>::recv, &GLOBAL_UPDATE_handler);
	ZCMHandler<state_t> STATE_handler;
	zcm.subscribe(maav::STATE_CHANNEL, &ZCMHandler<state_t>::recv, &STATE_handler);
	ZCMHandler<plane_fit_t> PLANE_FIT_handler;
	zcm.subscribe(maav::PLANE_FIT_CHANNEL, &ZCMHandler<plane_fit_t>::recv, &PLANE_FIT_handler);
	ZCMHandler<groundtruth_inertial_t> GT_INERTIAL_handler;
	zcm.subscribe(maav::GT_INERTIAL_CHANNEL, &ZCMHandler<groundtruth_inertial_t>::recv, &GT_INERTIAL_handler);
	ZCMHandler<groundtruth_imu_t> GT_IMU_handler;
	zcm.subscribe(maav::GT_IMU_CHANNEL, &ZCMHandler<groundtruth_imu_t>::recv, &GT_IMU_handler);

	zcm.start();

	while (RUNNING)
	{
		while(PID_ERROR_handler.ready()) {
			double time = elapsedTime(static_cast<double>(PID_ERROR_handler.msg().utime) / 1e6);
			dict_->dict["PID_ERROR_pos_error"]->addData(std::move(convertVector(time, PID_ERROR_handler.msg().pos_error)));
			dict_->dict["PID_ERROR_vel_error"]->addData(std::move(convertVector(time, PID_ERROR_handler.msg().vel_error)));
			dict_->dict["PID_ERROR_thrust"]->addData(std::move(convertVector(time, PID_ERROR_handler.msg().thrust)));
			dict_->dict["PID_ERROR_yaw_error"]->addData(std::move(convertVector(time, PID_ERROR_handler.msg().yaw_error)));
			dict_->dict["PID_ERROR_pitch"]->addData(std::move(convertVector(time, PID_ERROR_handler.msg().pitch)));
			dict_->dict["PID_ERROR_roll"]->addData(std::move(convertVector(time, PID_ERROR_handler.msg().roll)));
			PID_ERROR_handler.pop();
		}
		while(SIM_IMU_handler.ready()) {
			double time = elapsedTime(static_cast<double>(SIM_IMU_handler.msg().utime) / 1e6);
			dict_->dict["SIM_IMU_acceleration"]->addData(std::move(convertVector(time, SIM_IMU_handler.msg().acceleration)));
			dict_->dict["SIM_IMU_angular_rates"]->addData(std::move(convertVector(time, SIM_IMU_handler.msg().angular_rates)));
			dict_->dict["SIM_IMU_magnetometer"]->addData(std::move(convertVector(time, SIM_IMU_handler.msg().magnetometer)));
			SIM_IMU_handler.pop();
		}
		while(GT_SLAMDRIFT_handler.ready()) {
			double time = elapsedTime(static_cast<double>(GT_SLAMDRIFT_handler.msg().utime) / 1e6);
			dict_->dict["GT_SLAMDRIFT_attitude_drift"]->addData(std::move(convertQuaternion(time, GT_SLAMDRIFT_handler.msg().attitude_drift)));
			dict_->dict["GT_SLAMDRIFT_position_drift"]->addData(std::move(convertVector(time, GT_SLAMDRIFT_handler.msg().position_drift)));
			GT_SLAMDRIFT_handler.pop();
		}
		while(GT_WORLD_handler.ready()) {
			double time = elapsedTime(static_cast<double>(GT_WORLD_handler.msg().utime) / 1e6);
			dict_->dict["GT_WORLD_gravity"]->addData(std::move(convertVector(time, GT_WORLD_handler.msg().gravity)));
			dict_->dict["GT_WORLD_magnetic_field"]->addData(std::move(convertVector(time, GT_WORLD_handler.msg().magnetic_field)));
			GT_WORLD_handler.pop();
		}
		while(IMU_handler.ready()) {
			double time = elapsedTime(static_cast<double>(IMU_handler.msg().utime) / 1e6);
			dict_->dict["IMU_acceleration"]->addData(std::move(convertVector(time, IMU_handler.msg().acceleration)));
			dict_->dict["IMU_angular_rates"]->addData(std::move(convertVector(time, IMU_handler.msg().angular_rates)));
			dict_->dict["IMU_magnetometer"]->addData(std::move(convertVector(time, IMU_handler.msg().magnetometer)));
			IMU_handler.pop();
		}
		while(HEIGHT_LIDAR_handler.ready()) {
			double time = elapsedTime(static_cast<double>(HEIGHT_LIDAR_handler.msg().utime) / 1e6);
			dict_->dict["HEIGHT_LIDAR_distance"]->addData(std::move(convertVector(time, HEIGHT_LIDAR_handler.msg().distance)));
			HEIGHT_LIDAR_handler.pop();
		}
		while(GLOBAL_UPDATE_handler.ready()) {
			double time = elapsedTime(static_cast<double>(GLOBAL_UPDATE_handler.msg().utime) / 1e6);
			dict_->dict["GLOBAL_UPDATE_position"]->addData(std::move(convertVector(time, GLOBAL_UPDATE_handler.msg().position)));
			dict_->dict["GLOBAL_UPDATE_attitude"]->addData(std::move(convertQuaternion(time, GLOBAL_UPDATE_handler.msg().attitude)));
			GLOBAL_UPDATE_handler.pop();
		}
		while(STATE_handler.ready()) {
			double time = elapsedTime(static_cast<double>(STATE_handler.msg().utime) / 1e6);
			Eigen::MatrixXd covariance = convertMatrix(STATE_handler.msg().covariance);
			dict_->dict["STATE_attitude"]->addData(std::move(convertQuaternion(time, STATE_handler.msg().attitude)));
			std::shared_ptr<GaussianData> STATE_accel_biases = std::dynamic_pointer_cast<GaussianData>(dict_->dict["STATE_accel_biases"]);
			STATE_accel_biases->addData(std::move(convertVector(time, STATE_handler.msg().accel_biases)), covariance.block<3,3>(12,12));
			std::shared_ptr<GaussianData> STATE_velocity = std::dynamic_pointer_cast<GaussianData>(dict_->dict["STATE_velocity"]);
			STATE_velocity->addData(std::move(convertVector(time, STATE_handler.msg().velocity)), covariance.block<3,3>(6,6));
			dict_->dict["STATE_angular_velocity"]->addData(std::move(convertVector(time, STATE_handler.msg().angular_velocity)));
			dict_->dict["STATE_gravity"]->addData(std::move(convertVector(time, STATE_handler.msg().gravity)));
			std::shared_ptr<GaussianData> STATE_position = std::dynamic_pointer_cast<GaussianData>(dict_->dict["STATE_position"]);
			STATE_position->addData(std::move(convertVector(time, STATE_handler.msg().position)), covariance.block<3,3>(3,3));
			std::shared_ptr<GaussianData> STATE_gyro_biases = std::dynamic_pointer_cast<GaussianData>(dict_->dict["STATE_gyro_biases"]);
			STATE_gyro_biases->addData(std::move(convertVector(time, STATE_handler.msg().gyro_biases)), covariance.block<3,3>(9,9));
			dict_->dict["STATE_acceleration"]->addData(std::move(convertVector(time, STATE_handler.msg().acceleration)));
			dict_->dict["STATE_magnetic_field"]->addData(std::move(convertVector(time, STATE_handler.msg().magnetic_field)));
			STATE_handler.pop();
		}
		while(PLANE_FIT_handler.ready()) {
			double time = elapsedTime(static_cast<double>(PLANE_FIT_handler.msg().utime) / 1e6);
			dict_->dict["PLANE_FIT_pitch"]->addData(std::move(convertVector(time, PLANE_FIT_handler.msg().pitch)));
			dict_->dict["PLANE_FIT_z_dot"]->addData(std::move(convertVector(time, PLANE_FIT_handler.msg().z_dot)));
			dict_->dict["PLANE_FIT_z"]->addData(std::move(convertVector(time, PLANE_FIT_handler.msg().z)));
			dict_->dict["PLANE_FIT_roll"]->addData(std::move(convertVector(time, PLANE_FIT_handler.msg().roll)));
			PLANE_FIT_handler.pop();
		}
		while(GT_INERTIAL_handler.ready()) {
			double time = elapsedTime(static_cast<double>(GT_INERTIAL_handler.msg().utime) / 1e6);
			dict_->dict["GT_INERTIAL_attitude"]->addData(std::move(convertQuaternion(time, GT_INERTIAL_handler.msg().attitude)));
			dict_->dict["GT_INERTIAL_position"]->addData(std::move(convertVector(time, GT_INERTIAL_handler.msg().position)));
			dict_->dict["GT_INERTIAL_velocity"]->addData(std::move(convertVector(time, GT_INERTIAL_handler.msg().velocity)));
			dict_->dict["GT_INERTIAL_angular_velocity"]->addData(std::move(convertVector(time, GT_INERTIAL_handler.msg().angular_velocity)));
			dict_->dict["GT_INERTIAL_acceleration"]->addData(std::move(convertVector(time, GT_INERTIAL_handler.msg().acceleration)));
			GT_INERTIAL_handler.pop();
		}
		while(GT_IMU_handler.ready()) {
			double time = elapsedTime(static_cast<double>(GT_IMU_handler.msg().utime) / 1e6);
			dict_->dict["GT_IMU_accel_bias"]->addData(std::move(convertVector(time, GT_IMU_handler.msg().accel_bias)));
			dict_->dict["GT_IMU_gyro_bias"]->addData(std::move(convertVector(time, GT_IMU_handler.msg().gyro_bias)));
			GT_IMU_handler.pop();
		}
		std::this_thread::sleep_for(33ms);
	}

	zcm.stop();
}
