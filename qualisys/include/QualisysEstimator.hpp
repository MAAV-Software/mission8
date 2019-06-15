#ifndef QUALISYS_ESTIMATOR_HPP
#define QUALISYS_ESTIMATOR_HPP

#include <Eigen/Core>

#include "State.hpp"
#include "KalmanFilter.hpp"

namespace qualisys
{

/**
 * @brief	Estimates full pose, linear velocity, and angular velocity from 
 *		 	Qualisys pose tracking.
 */
class QualisysEstimator
{
public:
	
	/**
	 * @brief Constructs a default QualisysEstimator.
	 * @details Using default KalmanFilter constructor (see KalmanFilter.hpp) 
	 *			and Rprev = I_3x3, constructs a default QualisysEstimator.
	 */
	QualisysEstimator();
	
	/**
	 * @brief Constructs a QualisysEstimator with given parameters
	 * @details Constructs a QualisysEstimator using max_accel for process noise
	 *			and lin_meas_noise for measurement noise. Rprev is set to I_3x3.
	 *			Internally, the KalmanFilter will scale process noise by dt 
	 *			accordingly when calling operator() (see KalmanFilter.hpp).
	 * 			The suggested values for parameters below come from 
	 *			https://github.com/KumarRobotics/qualisys/blob/master/src/QualisysOdom.cpp.
	 * @param P	Initial covariance of the [pos, vel] Kalman filter. Suggested 
	 *			value is 0.01 * I_3x3.
	 * @param max_accel	Maximum linear acceleartion. Suggested value is 5.0.
	 *					This will be used to initalize Q as max_accel * I_6x6.
	 * @param lin_meas_noise 	3D position measurement noise of the Qualisys 
	 *							tracker. Suggested value is 0.0001.
	 */
	QualisysEstimator(const KalmanFilter::StateCov& P, float max_accel, float lin_meas_noise);
	
	/**
	 * @brief Runs one iteration of the Qualisys state estimator.
	 * @details Given dt, 3D position, and 3D orientation measurments, this 
	 *			function runs one iteration of the underlying Kalman Filter for 
	 *			estimating linear position and velocity and peforms a 1-step 
	 *			differentiation of the rotation matrices to compute the angular
	 *			velocities. Updates Rprev internally. Rotation differentiation 
	 *			is only performed if dt > 1e-6.
	 * @param dt 	Time [s] since last estimate (call to this function)
	 * @param pos 	3D position in world frame from the Qualisys tracker
	 * @param rot	Body-to-world rotation from the Qualisys tracker
	 */
	State operator()(float dt, const Eigen::Vector3f& pos, const Eigen::Matrix3f& rot);

private:
	Eigen::Matrix3f Rprev_;
	KalmanFilter kf_;
};

}

#endif /* QualisysEstimator */
