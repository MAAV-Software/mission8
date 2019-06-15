#ifndef QUALISYS_KALMAN_FILTER_HPP
#define QUALISYS_KALMAN_FITLER_HPP

#include <utility>

#include <Eigen/Core>

#include "State.hpp"

namespace qualisys
{

/**
 * @brief Linear Kalman Filter for double integrator under position feedback.
 */
class KalmanFilter
{
public:
	// useful aliases
	using StateVec   = Eigen::Matrix<float, State::num_lin_states, 1>;
	using StateCov   = Eigen::Matrix<float, State::num_lin_states, State::num_lin_states>;
	using MeasVec    = Eigen::Matrix<float, State::num_lin_meas, 1>;
	using MeasCov    = Eigen::Matrix<float, State::num_lin_meas, State::num_lin_meas>;
	using MeasJacob  = Eigen::Matrix<float, State::num_lin_meas, State::num_lin_states>;
	
	/**
	 * @brief Constructs a default linear Kalman Filter
	 * @details Constructs a linear Kalman filter for 3D position and velocity 
	 *			based on the double integrator dynamics model and 3D position 
	 *			measurements. Here the intial values are:
	 *			x = 0, P = 0.0001 * I_6x6, Q = 0.01 * I_6x6, R = 0.0001 * I_3x3
	 */
	KalmanFilter();
	
	/**
	 * @brief Constructs a linear Kalman Filter with given parameters
	 * @details Constructs a linear Kalman filter for 3D position and velocity 
	 *			based on the double integrator dynamics model and 3D position 
	 *			measurements.
	 * @param x	Initial state vector [x, xdot]
	 * @param P Initial state covariance
	 * @param Q Process noise based on double-integrator model (maximum 
	 *			acceleration as noise) before scaling by dt for velocity and 
	 *			0.5 * dt^2 for position. Suggested value is max_accel * I_6x6.
	 *			Actual process noise will be scaled according to dt in 
	 *			operator() when running the filter.
	 * @param R Measurement noise for Qualisys position measurement. Suggested 
	 *			value is 0.0001 * I_3x3.
	 */
	KalmanFilter(const StateVec& x, const StateCov& P, const StateCov& Q, const MeasCov& R);
	
	/**
	 * @brief Runs one iteration of the Kalman Filter
	 * @details Returns a pair of the state vector (first) and covariance 
	 * 			matrix (second) after running one iteration of the Kalman Filter
	 * 			for the given input dt for prediction and measurement for 
	 *			the correction. Dynamics are updated to account for dt and the 
	 *			measurment is applied following the prediction step to produce
	 *			the final result.
	 * @param dt	Time [s] since last estimate (call to this function)
	 * @param z		Measurement vector (3D position)
	 */
	std::pair<StateVec, StateCov> operator()(float dt, const MeasVec& z);
	
private:
	StateVec x_;	// state vector [x, xdot]
	StateCov P_;	// state covariance
	StateCov Q_;	// process noise (same dims as StateCov, so reusing type)
	MeasCov R_;		// measurement covariance
	StateCov A_;	// state dynamics model (same dims as StateCov, so reusing type)
	MeasJacob H_;	// measurement jacobian/model
};

}

#endif /* KalmanFilter.hpp */
