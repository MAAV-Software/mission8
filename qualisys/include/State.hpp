#ifndef QUALISYS_STATE_HPP
#define QUALISYS_STATE_HPP

#include <Eigen/Core>

namespace qualisys
{

/**
 * @brief Represents pose and velocity state estimate from Qualisys system.
 */
struct State
{
	static const int num_lin_states = 6; // number of linear states [x, xdot]
	static const int num_lin_meas   = 3; // number of linear measurements [x]
	
	Eigen::Vector3f pos;			// world-frame position
	Eigen::Vector3f vel;			// world-frame velocity
	Eigen::Matrix3f rot;			// body-to-world rotation
	Eigen::Vector3f ang_vel;		// body-frame angular velocities
	Eigen::Matrix3f pos_cov;		// cov(pos, pos)
	Eigen::Matrix3f vel_cov;		// cov(vel, vel)
	Eigen::Matrix3f pos_vel_cov;	// cov(pos, vel) = cov(vel, pos)^T

	/**
	 * @brief Default constructor for intializing vectors to 0 and matrices to 
	 *		  Identity (for rotation and covariances).
	 */
	State();
};

}
#endif /* State.hpp */
