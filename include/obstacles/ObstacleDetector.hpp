#ifndef OBSTACLEDETECTOR_HPP
#define OBSTACLEDETECTOR_HPP

//#include "Obstacle.hpp"
#include <Eigen/Core>
#include <vector>

// use the MAAV namespace
namespace maav
{
/**
 * @brief Detects obstacles in laser data
 * @author Daniel Woodworth (dascwo)
 *
 * @details [incomplete]
 */
class ObstacleDetector
{
   public:
	/**
	 * @brief Default
	 */
	ObstacleDetector();

	/**
	 * @brief Takes in laser's starting angle and step between angles
	 * @param startAngle laser's starting angle
	 * @param angleStep laser's step between measurements
	 */
	ObstacleDetector(float startAngle, float angleStep);

	/**
	 * @brief The upper margin for "far" values
	 * @details Any distance measurements that are less than this value are
	 * interpreted as being out of the sensor's range, and thus "far"
	 */
	float far_margin{0.4};

	/**
	 * @brief The minimum difference between two points to be considered a jump
	 * @details If two distance measurements vary by more than this value, they
	 * are assumed to be a boundary between obstacles
	 */
	float jump_margin{0.5};

	/**
	 * @brief The data returned for each obstacle by detect
	 */
	struct obstacle_data
	{
		/**
		 * @brief The midpoint of the obstacle
		 */
		Eigen::Vector2d midpoint;

		/**
		 * @brief The approximate width of the obstacle
		 */
		float width;
	};

	/**
	 * @brief Detects obstacles
	 * @param laser_data A vector containing radial distances as given by
	 * LaserReader
	 * @param start The start angle of the laser_data values
	 * @param step The angular distance between the laser_data values
	 */
	std::vector<obstacle_data> detect(const std::vector<float>& laser_data);

   private:
	float start;
	float step;
};
}

#endif
