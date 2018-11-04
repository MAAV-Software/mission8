#ifndef MAAV_FORMAPPER_HPP
#define MAAV_FORMAPPER_HPP

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <utility>
#include "FieldOfView.hpp"

/**
 * @brief Frame-of-Reference Mapper.
 *
 * @details Translates coordinates from camera coordinates to 3D world
 * coordinates. (0, 0) in camera coordinates is upper left corner of image.
 */
class FORMapper
{
    public:
    /**
     * @brief Constructor. Initializes the matrices from calibration
     */
    FORMapper(const YAML::Node &config, int cameraID);

    /**
     * @brief Calculates the world coordinates given the pixel coordinates and current vehicle state
     * @param px 	Pixel coordinates in homogeneous form (i.e. [u, v, 1])
     * @param x 	Vehicle [x, y, z] state in the world frame
     * @param q 	Quaternion for vehicle orientation
     */
    Eigen::Vector3d formapperWithDepth(const Eigen::Vector3d &px, const double depth,
        const Eigen::Vector3d &x, const Eigen::Quaterniond &q);

    Eigen::Vector3d operator()(
        const Eigen::Vector3d &px, const Eigen::Vector3d &x, const Eigen::Quaterniond &q);

    private:
    Eigen::Matrix3d Rc2i;       // rotation matrix camera to imu
    Eigen::Matrix3d Ri2c;       // rotation matrix camera to imu
    Eigen::Vector3d Tc;         // translation in camera
    Eigen::Matrix3d K;          // K
    Eigen::Matrix3d Kinv;       // Inverse of K
    Eigen::Vector3d principPt;  // principle point

    void fillMatrix3d(const YAML::Node &node, Eigen::Matrix3d &m);
    void fillVector3d(const YAML::Node &node, Eigen::Vector3d &v);
};

#endif  // MAAV_FORMAPPER_HPP
