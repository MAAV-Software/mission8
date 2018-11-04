#include "common/math/FORMapper.hpp"
#include <cassert>
#include <iostream>
#include <string>

using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Quaterniond;
using YAML::Node;
using std::string;
using std::to_string;

FORMapper::FORMapper(const Node &config, int cameraID)
{
    // Create the camera query for yaml
    string cameraName = "Camera" + to_string(cameraID);

    // Store the private variables from the config file
    fillMatrix3d(config["FORMapper"][cameraName]["K"], K);
    fillMatrix3d(config["FORMapper"][cameraName]["Kinv"], Kinv);
    fillMatrix3d(config["FORMapper"][cameraName]["Rc2i"], Rc2i);
    fillMatrix3d(config["FORMapper"][cameraName]["Ri2c"], Ri2c);
    fillVector3d(config["FORMapper"][cameraName]["Tc"], Tc);
    fillVector3d(config["FORMapper"][cameraName]["principalPt"], principPt);
}

void FORMapper::fillMatrix3d(const Node &node, Matrix3d &m)
{
    for (size_t row = 0; row < node.size(); ++row)
    {
        for (size_t col = 0; col < node[row].size(); ++col)
            m(row, col) = node[row][col].as<double>();
    }
}

void FORMapper::fillVector3d(const Node &node, Vector3d &v)
{
    for (size_t i = 0; i < node.size(); ++i) v(i) = node[i].as<double>();
}

Vector3d FORMapper::formapperWithDepth(
    const Vector3d &px, const double depth, const Vector3d &x, const Quaterniond &q)
{
    // rotation from imu/body to world as defined by quaternion
    Matrix3d Ri2w = q.toRotationMatrix();

    // NOTE: Order of operations matter!
    // use height in world to camera transform to get scale factor (z coord in pc)
    // Vector3d pw(0, 0, x.z());
    // Vector3d pc = (Ri2c * (Ri2w.transpose() * pw)) + Tc;

    // return projection of pixel coordinate into the world frame
    return (Ri2w * (Rc2i * ((depth * Kinv * px) - Tc))) + x;
    //	return (Rc2i * ((depth * Kinv * px) - Tc)); <- THIS IS WRONG!
}

Vector3d FORMapper::operator()(const Vector3d &px, const Vector3d &x, const Quaterniond &q)
{
    // rotation from imu/body to world as defined by quaternion
    Matrix3d Ri2w = q.toRotationMatrix();

    // NOTE: Order of operations matter!
    // use height in world to camera transform to get scale factor (z coord in pc)
    Vector3d pw(0, 0, x.z());
    Vector3d pc = (Ri2c * (Ri2w.transpose() * pw)) + Tc;

    // return projection of pixel coordinate into the world frame
    return (Ri2w * (Rc2i * ((pc.z() * Kinv * px) - Tc))) + x;
}
