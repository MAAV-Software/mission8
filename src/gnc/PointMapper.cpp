#include <gnc/PointMapper.hpp>

#include <yaml-cpp/yaml.h>
#include <sophus/so3.hpp>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <zcm/zcm-cpp.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <gnc/State.hpp>
#include <gnc/State.hpp>
#include <common/messages/state_t.hpp>
#include <common/messages/MsgChannels.hpp>
#include <gnc/utils/ZcmConversion.hpp>

#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>

using std::unique_lock;
using std::mutex;
using std::cerr;
using std::endl;

using Eigen::Matrix4d;
using Eigen::Vector3d;
using Eigen::Matrix3d;

using pcl::PointXYZ;
using pcl::PointCloud;
using pcl::transformPointCloud;

using maav::STATE_CHANNEL;
using maav::GT_INERTIAL_CHANNEL;
using maav::gnc::State;

maav::gnc::PointMapper::PointMapper(YAML::Node& config, zcm::ZCM &zcm, bool simulator)
    : zcm_ {zcm}, simulator_{simulator}, handler_ {Handler(latest_states_, mtx_, cv_)}
{
    unique_lock<mutex> lck(mtx_);
    // Fill in camera_to_body_ matrix using the camera config
    YAML::Node rotation_matrix, translation_vector;
    if(simulator)
    {
        rotation_matrix = config["sim_rotation"];
        translation_vector = config["sim_translation"];
    }
    else 
    {
        rotation_matrix = config["rotation"];
        translation_vector = config["translation"];
    }
    camera_to_body_ = Matrix4d::Zero(4, 4);
    for (size_t i = 0; i < rotation_matrix.size(); ++i)
    {
        camera_to_body_(i / 3, i % 3) = rotation_matrix[i].as<double>();
    }
    for (size_t i = 0; i < translation_vector.size(); ++i)
    {
        camera_to_body_(i, 3) = translation_vector[i].as<double>();
    }
    camera_to_body_(3, 3) = 1;
    // Start the zcm subscription to the localization states
    if(simulator_) { zcm.subscribe(GT_INERTIAL_CHANNEL, &Handler::handleSim, &handler_); }
    else { zcm.subscribe(STATE_CHANNEL, &Handler::handle, &handler_); }
    zcm.start();
}
// overloaded for legacy without simulator option
maav::gnc::PointMapper::PointMapper(YAML::Node& config, zcm::ZCM &zcm) 
: PointMapper(config, zcm, false) {}


maav::gnc::PointMapper::~PointMapper()
{
    zcm_.stop();
    kill();
}

void maav::gnc::PointMapper::kill()
{
    unique_lock<mutex> lck(mtx_);
    latest_states_.emplace_back();
    cv_.notify_all();
}

State maav::gnc::PointMapper::nearestState(uint64_t utime)
{
    // Wait until there exists at least one state
    unique_lock<mutex> lck(mtx_);
    while (latest_states_.empty())
    {
        cv_.wait(lck);
    }
    // Perform binary search for state nearest requested time
    // Uses int in order to avoid unsigned integer underflows
    int left = 0;
    int right = (int)latest_states_.size() - 1;
    while (left != right)
    {
        int diff = right - left;
        if (diff == 1)
        {
                // Compute left and right squared distances and choose the closer one
                long long left_dist = (utime - latest_states_[(size_t) left].timeUSec());
                left_dist *= left_dist;
                long long right_dist = (utime - latest_states_[(size_t) right].timeUSec());
                right_dist *= right_dist;
                
                if (left_dist < right_dist) {
                    return latest_states_[(size_t) left];
                } 
                else {
                    return latest_states_[(size_t) right];
                } 
        }
        size_t midpoint = (size_t) (left + ((right - left) / 2));
        if (latest_states_[midpoint].timeUSec() < utime)
            left = midpoint;
        else
            right = midpoint;
    }
    return latest_states_[left];
}

Matrix4d maav::gnc::PointMapper::getCameraToWorldMatrix(uint64_t utime)
{
    // get state information
    State nearest = nearestState(utime);
    Matrix3d rotation_matrix = nearest.attitude().matrix();
    Vector3d position_vector = nearest.position();
    Matrix4d body_to_world = Matrix4d::Zero(4, 4);
    for (size_t x = 0; x < 3; ++x)
    {
        for (size_t y = 0; y < 3; ++y)
        {
            body_to_world(x, y) = rotation_matrix(x, y);
        }
    }
    for (size_t y = 0; y < 3; ++y)
    {
        body_to_world(y, 3) = position_vector(y);
    }
    body_to_world(3, 3) = 1;
    return body_to_world * camera_to_body_;
}

// sensor origin required for octomap
Vector3d maav::gnc::PointMapper::getCameraOrigin(uint64_t utime)
{
    // get state information
    State nearest = nearestState(utime);
    Matrix3d rotation_matrix = nearest.attitude().matrix();
    Vector3d position_vector = nearest.position();

    Vector3d sensor_to_body_tr(camera_to_body_(0, 3), camera_to_body_(1, 3), 
        camera_to_body_(2, 3));
    // TODO: is it rotation_matrix or rotation_matrix.inverse()?
    Vector3d sensor_origin = rotation_matrix * sensor_to_body_tr +
                                position_vector;
    return sensor_origin;
}

// transforms point cloud to the global reference frame
PointCloud<PointXYZ>::Ptr maav::gnc::PointMapper::transformCloud(
    PointCloud<PointXYZ>::Ptr cloud, uint64_t utime)
{   
    PointCloud<PointXYZ>::Ptr new_cloud(new PointCloud<PointXYZ>());
    Matrix4d camera_to_world = getCameraToWorldMatrix(utime);
    transformPointCloud(*cloud, *new_cloud, camera_to_world);

    return new_cloud;
}
