#ifndef __POINT_MAPPER_HPP___
#define __POINT_MAPPER_HPP___

#include <yaml-cpp/yaml.h>
#include <sophus/so3.hpp>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <zcm/zcm-cpp.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <gnc/State.hpp>
#include <gnc/State.hpp>
#include <common/messages/groundtruth_inertial_t.hpp>
#include <common/messages/state_t.hpp>
#include <common/messages/MsgChannels.hpp>
#include <gnc/utils/ZcmConversion.hpp>

#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>

/* To use, create an instance of the PointMapper class.
 * It will automatically subscribe to STATE_CHANNEL
 * or GT_INERTIAL (for the simulator) and
 * store the last 100 received states, under the assumption
 * that states are sent at 100hz. The intention is to hold a
 * queue (but still needing the functionality of a deque for
 * random access) of the states we were in in the last second.
 * All that one needs to do to use it is pass in the utime in
 * order to access to closest state for any of its public
 * functions. Relies upon correct camera to body extrinsics
 * specified in the camera config that is passed into the
 * constructor to work correctly.
*/

namespace maav::gnc
{

// Maps points from the camera frame point cloud
// to the world frame. Also returns the matrix
// used for that transformation.
class PointMapper
{
public:
    // Requires Camera Extrinsic parameters from camera-config
    PointMapper(YAML::Node& config, zcm::ZCM &zcm);
    PointMapper(YAML::Node& config, zcm::ZCM &zcm, bool simulator);
    // Stops the zcm subscription
    ~PointMapper();
    // Returns the camera_to_world transformation matrix
    Eigen::Matrix4d getCameraToWorldMatrix(uint64_t utime);
    // Get the sensor origin used for octomap
    Eigen::Vector3d getCameraOrigin(uint64_t utime);
    // Transforms all points from the camera frame to the global
    // frame in the passed in point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr
        transformCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        uint64_t utime);
    // Unblocks the point mapper if waiting if program is being killed
    void kill();
private:
    // Queue of last 100 states
    std::deque<State> latest_states_;
    // Created during construction
    Eigen::Matrix4d camera_to_body_;
    // reference to zcm oject, used to receieve state messages and
    // update the latest states deque
    zcm::ZCM &zcm_;
    std::mutex mtx_;
    std::condition_variable cv_;
    // Used to find the state nearest to the passed in utime
    State nearestState(uint64_t utime);
    bool simulator_ = false; // used to subscribe to simulator zcm channels
    // Handler class for receiving the zcm messages and updating
    // the deque
    class Handler
    {
    public:
        Handler(std::deque<State>& latest_states, std::mutex& mtx,
                std::condition_variable& cv) :
            latest_states_ {latest_states}, mtx_ {mtx}, cv_ {cv}
        {

        }
        // Adds the latest state to the deque, and keeps the
        // deque at a max size of 100
        void handle(const zcm::ReceiveBuffer*, const std::string&,
                const state_t* message)
        {
            std::unique_lock<std::mutex>(mtx_);

            latest_states_.push_back(maav::gnc::ConvertState(*message));
            if (latest_states_.size() > 100)
            {
                latest_states_.pop_front();
            }
            // Wake the PointMapper if it is waiting
            cv_.notify_all();
        }

        // Adds the latest state to the deque, and keeps the
        // deque at a max size of 100. Simulator channel
        void handleSim(const zcm::ReceiveBuffer*, const std::string&,
                const groundtruth_inertial_t* message)
        {
            std::unique_lock<std::mutex>(mtx_);

            latest_states_.push_back(maav::gnc::ConvertGroundTruthState(*message));
            if (latest_states_.size() > 100)
            {
                latest_states_.pop_front();
            }
            // Wake the PointMapper if it is waiting
            cv_.notify_all();
        }
    private:
        std::deque<State>& latest_states_;
        std::mutex& mtx_;
        std::condition_variable& cv_;
    } handler_;
};
}

#endif
