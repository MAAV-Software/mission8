#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <math.h>

#include <yaml-cpp/yaml.h>
#include <zcm/zcm-cpp.hpp>

#include <gnc/measurements/Waypoint.hpp>
#include <gnc/State.hpp>

#include <common/messages/MsgChannels.hpp>
#include <common/utils/GetOpt.hpp>
#include <common/messages/waypoint_t.hpp>
#include <common/messages/state_t.hpp>
#include <common/messages/point_cloud_t.hpp>
#include <common/messages/point_t.hpp>
#include <common/messages/groundtruth_inertial_t.hpp>
#include <common/math/math.hpp>
#include <common/math/angle_functions.hpp>
#include <gnc/utils/ZcmConversion.hpp>

#include <vision/obstacle/NaiveObstacle.hpp>
#include <vision/core/utilities.hpp>

using YAML::Node;
using maav::gnc::Waypoint;
using std::cin;
using std::cout;
using std::endl;
using std::string;
using std::stringstream;
using std::this_thread::sleep_for;
using std::to_string;
using std::vector;
using std::mutex;
using std::condition_variable;
using std::unique_lock;
using std::deque;
using maav::GOAL_WAYPOINT_CHANNEL;
using maav::STATE_CHANNEL;
using maav::GT_INERTIAL_CHANNEL;
using maav::FORWARD_CAMERA_POINT_CLOUD_CHANNEL;
using maav::gnc::State;
using maav::deg_to_rad;
using maav::rad_to_deg;
using maav::PI;
using namespace std::chrono;
using namespace eecs467;

using pcl::PointXYZ;
using pcl::PointCloud;
using maav::vision::NaiveObstacle;

class StateMachine
{
public:
    StateMachine(zcm::ZCM* zcm) : history_ {deque<int>(3, 0)}, zcm_ {zcm}
    {
        current_target_.pose[0] = 0;
        current_target_.pose[1] = 0;
        current_target_.pose[2] = -1;
        current_target_.pose[3] = 0;

        current_target_.rate[0] = 0;
        current_target_.rate[1] = 0;
        current_target_.rate[2] = 0;
        current_target_.rate[3] = 0;

        current_target_.pmode = 0;
        current_target_.mode = 0;

        memset(&current_state_, 0, sizeof(state_t));
        current_state_.attitude.data[2] = 1;
    }
    void updatedCurrentState(state_t new_state)
    {
        unique_lock<mutex> lck(mtx_);
        current_state_ = new_state;
    }
    void updatedHistory(bool obstacle_exists)
    {
        unique_lock<mutex> lck(mtx_);
        if (obstacle_exists)
        {
            emergency();
            history_.push_back(obstacle_exists ? 1 : 0);
            history_.pop_front();
        }
        else if (internalSummarizeHistory() > 0)
        {
            history_.push_back(obstacle_exists ? 1 : 0);
            history_.pop_front();
            int after = internalSummarizeHistory();
            // Check history, if before greater than zero and after equal to zero
            if (after == 0)
            {
                exitEmergency();
            }
        }
        else
        {
            /* do nothing */
        }
    }
    int summarizeHistory()
    {
        unique_lock<mutex> lck(mtx_);
        return internalSummarizeHistory();
    }
    void takeoff()
    {
        current_target_.pose[0] = 0;
        current_target_.pose[1] = 0;
        current_target_.pose[2] = -1;
        current_target_.pose[3] = 0;
        path_t wpt_path = packageInPath(current_target_);
        zcm_->publish(maav::PATH_CHANNEL, &wpt_path);
    }
    void approachObstacle()
    {
        current_target_.pose[0] = 1;
        path_t wpt_path = packageInPath(current_target_);
        zcm_->publish(maav::PATH_CHANNEL, &wpt_path);
    }
private:
    int internalSummarizeHistory()
    {
        int summary = 0;
        for (int element : history_)
        {
            summary += element;
        }
        return summary;
    }
    void exitEmergency()
    {
        // then is no longer in emergency and
        // Can move forward 2 meters, therefore, send and set new goal waypoint 2 meters 
        // forward
        double yaw = get_heading(maav::gnc::ConvertState(current_state_));
        Eigen::Vector2d posChange;
        posChange[0] = 2;
        posChange[1] = 0;
        Eigen::Matrix2d rot;
        rot << cos(yaw), -sin(yaw),
                sin(yaw), cos(yaw);
        posChange = rot * posChange;
        current_target_.pose[0] = posChange.x();
        current_target_.pose[1] = posChange.y();
        current_target_.pose[3] = yaw;
        path_t wpt_path = packageInPath(current_target_);
        zcm_->publish(maav::PATH_CHANNEL, &wpt_path);
    }
    void emergency()
    {
        // If not close enough to goal yaw, do nothing
        double diff = angle_diff_abs(get_heading(maav::gnc::ConvertState(current_state_)), 
            current_target_.pose[3]);
        if (diff > (PI / 26))
        {
            return;
        }
        // Yaw and send new yawed goal
        // Also make sure stops moving forward
        current_target_.pose[0] = current_state_.position.data[0];
        current_target_.pose[1] = current_state_.position.data[1];
        current_target_.pose[3] += (PI / 6);
        current_target_.pose[3] = wrap_to_2pi(current_target_.pose[3]);
        path_t wpt_path = packageInPath(current_target_);
        zcm_->publish(maav::PATH_CHANNEL, &wpt_path);
    }
    // Helper to get heading out of state
    static double get_heading(const State& state)
    {
        double q0 = state.attitude().unit_quaternion().w();
        double q1 = state.attitude().unit_quaternion().x();
        double q2 = state.attitude().unit_quaternion().y();
        double q3 = state.attitude().unit_quaternion().z();
        return wrap_to_2pi(atan2((q1 * q2) + (q0 * q3), 0.5 - (q2 * q2) - (q3 * q3)));
    }
    static path_t packageInPath(waypoint_t& wpt)
    {
        path_t wpt_path;
        wpt_path.NUM_WAYPOINTS = 1;
        wpt_path.waypoints.push_back(wpt);
        return wpt_path;
    }
    static double mode2PI(double rads)
    {
        return wrap_to_2pi(rads);
    }
    deque<int> history_;
    zcm::ZCM* zcm_;
    waypoint_t current_target_;
    state_t current_state_;
    mutex mtx_;
};

class PointCloudHandler
{
public:
    PointCloudHandler(StateMachine& state_machine) : obstacle_detector_ {NaiveObstacle()}, 
        state_machine_ {state_machine} {}
    void handle(const zcm::ReceiveBuffer*, const std::string&,
        const point_cloud_t* message) 
    {
        if (!running)
        {
            running = true;
            std::thread handleThread(handleImpl, *message, this);
            handleThread.detach();
        }
    }
    static void handleImpl(const point_cloud_t message, PointCloudHandler* handler)
    {
        PointCloud<PointXYZ>::Ptr cloud = maav::vision::zcmTypeToPCLPointCloud(message);
        auto obstacles = handler->obstacle_detector_.detectObstacles(cloud);
        handler->state_machine_.updatedHistory(!obstacles.empty());
        handler->running = false;
    }
private:
    NaiveObstacle obstacle_detector_;
    StateMachine& state_machine_;
    std::atomic_bool running = false;
};

class StateHandler
{
public:
    StateHandler(StateMachine& state_machine) : state_machine_ {state_machine} {}
    void handle(const zcm::ReceiveBuffer*, const std::string&,
        const state_t* message)
    {
        state_machine_.updatedCurrentState(*message);
    }
    void handleInertial(const zcm::ReceiveBuffer*, const std::string&,
        const groundtruth_inertial_t* message)
    {
        state_machine_.updatedCurrentState(
            maav::gnc::ConvertState(maav::gnc::ConvertGroundTruthState(*message)));
    }
private:
    StateMachine& state_machine_;
};

int main(int argc, char** argv)
{
    // Send the takeoff command serveral times for 10 seconds to guarantee that
    // the vehicle is flying
    // Produce no new goal waypoints until the last goal is reached
    // If an obstacle is detected, then turn to the left until an obstacle is no longer detected
    // Then move forward 2 meters and go back to holding that position
    zcm::ZCM zcm{"ipc"};
    StateMachine state_machine(&zcm);
    PointCloudHandler point_cloud_handler(state_machine);
    StateHandler state_handler(state_machine);
    for (size_t i = 0; i < 5; ++i)
    {
        state_machine.takeoff();
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    // state_machine.approachObstacle();
    zcm.subscribe(FORWARD_CAMERA_POINT_CLOUD_CHANNEL,
        &PointCloudHandler::handle, &point_cloud_handler);
    zcm.subscribe(STATE_CHANNEL, &StateHandler::handle, &state_handler);
    zcm.subscribe(GT_INERTIAL_CHANNEL, &StateHandler::handleInertial, &state_handler);
    zcm.run();
}