// Octomap handler just updates the octomap and reruns a*
// State handler just updates the latest state
// Goal Handler updates the goal waypoint and attempts to rerun a*
// TODO THE BELOW DOES NOT EXIST YET
// Point Cloud handler runs the naive obstacle avoidance and puts the
//      vehicle into emergency evasion mode for a short while until it
//      is re-run and the obstacle is no longer nearby.
// Enemy drone handler updates enemy quadcopter locations and re-runs
// a* if one of the drones are close.
// Once enemy drone handler is added and related code, before each run of
// the a* will first update the recieved octomap with the potential
// locations of the drones

#include <atomic>
#include <common/messages/MsgChannels.hpp>
#include <common/messages/octomap_t.hpp>
#include <common/messages/path_t.hpp>
#include <common/messages/point_cloud_t.hpp>
#include <common/messages/state_t.hpp>
#include <common/utils/GetOpt.hpp>
#include <common/utils/ZCMHandler.hpp>
#include <condition_variable>
#include <gnc/Planner.hpp>
#include <gnc/measurements/Waypoint.hpp>
#include <gnc/planner/Path.hpp>
#include <gnc/State.hpp>
#include <gnc/utils/ZcmConversion.hpp>
#include <iostream>
#include <memory>
#include <mutex>
#include <octomap/OcTree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <thread>
#include <vision/core/utilities.hpp>
#include <zcm/zcm-cpp.hpp>

using maav::OCCUPANCY_MAP_CHANNEL;
using maav::STATE_CHANNEL;
using maav::PATH_CHANNEL;
using maav::GOAL_WAYPOINT_CHANNEL;
using maav::FORWARD_CAMERA_POINT_CLOUD_CHANNEL;
using maav::GT_INERTIAL_CHANNEL;
using maav::gnc::Planner;
using maav::vision::zcmTypeToOctomap;
using maav::vision::zcmTypeToPCLPointCloud;
using maav::gnc::State;
using maav::gnc::ConvertState;
using maav::gnc::ConvertGroundTruthState;
using maav::gnc::Waypoint;

using std::atomic;
using std::condition_variable;
using std::mutex;
using std::unique_lock;
using std::thread;
using std::shared_ptr;

using pcl::PointCloud;
using pcl::PointXYZ;

class GoalHandler;
class StateHandler;
class MapHandler;
class PointCloudHandler;

// Manages executions of astar only allowing one to occur at
// once asynchronously.
class AStarManager
{
public:
    AStarManager(Planner& planner, zcm::ZCM* zcm) : planner_ {planner},
        zcm_ {zcm} {}
    // Normal compute after new information arrives
    bool try_compute(bool updated_goal = false)
    {
        unique_lock<mutex> lck(mtx_);
        // In the case that this try doesn't actually cause an execution
        // remember for the end of the current execution that the goal
        // has been updated and therefore, should be run again
        if (updated_goal == true) updated_goal_ = true;
        if (running_) return false;
        running_ = true;
        // Creating a new execution, therefore, the latest goal is being
        // used
        updated_goal_ = false;
        lck.unlock();
        thread computation(compute, this, &running_);
        computation.detach();
        return true;
    }
    // Separate pool to allow instant response to threats
    bool emergency_compute()
    {
        unique_lock<mutex> lck(mtx_);
        if (emergency_running_) return false;
        emergency_running_ = true;
        lck.unlock();
        thread computation(compute, this, &emergency_running_);
        computation.detach();
        return true;
    }
    // Used to run a* and send the results in a detatched thread
    void static compute(AStarManager* self, bool* running);
    // Must be called before astar handler can be used
    void setHandlers(GoalHandler* goal_handler,
            MapHandler* map_handler,
            StateHandler* state_handler)
    {
        unique_lock<mutex> lck(mtx_);
        goal_handler_ = goal_handler;
        map_handler_ = map_handler;
        state_handler_ = state_handler;
    }
    Planner& planner_;
private:
    zcm::ZCM* zcm_;
    mutex mtx_;
    bool running_ = false;
    bool updated_goal_ = false;
    bool emergency_running_ = false;
    GoalHandler* goal_handler_ = nullptr;
    MapHandler* map_handler_ = nullptr;
    StateHandler* state_handler_ = nullptr;
};

// Receives new octomaps, replaces the old and tries to start a new
// compuation of a*
class MapHandler
{
public:
    explicit MapHandler(AStarManager& astar_manager) : astar_manager_ {astar_manager} {}
    void handle(const zcm::ReceiveBuffer*, const std::string&,
        const octomap_t* message)
    {
        unique_lock<mutex> lck(mtx_);
        run_ = true;
        octree_ = zcmTypeToOctomap(message);
        lck.unlock();
        astar_manager_.try_compute();
    }
    shared_ptr<octomap::OcTree> getMap()
    {
        unique_lock<mutex> lck(mtx_);
        return octree_;
    }
    bool run_ = false;
private:
    AStarManager& astar_manager_;
    mutex mtx_;
    shared_ptr<octomap::OcTree> octree_;
};

class StateHandler
{
public:
    explicit StateHandler(AStarManager& astar_manager) : astar_manager_ {astar_manager} {}
    // Updates the latest state and compares it to the target state
    // If close enough to the target state, for now attempt to
    // re-run a*, later may add additional behavior
    void handle(const zcm::ReceiveBuffer*, const std::string&,
        const state_t* message)
    {
        latest_state_ = ConvertState(*message);
        handle_impl();
    }
    void handle_impl()
    {
        unique_lock<mutex> lck(mtx_);
        run_ = true;
        // TODO Use config to set the value of distance before considered to
        // have reached the destination
        // TODO compute the distance from the goal
        lck.unlock();
    }
    void handleGTState(const zcm::ReceiveBuffer*, const std::string&,
        const groundtruth_inertial_t* message)
    {
        latest_state_ = ConvertGroundTruthState(*message);
        handle_impl();
    }
    // Returns the state in a thread safe manner
    State getState()
    {
        unique_lock<mutex> lck(mtx_);
        return latest_state_;
    }
    bool run_ = false;
private:
    AStarManager& astar_manager_;
    mutex mtx_;
    State latest_state_;
};

class GoalHandler
{
public:
    explicit GoalHandler(AStarManager& astar_manager) : astar_manager_ {astar_manager} {}
    void handle(const zcm::ReceiveBuffer*, const std::string&,
        const waypoint_t* message)
    {
        unique_lock<mutex> lck(mtx_);
        run_ = true;
        goal_ = maav::gnc::ConvertWaypoint(*message);
        lck.unlock();
            astar_manager_.try_compute(true);
    }
    // Returns the goal in a thread safe manner
    Waypoint getGoal()
    {
        unique_lock<mutex> lck(mtx_);
        return goal_;
    }
    bool run_ = false;
private:
    AStarManager& astar_manager_;
    mutex mtx_;
    Waypoint goal_;
};

// Receives point clouds from the front facing camera
// Runs a naive yet fast form of obstacle detection to
// avoid obstacles on short notice
class PointCloudHandler
{
public:
    explicit PointCloudHandler(AStarManager& astar_manager) : astar_manager_ {astar_manager} {}
    void handle(const zcm::ReceiveBuffer*, const std::string&,
        const point_cloud_t* message)
    {
        unique_lock<mutex> lck(mtx_);
        cloud_ = zcmTypeToPCLPointCloud(*message);
        // Run naive obstacle detection and run emergency logic if necessary
        // TODO
        // Should implement this in a class outside of this file
    }
private:
    AStarManager& astar_manager_;
    mutex mtx_;
    PointCloud<PointXYZ>::Ptr cloud_;
};

// TODO Add yolo obstacle handler once more is known about it

// Compute implemented here because it needs to know all info
// from the Handler classes
void AStarManager::compute(AStarManager* self, bool* running)
{
    std::cout << "Started compute thread" << std::endl;
    // Used make sure that running is set to false once this is exited
    class FalseRAI
    {
    public:
        FalseRAI(bool& in, mutex& mtx) : asset_ {in},
            mtx_ {mtx} {}
        ~FalseRAI()
        {
            unique_lock<mutex> lck(mtx_);
            asset_ = false;
        }
    private:
        bool& asset_;
        mutex& mtx_;
    };
    FalseRAI rai(*running, self->mtx_);
    // Do the a* computation and send the results over zcm
    // First check that all the handlers have been run at least once
    if (!self->map_handler_->run_ || !self->state_handler_->run_ ||
        !self->goal_handler_->run_)
        return;
    unique_lock<mutex> lck(self->mtx_);
    // Run at least once, keep running while the goal has been updated
    do
    {
        std::cout << "About to run astar." << std::endl;
        self->updated_goal_ = false;
        lck.unlock();
        self->planner_.update_map(self->map_handler_->getMap());
        self->planner_.update_state(self->state_handler_->getState());
        self->planner_.update_target(self->goal_handler_->getGoal());
        Path p = self->planner_.get_path();
        planner_.print_path();
        path_t path = maav::gnc::ConvertPath(p);
        self->zcm_->publish(PATH_CHANNEL, &path);
        std::cout << "Published path" << std::endl;
        lck.lock();
    } while (self->updated_goal_ == true);
    std::cout << "Finished compute thread" << std::endl;
}

// Keeps track of whether the kill signal has been received
atomic<bool> KILL{false};
mutex KILL_mutex;
condition_variable KILL_cv;
void sigHandler(int)
{
    unique_lock lck(KILL_mutex);
    KILL = true;
    KILL_cv.notify_one();
}

int main(int argc, char** argv)
{
    // Set up getopt
    GetOpt gopt;
    gopt.addBool('h', "help", false, "This message");
    gopt.addString('c', "config", "", "Path to config.");
    // Parse getopt and check for help flag being passed
    if (!gopt.parse(argc, argv, 1) || gopt.getBool("help"))
    {
        std::cout << "Guidance Driver" << std::endl;
        std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
        gopt.printHelp();
        return 1;
    }

    Planner planner(gopt.getString("config"));

    // Set up zcm

    zcm::ZCM zcm{"ipc"};

    AStarManager astar_manager(planner, &zcm);
    StateHandler state_handler(astar_manager);
    MapHandler map_handler(astar_manager);
    GoalHandler goal_handler(astar_manager);
    PointCloudHandler point_cloud_handler(astar_manager);
    astar_manager.setHandlers(&goal_handler, &map_handler, &state_handler);

    zcm.subscribe(OCCUPANCY_MAP_CHANNEL, &MapHandler::handle, &map_handler);
    // TODO Use when not testing with sim
    // zcm.subscribe(STATE_CHANNEL, &StateHandler::handle, &state_handler);
    zcm.subscribe(GT_INERTIAL_CHANNEL, &StateHandler::handleGTState, &state_handler);
    zcm.subscribe(GOAL_WAYPOINT_CHANNEL, &GoalHandler::handle, &goal_handler);
    zcm.subscribe(FORWARD_CAMERA_POINT_CLOUD_CHANNEL, &PointCloudHandler::handle,
        &point_cloud_handler);

    zcm.start();

    while (!KILL)
    {
        unique_lock lck(KILL_mutex);
        KILL_cv.wait(lck);
    }

    zcm.stop();
}
