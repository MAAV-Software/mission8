#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <memory>
#include <mutex>
#include <thread>
#include <tuple>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <zcm/zcm-cpp.hpp>

#include "common/messages/MsgChannels.hpp"
#include "common/messages/heartbeat_t.hpp"
#include "common/messages/nav_runstate_t.hpp"
#include "common/messages/plane_fit_t.hpp"
#include "common/messages/point_cloud_t.hpp"
#include "vision/depth-utils/PlaneFitter.hpp"

using std::atomic;
using std::make_tuple;
using std::shared_ptr;
using std::thread;
using std::tuple;
using std::chrono::milliseconds;
using zcm::ZCM;

// Keeps track of whether the kill signal has been received
atomic<bool> KILL{false};
void sigHandler(int) { KILL = true; }
// Used to get the latest job sent to this driver
// and ignore jobs that there was not enough time to do
class JobDispatcher
{
public:
    // Returns a tuple that holds the point cloud and the time it was recorded
    tuple<pcl::PointCloud<pcl::PointXYZ>::Ptr, long long> waitForTask()
    {
        std::unique_lock<std::mutex> lk(mtx_);
        while (!ready_)
        {
            cv_.wait(lk);
        }
        // Record that task has been taken
        ready_ = false;
        // Return task to be done
        return tuple<pcl::PointCloud<pcl::PointXYZ>::Ptr, long long>(data_, utime_);
    }
    // Updates the held task and notifies any waiting thread
    // that a new task is available to be run
    // Only works with one thread
    // Used to cause driver to work on latest request
    // and ignore requests that it does not have enough time
    // to do
    void addTask(const point_cloud_t* const dataIn)
    {
        // For thread safety
        std::lock_guard<std::mutex> lk(mtx_);
        // Create new point cloud to store in data
        data_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        for (unsigned int i = 0; i < static_cast<unsigned int>(dataIn->size); i++)
        {
            data_->push_back(pcl::PointXYZ(
                dataIn->point_cloud[i].x, dataIn->point_cloud[i].y, dataIn->point_cloud[i].z));
        }
        utime_ = dataIn->utime;
        ready_ = true;
        // Class is only meant to work with one receiver
        cv_.notify_one();
    }

private:
    std::condition_variable cv_;
    std::mutex mtx_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr data_;
    long long utime_;
    bool ready_ = false;
};

// Handler class that when receiving a zcm message
// will update the job dispatcher pointed to by its member pointer
// with the new task data (point cloud) that it has recieved
class Handler
{
public:
    // The task dispatcher associated with this handler
    shared_ptr<JobDispatcher> dispatcher;
    // Pass in shared pointer to the job dispatcher
    Handler(shared_ptr<JobDispatcher> dispatcherIn) : dispatcher{dispatcherIn} {}
    // Updates job dispatcher with new task data
    void handle(const zcm::ReceiveBuffer*, const std::string&, const point_cloud_t* message)
    {
        dispatcher->addTask(message);
    }
};

// Sends heartbeats every 100 milliseconds
// Also the main thread waits for this to end
// to know when the kill signal has been sent
void runHeartbeat(shared_ptr<ZCM> zcm)
{
    heartbeat_t msg;
    msg.alive = true;
    while (!KILL)
    {
        zcm->publish(maav::PLANE_FITTER_HEARTBEAT_CHANNEL, &msg);
        std::this_thread::sleep_for(milliseconds(100));
    }
}

void runFitPlane(shared_ptr<JobDispatcher> dispatcher, shared_ptr<ZCM> zcm, shared_ptr<ZCM> zcm_udp)
{
    // Initialize plane fitter
    maav::vision::PlaneFitter planeFitter(0.1f);
    // Run plane fitting whenever possible (using JobDispatcher) to prevent
    // jobs from pilling up that it takes too long too handle
    while (!KILL)
    {
        plane_fit_t output;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
        long long utime;
        tuple<pcl::PointCloud<pcl::PointXYZ>::Ptr, long long> out;
        std::tie(cloud, utime) = dispatcher->waitForTask();
        // out = dispatcher->waitForTask();
        // Run the plane fitter, upon success, send out new orientation data
        if (planeFitter.runPlaneFitting(
                cloud, output.z_dot, output.z, output.roll, output.pitch, utime))
        {
            output.utime = utime;
            zcm->publish(maav::PLANE_FIT_CHANNEL, &output);
            zcm_udp->publish(maav::PLANE_FIT_CHANNEL, &output);
        }
    }
}

int main()
{
    // Bind sigHandler
    signal(SIGINT, sigHandler);
    signal(SIGABRT, sigHandler);
    signal(SIGSEGV, sigHandler);
    signal(SIGTERM, sigHandler);
    // Initialize shared stuff
    shared_ptr<ZCM> zcm = shared_ptr<ZCM>(new ZCM("ipc"));
    shared_ptr<ZCM> zcm_udp = shared_ptr<ZCM>(new ZCM("udpm://239.255.76.67:7667?ttl=1"));
    shared_ptr<JobDispatcher> dispatcher = shared_ptr<JobDispatcher>(new JobDispatcher());
    // Subscribe and start zcm receive loop
    Handler handler(dispatcher);
    zcm->subscribe(maav::DOWNWARD_CAMERA_POINT_CLOUD_CHANNEL, &Handler::handle, &handler);
    zcm->start();
    // Start processing threads and heartbeat thread
    thread th1(runFitPlane, dispatcher, zcm, zcm_udp);
    thread th2(runHeartbeat, zcm);
    // Prevent main from ending until kill signal is received
    th2.join();
    zcm->stop();
    th1.detach();
}
